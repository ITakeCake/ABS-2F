local M = {}
M.type = "auxiliary"
M.version = "1.00"

-- PID ABS, no-cheat. Uses fusedSpeed only (no virtualAirspeed).
-- detectMu runs at 2kHz (sensor fusion -> fusedSpeed)
-- runTick runs at 200Hz (per-wheel PID + peak-decel D estimator)
--
-- (2): All vehicle-specific hardcodes removed. Init now builds wheel maps,
-- geometry, and static loads dynamically from wheels.wheelRotatorIDs,
-- wheels.wheelRotators[i].node1/node2, obj:getNodePositionRelative(),
-- obj:getMass(), and jbeamData. Falls back to safe universal defaults
-- when any lookup fails. No dynamic (per-tick) cheating anywhere.

local TICK_RATE_HZ = 200
local TICK_STEP = 1 / TICK_RATE_HZ
local timeAccum = 0

local origBrakeTorque = {}

-- wheelToBrakeMap[logicalIdx] = wheelRotator index (1-based)
-- Logical order: 1=RR, 2=RL, 3=FR, 4=FL  (our convention throughout)
-- Built dynamically in buildWheelMaps() via wheels.wheelRotatorIDs.
local wheelToBrakeMap = {1, 2, 3, 4}

-- Which logical indices are rear / front wheels
local rearLogicalIndices  = {1, 2}   -- default RR=1, RL=2
local frontLogicalIndices = {3, 4}   -- default FR=3, FL=4

local N_WHEELS = 4   -- set in init(); all tables sized to this

local wasBraking = false

-- Shared sensor state (detectMu writes at 2kHz, runTick reads at 200Hz)
local fusedSpeed = 0
local fusedPrevWs = {}
local fusedInitialized = false
local latestWheelSpeed = {}
local latestSensorY = 0
local wasBrakingMu = false

-- Snap-up gap gate (TEST): reject a fused up-snap whose one-tick gap to the fastest wheel
-- exceeds this. 0.75 m/s @ 2kHz implies a "speed-up" of ~1500 m/sÂ² â€” impossible for a braking
-- car, so it can only be a corrupted/spiking wheel reading. Real re-grips are <= ~0.33 m/s/tick,
-- so this never blocks a legitimate re-anchor. Equivalent to the (maxWs-fused)/dt accel bound.
local SNAP_GAP_MAX = 0.75
-- Snap-up low-speed cutout: don't re-anchor fused up to a wheel when BOTH fused and the wheel
-- average are essentially stopped (< 2.5 mph). Near standstill, locked/creeping wheels and sensor
-- noise can throw a spuriously-faster wheel reading; suppressing the up-snap there keeps fused from
-- twitching back up as the car settles to a stop.
local SNAP_MIN_SPEED = 1.1176   -- m/s = 2.5 mph

-- IMU speed ceiling (anti-wheelspin). imuSpeed integrates the accelerometer alone, so it is
-- immune to wheelspin; fused is capped at EXACTLY imuSpeed so pre-brake wheelspin (e.g. floor
-- it on water, then brake) can never inflate fused above what the car's own acceleration
-- actually supports. imuSpeed resyncs to the wheels only while they're trustworthy â€” a wheel
-- within IMU_TRUST_WINDOW above imuSpeed is "gripping" (resync); further above = "spinning".
local imuSpeed = 0
local IMU_TRUST_WINDOW = 2.0   -- m/s: how far a wheel may lead imuSpeed and still be trusted
-- Slip-anchored fused stepdown: during braking the wheels lag true speed by ~the commanded slip.
-- If fused sits further above the fastest wheel than that slip explains, sustained, fused has
-- drifted -> snap fused + IMU down to the wheel-implied true speed. A discrete reset (like the
-- off-brake resync), NOT a continuous override. All tunables live here (expect frequent tweaking).
local ENABLE_SLIP_STEPDOWN   = true
local SLIP_STEPDOWN_RATIO    = 1.01   -- fire when measured gap > this * expected(slip-explained) gap
local SLIP_STEPDOWN_SUSTAIN  = 0.02   -- s: gap must stay over threshold this long before firing
local SLIP_STEPDOWN_COOLDOWN = 0.30   -- s: no re-fire during this window after a correction
local SLIP_STEPDOWN_VMIN     = 2.2352 -- m/s: only active above this speed (= 5 mph)
local SLIP_STEPDOWN_GAPFLOOR = 0.5    -- m/s: ignore gaps smaller than this (noise / tiny s_t)
local SLIP_STEPDOWN_STFLOOR  = 0.03   -- min commanded slip target used in the expected-gap calc
local SLIP_STEPDOWN_SNAPFRAC = 0.5    -- where in the [fastest wheel, slip-implied speed] window to
                                      -- land fused on a fire. 0 = snap to the wheel (max down),
                                      -- 1 = full slip-implied speed (old behavior), 0.5 = middle.
local slipStepdownTimer    = 0
local slipStepdownCooldown = 0
local slipStepdownCount    = 0
-- IMU ceiling slew: fused is pulled DOWN toward imuSpeed at no more than this rate (m/s^2),
-- instead of snapping instantly. Below a car's real max decel (~11 m/s^2), so a brief imuSpeed
-- undershoot can't collapse fused to 0, while a genuine over-read still corrects in a few tenths.
local IMU_CLAMP_RATE = 8.0     -- m/s^2

-- Gradient re-anchor: a sustained one-sign pitch rate (crest/dip/bump) corrupts the body-axis
-- integral. Once the pitch event settles and the tires carry load again, snap fused ONCE to the
-- slip-corrected fastest wheel (bounded), both directions. +sensorZ = downward accel.
local ra = {
  ENABLE      = true,
  TAU         = 0.02,    -- s: EMA on pitch rate (finite-differenced, noisy)
  RATE_ON     = 0.15,    -- rad/s: gradient is changing
  ON_MIN_S    = 0.04,    -- s: one-sign persistence before it counts as an event
  RATE_OFF    = 0.05,    -- rad/s: settled
  SETTLE_S    = 0.05,    -- s: settled + loaded this long => fire
  SUPPORT_MIN = 6.9,     -- m/s^2: ~0.7 g tire support required to trust wheels
  MAX_FRAC    = 0.25,    -- cap |correction| at this fraction of fused
  COOLDOWN_S  = 0.20,
  VMIN        = 2.2352,  -- m/s (5 mph)
  SLIP_FRAC   = 0.15,    -- fastest wheel logged at ~2-3% slip under a 0.25 target
  ONSET_ENABLE = true,   -- brake onset: wheels carry no brake torque yet => downward-only snap to fastest wheel
  ONSET_SLIP  = 0.02,
  onsetFires  = 0,
  maxWs = 0, avgWs = 0,
  rate = 0, az = 0, support = 9.81, sign = 0, onTime = 0, armed = false,
  settleTime = 0, cooldown = 0, fires = 0, lastDelta = 0,
}

-- Airborne axle: pre-arm, don't pre-load. Both wheels of an axle holding speed (or spinning down
-- torque-only) while body support < 0.7 g => axle in the air. Output FLOOR, freeze PID state; on
-- touchdown re-arm the integral so duty resumes at REARM_DUTY instead of ramping from zero.
local air = {
  ENABLE       = false,
  FLOOR        = 0.0,     -- absCoef while airborne (0 = free wheel lands at road speed)
  SUPPORT_MIN  = 6.9,     -- m/s^2: below this the body is not on its tires
  HOLD_ACC     = 3.0,     -- m/s^2: |wheel accel| under this = holding speed
  BODY_DECEL   = 3.0,     -- m/s^2: car must be decelerating for "holding" to mean airborne
  SPINDOWN_ACC = 40.0,    -- m/s^2: rim spinning down faster than any loaded wheel
  SPINUP_ACC   = 30.0,    -- m/s^2: touchdown spin-up ends the event
  ENTRY_S      = 0.05,    -- s: both wheels must qualify this long (bump chatter is shorter)
  FLOOR_EARLY  = 0.025,   -- absCoef for the first FLOOR_EARLY_S of a flight (short hops: pads stay engaged)
  FLOOR_EARLY_S = 0.10,
  EXIT_S       = 0.01,    -- s: supported this long = landed
  TIMEOUT_S    = 0.5,
  REARM_DUTY   = 0.5,     -- integral seeded so duty >= this at touchdown
  wAcc = {0, 0, 0, 0}, tick = {0, 0, 0, 0},
  front = false, rear = false, frontT = 0, rearT = 0, supportedT = 0,
  events = 0, frontEvents = 0, rearEvents = 0,
}

-- Flight gate: brakes on, body support outside the flat band, and both wheels of an axle spinning
-- down faster than a loaded wheel can (per-wheel line = SAFETY * k_i * applied torque, k_i from jbeam
-- radius/inertia). A: hold the D-estimator through flight AND landing. B: once support has come
-- back into band from the landing spike, snap fused to the fastest wheel * (1 + B_PCT).
local fg = {
  ENABLE_FG  = true,
  A_ENABLE   = false,
  A_MODE     = "flight",  -- "hold": freeze D while gate open; "flight": no freeze, restore pre-event D once at landing of a real flight
  B_ENABLE   = true,
  B_PCT      = 0.03,
  B_MODE     = "above",   -- "above": need a landing spike first; "any": first return to band
  TRIGGER    = "support", -- "wheel": support band + free-wheel spin-down; "support": support band alone
  FLIGHT_SUP = 5.0,       -- m/s^2: support below this = real flight (B requires it when TRIGGER=support)
  REL_DUTY   = 0.05,      -- "release" trigger: duty under this ...
  REL_S      = 0.04,      -- ... for this long, wheel not spinning back up ...
  REL_SPINUP = 5.0,       -- m/s^2: accel above this = tire has re-gripped
  REL_SLIP   = 0.10,      -- ... still this far below fused ...
  REL_MEM_S  = 0.10,      -- ... and support left the band within this window
  relT = {0, 0, 0, 0}, wasRel = {false, false, false, false}, outBandAgo = 9,
  sawLow     = false,
  B_SETTLE   = 0.03,      -- s in band before the dump
  BAND_LO    = 6.8,  BAND_HI = 12.8,   -- m/s^2 support, flat measured 8.7-10.3
  FLOOR      = 250,       -- m/s^2: highest loaded-wheel spin-down seen on any surface
  SAFETY     = 0.7,
  TIRE_CORR  = 0.55,      -- measured free decel / (r/I) on etk800 fronts (tire inertia not in wd.inertia)
  DWELL      = 0.01,      -- s both wheels must qualify
  HOLD_S     = 0.10,      -- s in band before unlocking D
  TIMEOUT_S  = 1.0,
  k = {0, 0, 0, 0}, wAcc = {0, 0, 0, 0}, tick = {0, 0, 0, 0},
  open = false, openT = 0, inBandT = 0, sawHigh = false, dumped = false, lockD = 1.0, dumpTo = nil,
  events = 0, frontEvents = 0, rearEvents = 0, dumps = 0,
}

-- 2D planar speed (drift handling). A sliding/yawing car corrupts a single-axis forward-speed
-- integral via the yaw-Coriolis term (lateral velocity * yaw rate). Tracking lateral velocity
-- (vLat) and feeding that term back keeps the forward estimate honest through a slide, so fused
-- can't inflate while the back end is out. Toggle OFF => bit-identical to plain 1F integration.
-- Sign of sensorX and yaw rate must be verified before trusting (see notes).
local ENABLE_2D_SPEED = true
                                -- survive a multi-second open-loop slide). Wheels are the only
                                -- reliable speed source; keep IMU as a short re-anchored ceiling.
local fwdVel = 0               -- estimated forward velocity, body frame (m/s) â€” internal 2D state
local vLat = 0                 -- estimated lateral (sideslip) velocity, body frame (m/s)

local vVert = 0

-- IMU input filtering (change #2). Filters the finite-differenced pitch/roll rates (the dominant
-- vVert noise source: differentiation amplifies noise and it's multiplied by fwdVel below) and the
-- vertical accel (vVert enters imuSpeed SQUARED, so its noise always biases the ceiling UPWARD).
-- ax (forward) and ay (lateral) are deliberately NOT EMA-filtered: lag there would delay
-- brake-onset / slide-onset detection. All EMAs are dt-scaled => rate-independent behavior.
local ENABLE_IMU_FILTERS = false  -- V1.03 test: filters off to isolate regression vs V1.01
local TAU_ROT = 0.02         -- s: EMA time constant for pitch/roll rates
local TAU_AZ  = 0.05         -- s: EMA time constant for vertical accel
local ACCEL_DEADBAND = 0.05  -- m/s^2: kills stationary sensor creep on ay/az
local filtPitchRate = 0
local filtRollRate  = 0
local filtAz        = 0
local lastPitch = 0
local lastRoll = 0
local pitchRateLog = 0
local rollRateLog = 0
local pitchLog = 0
local rollLog = 0
local ENABLE_IMU_LOG = false  -- writes abs_imu_log_*.csv per stop (development only)
local isLogging = false
local logData = {}
local logTimer = 0
-- imuSpeed reports GROUND SPEED = sqrt(fwdVel^2 + vLat^2); pure yaw rotates speed between the two
-- axes and conserves the magnitude, so a sideways slide keeps the estimate at true speed (not 0).

-- Reverse support: bypass PID in reverse, handle arcade-mode input routing correctly.
local motionDirection = 1   -- +1 forward, -1 reverse (hysteresis)
local REVERSE_DETECT_THRESHOLD = 1.0   -- m/s â€” min wheel speed to trust direction signal
local REVERSE_LOCKIN_MPS       = 0.5   -- m/s â€” signed avg must exceed this to flip

-- Wheel decel lockup guard: if wheel decels faster than this, override PID and cut brake
local WHEEL_DECEL_LIMIT = -100  -- m/sÂ²

-- Brake-event recorder: FIFO of last 4 events with peak fused-vs-airspeed divergence.
-- Published to UI via guihooks.trigger('updateABSBrakeEvents', {events, live}).
local brakeEvents = {}
local currentBrakeEvent = nil
local brakeSimTime = 0
local brakeUiAccum = 0
local BRAKE_EVENT_MIN_AIR = 1.0           -- m/s: don't even start an event below this
local BRAKE_EVENT_MIN_PEAK_AIR = 2.2352   -- m/s (= 5 mph): don't update peak divergence below this
local BRAKE_EVENT_MIN_DUR = 0.3           -- s
local BRAKE_EVENT_PRESS_THRESHOLD = 0.05
local BRAKE_EVENT_MAX = 6                 -- FIFO size
local BRAKE_EVENT_FILE = "settings/dynamic_abs_brake_events.json"

-- wheelAvg speed estimator. Per-wheel lock flag with decel-buffer unlock:
--   LOCK   : wDecel < LOCK_DECEL
--   UNLOCK : latest buffer entry >= 0 AND at least one positive in buffer
-- "Not locked" wheels go into wheelAvg. wheelAvg is for UI, PID uses fusedSpeed.
local wa = {
  speed = 0,
  updated = false,
  speedTwo = 0,
  twoUpdated = false,
  prevWs = {},
  locked = {},
  decelBuf = {},
  decelIdx = {},
  LOCK_WINDOW = 8,       -- 40ms at 200Hz
  LOCK_DECEL = -5,       -- hard-lockup threshold (m/sÂ²)
}

-- UI state: which speed source PID is using this tick
local absSpeedSource = { wheelAvgActive = false, fusedActive = true }

-- Safety + counters: bundled to stay under LuaJIT's 60-upvalue limit
local safety = {
  snapUp = 0, snapUpRej = 0,   -- snapUpRej = up-snaps rejected by the gap gate
  imuClamps = 0,               -- times the IMU ceiling capped fused (anti-wheelspin)
  slipRatios = {},
  lastAbsCoefs = {},
  -- Low-speed brake boost: naturally deepens slip below 30mph to aid final stopping without PID windup
  lowSpeed = { BOOST_MAX = 1.25, SPEED_THRESH = 13.41 },  -- 1.25 = golden V1.00 low-speed boost (restored)
}

local lastLowSpeedBoost = 1.0

-- Per-wheel surface-grip detector (ANCHORED brake-acceptance).
-- The global consensusD (IMU peak-decel) sets the absolute LEVEL; per-wheel
-- brake-acceptance carves the SPLIT. grip proxy g_i = appliedTorque_i / Fz_i (~ mu_i),
-- where appliedTorque_i = absCoef_i * origBrakeTorque_i (PID output + known constant) and
-- Fz_i = static corner load + longitudinal transfer from the global decel we already have.
-- Honest signals only â€” no per-wheel downForce. Bundled in one table to stay under
-- LuaJIT's 60-upvalue limit. wheel order 1=RR,2=RL,3=FR,4=FL.
--
-- (2): FRONT_FRAC, H_CG, and WHEELBASE are now built in init()
-- from real vehicle geometry (wheel axle node positions + obj:getMass()).
-- None of these values are updated at runtime â€” static config only.
local grip = {
  ENABLE_PERWHEEL_D = false,  -- master switch; false => original global broadcast (escape hatch)
  FRONT_FRAC = 0.5,           -- static front weight fraction â€” computed in init(), fallback 0.5
  H_CG = 0.55,                -- CG height (m) â€” read from jbeamData or universal fallback
  WHEELBASE = 2.6,            -- wheelbase (m) â€” computed from axle node positions in init()
  GRAV = 9.81,
  FZ_MIN = 200, SAT = 0.97, ROLL_MIN = 1.0, TORQUE_MIN_FRAC = 0.05,
  G_SMOOTH = 0.9, DECAY = 0.05, EPS = 1e-3,
  mass = 1500,
  Fz0 = {},                    -- per-wheel static load (set in init)
  gEMA = {},
  Dwheel = {},
  confident = {},
  yawRate = 0,                 -- last read yaw rate (rad/s), published for sign-check
}

-- Per-wheel PID state
local slipIntegral = {}
local lastSlipError = {}
local prevTickWheelSpeed = {}
local lastEffectiveTargets = {0, 0, 0, 0}
local lastSlipErrors = {0, 0, 0, 0}
local lastSlipDerivatives = {0, 0, 0, 0}



-- Per-wheel adaptive slip targets (D-estimator writes, PID reads)
local slipTargets = {}
local SLIP_TARGET_MIN = 0.02
local SLIP_TARGET_MAX = 1.0
local TARGET_SMOOTHING = 0.95

-- [FLAT TARGET EXPERIMENT 2026-08-26] Bypass the D->target formula (and hunter trim)
-- with a flat base target on all four wheels. Deepener still applies on top at the use site.
-- Set USE_FIXED_SLIP_TARGET = false to restore the stock V1.15 D-law behavior.
local USE_FIXED_SLIP_TARGET = false
local FIXED_SLIP_TARGET = 0.145

-- Low-speed slip deepening: the (1+D)/carSpeed term added to effectiveTarget, which raises the
-- allowable slip as speed falls so ABS ramps out toward a firm stop near standstill. When OFF,
-- effectiveTarget == slipTargets, so the stepdown safety uses the base slipTargets with zero
-- first-order fused-dependence (1:1 with what the PID commands). If you turn this back ON, real
-- slip deepens at low speed and the base target under-predicts it -> switch the safety's s_t back
-- to lastEffectiveTargets (see the slip-stepdown block) or it will false-fire at low speed.
local ENABLE_LOWSPEED_SLIP_DEEPEN = true

-- D estimator (peak-decel window). D = peakDecel / g.
-- (D-estimator state: bundled to stay under LuaJIT 60-upvalue limit)
local dest = {
  EST_MIN          = 0.10,
  EST_MAX          = 1.5,
  consensusD       = 1.0,
  SMOOTHING        = 0.80,
  UPDATE_MIN_DECEL = 0.5,
  baseTarget       = 0.14,
  retroResets      = 0,
  WINDOW_SIZE      = 40,    -- 0.2s at 200Hz
  window           = {},
  windowIdx        = 0,
  stableD          = 1.0,
  stableTicks      = 0,
  SETTLE_TICKS     = 30,       -- 0.15s before surface-change detection arms
  CHANGE_THRESHOLD = 0.40,    -- >40% D jump = surface change, blow away the window
  -- snap+undo: upward snap is provisional, reverted if raw decel returns to pre-snap band
  SNAP_UNDO_ENABLE  = true,
  SNAP_UNDO_TICKS   = 20,     -- 100ms: wheel-hop transients are 30-100ms on road cars
  SNAP_UNDO_BAND    = 0.30,   -- +/-30% of the pre-snap level counts as "returned"
  SNAP_UNDO_CONFIRM = 3,      -- consecutive in-band ticks before reverting
  shadowWindow      = {},
  shadowIdx         = 0,
  shadowD           = 1.0,
  shadowStableD     = 1.0,
  shadowStableTicks = 0,
  shadowLevel       = 0,      -- pre-snap decel level (m/s^2)
  undoTicks         = 0,      -- >0 while a snap is provisional
  undoInBand        = 0,
  snapUndos         = 0,
  snapTotal         = 0,      -- snaps since vehicle reset (retroResets zeroes per ABS event)
  
  absEventActive   = false,
  absEventSum      = 0,
  absEventTicks    = 0,
  absEventTotalTicks = 0,
}

-- D-estimator window aggregation: how the sliding decel window collapses to one number.
-- Flip D_AGG_MODE to compare estimators on the same drive (peak is the historical behavior).
--   "peak" : max sample in the window  (over-reads on ice: latches transients)
--   "mean" : average of the window     (steadiest, but slow to catch a real grip peak)
--   "topn" : average of the D_AGG_TOPN largest samples (compromise; peak-ish but denoised)
-- Switchable live via M.setDAggMode(mode[, topn]) -- no reload needed.
local D_AGG_MODE = "peak"
local D_AGG_TOPN = 10

-- Collapse dest.window to a single decel value per D_AGG_MODE. peak/mean allocate nothing;
-- topn builds+sorts a small array (only that mode), fine for a diagnostic at 200Hz.
local function aggregateDecelWindow()
  local w = dest.window
  if D_AGG_MODE == "mean" then
    local sum, n = 0, 0
    for j = 1, dest.WINDOW_SIZE do local v = w[j]; if v then sum = sum + v; n = n + 1 end end
    return n > 0 and (sum / n) or 0
  elseif D_AGG_MODE == "topn" then
    local vals = {}
    for j = 1, dest.WINDOW_SIZE do local v = w[j]; if v then vals[#vals + 1] = v end end
    local n = #vals
    if n == 0 then return 0 end
    table.sort(vals)                              -- ascending
    local k = math.min(D_AGG_TOPN, n)
    local s = 0
    for i = n, n - k + 1, -1 do s = s + vals[i] end
    return s / k
  else                                            -- "peak" (default / historical)
    local peak = 0
    for j = 1, dest.WINDOW_SIZE do local v = w[j]; if v and v > peak then peak = v end end
    return peak
  end
end

-- Hybrid Peak-Hunter & Brake Simulator State
-- ENABLE=false: the front/rear trim extremum-seeker is OFF (fixed D-derived slip targets only).
local ph = {
  ENABLE = false,
  ENABLE_THROTTLE_LOCKOUT = true,
  seekerSuppressed = false,
  
  turn = 1,       -- 1 = Front, 2 = Rear
  phase = 0,      -- 0=STEP, 1=SETTLE, 2=MEASURE, 3=EVALUATE
  
  frontOffset = 0,
  rearOffset = 0,
  frontDirection = 1,
  rearDirection = 1,
  
  lastFrontEfficiency = 0,
  lastRearEfficiency = 0,
  
  settleTicks = 0,
  measureTicks = 0,
  startSpeed = 0,
}

-- PID tunables
local KP = 6.0
local KI = 0.8
local KD = 0.08
local INTEGRAL_MIN = -1.0
local INTEGRAL_MAX = 1.0
local MIN_SPEED = 5.0
local MIN_ADAPT_SPEED = 2.0

-- Misc
local NON_BRAKING_DECEL_FILTER = -5.0
local STANDSTILL_WS_THRESHOLD = 0.3
local STANDSTILL_FUSED_THRESHOLD = 2.0
local STANDSTILL_ACCEL_MAX = 0.5   -- m/s^2: only count as stopped when NOT decelerating. Locked wheels
                                   -- read ~0 while the car still moves under braking; the accelerometer
                                   -- (gravity-cancelled) does not, so it vetoes a premature standstill-zero.
local STANDSTILL_PHYS_TICKS = 100
local standstillCounter = 0

local uiAccum = 0
local function getCondition(slip)
  if slip <= 0.06 then return "Ice"
  elseif slip <= 0.10 then return "Snow"
  elseif slip <= 0.13 then return "Wet"
  else return "Dry Asphalt"
  end
end

-- Persist brake-event history to disk so it survives vehicle switches / config changes.
-- File path is relative to BeamNG userdata root (.../current/).
local function saveBrakeEvents()
  pcall(jsonWriteFile, BRAKE_EVENT_FILE, brakeEvents, false)
end

local function loadBrakeEvents()
  local ok, data = pcall(jsonReadFile, BRAKE_EVENT_FILE)
  if ok and type(data) == "table" then
    brakeEvents = {}
    for i, e in ipairs(data) do
      if i > BRAKE_EVENT_MAX then break end
      table.insert(brakeEvents, e)
    end
  end
end


local function initDecelWindow()
  dest.window = {}
  dest.windowIdx = 0
  dest.stableD = 1.0
  dest.stableTicks = 0
  dest.consensusD = 1.0
  dest.undoTicks = 0
  dest.undoInBand = 0
  dest.snapUndos = 0
  dest.snapTotal = 0
end


-- (2) buildWheelMaps: 1:1 copy of the stock BeamNG method from
-- drivingDynamics/sensors/vehicleData.lua â†’ initSecondStage().
-- Step 1: Filter to known corner wheel names {"FR","FL","RR","RL"}.
-- Step 2: Compute average wheel position from v.data.nodes[wheel.node1].pos.
-- Step 3: Build a local coordinate frame from the vehicle's reference nodes
--         (ref, back, up) via forward/up/right vectors.
-- Step 4: Classify each corner wheel as front/rear + left/right using
--         dot products against the forward and right vectors.
-- Maps logical index (1=RR, 2=RL, 3=FR, 4=FL) â†’ wheelRotator slot (1-based).
-- Falls back to identity map {1,2,3,4} if the stock method fails.
local function buildWheelMaps()
  wheelToBrakeMap = {}

  local ok, err = pcall(function()
    -- Stock: jbeamData.cornerWheels or {"FR", "FL", "RR", "RL"}
    local cornerWheelData = {"FR", "FL", "RR", "RL"}
    local cornerWheels = {}
    for _, wheelName in pairs(cornerWheelData) do
      cornerWheels[wheelName] = true
    end

    -- Stock: calculate average wheel position for later being able to determine where a given wheel is
    local avgWheelPos = vec3(0, 0, 0)
    for _, wheel in pairs(wheels.wheels) do
      if cornerWheels[wheel.name] then
        local wheelNodePos = v.data.nodes[wheel.node1].pos
        avgWheelPos = avgWheelPos + wheelNodePos
      end
    end
    avgWheelPos = avgWheelPos / #wheels.wheels  -- stock divides by total wheel count

    -- Stock: build reference frame from the vehicle's ref nodes
    local refNodes = v.data.refNodes[0]
    local vectorForward = vec3(v.data.nodes[refNodes.ref].pos) - vec3(v.data.nodes[refNodes.back].pos)
    local vectorUp      = vec3(v.data.nodes[refNodes.up].pos)  - vec3(v.data.nodes[refNodes.ref].pos)
    local vectorRight   = vectorForward:cross(vectorUp)

    local foundWheelsCount = 0

    -- Stock: classify each corner wheel using dot products
    for _, wheel in pairs(wheels.wheels) do
      if cornerWheels[wheel.name] then
        local wheelNodePos = vec3(v.data.nodes[wheel.node1].pos)
        local wheelVector  = wheelNodePos - avgWheelPos
        local dotForward   = vectorForward:dot(wheelVector)
        local dotRight     = vectorRight:dot(wheelVector)   -- stock calls this "dotLeft" but tests >= 0 for right

        -- Map wheel name â†’ wheelRotator index â†’ 1-based slot for our tables
        local rotIdx = wheels.wheelRotatorIDs[wheel.name]
        if rotIdx == nil then error("wheelRotatorIDs missing for '" .. wheel.name .. "'") end
        local slot = rotIdx + 1  -- wheelRotatorIDs is 0-based; our tables are 1-based

        -- Stock convention: dotRight >= 0 is right side, dotForward >= 0 is front
        if dotRight >= 0 then
          if dotForward >= 0 then
            wheelToBrakeMap[3] = slot   -- FR = logical 3
          else
            wheelToBrakeMap[1] = slot   -- RR = logical 1
          end
        else
          if dotForward >= 0 then
            wheelToBrakeMap[4] = slot   -- FL = logical 4
          else
            wheelToBrakeMap[2] = slot   -- RL = logical 2
          end
        end
        foundWheelsCount = foundWheelsCount + 1
      end
    end

    if foundWheelsCount ~= 4 or not (wheelToBrakeMap[1] and wheelToBrakeMap[2] and wheelToBrakeMap[3] and wheelToBrakeMap[4]) then
      error("Could not classify all 4 corner wheels (found " .. foundWheelsCount .. ")")
    end

    print("[ABS-1FEX] (2) wheelToBrakeMap built (stock geometry method): RR="
      .. tostring(wheelToBrakeMap[1]) .. " RL=" .. tostring(wheelToBrakeMap[2])
      .. " FR=" .. tostring(wheelToBrakeMap[3]) .. " FL=" .. tostring(wheelToBrakeMap[4]))
  end)

  if not ok then
    -- Fallback: identity map
    for i = 1, N_WHEELS do wheelToBrakeMap[i] = i end
    print("[ABS-1FEX] (2) WARNING: Stock geometry method failed (" .. tostring(err) .. "). Using fallback identity map.")
  end

  rearLogicalIndices  = {1, 2}
  frontLogicalIndices = {3, 4}
end


-- (2) buildGeometry: computes WHEELBASE and FRONT_FRAC from wheel axle
-- node positions. Uses wheels.wheelRotators[i].node1/.node2 (confirmed in official
-- BeamNG dev code) and obj:getNodePositionRelative() for body-frame positions.
-- All values are computed ONCE at init â€” zero runtime overhead.
-- jbeamData is checked first for an explicit cgHeight override; falls back to 0.55.
local function buildGeometry(jbeamData)
  -- CG height: try jbeamData first (our own ABS jbeam may define it), then universal fallback.
  local cgH = 0.55
  if jbeamData then
    pcall(function()
      if jbeamData.cgHeight and type(jbeamData.cgHeight) == "number" then
        cgH = jbeamData.cgHeight
      end
    end)
  end
  grip.H_CG = cgH

  -- Attempt to read axle node positions for each named wheel.
  -- node1 and node2 are the two axle nodes; their midpoint is the wheel center in body frame.
  local wheelNames = { "RR", "RL", "FR", "FL" }
  local logicalMap = { RR = 1, RL = 2, FR = 3, FL = 4 }
  local positions = {}   -- [logicalIdx] = {x, y, z} body-frame midpoint

  for _, name in ipairs(wheelNames) do
    local logical = logicalMap[name]
    if wheels.wheelRotatorIDs and wheels.wheelRotatorIDs[name] ~= nil then
      local physIdx = wheels.wheelRotatorIDs[name]
      local wr = wheels.wheelRotators[physIdx]
      if wr and wr.node1 ~= nil and wr.node2 ~= nil then
        local pos = nil
        pcall(function()
          local p1 = obj:getNodePositionRelative(wr.node1)
          local p2 = obj:getNodePositionRelative(wr.node2)
          -- Average of the two axle nodes = wheel center in body frame
          pos = {
            x = (p1.x + p2.x) * 0.5,
            y = (p1.y + p2.y) * 0.5,
            z = (p1.z + p2.z) * 0.5,
          }
        end)
        if pos then positions[logical] = pos end
      end
    end
  end

  -- Wheelbase: distance along Y between front axle avg and rear axle avg.
  -- BeamNG body frame: Y positive = rear, Y negative = front (matches suspension JBeam nodes).
  local frontY, rearY = nil, nil
  for _, li in ipairs(frontLogicalIndices) do
    if positions[li] then
      frontY = (frontY or 0) + positions[li].y
    end
  end
  for _, li in ipairs(rearLogicalIndices) do
    if positions[li] then
      rearY = (rearY or 0) + positions[li].y
    end
  end
  if frontY and rearY then
    frontY = frontY / #frontLogicalIndices
    rearY  = rearY  / #rearLogicalIndices
    local wb = math.abs(rearY - frontY)
    if wb > 0.5 then   -- sanity: anything under 0.5m is probably a bad read
      grip.WHEELBASE = wb
    end
  end

  -- FRONT_FRAC: fraction of static weight on front axle.
  -- Derived from front/rear axle Y positions relative to vehicle origin (body CG â‰ˆ origin).
  -- front_frac = distance_from_CG_to_rear / wheelbase  (static weight fraction at front).
  if frontY and rearY and grip.WHEELBASE > 0.5 then
    -- CG is at Y=0 in body frame (origin); rear is positive Y, front is negative Y.
    -- For front_frac: fraction of weight at front = rear_moment / wheelbase.
    -- rear_moment = distance from CG (0) to rear axle = rearY (positive).
    -- front_frac = rearY / wheelbase.
    local frac = rearY / grip.WHEELBASE
    if frac > 0.2 and frac < 0.8 then   -- sanity bounds
      grip.FRONT_FRAC = frac
    end
  end

  print(string.format("[ABS-1FEX] (2) Geometry: WB=%.2fm FRONT_FRAC=%.2f H_CG=%.2fm",
    grip.WHEELBASE, grip.FRONT_FRAC, grip.H_CG))
end


local function init(jbeamData)
  print("[ABS-1FEX] (2) canonical build loaded")

  -- Read wheel count first; everything else is sized to this.
  N_WHEELS = wheels.wheelRotatorCount or 4

  -- Initialize all N_WHEELS-sized tables
  origBrakeTorque       = {}
  slipIntegral          = {}
  lastSlipError         = {}
  prevTickWheelSpeed    = {}
  latestWheelSpeed      = {}
  fusedPrevWs           = {}
  slipTargets           = {}
  ph.simulatedTorque    = {}
  ph.brakeInRate        = {}
  ph.brakeOutRate       = {}
  safety.slipRatios     = {}
  safety.lastAbsCoefs   = {}
  grip.Fz0              = {}
  grip.gEMA             = {}
  grip.Dwheel           = {}
  grip.confident        = {}
  wa.prevWs             = {}
  wa.locked             = {}
  wa.decelBuf           = {}
  wa.decelIdx           = {}

  for i = 1, N_WHEELS do
    slipIntegral[i]       = 0
    lastSlipError[i]      = 0
    lastSlipErrors[i]     = 0
    lastSlipDerivatives[i]= 0
    lastEffectiveTargets[i]= 0.14
    prevTickWheelSpeed[i] = 0
    latestWheelSpeed[i]   = 0
    fusedPrevWs[i]        = 0
    slipTargets[i]        = 0.14
    ph.simulatedTorque[i] = 0
    ph.brakeInRate[i]     = 0
    ph.brakeOutRate[i]    = 0
    safety.slipRatios[i]  = 0
    safety.lastAbsCoefs[i]= 1
    grip.Fz0[i]           = 0
    grip.gEMA[i]          = 1
    grip.Dwheel[i]        = 1
    grip.confident[i]     = false
    wa.prevWs[i]          = 0
    wa.locked[i]          = false
    wa.decelBuf[i]        = {}
    wa.decelIdx[i]        = 0
    wheelToBrakeMap[i]    = i   -- safe default until buildWheelMaps overrides
  end

  wasBraking = false
  wasBrakingMu = false
  timeAccum = 0
  uiAccum = 0
  standstillCounter = 0
  safety.snapUp = 0
  safety.snapUpRej = 0
  safety.imuClamps = 0
  slipStepdownTimer = 0
  slipStepdownCooldown = 0
  slipStepdownCount = 0
  -- Reset mid-event state; load persisted history from disk (survives vehicle switches).
  currentBrakeEvent = nil
  brakeSimTime = 0
  brakeUiAccum = 0
  loadBrakeEvents()

  initDecelWindow()
  dest.retroResets = 0
  dest.baseTarget = 0.14
  dest.consensusD = 1.0
  dest.window = {}
  dest.windowIdx = 0
  dest.stableD = 1.0
  dest.stableTicks = 0
  dest.absEventActive = false
  dest.absEventSum = 0
  dest.absEventTicks = 0
  dest.absEventTotalTicks = 0

  ph.frontOffset = 0
  ph.rearOffset = 0
  ph.frontDirection = 1
  ph.rearDirection = 1
  ph.phase = 0
  ph.turn = 1

  ph.seekerSuppressed = false

  fusedSpeed = 0
  imuSpeed = 0
  fwdVel = 0
  vLat = 0
  vVert = 0
  lastPitch = 0
  lastRoll = 0
  pitchRateLog = 0
  rollRateLog = 0
  pitchLog = 0
  rollLog = 0
  
  -- Logging state
  isLogging = false
  logData = {}
  logTimer = 0
  fusedInitialized = false
  latestSensorY = 0

  -- Read static brake torques and hydraulic delays.
  -- IMPORTANT: index by LOGICAL order (1=RR,2=RL,3=FR,4=FL) via wheelToBrakeMap,
  -- NOT by raw wheelRotator iteration order. This ensures origBrakeTorque[i]
  -- matches the same wheel that latestWheelSpeed[i] and the PID loop use.
  -- First pass: store by raw wheelRotator order (temporary)
  local rawBrakeTorque = {}
  local rawInDelay = {}
  local rawOutDelay = {}
  for i = 0, N_WHEELS - 1 do
    rawBrakeTorque[i + 1] = wheels.wheelRotators[i].brakeTorque or 0
    rawInDelay[i + 1] = wheels.wheelRotators[i].brakePressureInDelay or 0.04
    rawOutDelay[i + 1] = wheels.wheelRotators[i].brakePressureOutDelay or 0.04
  end

  -- (2) Build wheel name -> index map (replaces hardcoded wheelToBrakeMap = {3,4,1,2})
  buildWheelMaps()

  -- Second pass: remap brake torques from raw wheelRotator order to logical order
  for i = 1, N_WHEELS do
    local slot = wheelToBrakeMap[i]  -- physical wheelRotator slot (1-based)
    local maxT = slot and rawBrakeTorque[slot] or rawBrakeTorque[i]
    origBrakeTorque[i] = maxT
    local wr = wheels.wheelRotators[(slot or i) - 1]
    fg.k[i] = fg.TIRE_CORR * ((wr and wr.radius) or 0.33) / math.max((wr and wr.inertia) or 1.0, 0.05)
    local inD = slot and rawInDelay[slot] or rawInDelay[i]
    local outD = slot and rawOutDelay[slot] or rawOutDelay[i]
    ph.brakeInRate[i] = maxT / (inD + 1e-30)
    ph.brakeOutRate[i] = maxT / (outD + 1e-30)
  end

  -- (2) Vehicle mass â€” static property, read once at init. obj:getMass() is confirmed API.
  pcall(function() grip.mass = obj:getMass() or grip.mass end)

  -- Disable per-wheel D on vehicles with fewer than 4 wheels
  grip.ENABLE_PERWHEEL_D = (N_WHEELS >= 4) and grip.ENABLE_PERWHEEL_D or false

  -- (2) Compute geometry from wheel axle node positions (wheelbase, FRONT_FRAC)
  -- and read H_CG from jbeamData if provided. All one-time static reads, no runtime cheating.
  buildGeometry(jbeamData)

  -- Static corner loads from mass + computed front/rear fraction
  do
    local fAxle = grip.mass * grip.GRAV * grip.FRONT_FRAC / 2          -- per front wheel
    local rAxle = grip.mass * grip.GRAV * (1 - grip.FRONT_FRAC) / 2    -- per rear wheel
    for _, li in ipairs(rearLogicalIndices)  do grip.Fz0[li] = rAxle end
    for _, li in ipairs(frontLogicalIndices) do grip.Fz0[li] = fAxle end
    -- Any logical index not in either list (non-standard layout) gets average
    local avgFz = grip.mass * grip.GRAV / N_WHEELS
    for i = 1, N_WHEELS do
      if grip.Fz0[i] == 0 then grip.Fz0[i] = avgFz end
    end
  end

  -- D-estimator v2 per-wheel constants from the car itself
  do
    local d2 = safety.d2
    for i = 1, N_WHEELS do
      local slot = wheelToBrakeMap[i] or i
      local wr = wheels.wheelRotators[slot - 1]
      local rr = (wr and wr.radius) or 0.33
      local inertia = math.max((wr and wr.inertia) or 1.0, 0.05)
      local corr = (wr and wr.isPropulsed) and d2.TIRE_CORR_DRIVEN or d2.TIRE_CORR
      d2.r[i] = rr
      d2.Ieff[i] = inertia / corr
      d2.front[i] = false
      for _, li in ipairs(frontLogicalIndices) do if li == i then d2.front[i] = true end end
      d2.sHist[i] = {}; d2.bufS[i] = {}; d2.bufM[i] = {}; d2.bufBad[i] = {}
    end
  end

  extensions.load('abstelemv2')
end


-- detectMu(dtPhys) â€” 2kHz sensor fusion only
safety.air = air   -- reachable from runTick/detectMu without a new upvalue
safety.fg = fg

-- D-estimator v2 (notes/D_ESTIMATOR_V2_DESIGN.md). L1: per-wheel utilized grip from wheel torque
-- balance, level anchored to the accelerometer. L2: local quadratic fit of mu vs slip -> per-wheel
-- state 0 unknown / 1 below peak / 2 near peak / 3 past peak. L3: per-axle slip-target offset stepped
-- by state, kept only if the cycle-mean grip rose. Grip LEVEL never sets a target.
local d2 = {
  ENABLE   = true,
  CONTROL  = true,
  TIRE_CORR = 0.55, TIRE_CORR_DRIVEN = 0.40,   -- measured free-decel / (r/I) on etk800 (ledge session)
  SIGMA    = 0.3,       -- m: tire relaxation length, lags slip to line up with force
  WIN      = 30,        -- samples @200 Hz = 150 ms fit window
  STEP     = 10,        -- fit every 50 ms
  EXC_MIN  = 0.03,      -- slip excursion needed inside a window
  Z_SIG    = 2.0,
  BAND_LO  = 6.8, BAND_HI = 12.8,
  VMIN     = 5.0,
  STEP_SIZE = 0.0075, HOLD_S = 0.20, OFF_MAX = 0.08, REVERT_DROP = 0.015, PROBE_S = 0.40,
  GAIN_MIN = 0.005, MUTE_S = 1.0,
  GUARD_DROP = 0.40,    -- mu falling this fraction in 100 ms = surface change -> offsets reset
  LOW_MU   = 0.5,       -- below this cycle-mean grip: scaled steps, no probing
  r = {0.33, 0.33, 0.33, 0.33}, Ieff = {1.8, 1.8, 1.8, 1.8}, front = {false, false, true, true},
  aw = {0, 0, 0, 0}, fx = {0, 0, 0, 0}, fz = {0, 0, 0, 0}, mu = {0, 0, 0, 0}, s = {0, 0, 0, 0},
  sHist = {{}, {}, {}, {}}, histI = 0,
  bufS = {{}, {}, {}, {}}, bufM = {{}, {}, {}, {}}, bufBad = {{}, {}, {}, {}}, bufI = 0, bufN = 0, sinceFit = 0,
  raw = {0, 0, 0, 0}, prevRaw = {0, 0, 0, 0}, state = {0, 0, 0, 0}, stateAge = {0, 0, 0, 0},
  winMean = {0, 0, 0, 0}, prevWinMean = {0, 0, 0, 0}, b = {0, 0, 0, 0}, q = {0, 0, 0, 0},
  muPeak = {0, 0, 0, 0}, muLB = {0, 0, 0, 0}, sStar = {0, 0, 0, 0}, muSlow = {0, 0, 0, 0},
  c = 1.0,
  axleState = {0, 0}, off = {0, 0}, holdT = {0, 0}, holdSum = {0, 0}, holdN = {0, 0},
  lastMean = {0, 0}, lastDir = {0, 0}, unknownT = {0, 0},
  steps = 0, reverts = 0, guards = 0, fits = 0,
}
safety.d2 = d2

-- Loose-surface deep-slip regime (design doc section 13): probe a deep target when the grip level
-- sits in the loose band, keep it only while it keeps beating the normal target
local deep = {
  ENABLE_DEEP = true,
  TARGET = 1.00, LO = 0.45, HI = 0.85, HI_MARGIN = 0.05, DROP = 0.40,
  SETTLE_S = 0.30, SKIP_S = 0.20, PROBE_S = 0.45, GAIN = 0.03, RECHECK_S = 1.5, TREND_MAX = 0.03, lowT = 0,
  FREEZE_TEST = false, LEVEL_GATE = true, RECOVER_MIN_S = 0.4, RECOVER_MAX_S = 1.5, recover = 0, recoverT = 0, regulated = false,
  FAIL_COOL = 1.5, EXIT_COOL = 1.0, ABORT_COOL = 0.3, STEER_COOL = 0.5,
  VMIN = 8.0, STEER_MAX = 0.05, YAW_MAX = 0.25, HEADING_MAX = 0.09, heading = 0,
  mode = 0, active = false, lvl = 0, cool = 0, regT = 0, stopT = 0, t = 0, sum = 0, n = 0,
  base = 0, mean = 0, deepT = 0, hiT = 0, refT = 0, refSum = 0, refN = 0, ref1 = 0, ref2 = 0,
  probes = 0, keeps = 0, exits = 0, time = 0, fails = 0, aborts = 0, lastBase = 0, lastMeas = 0, lastN = 0, lastExit = 0,
}
safety.deep = deep

local function detectMu(dtPhys)
  local rawSY = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorY) or 0
  -- BeamNG's ffiSensors.sensorY is already body-frame, gravity-cancelled.
  latestSensorY = rawSY

  local signedSum, signedCount = 0, 0
  -- Read wheel speeds indexed by LOGICAL order (1=RR,2=RL,3=FR,4=FL).
  -- wheelToBrakeMap[i] gives the physical wheelRotator slot (1-based) for logical wheel i.
  -- This ensures latestWheelSpeed[1] is always RR, [2] is always RL, etc.
  for i = 1, N_WHEELS do
    local slot = wheelToBrakeMap[i]
    local signedWs = wheels.wheelRotators[slot - 1].wheelSpeed or 0  -- wheelRotators is 0-based
    latestWheelSpeed[i] = math.abs(signedWs)
    if math.abs(signedWs) > REVERSE_DETECT_THRESHOLD then
      signedSum = signedSum + signedWs
      signedCount = signedCount + 1
    end
  end

  -- Motion-direction detection with hysteresis: flip only when signed avg clearly crosses threshold.
  if signedCount > 0 then
    local signedAvg = signedSum / signedCount
    if signedAvg > REVERSE_LOCKIN_MPS then motionDirection = 1
    elseif signedAvg < -REVERSE_LOCKIN_MPS then motionDirection = -1 end
  end

  if not fusedInitialized then
    local maxInit = 0
    for i = 1, N_WHEELS do
      if latestWheelSpeed[i] > maxInit then maxInit = latestWheelSpeed[i] end
    end
    fusedSpeed = maxInit
    imuSpeed = maxInit
    fwdVel = maxInit
    for i = 1, N_WHEELS do fusedPrevWs[i] = latestWheelSpeed[i] end
    fusedInitialized = true
  end

  local brakeInput = input.brake or 0
  local isBraking = brakeInput > 0

  -- Integrate raw decel, floor at 0.
  fusedSpeed = math.max(0, fusedSpeed - latestSensorY * dtPhys)

  local maxWs, minWs, maxIdx = 0, math.huge, 1
  local wsSum = 0
  for i = 1, N_WHEELS do
    local ws = latestWheelSpeed[i]
    wsSum = wsSum + ws
    if ws > maxWs then maxWs = ws; maxIdx = i end
    if ws < minWs then minWs = ws end
  end
  local avgWs = wsSum / N_WHEELS

  if isBraking then
    -- Snap-up gap gate (TEST): only re-anchor fused up to the fastest wheel if the one-tick
    -- gap is physically plausible (<= SNAP_GAP_MAX). A larger gap implies an impossible
    -- speed-up for a braking car => spiking/corrupt wheel reading => reject it.
    -- Low-speed cutout: suppress entirely when both fused and wheel-avg are essentially stopped.
    local bothStopped = fusedSpeed < SNAP_MIN_SPEED and avgWs < SNAP_MIN_SPEED
    if maxWs > fusedSpeed and not bothStopped then
      if (maxWs - fusedSpeed) <= SNAP_GAP_MAX then
        fusedSpeed = maxWs
        safety.snapUp = safety.snapUp + 1
      else
        safety.snapUpRej = safety.snapUpRej + 1
      end
    end
  else
    local goodSum, goodCount = 0, 0
    for i = 1, N_WHEELS do
      local d = (latestWheelSpeed[i] - fusedPrevWs[i]) / dtPhys
      if d > NON_BRAKING_DECEL_FILTER then
        goodSum = goodSum + latestWheelSpeed[i]
        goodCount = goodCount + 1
      end
    end
    if goodCount > 0 then fusedSpeed = goodSum / goodCount end
  end

  -- Speed integral.
  -- 1F (toggle OFF): single longitudinal integral, floored at 0 (bit-identical to plain 1F).
  -- 2D (toggle ON): integrate forward (fwdVel) AND lateral (vLat) velocity with the yaw-Coriolis
  -- coupling, then report GROUND SPEED = sqrt(fwd^2 + lat^2). Pure yaw rotates speed between the
  -- two axes and conserves the magnitude, so a 90-deg slide keeps the estimate at TRUE speed
  -- instead of collapsing the forward component (and fused) to 0.
  if ENABLE_2D_SPEED then
    local yr = 0
    pcall(function() yr = obj:getYawAngularVelocity() or 0 end)
    local roll, pitch = 0, 0
    pcall(function() roll, pitch, _ = obj:getRollPitchYaw() end)

    local pitchRate, rollRate = 0, 0
    if dtPhys > 0 then
      local dPitch = (pitch - (lastPitch or 0)) % (2 * math.pi)
      if dPitch > math.pi then dPitch = dPitch - 2 * math.pi end
      pitchRate = dPitch / dtPhys

      local dRoll = (roll - (lastRoll or 0)) % (2 * math.pi)
      if dRoll > math.pi then dRoll = dRoll - 2 * math.pi end
      rollRate = dRoll / dtPhys
    end
    lastPitch = pitch
    lastRoll = roll
    pitchLog = pitch
    rollLog = roll
    pitchRateLog = pitchRate
    rollRateLog = rollRate

    local ax = -latestSensorY                                                       -- forward accel
    local ay = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorX) or 0 -- lateral accel
    local az = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorZ) or 0 -- vertical accel

    -- (change #2) Filter the noisy signals before integration. dt-scaled EMAs => rate-independent.
    -- pitch/roll rates: finite-differenced => noise-amplified, and multiplied by fwdVel below, so
    -- they dominate vVert corruption. az: enters imuSpeed squared => noise biases the ceiling up.
    -- ax deliberately untouched (brake-onset fidelity); ay gets deadband only (slide-onset fidelity).
    if ENABLE_IMU_FILTERS then
      local aRot = dtPhys / (TAU_ROT + dtPhys)
      filtPitchRate = filtPitchRate + (pitchRate - filtPitchRate) * aRot
      filtRollRate  = filtRollRate  + (rollRate  - filtRollRate)  * aRot
      pitchRate = filtPitchRate
      rollRate  = filtRollRate

      local aAz = dtPhys / (TAU_AZ + dtPhys)
      filtAz = filtAz + (az - filtAz) * aAz
      az = filtAz
      if math.abs(az) < ACCEL_DEADBAND then az = 0 end
      if math.abs(ay) < ACCEL_DEADBAND then ay = 0 end
    end

    -- Full 3D strapdown integration (Coriolis/Centripetal cross-coupling)
    local dotFwd  = ax + vLat * yr + (vVert or 0) * pitchRate
    local dotLat  = -ay - fwdVel * yr + (vVert or 0) * rollRate
    local dotVert = -az - fwdVel * pitchRate - vLat * rollRate

    local newFwd = fwdVel + dotFwd * dtPhys
    vLat         = vLat   + dotLat * dtPhys
    vVert        = (vVert or 0) + dotVert * dtPhys
    fwdVel = newFwd                                          -- forward comp may pass through/below 0

    -- Kinematic Lateral Anchor: Tie vLat decay to steering angle and yaw rate
    local steering = math.abs(electrics.values.steering or 0)
    local absYaw = math.abs(yr)
    if absYaw < 0.05 and steering < 0.05 then
      -- Driving perfectly straight: scrub phantom lateral noise aggressively
      vLat = vLat * 0.95
    else
      -- Steering or drifting: use a dynamic decay that scales with yaw/steering severity.
      -- This preserves the drift vector while slowing bleeding extreme sensor noise.
      local adaptiveDecay = math.max(0.99, 1.0 - (0.01 / (1.0 + absYaw * 10.0 + steering * 5.0)))
      vLat = vLat * adaptiveDecay
    end
    
    vVert = vVert * 0.98                                     -- always gently bleed vertical velocity to prevent drift
    imuSpeed = math.sqrt(fwdVel * fwdVel + vLat * vLat + vVert * vVert)      -- true 3D ground speed (magnitude)
  else
    vLat = 0
    vVert = 0
    fwdVel = math.max(0, fwdVel - latestSensorY * dtPhys)
    imuSpeed = fwdVel
  end

  -- IMU speed ceiling: cap fused at the accelerometer-derived ground speed. Resync the estimate to
  -- the wheels ONLY while NOT braking and the wheels are trustworthy (agree AND not running away).
  -- During braking the wheels under-read, so we never resync then â€” the integral is the truth.
  if (not isBraking) and (maxWs - minWs) < 1.0 and maxWs <= imuSpeed + IMU_TRUST_WINDOW then
    fwdVel = maxWs
    vLat = 0                              -- wheels agree => assume no sideslip, reset lateral est.
    vVert = 0                             -- reset vertical est.
    imuSpeed = maxWs
  end

  -- IMU ceiling constraint:
  -- When NOT braking, use an EXACT hard cap so continuous wheelspin can't fight a slew rate.
  -- When braking STARTS, do one final hard snap to guarantee a clean starting speed.
  -- During the rest of the brake event, allow the integral to track freely.
  local brakingJustStartedMu = isBraking and not wasBrakingMu
  if not isBraking then
    if fusedSpeed > imuSpeed then
      fusedSpeed = imuSpeed
      safety.imuClamps = safety.imuClamps + 1
    end
  elseif brakingJustStartedMu then
    if fusedSpeed > imuSpeed then
      fusedSpeed = imuSpeed
      safety.imuClamps = safety.imuClamps + 1
    end
    if ra.ENABLE and ra.ONSET_ENABLE and maxWs > ra.VMIN then
      local onsetTarget = maxWs / (1 - ra.ONSET_SLIP)
      if fusedSpeed > onsetTarget then
        fusedSpeed = onsetTarget; fwdVel = onsetTarget; vLat = 0; vVert = 0; imuSpeed = onsetTarget
        ra.onsetFires = ra.onsetFires + 1
      end
    end
  end
  wasBrakingMu = isBraking

  -- Slip-anchored fused stepdown (tunables at top). wheelMax lags true speed by ~the commanded
  -- slip; if fused sits > RATIO * that expected gap above wheelMax for SUSTAIN seconds, snap fused
  -- and the IMU strapdown down toward the wheel-implied true speed (SNAPFRAC into the [wheel,
  -- slip-speed] window; 0.5 = middle). wheelMax is a hard floor (no wheel
  -- can exceed true speed while braking), so this can only ever pull fused DOWN. s_t is the base
  -- commanded slip target (slipTargets). With ENABLE_LOWSPEED_SLIP_DEEPEN OFF, effectiveTarget ==
  -- slipTargets, so this IS the controller's actual commanded slip AND it's fused-independent
  -- (slipTargets is D/accelerometer-derived) -- no circularity, 1:1 with the PID. If you re-enable
  -- deepening, real slip grows at low speed and this under-predicts it (low-speed false fires);
  -- switch s_t back to lastEffectiveTargets then.
  if slipStepdownCooldown > 0 then slipStepdownCooldown = slipStepdownCooldown - dtPhys end
  local deepHold = safety.deep and (safety.deep.active or safety.deep.recover > 0 or safety.deep.FREEZE_TEST)   -- wheels far below truth on purpose
  if ENABLE_SLIP_STEPDOWN and isBraking and fusedSpeed > SLIP_STEPDOWN_VMIN and not deepHold then
    local st = math.min(math.max(lastEffectiveTargets[maxIdx] or 0.14, SLIP_STEPDOWN_STFLOOR), 0.5)
    local estimatedSlipSpeed = maxWs / (1 - st)     -- where true speed should be
    local expectedGap = estimatedSlipSpeed - maxWs  -- = maxWs * st / (1 - st)
    local measuredGap = fusedSpeed - maxWs
    if measuredGap > SLIP_STEPDOWN_GAPFLOOR and measuredGap > SLIP_STEPDOWN_RATIO * expectedGap then
      slipStepdownTimer = slipStepdownTimer + dtPhys
      if slipStepdownTimer >= SLIP_STEPDOWN_SUSTAIN and slipStepdownCooldown <= 0 then
        -- Land fused SNAPFRAC of the way from the fastest wheel up to the slip-implied speed
        -- (0.5 = middle). wheelMax stays a hard floor since SNAPFRAC >= 0.
        local snapTarget = maxWs + SLIP_STEPDOWN_SNAPFRAC * expectedGap
        fusedSpeed = snapTarget
        fwdVel     = snapTarget
        vLat       = 0
        vVert      = 0
        imuSpeed   = snapTarget
        slipStepdownTimer = 0
        slipStepdownCooldown = SLIP_STEPDOWN_COOLDOWN
        -- Count only real moving-car fires (airspeed > 1 mph); ignore stepdowns while the
        -- brakes are just holding the car steady near standstill.
        if (electrics.values.airspeed or 0) > 0.44704 then
          slipStepdownCount = slipStepdownCount + 1
        end
      end
    else
      slipStepdownTimer = 0
    end
  else
    slipStepdownTimer = 0
  end

  for i = 1, N_WHEELS do fusedPrevWs[i] = latestWheelSpeed[i] end

  -- Flight-gate fused dump (B), requested from runTick
  if safety.fg and safety.fg.dumpTo then
    local v = safety.fg.dumpTo; safety.fg.dumpTo = nil
    if v > 1.0 and not deepHold then fusedSpeed = v; fwdVel = v; vLat = 0; vVert = 0; imuSpeed = v end
  end

  -- Gradient re-anchor (tunables in ra)
  if ra.ENABLE then
    local aRa = dtPhys / (ra.TAU + dtPhys)
    ra.rate = ra.rate + (pitchRateLog - ra.rate) * aRa
    ra.az = ra.az + (((sensors and sensors.ffiSensors and sensors.ffiSensors.sensorZ) or 0) - ra.az) * aRa
    ra.support = 9.81 - ra.az
    safety.air.support = ra.support
    safety.fg.raFires = ra.fires; safety.fg.raDelta = ra.lastDelta; safety.fg.onsetFires = ra.onsetFires
    if ra.cooldown > 0 then ra.cooldown = ra.cooldown - dtPhys end
    local mag = math.abs(ra.rate)
    if mag > ra.RATE_ON then
      local sgn = ra.rate > 0 and 1 or -1
      ra.onTime = (sgn == ra.sign) and (ra.onTime + dtPhys) or dtPhys
      ra.sign = sgn
      ra.settleTime = 0
      if ra.onTime >= ra.ON_MIN_S then ra.armed = true end
    elseif mag < ra.RATE_OFF then
      ra.onTime = 0
      ra.settleTime = (ra.support >= ra.SUPPORT_MIN) and (ra.settleTime + dtPhys) or 0
      if ra.armed and ra.settleTime >= ra.SETTLE_S then
        ra.armed = false
        if ra.cooldown <= 0 and fusedSpeed > ra.VMIN and maxWs > 0 and not deepHold then
          -- fastest wheel runs well under the commanded slip: use half the target; off-brake wheels = true
          local st = math.min(math.max(lastEffectiveTargets[maxIdx] or 0.14, SLIP_STEPDOWN_STFLOOR), 0.5)
          local target = isBraking and (maxWs / (1 - math.min(ra.SLIP_FRAC * st, 0.04))) or avgWs
          local cap = ra.MAX_FRAC * fusedSpeed
          local delta = math.max(-cap, math.min(cap, target - fusedSpeed))
          fusedSpeed = fusedSpeed + delta
          fwdVel = fusedSpeed; vLat = 0; vVert = 0; imuSpeed = fusedSpeed
          ra.cooldown = ra.COOLDOWN_S
          ra.fires = ra.fires + 1
          ra.lastDelta = delta
        end
      end
    end
    ra.maxWs = maxWs; ra.avgWs = avgWs
  end

  if maxWs < STANDSTILL_WS_THRESHOLD and fusedSpeed < STANDSTILL_FUSED_THRESHOLD
     and math.abs(latestSensorY) < STANDSTILL_ACCEL_MAX then
    standstillCounter = standstillCounter + 1
    if standstillCounter >= STANDSTILL_PHYS_TICKS then
      fusedSpeed = 0
      imuSpeed = 0
      fwdVel = 0
      vLat = 0
      vVert = 0
    end
  else
    standstillCounter = 0
  end

end


-- runTick(dt) â€” 200Hz: wheel-avg + PID + D-estimator
local function runTick(dt)
  local air = safety.air
  local brakeInput = input.brake or 0
  local brakingJustStarted = brakeInput > 0 and not wasBraking
  wasBraking = brakeInput > 0
  local isBraking = brakeInput > 0

  local abstelem = extensions.abstelemv2
  local haveTelem = abstelem ~= nil and abstelem.setBrakes ~= nil

  -- Brake-event recorder: track peak fused-vs-airspeed divergence per event
  do
    local airspeed = electrics.values.airspeed or 0
    local pressed = brakeInput > BRAKE_EVENT_PRESS_THRESHOLD
    if pressed and airspeed > BRAKE_EVENT_MIN_AIR then
      if currentBrakeEvent == nil then
        currentBrakeEvent = {
          peakDiffAbs = 0, peakDiffMs = 0,  -- m/s absolute + signed; UI converts to mph for display
          airAtPeak = airspeed, fusedAtPeak = fusedSpeed,
          startAir = airspeed, startTime = brakeSimTime, duration = 0,
        }
      end
      currentBrakeEvent.duration = brakeSimTime - currentBrakeEvent.startTime
      -- Track peak ABSOLUTE divergence (m/s) â€” only above 5 mph so low-speed sensor noise
      -- doesn't corrupt the peak with meaningless tiny-denominator readings.
      if airspeed >= BRAKE_EVENT_MIN_PEAK_AIR then
        local diffMs = fusedSpeed - airspeed
        if math.abs(diffMs) > currentBrakeEvent.peakDiffAbs then
          currentBrakeEvent.peakDiffAbs = math.abs(diffMs)
          currentBrakeEvent.peakDiffMs = diffMs
          currentBrakeEvent.airAtPeak = airspeed
          currentBrakeEvent.fusedAtPeak = fusedSpeed
        end
      end
    elseif currentBrakeEvent and not pressed then
      if currentBrakeEvent.duration >= BRAKE_EVENT_MIN_DUR then
        table.insert(brakeEvents, 1, currentBrakeEvent)
        while #brakeEvents > BRAKE_EVENT_MAX do table.remove(brakeEvents) end
        saveBrakeEvents()
      end
      currentBrakeEvent = nil
    end
  end

  -- Reverse bypass: no PID modulation when moving backward. Reverse speeds are low so ABS
  -- isn't critical, and forward-assuming slip math can get sign-confused. Three arcade-mode
  -- cases handled:
  --   1) Gamepad brake axis pressed: pass through directly.
  --   2) Driver hits "forward" while reversing -> arcade writes electrics.values.brake
  --      itself; we must release so the stock pipeline applies it (calling setBrakes(0,0,0,0)
  --      would zero wd.ref.brakeTorque and suppress arcade's auto-brake).
  --   3) Driver holds brake pedal in reverse -> arcade reinterprets as "more reverse throttle"
  --      and writes electrics.values.throttle > 0. Ignore input.brake in that case.
  if motionDirection < 0 then
    local arcadeWantsReverseThrottle = (electrics.values.throttle or 0) > 0.05
    local effectiveBrake = arcadeWantsReverseThrottle and 0 or brakeInput

    for i = 1, N_WHEELS do
      slipIntegral[i] = 0
      lastSlipError[i] = 0
      prevTickWheelSpeed[i] = latestWheelSpeed[i]
    end
    if effectiveBrake > 0 then
      local cmd = {}
      for i = 1, N_WHEELS do cmd[wheelToBrakeMap[i]] = effectiveBrake end
      if haveTelem then abstelem.setBrakes(cmd) end
    else
      if haveTelem and abstelem.releaseBrakes then abstelem.releaseBrakes() end
    end
    return
  end

  -- Wheel-average speed estimator (200Hz in the PID tick).
  -- Each wheel locks on a hard decel event; unlocks when the latest decel >= 0
  -- AND the buffer has seen at least one positive sample recently.
  do
    local goodSpeeds = {}
    local goodSum, goodCount = 0, 0
    for i = 1, N_WHEELS do
      local wDecel = (latestWheelSpeed[i] - wa.prevWs[i]) / dt

      wa.decelIdx[i] = (wa.decelIdx[i] % wa.LOCK_WINDOW) + 1
      wa.decelBuf[i][wa.decelIdx[i]] = wDecel

      if wa.locked[i] then
        local latest = wa.decelBuf[i][wa.decelIdx[i]]
        if latest >= 0 then
          local hasPositive = false
          for j = 1, wa.LOCK_WINDOW do
            local v = wa.decelBuf[i][j]
            if v and v > 0 then
              hasPositive = true
              break
            end
          end
          if hasPositive then
            wa.locked[i] = false
          end
        end
      else
        if wDecel < wa.LOCK_DECEL then
          wa.locked[i] = true
        end
      end

      if not wa.locked[i] and wDecel > -0.01 then
        goodSum = goodSum + latestWheelSpeed[i]
        goodCount = goodCount + 1
        goodSpeeds[#goodSpeeds + 1] = latestWheelSpeed[i]
      end
    end
    wa.updated = goodCount > 0
    if wa.updated then
      wa.speed = goodSum / goodCount
    end

    -- Two-wheel consensus: pairs within 2mph of each other
    wa.twoUpdated = false
    for a = 1, #goodSpeeds - 1 do
      for b = a + 1, #goodSpeeds do
        if math.abs(goodSpeeds[a] - goodSpeeds[b]) < 0.894 then
          wa.speedTwo = (goodSpeeds[a] + goodSpeeds[b]) / 2
          wa.twoUpdated = true
        end
      end
    end

    for i = 1, N_WHEELS do wa.prevWs[i] = latestWheelSpeed[i] end
  end

  -- carSpeed = fusedSpeed only (this is the 1F variant â€” no virtualAirspeed)
  local carSpeed = fusedSpeed
  absSpeedSource.wheelAvgActive = false
  absSpeedSource.fusedActive = true

  -- Turn handling: read yaw rate (honest ESC sensor) + soft deadband. Used below to build a
  -- per-wheel reference speed so each wheel's slip is measured against where IT should be
  -- rolling in the turn (outside wheels travel faster). Straight => yrEff 0 => no change.
  local yawRate = 0
  pcall(function() yawRate = obj:getYawAngularVelocity() or 0 end)
  grip.yawRate = yawRate
  

  -- Reset per-wheel PID and counters on new brake event
  if brakingJustStarted then
    for i = 1, N_WHEELS do
      slipIntegral[i] = 0
      lastSlipError[i] = 0
    end
    safety.snapUp = 0
    safety.snapUpRej = 0
  end

  -- Airborne axle detection (tunables in air); logical order 1=RR 2=RL 3=FR 4=FL
  if air.ENABLE then
    if not isBraking then
      air.front = false; air.rear = false; air.frontT = 0; air.rearT = 0
      for i = 1, N_WHEELS do air.tick[i] = 0 end
    else
      local supported = (air.support or 9.81) >= air.SUPPORT_MIN
      air.supportedT = supported and (air.supportedT + dt) or 0
      local bodyDecel = latestSensorY
      for i = 1, N_WHEELS do
        local a = (latestWheelSpeed[i] - prevTickWheelSpeed[i]) / dt
        air.wAcc[i] = air.wAcc[i] + (a - air.wAcc[i]) * 0.5
        local freeHold = math.abs(air.wAcc[i]) < air.HOLD_ACC and bodyDecel > air.BODY_DECEL
                         and (safety.lastAbsCoefs[i] or 1) < 0.05
        local spinDown = air.wAcc[i] < -air.SPINDOWN_ACC
        air.tick[i] = (freeHold or spinDown or not supported) and (air.tick[i] + dt) or 0
      end
      local function axleSpinDown(a1, a2)
        return air.wAcc[a1] < -air.SPINDOWN_ACC or air.wAcc[a2] < -air.SPINDOWN_ACC
      end
      local function axle(a1, a2, flag, tKey, cKey)
        if not air[flag] then
          local o1, o2 = (a1 == 1) and 3 or 1, (a1 == 1) and 4 or 2
          local iceLock = axleSpinDown(o1, o2) and not (air.support < air.SUPPORT_MIN)
          if air.tick[a1] >= air.ENTRY_S and air.tick[a2] >= air.ENTRY_S and not iceLock then
            air[flag] = true; air[tKey] = 0; air.events = air.events + 1; air[cKey] = air[cKey] + 1
          end
        else
          air[tKey] = air[tKey] + dt
          local landed = air.supportedT >= air.EXIT_S or air.wAcc[a1] > air.SPINUP_ACC
                         or air.wAcc[a2] > air.SPINUP_ACC or air[tKey] >= air.TIMEOUT_S
          if landed then
            air[flag] = false
            for _, i in ipairs({a1, a2}) do
              local landSlip = 1 - latestWheelSpeed[i] / math.max(carSpeed, 0.5)
              if landSlip < (slipTargets[i] or 0.14) then
                slipIntegral[i] = math.max(slipIntegral[i], air.REARM_DUTY / KI)
                air.rearms = (air.rearms or 0) + 1
              end
              lastSlipError[i] = 0
            end
          end
        end
      end
      axle(1, 2, "rear", "rearT", "rearEvents")
      axle(3, 4, "front", "frontT", "frontEvents")
    end
  end

  -- Flight gate (tunables in fg)
  do
    local fg = safety.fg
    if fg.ENABLE_FG then
      if not isBraking then
        fg.open = false; fg.dumped = false; fg.sawHigh = false; fg.inBandT = 0
        for i = 1, N_WHEELS do fg.tick[i] = 0 end
      else
        local sup = air.support or 9.81
        local outBand = sup < fg.BAND_LO or sup > fg.BAND_HI
        fg.outBandAgo = outBand and 0 or (fg.outBandAgo + dt)
        for i = 1, N_WHEELS do
          local a = (latestWheelSpeed[i] - prevTickWheelSpeed[i]) / dt
          fg.wAcc[i] = fg.wAcc[i] + (a - fg.wAcc[i]) * 0.5
          local tq = (safety.lastAbsCoefs[i] or 1) * (origBrakeTorque[i] or 0) * brakeInput
          local thr = math.max(fg.FLOOR, fg.SAFETY * (fg.k[i] or 0) * tq)
          local hit
          local duty = safety.lastAbsCoefs[i] or 1
          fg.relT[i] = (duty < fg.REL_DUTY) and (fg.relT[i] + dt) or 0
          if fg.relT[i] >= fg.REL_S then fg.wasRel[i] = true end
          if fg.TRIGGER == "release" then
            local slip = 1 - latestWheelSpeed[i] / math.max(carSpeed, 0.5)
            hit = fg.relT[i] >= fg.REL_S and fg.wAcc[i] < fg.REL_SPINUP and slip > fg.REL_SLIP
                  and fg.outBandAgo <= fg.REL_MEM_S
          else
            hit = outBand and (fg.TRIGGER == "support" or fg.wAcc[i] < -thr)
          end
          fg.tick[i] = hit and (fg.tick[i] + dt) or 0
        end
        local frontHit = fg.tick[3] >= fg.DWELL and fg.tick[4] >= fg.DWELL
        local rearHit  = fg.tick[1] >= fg.DWELL and fg.tick[2] >= fg.DWELL
        if not fg.open then
          if frontHit or rearHit then
            fg.open = true; fg.openT = 0; fg.sawHigh = false; fg.sawLow = false; fg.inBandT = 0; fg.dumped = false
            for i = 1, N_WHEELS do fg.wasRel[i] = fg.relT[i] >= fg.REL_S end
            fg.lockD = dest.consensusD; fg.events = fg.events + 1
            if frontHit then fg.frontEvents = fg.frontEvents + 1 else fg.rearEvents = fg.rearEvents + 1 end
          end
        else
          fg.openT = fg.openT + dt
          if sup > fg.BAND_HI then fg.sawHigh = true end
          if sup < fg.FLIGHT_SUP then fg.sawLow = true end
          fg.inBandT = outBand and 0 or (fg.inBandT + dt)
          local bReady
          if fg.B_MODE == "duty" then
            -- a wheel the PID had released is taking torque again and has finished spinning up
            bReady = false
            for i = 1, N_WHEELS do
              if fg.wasRel[i] and (safety.lastAbsCoefs[i] or 0) > 2 * fg.REL_DUTY and fg.wAcc[i] < fg.REL_SPINUP then bReady = true end
            end
          else
            bReady = fg.inBandT >= fg.B_SETTLE and (fg.sawHigh or fg.B_MODE == "any")
                     and (fg.TRIGGER ~= "support" or fg.sawLow)
          end
          if fg.B_ENABLE and not fg.dumped and bReady then
            local mw = 0
            for i = 1, N_WHEELS do if latestWheelSpeed[i] > mw then mw = latestWheelSpeed[i] end end
            fg.dumpTo = mw * (1 + fg.B_PCT); fg.dumped = true; fg.dumps = fg.dumps + 1
          end
          if fg.inBandT >= fg.HOLD_S or fg.openT >= fg.TIMEOUT_S then
            fg.open = false
            if fg.A_ENABLE and (fg.A_MODE == "hold" or fg.sawLow) then
              local w = dest.window
              for j = 1, dest.WINDOW_SIZE do w[j] = nil end
              dest.windowIdx = 1; dest.stableTicks = 0
              dest.consensusD = fg.lockD; dest.stableD = fg.lockD
            end
          end
        end
      end
    end
  end

  -- Per-wheel PID on slip error
  local cmd = {}
  local slipRatios = {}
  local slipErrors = {}
  local absCoefs = {}
  local effectiveTargets = {}
  for i = 1, N_WHEELS do
    cmd[i] = 0
    slipRatios[i] = 0
    slipErrors[i] = 0
    absCoefs[i] = 0
    effectiveTargets[i] = 0
  end

  local lowSpeedBoost = 1.0
  if carSpeed < safety.lowSpeed.SPEED_THRESH then
    local ratio = 1.0 - (carSpeed / safety.lowSpeed.SPEED_THRESH)
    lowSpeedBoost = 1.0 + (safety.lowSpeed.BOOST_MAX - 1.0) * ratio * ratio
  end
  lastLowSpeedBoost = lowSpeedBoost

  for i = 1, N_WHEELS do
    local slot = wheelToBrakeMap[i]

    local axleAir = air.ENABLE and ((i >= 3) and air.front or air.rear)
    if carSpeed > MIN_SPEED and isBraking and axleAir then
      -- airborne: hold PID state, output the floor only (early floor for short hops)
      local tAir = (i >= 3) and air.frontT or air.rearT
      local floor = (tAir < air.FLOOR_EARLY_S) and air.FLOOR_EARLY or air.FLOOR
      absCoefs[i] = floor
      effectiveTargets[i] = slipTargets[i]
      safety.slipRatios[i] = 0
      cmd[slot] = floor * brakeInput
    elseif carSpeed > MIN_SPEED and isBraking then
      -- per-wheel turn-compensated reference speed (vehicle speed + yaw geometry)
      -- Yaw compensation disabled as requested:
      
      local vRef = math.max(carSpeed, 0.5)
      local slip = math.max(0, math.min((vRef - latestWheelSpeed[i]) / vRef, 1))
      local deepen = ENABLE_LOWSPEED_SLIP_DEEPEN and ((1 + dest.consensusD) / carSpeed) or 0
      local effectiveTarget = math.min(slipTargets[i] + deepen, 1.0)
      local slipError = effectiveTarget - slip

      slipIntegral[i] = math.max(INTEGRAL_MIN, math.min(INTEGRAL_MAX, slipIntegral[i] + slipError * dt))

      local slipErrorDerivative = 0
      if lastSlipError[i] ~= 0 then
        slipErrorDerivative = (slipError - lastSlipError[i]) / dt
      end
      lastSlipError[i] = slipError

      local absCoef = math.max(0.0, math.min(1,
        slipError * KP + slipIntegral[i] * KI + slipErrorDerivative * KD))

      -- Low-speed brake boost: lightly scales up brake command below 30mph (13.41m/s) 
      -- to firmly halt the car. Uses a quadratic curve so it's gentle at 25mph and stronger near 0mph.
      if carSpeed < safety.lowSpeed.SPEED_THRESH then
        absCoef = math.min(1.0, absCoef * lowSpeedBoost)
      end

      -- Lockup guard: wheel decel too fast -> soft dump brake to 70% immediately
      local wheelDecel = (latestWheelSpeed[i] - prevTickWheelSpeed[i]) / dt
      if wheelDecel < WHEEL_DECEL_LIMIT then
        absCoef = absCoef * 0.70
        slipIntegral[i] = math.max(slipIntegral[i], 0)  -- no negative windup
      end

      slipRatios[i] = slip
      slipErrors[i] = slipError
      lastSlipErrors[i] = slipError
      lastSlipDerivatives[i] = slipErrorDerivative

      absCoefs[i] = absCoef
      effectiveTargets[i] = effectiveTarget
      safety.slipRatios[i] = slip

      cmd[slot] = absCoef * brakeInput
    else
      cmd[slot] = brakeInput
      slipIntegral[i] = 0
      lastSlipError[i] = 0
      absCoefs[i] = 1.0
      effectiveTargets[i] = slipTargets[i]
      safety.slipRatios[i] = 0
    end
  end
  for i = 1, N_WHEELS do safety.lastAbsCoefs[i] = absCoefs[i] end

  -- D-estimator v2 (tunables in d2; see notes/D_ESTIMATOR_V2_DESIGN.md)
  do
    local d2 = safety.d2
    if d2.ENABLE then
      local sup = (safety.air and safety.air.support) or 9.81
      local inBand = sup >= d2.BAND_LO and sup <= d2.BAND_HI
      local ax = latestSensorY                                   -- + = decel (body)
      local dFz = grip.mass * ax * grip.H_CG / (2 * grip.WHEELBASE)
      local sumFx = 0
      d2.histI = (d2.histI % 8) + 1
      for i = 1, N_WHEELS do
        local slot = wheelToBrakeMap[i] or i
        local wr = wheels.wheelRotators[slot - 1]
        local a = (latestWheelSpeed[i] - prevTickWheelSpeed[i]) / dt
        d2.aw[i] = d2.aw[i] + (a - d2.aw[i]) * 0.5
        local tb = (wr and wr.brakingTorque) or 0
        local tp = (wr and wr.propulsionTorque) or 0
        local rr = d2.r[i]
        local fx = (tb - tp) / rr + (d2.Ieff[i] / (rr * rr)) * d2.aw[i]
        local fz = grip.Fz0[i] * (sup / 9.81) + (d2.front[i] and dFz or -dFz)
        if fz < 200 then fz = 200 end
        d2.fx[i] = fx; d2.fz[i] = fz; sumFx = sumFx + fx
        local mu = d2.c * fx / fz
        d2.mu[i] = d2.mu[i] + (mu - d2.mu[i]) * 0.5              -- same EMA as slip
        local sr = 1 - latestWheelSpeed[i] / math.max(carSpeed, 0.5)
        d2.s[i] = d2.s[i] + (sr - d2.s[i]) * 0.5
        d2.sHist[i][d2.histI] = d2.s[i]
        -- surface-change guard on the slow level
        local prev = d2.muSlow[i]
        d2.muSlow[i] = d2.muSlow[i] + (d2.mu[i] - d2.muSlow[i]) * 0.05    -- ~100 ms
        if isBraking and prev > 0.3 and d2.mu[i] < prev * (1 - d2.GUARD_DROP) and d2.stateAge[i] > 0.2 then
          d2.off[1] = 0; d2.off[2] = 0; d2.guards = d2.guards + 1; d2.stateAge[i] = 0
        end
        if isBraking then
          if d2.mu[i] > d2.muLB[i] then d2.muLB[i] = d2.mu[i] end
        end
      end
      if isBraking and ax > 3 and sumFx > 2000 and inBand then
        d2.cNum = (d2.cNum or 0) + (grip.mass * ax - (d2.cNum or 0)) * 0.1     -- 50 ms EMAs @200 Hz
        d2.cDen = (d2.cDen or 1) + (sumFx - (d2.cDen or 1)) * 0.1
        local cInst = d2.cNum / math.max(d2.cDen, 1)
        if cInst < 0.5 then cInst = 0.5 elseif cInst > 1.5 then cInst = 1.5 end
        d2.c = d2.c + (cInst - d2.c) * 0.01
      end
      if not isBraking then
        d2.bufN = 0; d2.bufI = 0; d2.sinceFit = 0
        for i = 1, N_WHEELS do d2.state[i] = 0; d2.raw[i] = 0; d2.prevRaw[i] = 0; d2.muLB[i] = 0 end
        for a2 = 1, 2 do d2.holdT[a2] = 0; d2.holdSum[a2] = 0; d2.holdN[a2] = 0; d2.lastDir[a2] = 0; d2.lastMean[a2] = 0; d2.unknownT[a2] = 0 end
        if d2.failT then d2.failT[1][1] = 0; d2.failT[1][2] = 0; d2.failT[2][1] = 0; d2.failT[2][2] = 0 end
        if d2.lastLastMean then d2.lastLastMean[1] = 0; d2.lastLastMean[2] = 0 end
      elseif carSpeed > d2.VMIN then
        -- ring buffer (200 Hz): slip lagged by SIGMA / v to line up with force
        local lag = math.floor((d2.SIGMA / math.max(carSpeed, 1.0)) / dt + 0.5)
        if lag > 7 then lag = 7 end
        d2.bufI = (d2.bufI % d2.WIN) + 1
        if d2.bufN < d2.WIN then d2.bufN = d2.bufN + 1 end
        local hi = ((d2.histI - lag - 1) % 8) + 1
        for i = 1, N_WHEELS do
          d2.bufS[i][d2.bufI] = d2.sHist[i][hi] or d2.s[i]
          d2.bufM[i][d2.bufI] = d2.mu[i]
          d2.bufBad[i][d2.bufI] = (not inBand or (safety.lastAbsCoefs[i] or 1) < 0.05) and 1 or 0
          d2.stateAge[i] = d2.stateAge[i] + dt
        end
        d2.sinceFit = d2.sinceFit + 1
        if d2.bufN >= d2.WIN and d2.sinceFit >= d2.STEP then
          d2.sinceFit = 0; d2.fits = d2.fits + 1
          for i = 1, N_WHEELS do
            local S, M, B = d2.bufS[i], d2.bufM[i], d2.bufBad[i]
            local n = d2.WIN; local bad = 0; local smin, smax, sx = 1, -1, 0
            for k = 1, n do bad = bad + B[k]; local v = S[k]; sx = sx + v; if v < smin then smin = v end; if v > smax then smax = v end end
            local raw = 0
            if bad == 0 and (smax - smin) >= d2.EXC_MIN then
              local mx = sx / n
              -- detrend slip and mu against time first: both drift up as the car slows
              local tm = (n + 1) / 2; local stt, sty, sts, sym = 0, 0, 0, 0
              for k = 1, n do sym = sym + M[k] end
              local my = sym / n
              for k = 1, n do
                local kk = ((k - d2.bufI - 1) % n) + 1          -- chronological index of ring slot k
                local tc = kk - tm
                stt = stt + tc * tc; sty = sty + tc * (M[k] - my); sts = sts + tc * (S[k] - mx)
              end
              local trM = (stt > 0) and (sty / stt) or 0
              local trS = (stt > 0) and (sts / stt) or 0
              local s2, s3, s4, sy, sxy, sx2y = 0, 0, 0, 0, 0, 0
              for k = 1, n do
                local kk = ((k - d2.bufI - 1) % n) + 1
                local tc = kk - tm
                local x = (S[k] - mx) - trS * tc; local y = M[k] - trM * tc; local x2 = x * x
                s2 = s2 + x2; s3 = s3 + x2 * x; s4 = s4 + x2 * x2; sy = sy + y; sxy = sxy + x * y; sx2y = sx2y + x2 * y
              end
              -- normal equations [n 0 s2; 0 s2 s3; s2 s3 s4] [a b q] = [sy sxy sx2y]
              local det = n * (s2 * s4 - s3 * s3) - s2 * (s2 * s2)
              if det > 1e-12 then
                local A = (sy * (s2 * s4 - s3 * s3) + s2 * (sxy * s3 - s2 * sx2y)) / det
                local Bc = (n * (sxy * s4 - s3 * sx2y) + sy * s3 * s2 - s2 * s2 * sxy) / det
                local Q = (n * (s2 * sx2y - s3 * sxy) - sy * s2 * s2) / det
                local rss = 0
                for k = 1, n do
                  local kk = ((k - d2.bufI - 1) % n) + 1
                  local tc = kk - tm
                  local x = (S[k] - mx) - trS * tc; local y = M[k] - trM * tc
                  local e = y - (A + Bc * x + Q * x * x); rss = rss + e * e
                end
                local sig2 = rss / (n - 3)
                local invBB = (n * s4 - s2 * s2) / det          -- (X'X)^-1 [2][2]
                local invQQ = (n * s2) / det                     -- (X'X)^-1 [3][3]
                local seB = math.sqrt(math.max(sig2 * invBB, 1e-12))
                local seQ = math.sqrt(math.max(sig2 * invQQ, 1e-12))
                local zb, zq = Bc / seB, Q / seQ
                if zb > d2.Z_SIG then raw = 1 elseif zb < -d2.Z_SIG then raw = 3 elseif zq < -d2.Z_SIG then raw = 2 end
                d2.b[i] = Bc; d2.q[i] = Q
              end
              d2.prevWinMean[i] = d2.winMean[i]; d2.winMean[i] = mx
            end
            -- publish: two consecutive agree, or a crossing (below <-> past) = near peak
            local pub = 0
            if raw ~= 0 and raw == d2.prevRaw[i] then pub = raw
            elseif (raw == 1 and d2.prevRaw[i] == 3) or (raw == 3 and d2.prevRaw[i] == 1) then
              pub = 2; d2.sStar[i] = 0.5 * (d2.winMean[i] + d2.prevWinMean[i])
            end
            if pub ~= d2.state[i] then d2.stateAge[i] = 0 end
            d2.state[i] = pub; d2.prevRaw[i] = raw; d2.raw[i] = raw
            if pub == 2 then
              local mmax = 0
              for k = 1, n do if M[k] > mmax then mmax = M[k] end end
              d2.muPeak[i] = (d2.muPeak[i] > 0) and (d2.muPeak[i] * 0.7 + mmax * 0.3) or mmax
            end
          end
          -- axle states: 1 = rear (logical 1,2), 2 = front (3,4); rear select-low (past wins)
          for a2 = 1, 2 do
            local w1, w2 = (a2 == 1) and 1 or 3, (a2 == 1) and 2 or 4
            local s1, s2s = d2.state[w1], d2.state[w2]
            if s1 == s2s then d2.axleState[a2] = s1
            elseif a2 == 1 and (s1 == 3 or s2s == 3) then d2.axleState[a2] = 3
            else d2.axleState[a2] = 0 end
          end
        end
        -- L3A: per-axle target offset, stepped by state, kept only if cycle-mean grip rose
        if d2.CONTROL and inBand and safety.deep.mode == 0 then
          for a2 = 1, 2 do
            local w1, w2 = (a2 == 1) and 1 or 3, (a2 == 1) and 2 or 4
            d2.holdT[a2] = d2.holdT[a2] + dt
            local mAx = 0.5 * (d2.mu[w1] + d2.mu[w2])
            d2.holdSum[a2] = d2.holdSum[a2] + mAx; d2.holdN[a2] = d2.holdN[a2] + 1
            d2.holdSq = d2.holdSq or {0, 0}; d2.holdSq[a2] = d2.holdSq[a2] + mAx * mAx
            if d2.holdT[a2] >= d2.HOLD_S then
              local nH = math.max(d2.holdN[a2], 1)
              local mean = d2.holdSum[a2] / nH
              local var = math.max(d2.holdSq[a2] / nH - mean * mean, 0)
              local se = math.sqrt(var / nH) * 2.0
              d2.holdSq[a2] = 0
              d2.lastLastMean = d2.lastLastMean or {0, 0}
              local predicted = d2.lastMean[a2]
              if d2.lastLastMean[a2] > 0 then predicted = d2.lastMean[a2] + 0.5 * (d2.lastMean[a2] - d2.lastLastMean[a2]) end
              d2.failT = d2.failT or {{0, 0}, {0, 0}}; d2.lastProbe = d2.lastProbe or {0, 0}
              local ft = d2.failT[a2]
              ft[1] = math.max(ft[1] - d2.HOLD_S, 0); ft[2] = math.max(ft[2] - d2.HOLD_S, 0)
              local ld = d2.lastDir[a2]
              if ld ~= 0 and d2.lastMean[a2] > 0 and (mean - predicted) < math.max(d2.GAIN_MIN * predicted, se) then
                -- no clear gain above the trend: undo the step and mute that direction for a while
                d2.off[a2] = d2.off[a2] - ld * (d2.lastStep or d2.STEP_SIZE); d2.reverts = d2.reverts + 1
                ft[(ld > 0) and 1 or 2] = d2.MUTE_S; d2.lastDir[a2] = 0
              else
                local st = d2.axleState[a2]; local dir = 0
                local lowGrip = mean < d2.LOW_MU          -- narrow peak: finer steps, no probing
                if st == 1 then dir = 1 elseif st == 3 then dir = -1 elseif st == 2 then dir = 0
                elseif not lowGrip then
                  d2.unknownT[a2] = d2.unknownT[a2] + d2.HOLD_S
                  if d2.unknownT[a2] >= d2.PROBE_S then
                    dir = (d2.lastProbe[a2] > 0) and -1 or 1          -- alternate probe direction
                    d2.unknownT[a2] = 0
                  end
                end
                if st ~= 0 then d2.unknownT[a2] = 0 end
                if dir ~= 0 and ft[(dir > 0) and 1 or 2] > 0 then dir = 0 end
                if dir ~= 0 and st == 0 then d2.lastProbe[a2] = dir end
                local stepSz = d2.STEP_SIZE * math.max(0.25, math.min(1.0, mean / 1.0))
                local o = d2.off[a2] + dir * stepSz
                if o > d2.OFF_MAX then o = d2.OFF_MAX elseif o < -d2.OFF_MAX then o = -d2.OFF_MAX end
                d2.off[a2] = o; d2.lastDir[a2] = dir; d2.lastStep = stepSz
                if dir ~= 0 then d2.steps = d2.steps + 1 end
              end
              d2.lastLastMean[a2] = d2.lastMean[a2]; d2.lastMean[a2] = mean; d2.holdT[a2] = 0; d2.holdSum[a2] = 0; d2.holdN[a2] = 0
            end
          end
        end
      end
      -- deep-slip regime: 0 normal, 1 deep target on trial, 2 deep, 3 normal target on trial
      local deep = safety.deep
      if deep.ENABLE_DEEP then
        local lvl = (sup > 3) and (ax / sup) or 0                  -- body decel in g, support-normalized
        deep.lvl = deep.lvl + (lvl - deep.lvl) * 0.1
        if deep.active then deep.heading = deep.heading + (grip.yawRate or 0) * dt else deep.heading = 0 end
        if deep.wasActive and not deep.active then deep.recover = 1; deep.recoverT = 0 end
        deep.wasActive = deep.active
        if deep.recover > 0 then
          deep.recoverT = deep.recoverT + dt
          local maxWs = 0
          for i = 1, N_WHEELS do if latestWheelSpeed[i] > maxWs then maxWs = latestWheelSpeed[i] end end
          local tgt = lastEffectiveTargets[1] or 0.2
          local back = maxWs >= (1 - 2 * tgt) * carSpeed
          if deep.recoverT >= deep.RECOVER_MAX_S or (deep.recoverT >= deep.RECOVER_MIN_S and back) then deep.recover = 0 end
        end
        -- the regime only exists under a real stop: reference and regulation flag belong to this stop
        local hard = isBraking and (input.brake or 0) > 0.5
        if not hard then deep.regulated = false; deep.refT = 0; deep.refSum = 0; deep.refN = 0
        else for i = 1, N_WHEELS do if (safety.lastAbsCoefs[i] or 1) < 0.95 then deep.regulated = true end end end
        local turning = math.abs(electrics.values.steering or 0) > deep.STEER_MAX or math.abs(grip.yawRate or 0) > deep.YAW_MAX
          or math.abs(deep.heading) > deep.HEADING_MAX
        deep.cool = math.max(deep.cool - dt, 0)
        if not hard then
          deep.mode = 0; deep.active = false; deep.regT = 0; deep.stopT = 0; deep.ref1 = 0; deep.ref2 = 0
        else
          deep.stopT = deep.stopT + dt
          if inBand and deep.regulated then
            deep.refT = deep.refT + dt; deep.refSum = deep.refSum + deep.lvl; deep.refN = deep.refN + 1
            if deep.refT >= 0.1 then
              deep.ref1 = deep.ref2; deep.ref2 = deep.refSum / math.max(deep.refN, 1)
              deep.refT = 0; deep.refSum = 0; deep.refN = 0
            end
          end
          local m = deep.mode
          if m == 0 then
            -- entry: no level or flatness gate, only support in band, straight, speed, cooldown, and a 0.2 s reference
            local levelOk = (not deep.LEVEL_GATE) or (deep.ref2 >= deep.LO and deep.ref2 <= deep.HI)
            if inBand and deep.ref1 > 0 and deep.regulated and levelOk and not turning and carSpeed > deep.VMIN and deep.cool <= 0 then
              deep.base = 0.5 * (deep.ref1 + deep.ref2)
              deep.mode = 1; deep.active = true; deep.t = 0; deep.sum = 0; deep.n = 0; deep.probes = deep.probes + 1
            end
          elseif m == 1 or m == 3 then
            deep.t = deep.t + dt
            if turning then
              deep.mode = 0; deep.active = false; deep.cool = deep.STEER_COOL; deep.exits = deep.exits + 1; deep.lastExit = 1
            else
              if deep.t >= deep.SKIP_S and inBand then deep.sum = deep.sum + deep.lvl; deep.n = deep.n + 1 end
              if deep.t >= deep.PROBE_S then
                local meas = (deep.n > 0) and (deep.sum / deep.n) or 0
                deep.lastMeas = meas; deep.lastN = deep.n
                if m == 1 then
                  deep.lastBase = deep.base
                  if deep.n < 10 then
                    deep.mode = 0; deep.active = false; deep.cool = deep.ABORT_COOL; deep.aborts = deep.aborts + 1   -- bumps: no verdict
                  elseif meas >= deep.base * (1 + deep.GAIN) then
                    deep.mode = 2; deep.active = true; deep.keeps = deep.keeps + 1; deep.deepT = 0; deep.mean = meas; deep.hiT = 0
                  else
                    deep.mode = 0; deep.active = false; deep.cool = deep.FAIL_COOL; deep.fails = deep.fails + 1
                  end
                elseif deep.n >= 10 and meas > deep.mean * (1 - deep.GAIN) then
                  deep.mode = 0; deep.active = false; deep.cool = deep.EXIT_COOL; deep.exits = deep.exits + 1; deep.lastExit = 4   -- normal is as good
                else
                  deep.mode = 2; deep.active = true; deep.deepT = 0
                end
              end
            end
          elseif m == 2 then
            deep.deepT = deep.deepT + dt; deep.time = deep.time + dt
            if inBand then
              deep.mean = deep.mean + (deep.lvl - deep.mean) * 0.05
              deep.hiT = (deep.lvl > deep.HI + deep.HI_MARGIN) and (deep.hiT + dt) or 0
              deep.lowT = (deep.deepT > 0.3 and deep.lvl < deep.mean * (1 - deep.DROP)) and (deep.lowT + dt) or 0
            end
            if turning then
              deep.mode = 0; deep.active = false; deep.cool = deep.STEER_COOL; deep.exits = deep.exits + 1; deep.lastExit = 1
            elseif deep.hiT >= 0.1 then
              deep.mode = 0; deep.active = false; deep.cool = deep.EXIT_COOL; deep.exits = deep.exits + 1; deep.hiT = 0; deep.lastExit = 2   -- asphalt again
            elseif deep.lowT >= 0.1 then
              deep.mode = 0; deep.active = false; deep.cool = deep.EXIT_COOL; deep.exits = deep.exits + 1; deep.lastExit = 3; deep.lowT = 0   -- grip collapsed
            elseif deep.deepT >= deep.RECHECK_S and carSpeed > deep.VMIN then
              deep.mode = 3; deep.active = false; deep.t = 0; deep.sum = 0; deep.n = 0
              if deep.ref1 > 0 then deep.mean = 0.5 * (deep.ref1 + deep.ref2) end
            end
          end
        end
      end
    end
  end
  for i = 1, N_WHEELS do lastEffectiveTargets[i] = effectiveTargets[i] end

  -- Brake Delay Simulator & Global Peak-Hunter
  local totalTorque = 0
  local avgWheelDecel = 0
  local decelCount = 0
  
  for i = 1, N_WHEELS do
    local commandedTorque = (safety.lastAbsCoefs[i] or 1) * (origBrakeTorque[i] or 0) * brakeInput
    local current = ph.simulatedTorque[i] or 0
    if commandedTorque > current then
      current = math.min(current + (ph.brakeInRate[i] or 9999) * dt, commandedTorque)
    else
      current = math.max(current - (ph.brakeOutRate[i] or 9999) * dt, commandedTorque)
    end
    ph.simulatedTorque[i] = current
    totalTorque = totalTorque + current
    
    local wDecel = (prevTickWheelSpeed[i] - latestWheelSpeed[i]) / dt
    if wDecel > 0.1 then 
      avgWheelDecel = avgWheelDecel + wDecel
      decelCount = decelCount + 1
    end
  end

  local throttleInput = input.throttle or 0
  if ph.ENABLE_THROTTLE_LOCKOUT then
    ph.seekerSuppressed = (brakeInput > 0.05 and throttleInput > 0.05)
  else
    ph.seekerSuppressed = false
  end

  if ph.ENABLE and isBraking and not ph.seekerSuppressed then
    local latAccel = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorX) or 0
    local isStraight = math.abs(latAccel) < 3.0 -- roughly 0.3g
    
    local frontError = (math.abs(lastSlipError[3] or 0) + math.abs(lastSlipError[4] or 0)) / 2
    local rearError = (math.abs(lastSlipError[1] or 0) + math.abs(lastSlipError[2] or 0)) / 2
    local currentError = (ph.turn == 1) and frontError or rearError
    local pidSettled = currentError < 0.02
    
    if carSpeed > 6.7 and isStraight then
      if ph.phase == 0 then -- STEP PHASE
        -- Adaptive step size based on speed (fusedSpeed mapped from 15mph to 60mph)
        local speedRatio = math.max(0, math.min(1, (carSpeed - 6.7) / 20.0))
        local stepSize = 0.005 + (0.05 - 0.005) * speedRatio
        
        if ph.turn == 1 then
          ph.frontOffset = math.max(-0.05, math.min(0.15, ph.frontOffset + ph.frontDirection * stepSize))
        else
          ph.rearOffset = math.max(-0.05, math.min(0.15, ph.rearOffset + ph.rearDirection * stepSize))
        end
        ph.phase = 1
        ph.settleTicks = 0
        
      elseif ph.phase == 1 then -- SETTLE PHASE
        if pidSettled then
          ph.settleTicks = ph.settleTicks + 1
          if ph.settleTicks >= 8 then -- 40ms settled (at 200Hz)
            ph.phase = 2
            ph.measureTicks = 0
            ph.startSpeed = fusedSpeed
          end
        else
          ph.settleTicks = 0 -- Reset if disturbed
        end
        
      elseif ph.phase == 2 then -- MEASURE PHASE
        if pidSettled and isStraight then
          ph.measureTicks = ph.measureTicks + 1
          if ph.measureTicks >= 10 then -- 50ms measured
            ph.phase = 3
          end
        else
          ph.phase = 1
          ph.settleTicks = 0
        end
        
      elseif ph.phase == 3 then -- EVALUATE PHASE
        local decel = (ph.startSpeed - fusedSpeed) / (ph.measureTicks * dt)
        
        if ph.turn == 1 then
          local diff = decel - ph.lastFrontEfficiency
          if math.abs(diff) < math.max(0.001, math.abs(ph.lastFrontEfficiency) * 0.005) then
            -- Flat, hold direction (kills noise thrashing bug)
          elseif diff < 0 then
            ph.frontDirection = -ph.frontDirection
          end
          ph.lastFrontEfficiency = decel
          ph.turn = 2 -- handoff
        else
          local diff = decel - ph.lastRearEfficiency
          if math.abs(diff) < math.max(0.001, math.abs(ph.lastRearEfficiency) * 0.005) then
            -- Flat, hold direction
          elseif diff < 0 then
            ph.rearDirection = -ph.rearDirection
          end
          ph.lastRearEfficiency = decel
          ph.turn = 1 -- handoff
        end
        ph.phase = 0
      end
    end
  else
    if not isBraking then
      if carSpeed > 13.41 then -- ~30mph (Reset fully indicating a new driving context)
        ph.frontOffset = 0
        ph.rearOffset = 0
        ph.lastFrontEfficiency = 0
        ph.lastRearEfficiency = 0
      else
        -- Soft decay toward 0 instead of snap to 0
        ph.frontOffset = ph.frontOffset * 0.999
        ph.rearOffset = ph.rearOffset * 0.999
      end
      ph.phase = 0
      ph.settleTicks = 0
      ph.measureTicks = 0
      ph.turn = 1
    end
  end

  -- D estimator: peak sensorY decel over a sliding window, D = peak / g.
  -- No Pacejka shape assumption â€” avoids the overestimation the particle filter had on ice.
  if isBraking and carSpeed > MIN_ADAPT_SPEED and not (safety.fg.A_ENABLE and safety.fg.A_MODE == "hold" and safety.fg.open) then
    local measuredDecel = latestSensorY  -- friction-only, slope-independent

    if dest.undoTicks > 0 then
      dest.undoTicks = dest.undoTicks - 1
      local lvl = dest.shadowLevel
      local inBand = measuredDecel >= lvl * (1 - dest.SNAP_UNDO_BAND)
                 and measuredDecel <= lvl * (1 + dest.SNAP_UNDO_BAND)
      dest.undoInBand = inBand and (dest.undoInBand + 1) or 0
      if dest.undoInBand >= dest.SNAP_UNDO_CONFIRM then
        -- transient confirmed: put the pre-snap estimator back
        local sw, w = dest.shadowWindow, dest.window
        for j = 1, dest.WINDOW_SIZE do w[j] = sw[j] end
        dest.windowIdx = dest.shadowIdx
        dest.consensusD = dest.shadowD
        dest.stableD = dest.shadowStableD
        dest.stableTicks = dest.shadowStableTicks
        dest.snapUndos = dest.snapUndos + 1
        dest.undoTicks = 0
        dest.undoInBand = 0
      end
    end

    if measuredDecel > dest.UPDATE_MIN_DECEL then
      dest.windowIdx = (dest.windowIdx % dest.WINDOW_SIZE) + 1
      dest.window[dest.windowIdx] = measuredDecel

      local aggDecel = aggregateDecelWindow()   -- peak / mean / topN per D_AGG_MODE

      -- 0.91 = weight-transfer grip boost + 200Hz peak-sampling bias
      local instantD = math.max(dest.EST_MIN, math.min(dest.EST_MAX, aggDecel / 9.81 * 0.91))

      -- Surface-change detection: D jump >40% = reset window
      dest.stableTicks = dest.stableTicks + 1
      if dest.stableTicks > dest.SETTLE_TICKS then
        local dChange = math.abs(instantD - dest.stableD) / math.max(dest.stableD, 0.1)
        if dChange > dest.CHANGE_THRESHOLD then
          local w = dest.window
          if dest.SNAP_UNDO_ENABLE and instantD > dest.stableD then
            local sw = dest.shadowWindow
            for j = 1, dest.WINDOW_SIZE do sw[j] = w[j] end
            dest.shadowIdx = dest.windowIdx
            dest.shadowD = dest.consensusD
            dest.shadowStableD = dest.stableD
            dest.shadowStableTicks = dest.stableTicks
            dest.shadowLevel = dest.stableD * 9.81 / 0.91   -- inverse of the D mapping
            dest.undoTicks = dest.SNAP_UNDO_TICKS
            dest.undoInBand = 0
          end
          for j = 1, dest.WINDOW_SIZE do w[j] = nil end
          dest.windowIdx = 1
          w[1] = measuredDecel
          dest.retroResets = dest.retroResets + 1
          dest.snapTotal = dest.snapTotal + 1
          dest.stableTicks = 0
          dest.consensusD = instantD
          ph.frontOffset = 0; ph.rearOffset = 0
          ph.lastFrontEfficiency = 0; ph.lastRearEfficiency = 0
        end
        dest.stableD = dest.consensusD
      end

      -- Smooth consensus toward peak-decel estimate
      dest.consensusD = dest.consensusD * dest.SMOOTHING + instantD * (1.0 - dest.SMOOTHING)

      -- Map D to slip target ([FLAT TARGET EXPERIMENT] fixed base bypasses the D-law)
      if USE_FIXED_SLIP_TARGET then
        dest.baseTarget = FIXED_SLIP_TARGET
      else
        dest.baseTarget = math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX,
          0.04 + dest.consensusD * 0.10))
      end

      -- Track average D over the ABS event (Bug/Feature request)
      local isAbsActive = false
      for j = 1, N_WHEELS do
        if (safety.lastAbsCoefs[j] or 1) < 0.99 then
          isAbsActive = true
          break
        end
      end
      
      if isAbsActive then
        if not dest.absEventActive then
          dest.absEventActive = true
          dest.absEventSum = 0
          dest.absEventTicks = 0
          dest.absEventTotalTicks = 0
          dest.retroResets = 0
        end
        
        dest.absEventTotalTicks = dest.absEventTotalTicks + 1
        
        -- Filter out first 100ms (20 ticks at 200Hz) and any readings below 18mph (8.04 m/s)
        if dest.absEventTotalTicks > 20 and carSpeed >= 8.04 then
          dest.absEventSum = dest.absEventSum + dest.consensusD
          dest.absEventTicks = dest.absEventTicks + 1
        end
      else
        if dest.absEventActive then
          if dest.absEventTicks >= 10 and dest.retroResets == 0 then -- >50ms at 200Hz
            local avgD = dest.absEventSum / dest.absEventTicks
            dest.consensusD = avgD
            dest.stableD = avgD
          end
          dest.absEventActive = false
        end
      end

      if not grip.ENABLE_PERWHEEL_D then
        -- Escape hatch: original global broadcast â€” every wheel takes the global target.
        for j = 1, N_WHEELS do
          grip.Dwheel[j] = dest.consensusD
          local isFront = false
          for _, li in ipairs(frontLogicalIndices) do if li == j then isFront = true end end
          local trim = isFront and ph.frontOffset or ph.rearOffset
          local d2off = (safety.d2.ENABLE and safety.d2.CONTROL) and safety.d2.off[isFront and 2 or 1] or 0
          local finalTarget = math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX, dest.baseTarget + trim + d2off))
          if safety.deep.active then
            slipTargets[j] = safety.deep.TARGET                     -- deep regime: no smoothing either way
          else
            if slipTargets[j] > finalTarget + 0.15 then slipTargets[j] = finalTarget end
            slipTargets[j] = slipTargets[j] * TARGET_SMOOTHING + finalTarget * (1.0 - TARGET_SMOOTHING)
          end
        end
      else
        -- Per-wheel ANCHORED brake-acceptance. dest.consensusD sets the LEVEL; per-wheel
        -- brake-acceptance carves the SPLIT. g_j = appliedTorque_j / Fz_j  (~ mu_j).
        -- Longitudinal load transfer from the smoothed global decel (dest.consensusD*g); left/right
        -- loads are symmetric so the split there is pure brake-acceptance.
        local aLong = dest.consensusD * grip.GRAV
        local dFz = grip.mass * aLong * grip.H_CG / (2 * grip.WHEELBASE)
        local sumG, nG = 0, 0
        for j = 1, N_WHEELS do
          local maxT = origBrakeTorque[j] or 0
          local applied = absCoefs[j] * maxT
          -- (2) front/rear distinction via frontLogicalIndices: front gets +dFz, rear gets -dFz
          local isFront = false
          for _, fi in ipairs(frontLogicalIndices) do
            if j == fi then isFront = true; break end
          end
          local fz = grip.Fz0[j] + (isFront and dFz or -dFz)
          if fz < grip.FZ_MIN then fz = grip.FZ_MIN end
          local gj = applied / fz
          -- confident only while actively ABS-regulating (not saturated), rolling, real brake on
          local conf = (absCoefs[j] < grip.SAT)
            and (latestWheelSpeed[j] > grip.ROLL_MIN)
            and (applied > grip.TORQUE_MIN_FRAC * maxT)
          grip.confident[j] = conf

          if conf then
            grip.gEMA[j] = grip.gEMA[j] * grip.G_SMOOTH + gj * (1 - grip.G_SMOOTH)
            sumG = sumG + grip.gEMA[j]
            nG = nG + 1
          end
        end

        if nG > 0 then
          local gBar = sumG / nG
          if gBar < grip.EPS then gBar = grip.EPS end
          for j = 1, N_WHEELS do
            -- non-confident wheels drift their EMA back toward the confident mean
            if not grip.confident[j] then
              grip.gEMA[j] = grip.gEMA[j] + (gBar - grip.gEMA[j]) * grip.DECAY
            end
            -- anchored: mean of D stays = dest.consensusD; ratio carries the per-wheel split
            local Dj = dest.consensusD * (grip.gEMA[j] / gBar)
            if Dj < dest.EST_MIN then Dj = dest.EST_MIN elseif Dj > dest.EST_MAX then Dj = dest.EST_MAX end
            grip.Dwheel[j] = Dj
            local isFront = false
            for _, li in ipairs(frontLogicalIndices) do if li == j then isFront = true end end
            local trim = isFront and ph.frontOffset or ph.rearOffset
            local baseTarget = USE_FIXED_SLIP_TARGET and FIXED_SLIP_TARGET
              or math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX, 0.04 + Dj * 0.10 + trim))
            slipTargets[j] = slipTargets[j] * TARGET_SMOOTHING + baseTarget * (1.0 - TARGET_SMOOTHING)
          end
        else
          -- no confident wheel (light braking / all locked / first ticks): fall back to global
          for j = 1, N_WHEELS do
            grip.Dwheel[j] = dest.consensusD
            local isFront = false
            for _, li in ipairs(frontLogicalIndices) do if li == j then isFront = true end end
            local trim = isFront and ph.frontOffset or ph.rearOffset
            local finalTarget = USE_FIXED_SLIP_TARGET and FIXED_SLIP_TARGET
              or math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX, dest.baseTarget + trim))
            slipTargets[j] = slipTargets[j] * TARGET_SMOOTHING + finalTarget * (1.0 - TARGET_SMOOTHING)
          end
        end
      end
    end
  end

  -- Push brake commands
  if haveTelem then
    abstelem.setBrakes(cmd)
  end
  
  local telemetryLogger = extensions.absTelemetryLogger
  if telemetryLogger and telemetryLogger.setCustomTelemetry then
    telemetryLogger.setCustomTelemetry("Dynamic_ABS", {
      fusedSpeed = fusedSpeed,
      nextD = dest.consensusD,
      slipTargets = {slipTargets[1], slipTargets[2], slipTargets[3], slipTargets[4]},
      pidOut = {safety.lastAbsCoefs[1] or 1, safety.lastAbsCoefs[2] or 1, safety.lastAbsCoefs[3] or 1, safety.lastAbsCoefs[4] or 1},
      maxBrake = {origBrakeTorque[1] or 0, origBrakeTorque[2] or 0, origBrakeTorque[3] or 0, origBrakeTorque[4] or 0},
      trimOffset = ph.rearOffset or 0,
      trimDirection = ph.rearDirection or 1,
      pidGains = {KP, KI, KD},
      imuSpeed = imuSpeed,
      imuClamps = safety.imuClamps,
      fwdVel = fwdVel,
      vLat = vLat,
      vVert = vVert,
      effectiveTarget = {lastEffectiveTargets[4], lastEffectiveTargets[3], lastEffectiveTargets[2], lastEffectiveTargets[1]},
      gripD = {grip.Dwheel[4], grip.Dwheel[3], grip.Dwheel[2], grip.Dwheel[1]},
      gripConfident = (function() local n=0 for j=1,4 do if grip.confident[j] then n=n+1 end end return n end)(),
      lowSpeedBoost = lastLowSpeedBoost,
      frontTrimOffset = ph.frontOffset or 0,
      frontTrimDirection = ph.frontDirection or 1,
      slipError = {lastSlipErrors[4], lastSlipErrors[3], lastSlipErrors[2], lastSlipErrors[1]},
      slipIntegralState = {slipIntegral[4], slipIntegral[3], slipIntegral[2], slipIntegral[1]},
      slipDerivative = {lastSlipDerivatives[4], lastSlipDerivatives[3], lastSlipDerivatives[2], lastSlipDerivatives[1]},
      snapUpCount = safety.snapUp
    })
  end

  -- prevTickWheelSpeed for next tick's lockup guard
  for i = 1, N_WHEELS do
    prevTickWheelSpeed[i] = latestWheelSpeed[i]
  end
end


-- update(dtPhys) â€” 2kHz orchestrator
local function update(dtPhys)
  detectMu(dtPhys)
  brakeSimTime = brakeSimTime + dtPhys

  timeAccum = timeAccum + dtPhys
  while timeAccum >= TICK_STEP do
    runTick(TICK_STEP)
    timeAccum = timeAccum - TICK_STEP
  end

  -- Data Logging (50Hz) to track down IMU drift
  local brakeInput = math.max(input.brake or 0, electrics.values.brake or 0)  -- test machine drives input.brake
  if brakeInput > 0.01 then
    if not isLogging then
      isLogging = ENABLE_IMU_LOG
      logData = {}
      table.insert(logData, "Time,Airspeed,FusedSpeed,ImuSpeed,FwdVel,VLat,VVert,Ax,Ay,Az,YawRate,PitchRate,RollRate,Pitch,Roll,RaRate,Armed,Fires,Support,MaxWs,AvgWs,St,AirF,AirR")
      logTimer = 0
    end
    
    logTimer = logTimer + dtPhys
    if logTimer >= 0.02 then
      logTimer = 0
      local row = string.format("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%d,%d,%.2f,%.2f,%.2f,%.3f,%d,%d", 
        brakeSimTime, electrics.values.airspeed or 0, fusedSpeed, imuSpeed, fwdVel, vLat, vVert,
        -latestSensorY, (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorX) or 0,
        (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorZ) or 0,
        (obj:getYawAngularVelocity() or 0), pitchRateLog, rollRateLog, pitchLog, rollLog,
        ra.rate, ra.armed and 1 or 0, ra.fires, ra.support, ra.maxWs, ra.avgWs, (lastEffectiveTargets[1] or 0), safety.air.front and 1 or 0, safety.air.rear and 1 or 0
      )
      table.insert(logData, row)
    end
  else
    if isLogging then
      isLogging = false
      if #logData > 40 then  -- skip brake taps (20 ms rows)
        local file = io.open(string.format("abs_imu_log_%07d.csv", math.floor(brakeSimTime * 10)), "w")
        if file then
          file:write(table.concat(logData, "\n"))
          file:close()
        end
      end
      logData = {}
    end
  end

  -- Broadcast brake events to UI at 5Hz
  brakeUiAccum = brakeUiAccum + dtPhys
  if brakeUiAccum >= 0.2 then
    brakeUiAccum = 0
    if guihooks then
      guihooks.trigger('updateABSBrakeEvents', {
        events = brakeEvents,
        live = currentBrakeEvent,
      })
    end
  end

  -- Publish D-estimator state for external reading (test harness etc.)
  electrics.values.abs_consensusD = dest.consensusD
  electrics.values.abs_baseTarget = dest.baseTarget
  electrics.values.abs_retroResets = dest.retroResets
  electrics.values.abs_snapUndos = dest.snapUndos
  electrics.values.abs_snapTotal = dest.snapTotal
  electrics.values.abs_snapPending = dest.undoTicks
  electrics.values.abs_reanchors = ra.fires
  electrics.values.abs_reanchorDelta = ra.lastDelta
  electrics.values.abs_onsetSnaps = ra.onsetFires
  electrics.values.abs_airEvents = air.events
  electrics.values.abs_airFront = air.frontEvents
  electrics.values.abs_airRear = air.rearEvents
  electrics.values.abs_airRearms = air.rearms or 0
  electrics.values.abs_fgEvents = fg.events
  electrics.values.abs_fgDumps = fg.dumps
  electrics.values.abs_fgFront = fg.frontEvents
  electrics.values.abs_fgRear = fg.rearEvents
  electrics.values.abs_d2_offF = d2.off[2]
  electrics.values.abs_d2_offR = d2.off[1]
  electrics.values.abs_d2_c = d2.c
  electrics.values.abs_d2_steps = d2.steps
  electrics.values.abs_d2_reverts = d2.reverts
  electrics.values.abs_d2_guards = d2.guards
  electrics.values.abs_d2_stateF = d2.axleState[2]
  electrics.values.abs_d2_stateR = d2.axleState[1]
  electrics.values.abs_deep_mode = safety.deep.mode
  electrics.values.abs_deep_probes = safety.deep.probes
  electrics.values.abs_deep_keeps = safety.deep.keeps
  electrics.values.abs_deep_exits = safety.deep.exits
  electrics.values.abs_deep_time = safety.deep.time
  electrics.values.abs_deep_fails = safety.deep.fails
  electrics.values.abs_deep_aborts = safety.deep.aborts
  electrics.values.abs_deep_lastBase = safety.deep.lastBase
  electrics.values.abs_deep_lastMeas = safety.deep.lastMeas
  electrics.values.abs_deep_lastN = safety.deep.lastN
  electrics.values.abs_deep_lastExit = safety.deep.lastExit
  electrics.values.abs_surface = getCondition(slipTargets[1])
  electrics.values.abs_wheelAvgSpeed = wa.speed
  electrics.values.abs_wheelAvgSpeedTwo = wa.speedTwo
  -- Per-wheel surface grip (RR/RL/FR/FL) + how many wheels are confidently sensed
  electrics.values.abs_D_RR = grip.Dwheel[1]
  electrics.values.abs_D_RL = grip.Dwheel[2]
  electrics.values.abs_D_FR = grip.Dwheel[3]
  electrics.values.abs_D_FL = grip.Dwheel[4]
  do
    local confCount = 0
    for j = 1, N_WHEELS do if grip.confident[j] then confCount = confCount + 1 end end
    electrics.values.abs_grip_conf = confCount
  end
  electrics.values.abs_yawRate = grip.yawRate   -- rad/s; for verifying the turn-correction sign

  -- UI update at 5Hz
  local d2ui = function(i)
    local d2 = safety.d2
    if not d2.ENABLE then return string.format("%.2f", grip.Dwheel[i]) end
    local L = ({[0] = "?", [1] = "B", [2] = "N", [3] = "P"})[d2.state[i]] or "?"
    return string.format("%.2f %s", d2.mu[i], L)
  end
  uiAccum = uiAccum + dtPhys
  if uiAccum >= 0.2 then
    uiAccum = 0
    if guihooks then
      guihooks.trigger('updateABSGrip', {
        RR = { surfaceMu = d2ui(1), slipMu = string.format("%.2f", safety.slipRatios[1]) },
        RL = { surfaceMu = d2ui(2), slipMu = string.format("%.2f", safety.slipRatios[2]) },
        FR = { surfaceMu = d2ui(3), slipMu = string.format("%.2f", safety.slipRatios[3]) },
        FL = { surfaceMu = d2ui(4), slipMu = string.format("%.2f", safety.slipRatios[4]) },
        speeds = {
          airspeed = string.format("%.1f", electrics.values.airspeed or 0),
          fusedSpeed = string.format("%.1f", fusedSpeed or 0),
          plausibleSpeed = string.format("%.1f", wa.speed or 0),
          virtualAirspeed = string.format("%.1f", electrics.values.virtualAirspeed or 0),
          snapUpCount = safety.snapUp,
          snapUpRejCount = safety.snapUpRej,
          imuSpeed = string.format("%.1f", imuSpeed or 0),
          imuClampCount = safety.imuClamps,
          slipStepdownCount = slipStepdownCount,
          vLat = string.format("%.1f", vLat or 0),
          yaw2d = string.format("%.2f", grip.yawRate or 0),
          fusedActive = absSpeedSource.fusedActive,
          nextDEstimate = string.format("%.2f", dest.consensusD),
          reanchorCount = safety.fg.raFires or 0,
          reanchorDelta = string.format("%+.1f", safety.fg.raDelta or 0),
          onsetSnaps = safety.fg.onsetFires or 0,
          flightEvents = safety.fg.events or 0,
          flightDumps = safety.fg.dumps or 0,
          flightOpen = safety.fg.open and true or false,
          support = string.format("%.1f", safety.air.support or 9.81),
          d2off = string.format("%+.3f/%+.3f", safety.d2.off[2], safety.d2.off[1]),
          d2c = string.format("%.2f", safety.d2.c),
          d2steps = safety.d2.steps, d2reverts = safety.d2.reverts,
        }
      })
    end
  end
end


local function reset(jbeamData)
  init(jbeamData)
end


-- Clear brake event history (callable from UI via controller.getControllerSafe).
local function clearBrakeEvents()
  brakeEvents = {}
  currentBrakeEvent = nil
  pcall(jsonWriteFile, BRAKE_EVENT_FILE, {}, false)
  if guihooks then
    guihooks.trigger('updateABSBrakeEvents', { events = {}, live = nil })
  end
end


local function getCounts()
  return {snapUp = safety.snapUp, snapUpRej = safety.snapUpRej,
          imuClamps = safety.imuClamps}
end
local function resetCounts()
  safety.snapUp = 0; safety.snapUpRej = 0; safety.imuClamps = 0
end
local function setPerWheelD(val) grip.ENABLE_PERWHEEL_D = val end

-- Switch the D-estimator window aggregation live (no reload). mode = "peak"|"mean"|"topn".
local function setDAggMode(mode, topn)
  if mode then D_AGG_MODE = mode end
  if topn then D_AGG_TOPN = topn end
  print("[ABS-1FEX] D_AGG_MODE = " .. tostring(D_AGG_MODE) .. "  TOPN = " .. tostring(D_AGG_TOPN))
end

local function getGripDebug()
  return {D = {grip.Dwheel[1], grip.Dwheel[2], grip.Dwheel[3], grip.Dwheel[4]},
          conf = {grip.confident[1], grip.confident[2], grip.confident[3], grip.confident[4]},
          Fz0 = {grip.Fz0[1], grip.Fz0[2], grip.Fz0[3], grip.Fz0[4]},
          consensusD = dest.consensusD, enabled = grip.ENABLE_PERWHEEL_D}
end

local function getTelemetry()
  return {
    fusedSpeed = fusedSpeed,
    nextD = dest.consensusD,
    slipTargets = {slipTargets[1], slipTargets[2], slipTargets[3], slipTargets[4]},
    pidOut = {safety.lastAbsCoefs[1] or 1, safety.lastAbsCoefs[2] or 1, safety.lastAbsCoefs[3] or 1, safety.lastAbsCoefs[4] or 1},
    trimOffset = ph.rearOffset or 0,
    trimDirection = ph.rearDirection or 1,
    pidGains = {KP, KI, KD},
    imuSpeed = imuSpeed,
    imuClamps = safety.imuClamps,
    fwdVel = fwdVel,
    vLat = vLat,
    vVert = vVert,
    effectiveTarget = {lastEffectiveTargets[4], lastEffectiveTargets[3], lastEffectiveTargets[2], lastEffectiveTargets[1]},
    gripD = {grip.Dwheel[4], grip.Dwheel[3], grip.Dwheel[2], grip.Dwheel[1]},
    gripConfident = (function() local n=0 for j=1,4 do if grip.confident[j] then n=n+1 end end return n end)(),
    lowSpeedBoost = lastLowSpeedBoost,
    frontTrimOffset = ph.frontOffset or 0,
    frontTrimDirection = ph.frontDirection or 1,
    slipError = {lastSlipErrors[4], lastSlipErrors[3], lastSlipErrors[2], lastSlipErrors[1]},
    slipIntegralState = {slipIntegral[4], slipIntegral[3], slipIntegral[2], slipIntegral[1]},
    slipDerivative = {lastSlipDerivatives[4], lastSlipDerivatives[3], lastSlipDerivatives[2], lastSlipDerivatives[1]},
    snapUpCount = safety.snapUp
  }
end

M.init = init
M.update = update
M.reset = reset
M.clearBrakeEvents = clearBrakeEvents
M.getCounts = getCounts
M.resetCounts = resetCounts
M.setPerWheelD = setPerWheelD
M.setDAggMode = setDAggMode
M.getTelemetry = getTelemetry

M.getGripDebug = getGripDebug

return M
