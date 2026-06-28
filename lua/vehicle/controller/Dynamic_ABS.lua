local M = {}
M.type = "auxiliary"
M.version = "1.01"

-- PID ABS, no-cheat. Uses fusedSpeed only (no virtualAirspeed).
-- detectMu runs at 2kHz (sensor fusion -> fusedSpeed)
-- runTick runs at 200Hz (per-wheel PID + peak-decel D estimator)
--
-- (2): All vehicle-specific hardcodes removed. Init now builds wheel maps,
-- geometry, and static loads dynamically from wheels.wheelRotatorIDs,
-- wheels.wheelRotators[i].node1/node2, obj:getNodePositionRelative(),
-- obj:getMass(), and jbeamData. Falls back to safe universal defaults
-- when any lookup fails. No dynamic (per-tick) cheating anywhere.

local TICK_RATE_HZ = 100
local TICK_STEP = 1 / TICK_RATE_HZ
local timeAccum = 0

local origBrakeTorque = {}

-- wheelToBrakeMap[logicalIdx] = wheelRotator index (1-based)
-- Logical order: 1=RR, 2=RL, 3=FR, 4=FL  (our convention throughout)
-- Built dynamically in buildWheelMaps() via wheels.wheelRotatorIDs.
local wheelToBrakeMap = {1, 2, 3, 4}

-- Which logical indices are rear / front wheels (used by probe rearNow calc)
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
local latestRawSY = 0       -- raw accel (friction-only, no pitch correction)

-- Snap-up gap gate (TEST): reject a fused up-snap whose one-tick gap to the fastest wheel
-- exceeds this. 0.75 m/s @ 2kHz implies a "speed-up" of ~1500 m/s² — impossible for a braking
-- car, so it can only be a corrupted/spiking wheel reading. Real re-grips are <= ~0.33 m/s/tick,
-- so this never blocks a legitimate re-anchor. Equivalent to the (maxWs-fused)/dt accel bound.
local SNAP_GAP_MAX = 0.75

-- IMU speed ceiling (anti-wheelspin). imuSpeed integrates the accelerometer alone, so it is
-- immune to wheelspin; fused is capped at EXACTLY imuSpeed so pre-brake wheelspin (e.g. floor
-- it on water, then brake) can never inflate fused above what the car's own acceleration
-- actually supports. imuSpeed resyncs to the wheels only while they're trustworthy — a wheel
-- within IMU_TRUST_WINDOW above imuSpeed is "gripping" (resync); further above = "spinning".
local imuSpeed = 0
local IMU_TRUST_WINDOW = 2.0   -- m/s: how far a wheel may lead imuSpeed and still be trusted

-- 2D planar speed (drift handling). A sliding/yawing car corrupts a single-axis forward-speed
-- integral via the yaw-Coriolis term (lateral velocity * yaw rate). Tracking lateral velocity
-- (vLat) and feeding that term back keeps the forward estimate honest through a slide, so fused
-- can't inflate while the back end is out. Toggle OFF => bit-identical to plain 1F integration.
-- Sign of sensorX and yaw rate must be verified before trusting (see notes).
local ENABLE_2D_SPEED = true
                                -- survive a multi-second open-loop slide). Wheels are the only
                                -- reliable speed source; keep IMU as a short re-anchored ceiling.
local fwdVel = 0               -- estimated forward velocity, body frame (m/s) — internal 2D state
local vLat = 0                 -- estimated lateral (sideslip) velocity, body frame (m/s)
-- imuSpeed reports GROUND SPEED = sqrt(fwdVel^2 + vLat^2); pure yaw rotates speed between the two
-- axes and conserves the magnitude, so a sideways slide keeps the estimate at true speed (not 0).

-- Reverse support: bypass PID in reverse, handle arcade-mode input routing correctly.
local motionDirection = 1   -- +1 forward, -1 reverse (hysteresis)
local REVERSE_DETECT_THRESHOLD = 1.0   -- m/s — min wheel speed to trust direction signal
local REVERSE_LOCKIN_MPS       = 0.5   -- m/s — signed avg must exceed this to flip

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
local BRAKE_EVENT_FILE = "settings/blake_abs_brake_events.json"

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
  LOCK_DECEL = -5,       -- hard-lockup threshold (m/s²)
}

-- UI state: which speed source PID is using this tick
local absSpeedSource = { wheelAvgActive = false, fusedActive = true }

-- Safety + counters: bundled to stay under LuaJIT's 60-upvalue limit
local safety = {
  snapUp = 0, snapDn = 0, snapUpRej = 0,   -- snapUpRej = up-snaps rejected by the gap gate
  imuClamps = 0,                           -- times the IMU ceiling capped fused (anti-wheelspin)
  phantomVetoes = 0,                       -- times the decel cross-check vetoed a phantom-slip release
  stuckResyncs = 0,                        -- times the decel-sanity check re-anchored a stuck fused
  probes = 0, probesF = 0,    -- probesF = "Fast" probe (combined) fire count
  probesDrift = 0,            -- "Drift" probe (extended) fire count
  slipRatios = {},
  lastAbsCoefs = {},
  -- Safety toggle: BOTH probes (combined + extended). Back ON as the fused-too-high fallback while
  -- we work out a better fix. (Known tradeoff: they can false-fire on ice — see history.)
  ENABLE_PROBE = true,
  absEventID = 0,
  probeLogBuffer = {},
  -- Wheel decel lockup guard: if wheel decels faster than this, override PID and cut brake
  WHEEL_DECEL_LIMIT = -80,  -- m/s²
  -- Stuck-fused recovery (decel sanity). Re-anchor imuSpeed to the wheels DURING braking when the
  -- IMU proves the car isn't really moving that fast.
  stuck = { MARGIN=2.0, DECEL=0.5, SUSTAIN=0.15, timer=0 },
  engineFightTimer = {0, 0, 0, 0},
  -- Low-speed brake boost: naturally deepens slip below 30mph to aid final stopping without PID windup
  lowSpeed = { BOOST_MAX = 1.25, SPEED_THRESH = 13.41 },
}

-- Per-wheel surface-grip detector (ANCHORED brake-acceptance).
-- The global consensusD (IMU peak-decel) sets the absolute LEVEL; per-wheel
-- brake-acceptance carves the SPLIT. grip proxy g_i = appliedTorque_i / Fz_i (~ mu_i),
-- where appliedTorque_i = absCoef_i * origBrakeTorque_i (PID output + known constant) and
-- Fz_i = static corner load + longitudinal transfer from the global decel we already have.
-- Honest signals only — no per-wheel downForce. Bundled in one table to stay under
-- LuaJIT's 60-upvalue limit. wheel order 1=RR,2=RL,3=FR,4=FL.
--
-- (2): FRONT_FRAC, H_CG, WHEELBASE, and yawOffset are now built in init()
-- from real vehicle geometry (wheel axle node positions + obj:getMass()).
-- None of these values are updated at runtime — static config only.
local grip = {
  ENABLE_PERWHEEL_D = false,  -- master switch; false => original global broadcast (escape hatch)
  FRONT_FRAC = 0.5,           -- static front weight fraction — computed in init(), fallback 0.5
  H_CG = 0.55,                -- CG height (m) — read from jbeamData or universal fallback
  WHEELBASE = 2.6,            -- wheelbase (m) — computed from axle node positions in init()
  GRAV = 9.81,
  FZ_MIN = 200, SAT = 0.97, ROLL_MIN = 1.0, TORQUE_MIN_FRAC = 0.05,
  G_SMOOTH = 0.9, DECAY = 0.05, EPS = 1e-3,
  mass = 1500,
  Fz0 = {},                    -- per-wheel static load (set in init)
  gEMA = {},
  Dwheel = {},
  confident = {},
  -- Turn handling: yaw-compensated per-wheel reference speed.
  -- v_ref_i = fusedSpeed - yawRate * yawOffset_i. Outside wheels get a higher reference so
  -- their geometrically-faster rotation isn't mistaken for slip. Soft deadband ignores noise.
    yawOffset = {},              -- ±half-track (m): computed in init() from axle node X positions
  yawRate = 0,                 -- last read yaw rate (rad/s), published for sign-check
}

-- Per-wheel PID state
local slipIntegral = {}
local lastSlipError = {}
local prevTickWheelSpeed = {}



-- Rear-only brake-rotation assist (ESC-lite, EXPERIMENTAL). Under braking + steering, run the
-- INSIDE-rear wheel a touch deeper in slip to rotate the car into the corner. Yaw-rate limited:
-- the boost fades to 0 as actual yaw approaches what the steering commands, so it can't over-rotate.
-- Target-side (the slip PID cancels a raw brake boost). A wrong steering sign brakes the OUTSIDE
-- rear -> stabilizes (fail-safe, just no turn-in help), so a sign error can't spin you.
-- Bundled in one table to stay under LuaJIT's 60-upvalue-per-function limit.
local rot = {
  ENABLE = false,
  ASSIST_MAX = 0.05,      -- max added slip on the inside rear (0.14 -> 0.19 at full) ~ "5%"
  DEADBAND = 0.30,        -- steering below this = OFF (excludes 90mph lane changes; on for hairpins)
  MIN_SPEED = 5.0,        -- m/s: no assist below this
  YAW_CMD_GAIN = 0.18,    -- maps steering*speed -> commanded yaw rate (bicycle-model approx)
  YAW_LIMIT_BAND = 0.15,  -- rad/s: fade window; assist -> 0 as actual yaw reaches commanded
}

-- Per-wheel adaptive slip targets (D-estimator writes, PID reads)
local slipTargets = {}
local SLIP_TARGET_MIN = 0.02
local SLIP_TARGET_MAX = 1.0
local TARGET_SMOOTHING = 0.95

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
}

-- Hybrid Peak-Hunter & Brake Simulator State
local ph = {
  trimOffset = 0,
  trimDirection = 1,
  trimStep = 0.01,
  trimMax = 1.0,
  trimMin = 0.0, -- TEST: Prevents the seeker from dropping the ABS target below the baseline
  trimTimer = 0,
  TRIM_INTERVAL = 0.05,
  lastEfficiency = 0,
  effSum = 0,
  effCount = 0,
  simulatedTorque = {},
  brakeInRate = {},
  brakeOutRate = {},
  -- Gas+Brake seeker disable: when throttle AND brake are simultaneously pressed,
  -- the Peak-Hunter seeker is paused. The instant gas is released it resumes --
  -- no need to re-press the brake. Toggleable.
  ENABLE_THROTTLE_LOCKOUT = true,
  seekerSuppressed = false,
}

-- EBD (Electronic Brakeforce Distribution) safety:
-- Detects when front brakes are failing (front doing <25% of total braking force)
-- and smoothly reduces rear brake authority to prevent spinout.
local ebd = {
  ENABLE        = false,
  TRIGGER_FRAC  = 0.25,   -- front must fall below 25% of total brake force to activate
  SUSTAIN       = 0.40,   -- seconds of sustained low front share before activating
  MIN_SPEED     = 8.0,    -- m/s (~18mph): don't activate at standstill
  SMOOTH        = 0.97,   -- EMA smoothing for frontShareEMA (very heavy, avoids transients)
  REAR_MIN      = 0.40,   -- rear brakes never reduced below 40% even in worst case
  frontShareEMA = 1.0,    -- start assuming healthy (front doing 100%)
  sustainTimer  = 0,
  active        = false,
  rearScale     = 1.0,
}

-- Front/rear bias — zeroed, no measurable effect in testing
local AXLE_BIAS = {}

-- PID tunables
local KP = 6.0
local KI = 0.8
local KD = 0.08
local INTEGRAL_MIN = -1.0
local INTEGRAL_MAX = 1.0
local MIN_SPEED = 2.236
local MIN_ADAPT_SPEED = 2.0

-- Misc
local NON_BRAKING_DECEL_FILTER = -5.0
local STANDSTILL_WS_THRESHOLD = 0.3
local STANDSTILL_FUSED_THRESHOLD = 2.0
local STANDSTILL_PHYS_TICKS = 100
local standstillCounter = 0

-- Fused-speed safety probe: single combined config.
-- Fires when all wheels agree (within wheelAgree) AND fused floats > fusedRatio above the
-- wheel avg, sustained, AND the total applied brake (from the PID's absCoef output) is below
-- lowBrakeThreshold — then snaps fused down to the captured rear-wheel speed.
safety.configs = {
  combined = {
    wheelAgree = 1.788,    -- m/s (4 mph) — wheels must agree within this
    fusedRatio = 1.50,     -- RAISED from 1.30: normal 100mph stops peak at 1.31x; fire only at 1.50x+ (genuine runaway)
    sustain = 0.04,        -- seconds the divergence must hold before firing
    duration = 0.04,       -- seconds active, capturing max rear-wheel speed
    rearmCooldown = 1.0,   -- seconds latched before it can fire again
    requireLowBrake = true,
    lowBrakeThreshold = 0.60,  -- only fire when total applied brake (PID absCoef-weighted) <= 60%
  },
  -- Secondary "extended" probe: catches fused sitting above the wheel avg for a LONG time.
  extended = {
    wheelAgree = 2.682,    -- m/s (6 mph)
    fusedRatio = 1.28,     -- RAISED from 1.10: normal stops sustain 1.10-1.15x throughout; fire only at 1.28x+
    sustain = 1.5,         -- seconds — long hold = "extended period"
    duration = 0.10,       -- seconds active, capturing max rear-wheel speed
    rearmCooldown = 1.0,   -- seconds latched before it can fire again
    requireLowBrake = true,
    lowBrakeThreshold = 0.80,  -- fire when applied brake (PID absCoef-weighted) <= 80%
  },
}

local function newProbeState()
  return { divergeTimer = 0, active = false, timer = 0, maxRear = 0, latched = false, latchCooldown = 0 }
end

safety.probe = newProbeState()
safety.probe2 = newProbeState()

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
end


local function resetProbe()
  safety.probe = newProbeState()
  safety.probe2 = newProbeState()
  safety.lastAbsCoefs = {}
  for i = 1, N_WHEELS do safety.lastAbsCoefs[i] = 1 end
end


-- (2) buildWheelMaps: 1:1 copy of the stock BeamNG method from
-- drivingDynamics/sensors/vehicleData.lua → initSecondStage().
-- Step 1: Filter to known corner wheel names {"FR","FL","RR","RL"}.
-- Step 2: Compute average wheel position from v.data.nodes[wheel.node1].pos.
-- Step 3: Build a local coordinate frame from the vehicle's reference nodes
--         (ref, back, up) via forward/up/right vectors.
-- Step 4: Classify each corner wheel as front/rear + left/right using
--         dot products against the forward and right vectors.
-- Maps logical index (1=RR, 2=RL, 3=FR, 4=FL) → wheelRotator slot (1-based).
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

        -- Map wheel name → wheelRotator index → 1-based slot for our tables
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


-- (2) buildGeometry: computes WHEELBASE, yawOffset, and FRONT_FRAC from wheel axle
-- node positions. Uses wheels.wheelRotators[i].node1/.node2 (confirmed in official
-- BeamNG dev code) and obj:getNodePositionRelative() for body-frame positions.
-- All values are computed ONCE at init — zero runtime overhead.
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

  local canReadPos = (obj.getNodePositionRelative ~= nil) or true  -- optimistic; pcall guards below

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

  -- yawOffset per logical wheel: X position of axle center in body frame.
  -- BeamNG body frame: X positive = left, X negative = right (matches suspension JBeam nodes).
  -- Sign convention matches existing code: negative = right side, positive = left side.
  local halfTrackSum, halfTrackCount = 0, 0
  for li = 1, N_WHEELS do
    if positions[li] then
      grip.yawOffset[li] = positions[li].x
      halfTrackSum = halfTrackSum + math.abs(positions[li].x)
      halfTrackCount = halfTrackCount + 1
    else
      -- Fallback: use a reasonable default half-track; will be overwritten for any wheel we CAN read
      grip.yawOffset[li] = (li == 1 or li == 3) and -0.75 or 0.75
    end
  end

  -- FRONT_FRAC: fraction of static weight on front axle.
  -- Derived from front/rear axle Y positions relative to vehicle origin (body CG ≈ origin).
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
  print(string.format("[ABS-1FEX] (2) yawOffset: RR=%.3f RL=%.3f FR=%.3f FL=%.3f",
    grip.yawOffset[1] or 0, grip.yawOffset[2] or 0,
    grip.yawOffset[3] or 0, grip.yawOffset[4] or 0))
end


local function init(jbeamData)
  print("[ABS-1FEX] (2) canonical build loaded — per-wheel D + 2 probes (combined + extended) ON")

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
  AXLE_BIAS             = {}
  safety.slipRatios     = {}
  safety.lastAbsCoefs   = {}
  grip.Fz0              = {}
  grip.gEMA             = {}
  grip.Dwheel           = {}
  grip.confident        = {}
  grip.yawOffset        = {}
  wa.prevWs             = {}
  wa.locked             = {}
  wa.decelBuf           = {}
  wa.decelIdx           = {}

  for i = 1, N_WHEELS do
    slipIntegral[i]       = 0
    lastSlipError[i]      = 0
    prevTickWheelSpeed[i] = 0
    latestWheelSpeed[i]   = 0
    fusedPrevWs[i]        = 0
    slipTargets[i]        = 0.14
    ph.simulatedTorque[i] = 0
    ph.brakeInRate[i]     = 0
    ph.brakeOutRate[i]    = 0
    AXLE_BIAS[i]          = 0
    safety.slipRatios[i]  = 0
    safety.lastAbsCoefs[i]= 1
    safety.engineFightTimer[i] = 0
    grip.Fz0[i]           = 0
    grip.gEMA[i]          = 1
    grip.Dwheel[i]        = 1
    grip.confident[i]     = false
    grip.yawOffset[i]     = 0
    wa.prevWs[i]          = 0
    wa.locked[i]          = false
    wa.decelBuf[i]        = {}
    wa.decelIdx[i]        = 0
    wheelToBrakeMap[i]    = i   -- safe default until buildWheelMaps overrides
  end

  wasBraking = false
  timeAccum = 0
  uiAccum = 0
  standstillCounter = 0
  resetProbe()
  safety.snapUp = 0
  safety.snapDn = 0
  safety.snapUpRej = 0
  safety.imuClamps = 0
  safety.phantomVetoes = 0
  safety.stuckResyncs = 0
  safety.stuck.timer = 0
  safety.probes = 0
  safety.probesF = 0
  safety.probesDrift = 0
  safety.absEventID = 0
  safety.probeLogBuffer = {}
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

  ph.trimOffset = 0
  ph.trimDirection = 1
  ph.trimTimer = 0
  ph.lastEfficiency = 0
  ph.effSum = 0
  ph.effCount = 0

  ph.seekerSuppressed = false
  ebd.frontShareEMA = 1.0
  ebd.sustainTimer  = 0
  ebd.active        = false
  ebd.rearScale     = 1.0

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
  latestRawSY = 0

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
    local inD = slot and rawInDelay[slot] or rawInDelay[i]
    local outD = slot and rawOutDelay[slot] or rawOutDelay[i]
    ph.brakeInRate[i] = maxT / (inD + 1e-30)
    ph.brakeOutRate[i] = maxT / (outD + 1e-30)
  end

  -- (2) Vehicle mass — static property, read once at init. obj:getMass() is confirmed API.
  pcall(function() grip.mass = obj:getMass() or grip.mass end)

  -- Disable per-wheel D on vehicles with fewer than 4 wheels
  grip.ENABLE_PERWHEEL_D = (N_WHEELS >= 4) and grip.ENABLE_PERWHEEL_D or false

  -- (2) Compute geometry from wheel axle node positions (wheelbase, yawOffset, FRONT_FRAC)
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

  extensions.load('abstelemetry')
end


-- detectMu(dtPhys) — 2kHz sensor fusion only
local function detectMu(dtPhys)
  local rawSY = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorY) or 0
  latestRawSY = rawSY                              -- friction-only (for D-estimator)
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

  local maxWs, minWs = 0, math.huge
  for i = 1, N_WHEELS do
    local ws = latestWheelSpeed[i]
    if ws > maxWs then maxWs = ws end
    if ws < minWs then minWs = ws end
  end

  if isBraking then
    -- Snap-up gap gate (TEST): only re-anchor fused up to the fastest wheel if the one-tick
    -- gap is physically plausible (<= SNAP_GAP_MAX). A larger gap implies an impossible
    -- speed-up for a braking car => spiking/corrupt wheel reading => reject it.
    if maxWs > fusedSpeed then
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

    -- Full 3D strapdown integration (Coriolis/Centripetal cross-coupling)
    local dotFwd  = ax + vLat * yr + (vVert or 0) * pitchRate
    local dotLat  = -ay - fwdVel * yr + (vVert or 0) * rollRate
    local dotVert = -az - fwdVel * pitchRate - vLat * rollRate

    local newFwd = fwdVel + dotFwd * dtPhys
    vLat         = vLat   + dotLat * dtPhys
    vVert        = (vVert or 0) + dotVert * dtPhys
    fwdVel = newFwd                                          -- forward comp may pass through/below 0

    if math.abs(yr) < 0.05 then vLat = vLat * 0.99 end       -- ~straight: bleed sideslip est. (noise)
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
  -- During braking the wheels under-read, so we never resync then — the integral is the truth.
  if (not isBraking) and (maxWs - minWs) < 1.0 and maxWs <= imuSpeed + IMU_TRUST_WINDOW then
    fwdVel = maxWs
    vLat = 0                              -- wheels agree => assume no sideslip, reset lateral est.
    vVert = 0                             -- reset vertical est.
    imuSpeed = maxWs
  end

  if fusedSpeed > imuSpeed then          -- exact ceiling: fused may not exceed the IMU ground speed
    fusedSpeed = imuSpeed
    safety.imuClamps = safety.imuClamps + 1
  end

  for i = 1, N_WHEELS do fusedPrevWs[i] = latestWheelSpeed[i] end

  if maxWs < STANDSTILL_WS_THRESHOLD and fusedSpeed < STANDSTILL_FUSED_THRESHOLD then
    standstillCounter = standstillCounter + 1
    if standstillCounter >= STANDSTILL_PHYS_TICKS then
      fusedSpeed = 0
      imuSpeed = 0
      fwdVel = 0
      vLat = 0
      vVert = 0
      safety.stuck.timer = 0
    end
  else
    standstillCounter = 0
  end

end


-- runProbe: config-driven probe, operates on its own state table
-- returns corrected speed or nil
local function runProbe(dt, isBraking, cfg, st, probeName)
  if not isBraking then
    st.divergeTimer = 0
    st.active = false
    st.timer = 0
    st.maxRear = 0
    st.latched = false
    st.latchCooldown = 0
    return nil
  end

  if st.latched then
    st.latchCooldown = st.latchCooldown - dt
    if st.latchCooldown <= 0 then
      st.latched = false
    else
      return nil
    end
  end

  if st.active then
    -- (2) rearNow uses rearLogicalIndices built from wheelRotatorIDs — no hardcoded 1,2
    local rearSum, rearCount = 0, 0
    for _, li in ipairs(rearLogicalIndices) do
      rearSum = rearSum + (latestWheelSpeed[li] or 0)
      rearCount = rearCount + 1
    end
    local rearNow = rearCount > 0 and (rearSum / rearCount) or 0
    if rearNow > st.maxRear then st.maxRear = rearNow end

    st.timer = st.timer - dt
    if st.timer <= 0 then
      st.active = false
      st.latched = true
      st.latchCooldown = cfg.rearmCooldown
      return st.maxRear
    end
    return nil
  end

  local wsMin, wsMax, wsSum = math.huge, -math.huge, 0
  for i = 1, N_WHEELS do
    if latestWheelSpeed[i] < wsMin then wsMin = latestWheelSpeed[i] end
    if latestWheelSpeed[i] > wsMax then wsMax = latestWheelSpeed[i] end
    wsSum = wsSum + latestWheelSpeed[i]
  end

  local wheelsAgree = (wsMax - wsMin) < cfg.wheelAgree
  local wheelAvg = wsSum / N_WHEELS
  local fusedDiverged = fusedSpeed > (wheelAvg * cfg.fusedRatio)

  local brakeOk = true
  if cfg.requireLowBrake then
    local totalApplied, totalMax = 0, 0
    for i = 1, N_WHEELS do
      totalApplied = totalApplied + safety.lastAbsCoefs[i] * origBrakeTorque[i]
      totalMax = totalMax + origBrakeTorque[i]
    end
    brakeOk = totalMax > 0 and (totalApplied / totalMax) <= cfg.lowBrakeThreshold
  end

  if wheelsAgree and fusedDiverged and brakeOk then
    st.divergeTimer = st.divergeTimer + dt
    table.insert(safety.probeLogBuffer, string.format("%s,%.3f,%d,%.3f,%.3f,%.3f,%.3f,%s", probeName, os.clock(), safety.absEventID, fusedSpeed, wheelAvg, wsMax, wsMin, tostring(st.divergeTimer >= cfg.sustain)))
    if st.divergeTimer >= cfg.sustain then
      st.active = true
      st.timer = cfg.duration
      -- (2) initial maxRear from rearLogicalIndices
      local rearSum, rearCount = 0, 0
      for _, li in ipairs(rearLogicalIndices) do
        rearSum = rearSum + (latestWheelSpeed[li] or 0)
        rearCount = rearCount + 1
      end
      st.maxRear = rearCount > 0 and (rearSum / rearCount) or 0
      st.divergeTimer = 0
    end
  else
    st.divergeTimer = 0
  end

  return nil
end


-- runTick(dt) — 200Hz: wheel-avg + PID + D-estimator
local function runTick(dt)
  local brakeInput = input.brake or 0
  local brakingJustStarted = brakeInput > 0 and not wasBraking
  wasBraking = brakeInput > 0
  local isBraking = brakeInput > 0

  local abstelem = extensions.abstelemetry
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
      -- Track peak ABSOLUTE divergence (m/s) — only above 5 mph so low-speed sensor noise
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
      if #safety.probeLogBuffer > 0 then
        local f = io.open("probe_log.csv", "a")
        if f then
          for _, line in ipairs(safety.probeLogBuffer) do
            f:write(line .. "\n")
          end
          f:close()
        end
        safety.probeLogBuffer = {}
      end
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

  -- Safety probes (both run every tick; combined wins if both fire same tick).
  -- Probe 1 = combined (fast). Probe 2 = extended (catches long-held fused-above-avg).
  -- Both use the PID's per-wheel absCoef output (safety.lastAbsCoefs) for the low-brake gate.
  if safety.ENABLE_PROBE then
    local c1 = runProbe(TICK_STEP, isBraking, safety.configs.combined, safety.probe, "FAST_PROBE")
    local c2 = runProbe(TICK_STEP, isBraking, safety.configs.extended, safety.probe2, "DRIFT_PROBE")
    if c1 then safety.probesF = safety.probesF + 1 end          -- "Fast" probe (combined) fired
    if c2 then safety.probesDrift = safety.probesDrift + 1 end  -- "Drift" probe (extended) fired
    local probeCorrection = c1 or c2
    if probeCorrection then
      fusedSpeed = probeCorrection
      safety.snapDn = safety.snapDn + 1
      safety.probes = safety.probes + 1
    end
  end

  -- carSpeed = fusedSpeed only (this is the 1F variant — no virtualAirspeed)
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
    safety.absEventID = safety.absEventID + 1
    safety.probeLogBuffer = {}
    for i = 1, N_WHEELS do
      slipIntegral[i] = 0
      lastSlipError[i] = 0
      safety.engineFightTimer[i] = 0
    end
    safety.snapUp = 0
    safety.snapDn = 0
    safety.snapUpRej = 0
    safety.probes = 0
    safety.probesF = 0
    safety.probesDrift = 0
  end

  -- Rear-only brake-rotation assist (experimental): pick the inside-rear wheel from steering, scale
  -- by steering magnitude past a decent deadband (90mph lane changes use small steer -> stay off;
  -- hairpins/chicanes use big steer -> on), then fade via the yaw limiter so the car can't rotate
  -- past what the steering commands.
  local rotInsideRear, rotAssist = 0, 0
  if rot.ENABLE and isBraking and carSpeed > rot.MIN_SPEED then
    local steer = electrics.values.steering_input or 0
    local steerMag = math.abs(steer)
    if steerMag > rot.DEADBAND then
      -- (2) pick inside rear from rearLogicalIndices: steer>0 = LEFT turn -> inside = RL (logical 2)
      -- rearLogicalIndices[1]=RR (right rear), rearLogicalIndices[2]=RL (left rear) — convention held
      rotInsideRear = (steer > 0) and rearLogicalIndices[2] or rearLogicalIndices[1]
      local steerEff = math.min((steerMag - rot.DEADBAND) / (1 - rot.DEADBAND), 1)
      local commandedYaw = steer * carSpeed * rot.YAW_CMD_GAIN
      local yawScale = 1
      if math.abs(commandedYaw) > 0.01 then
        yawScale = math.max(0, math.min(1, (math.abs(commandedYaw) - math.abs(yawRate)) / rot.YAW_LIMIT_BAND))
      end
      rotAssist = rot.ASSIST_MAX * steerEff * yawScale
    end
  end
  electrics.values.abs_rotAssist = rotAssist
  electrics.values.abs_rotWheel = rotInsideRear

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

  for i = 1, N_WHEELS do
    local slot = wheelToBrakeMap[i]

    if carSpeed > MIN_SPEED and isBraking then
      -- per-wheel turn-compensated reference speed (vehicle speed + yaw geometry)
      -- Yaw compensation disabled as requested:
      
      local vRef = math.max(carSpeed, 0.5)
      local slip = math.max(0, math.min((vRef - latestWheelSpeed[i]) / vRef, 1))
      local effectiveTarget = math.min(slipTargets[i] + (AXLE_BIAS[i] or 0) + (1 + dest.consensusD) / carSpeed, 1.0)
      if i == rotInsideRear then effectiveTarget = math.min(effectiveTarget + rotAssist, 1.0) end
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
        local ratio = 1.0 - (carSpeed / safety.lowSpeed.SPEED_THRESH)
        local boost = 1.0 + (safety.lowSpeed.BOOST_MAX - 1.0) * ratio * ratio
        absCoef = math.min(1.0, absCoef * boost)
      end

      slipRatios[i] = slip
      slipErrors[i] = slipError
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

  -- EBD: compute front brake share from this tick's absCoefs + origBrakeTorque.
  -- Use a very heavy EMA so single-frame ABS pumps don't falsely trigger.
  if isBraking and carSpeed > ebd.MIN_SPEED and ebd.ENABLE then
    local frontT, rearT = 0, 0
    for i = 1, N_WHEELS do
      local t = absCoefs[i] * (origBrakeTorque[i] or 0)
      local isFront = false
      for _, fi in ipairs(frontLogicalIndices) do if i == fi then isFront = true; break end end
      if isFront then frontT = frontT + t else rearT = rearT + t end
    end
    local totalT = frontT + rearT
    local rawShare = totalT > 1 and (frontT / totalT) or 1.0
    ebd.frontShareEMA = ebd.frontShareEMA * ebd.SMOOTH + rawShare * (1.0 - ebd.SMOOTH)

    if ebd.frontShareEMA < ebd.TRIGGER_FRAC then
      ebd.sustainTimer = ebd.sustainTimer + dt
      if ebd.sustainTimer >= ebd.SUSTAIN then
        ebd.active = true
        -- Logarithmic scale: deeper the front share drops, harder the rear is cut.
        -- At frontShareEMA = TRIGGER_FRAC (0.25) -> rearScale = 1.0 (just activated)
        -- At frontShareEMA = 0.10 -> rearScale ~0.56
        -- At frontShareEMA = 0.0  -> rearScale = REAR_MIN (0.40)
        local ratio = math.max(0, ebd.frontShareEMA / ebd.TRIGGER_FRAC)
        ebd.rearScale = math.max(ebd.REAR_MIN, ratio * ratio)  -- quadratic ramp
      end
    else
      ebd.sustainTimer = 0
      -- Self-healing: ramp rearScale back to 1.0 when fronts recover
      ebd.rearScale = math.min(1.0, ebd.rearScale + dt * 0.5)
      if ebd.rearScale >= 1.0 then ebd.active = false end
    end

    -- Apply rearScale to rear wheel commands
    if ebd.active then
      for i = 1, N_WHEELS do
        local isFront = false
        for _, fi in ipairs(frontLogicalIndices) do if i == fi then isFront = true; break end end
        if not isFront then
          local slot = wheelToBrakeMap[i]
          cmd[slot] = cmd[slot] * ebd.rearScale
          absCoefs[i] = absCoefs[i] * ebd.rearScale
        end
      end
    end
  else
    if not isBraking then
      ebd.frontShareEMA = 1.0
      ebd.sustainTimer  = 0
      ebd.active        = false
      ebd.rearScale     = 1.0
    end
  end
  electrics.values.abs_ebdActive = ebd.active and ebd.rearScale or 1.0

  -- prevTickWheelSpeed for next tick's lockup guard
  for i = 1, N_WHEELS do
    prevTickWheelSpeed[i] = latestWheelSpeed[i]
  end

  -- Gas+Brake seeker lockout: if throttle and brake are both pressed, pause the
  -- Peak-Hunter. The instant gas is released the seeker resumes mid-brake event.
  if ph.ENABLE_THROTTLE_LOCKOUT then
    local throttleIn = input.throttle or 0
    if throttleIn > 0.05 and isBraking then
      ph.seekerSuppressed = true
    else
      ph.seekerSuppressed = false  -- gas released: resume immediately, even if still braking
    end
  else
    ph.seekerSuppressed = false
  end

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

  if isBraking and carSpeed > MIN_ADAPT_SPEED and decelCount > 0 and not ph.seekerSuppressed then
    avgWheelDecel = avgWheelDecel / decelCount
    local instantEff = totalTorque / (avgWheelDecel + 1e-5)
    
    ph.effSum = ph.effSum + instantEff
    ph.effCount = ph.effCount + 1
    
    ph.trimTimer = ph.trimTimer + dt
    if ph.trimTimer >= ph.TRIM_INTERVAL then
      ph.trimTimer = 0
      
      local blockAverageEff = ph.effSum / ph.effCount
      if blockAverageEff < ph.lastEfficiency then
        ph.trimDirection = -ph.trimDirection
      end
      ph.lastEfficiency = blockAverageEff
      ph.trimOffset = math.max(ph.trimMin, math.min(ph.trimMax, ph.trimOffset + (ph.trimStep * ph.trimDirection)))
      
      ph.effSum = 0
      ph.effCount = 0
    end
  elseif not isBraking or ph.seekerSuppressed then
    if ph.seekerSuppressed then
      -- Flush stale accumulator so old data doesn't contaminate the next window
      ph.effSum  = 0
      ph.effCount = 0
      -- Do NOT reset trimOffset — seeker resumes from last good position
    else
      ph.trimOffset = 0
      ph.trimTimer = 0
      ph.lastEfficiency = 0
      ph.effSum = 0
      ph.effCount = 0
    end
  end

  -- D estimator: peak sensorY decel over a sliding window, D = peak / g.
  -- No Pacejka shape assumption — avoids the overestimation the particle filter had on ice.
  if isBraking and carSpeed > MIN_ADAPT_SPEED then
    local measuredDecel = latestRawSY  -- friction-only, slope-independent

    if measuredDecel > dest.UPDATE_MIN_DECEL then
      dest.windowIdx = (dest.windowIdx % dest.WINDOW_SIZE) + 1
      dest.window[dest.windowIdx] = measuredDecel

      local peakDecel = 0
      for j = 1, dest.WINDOW_SIZE do
        if dest.window[j] and dest.window[j] > peakDecel then
          peakDecel = dest.window[j]
        end
      end

      -- 0.91 = weight-transfer grip boost + 200Hz peak-sampling bias
      local instantD = math.max(dest.EST_MIN, math.min(dest.EST_MAX, peakDecel / 9.81 * 0.91))

      -- Surface-change detection: D jump >40% = reset window
      dest.stableTicks = dest.stableTicks + 1
      if dest.stableTicks > dest.SETTLE_TICKS then
        local dChange = math.abs(instantD - dest.stableD) / math.max(dest.stableD, 0.1)
        if dChange > dest.CHANGE_THRESHOLD then
          dest.window = {}
          dest.windowIdx = 1
          dest.window[1] = measuredDecel
          dest.retroResets = dest.retroResets + 1
          dest.stableTicks = 0
          dest.consensusD = instantD
          ph.trimOffset = 0  -- Reset Peak-Hunter on massive surface change
        end
        dest.stableD = dest.consensusD
      end

      -- Smooth consensus toward peak-decel estimate
      dest.consensusD = dest.consensusD * dest.SMOOTHING + instantD * (1.0 - dest.SMOOTHING)

      -- Map D to slip target
      dest.baseTarget = math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX,
        0.04 + dest.consensusD * 0.10))

      if not grip.ENABLE_PERWHEEL_D then
        -- Escape hatch: original global broadcast — every wheel takes the global target.
        local finalTarget = math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX, dest.baseTarget + ph.trimOffset))
        for j = 1, N_WHEELS do
          slipTargets[j] = slipTargets[j] * TARGET_SMOOTHING + finalTarget * (1.0 - TARGET_SMOOTHING)
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
            local baseTarget = math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX, 0.04 + Dj * 0.10))
            slipTargets[j] = slipTargets[j] * TARGET_SMOOTHING + baseTarget * (1.0 - TARGET_SMOOTHING)
          end
        else
          -- no confident wheel (light braking / all locked / first ticks): fall back to global
          local finalTarget = math.max(SLIP_TARGET_MIN, math.min(SLIP_TARGET_MAX, dest.baseTarget + ph.trimOffset))
          for j = 1, N_WHEELS do
            grip.Dwheel[j] = dest.consensusD
            slipTargets[j] = slipTargets[j] * TARGET_SMOOTHING + finalTarget * (1.0 - TARGET_SMOOTHING)
          end
        end
      end
    end
  end

  -- (Rear 20% ease-off during active probe removed per request — probing now captures the rear
  -- wheel speed at full PID brake command, with no brake reduction.)

  -- Push brake commands
  if haveTelem then
    abstelem.setBrakes(cmd)
  end
end


-- update(dtPhys) — 2kHz orchestrator
local function update(dtPhys)
  detectMu(dtPhys)
  brakeSimTime = brakeSimTime + dtPhys

  timeAccum = timeAccum + dtPhys
  while timeAccum >= TICK_STEP do
    runTick(TICK_STEP)
    timeAccum = timeAccum - TICK_STEP
  end

  -- Data Logging (50Hz) to track down IMU drift
  local brakeInput = electrics.values.brake or 0
  if brakeInput > 0.01 then
    if not isLogging then
      isLogging = true
      logData = {}
      table.insert(logData, "Time,Airspeed,FusedSpeed,ImuSpeed,FwdVel,VLat,VVert,Ax,Ay,Az,YawRate,PitchRate,RollRate,Pitch,Roll")
      logTimer = 0
    end
    
    logTimer = logTimer + dtPhys
    if logTimer >= 0.02 then
      logTimer = 0
      local row = string.format("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
        brakeSimTime, electrics.values.airspeed or 0, fusedSpeed, imuSpeed, fwdVel, vLat, vVert,
        -latestSensorY, (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorX) or 0,
        (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorZ) or 0,
        (obj:getYawAngularVelocity() or 0), pitchRateLog, rollRateLog, pitchLog, rollLog
      )
      table.insert(logData, row)
    end
  else
    if isLogging then
      isLogging = false
      if #logData > 1 then
        local file = io.open("abs_imu_log.csv", "w")
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
  uiAccum = uiAccum + dtPhys
  if uiAccum >= 0.2 then
    uiAccum = 0
    if guihooks then
      guihooks.trigger('updateABSGrip', {
        RR = { surfaceMu = string.format("%.2f", grip.Dwheel[1]), slipMu = string.format("%.2f", safety.slipRatios[1]) },
        RL = { surfaceMu = string.format("%.2f", grip.Dwheel[2]), slipMu = string.format("%.2f", safety.slipRatios[2]) },
        FR = { surfaceMu = string.format("%.2f", grip.Dwheel[3]), slipMu = string.format("%.2f", safety.slipRatios[3]) },
        FL = { surfaceMu = string.format("%.2f", grip.Dwheel[4]), slipMu = string.format("%.2f", safety.slipRatios[4]) },
        speeds = {
          airspeed = string.format("%.1f", electrics.values.airspeed or 0),
          fusedSpeed = string.format("%.1f", fusedSpeed or 0),
          plausibleSpeed = string.format("%.1f", wa.speed or 0),
          virtualAirspeed = string.format("%.1f", electrics.values.virtualAirspeed or 0),
          snapUpCount = safety.snapUp,
          snapDnCount = safety.snapDn,
          snapUpRejCount = safety.snapUpRej,
          imuSpeed = string.format("%.1f", imuSpeed or 0),
          imuClampCount = safety.imuClamps,
          phantomVetoCount = safety.phantomVetoes,
          stuckResyncCount = safety.stuckResyncs,
          vLat = string.format("%.1f", vLat or 0),
          yaw2d = string.format("%.2f", grip.yawRate or 0),
          probeCount = safety.probes,
          probeFastCount = safety.probesF,
          probeDriftCount = safety.probesDrift,
          fusedActive = absSpeedSource.fusedActive,
          engineFight = (safety.engineFightTimer[1] > 0 or safety.engineFightTimer[2] > 0 or safety.engineFightTimer[3] > 0 or safety.engineFightTimer[4] > 0),
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


local function setProbes(val) safety.ENABLE_PROBE = val end
local function setProbeSustain(val) safety.configs.combined.sustain = val end
local function getCounts()
  return {probes = safety.probes, probesF = safety.probesF,
          probesDrift = safety.probesDrift,
          snapUp = safety.snapUp, snapDn = safety.snapDn,
          }
end
local function resetCounts()
  safety.probes = 0; safety.probesF = 0; safety.probesDrift = 0
  safety.snapUp = 0; safety.snapDn = 0; safety.snapUpRej = 0; safety.imuClamps = 0
  safety.phantomVetoes = 0; safety.stuckResyncs = 0
end
local function setPerWheelD(val) grip.ENABLE_PERWHEEL_D = val end

local function getGripDebug()
  return {D = {grip.Dwheel[1], grip.Dwheel[2], grip.Dwheel[3], grip.Dwheel[4]},
          conf = {grip.confident[1], grip.confident[2], grip.confident[3], grip.confident[4]},
          Fz0 = {grip.Fz0[1], grip.Fz0[2], grip.Fz0[3], grip.Fz0[4]},
          consensusD = dest.consensusD, enabled = grip.ENABLE_PERWHEEL_D}
end

M.init = init
M.update = update
M.reset = reset
M.clearBrakeEvents = clearBrakeEvents
M.setProbes = setProbes
M.setProbeSustain = setProbeSustain
M.getCounts = getCounts
M.resetCounts = resetCounts
M.setPerWheelD = setPerWheelD

M.getGripDebug = getGripDebug

return M
