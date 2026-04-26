local M = {}
M.type = "auxiliary"

-- Blake ABS-2F — per-wheel PID anti-lock braking controller.
-- Toggle any feature below to false to disable that piece.

-- ===== Toggles =====
local USE_REVERSE_SUPPORT  = true  -- required for reverse brake + arcade-mode handling
local REVERSE_KILL_SWITCH  = false -- off — our code runs in reverse, bypass handles it
local USE_THROTTLE_SLIP    = true  -- continuously compute per-wheel slip during throttle events
local MIN_BRAKE_FLOOR      = 0.01  -- per-wheel absCoef floor when driver is braking (1% baseline)
local THROTTLE_SLIP_MIN_THROTTLE = 0.1   -- throttle must exceed this to activate
local THROTTLE_SLIP_MIN_REF      = 0.5   -- reference wheel speed must exceed this (m/s)

-- Rolling plausibility checker: maintain a 5000ms ring buffer of (fusedSpeed, ΔsensorY*dt).
-- Each tick, the physics-predicted speed = oldest_entry_speed − ∫sensorY·dt over window.
-- If current fusedSpeed diverges from this prediction beyond tolerance, snap to prediction.
-- Runs at ALL times so it has valid history going into any brake event.
local USE_PLAUSIBILITY         = true
local PLAUSIBILITY_WINDOW_TICKS = 10000 -- 5000ms at 2kHz physics rate
local PLAUSIBILITY_TOLERANCE    = 2.0   -- m/s: divergence beyond this triggers correction (currently unused)
-- Braking-branch wheel-reading gate: during a brake event, a wheel reading more than
-- this many m/s above plausibleSpeed is treated as wheelspin contamination and rejected
-- (fused does NOT snap up to it). Keeps Event-3-style 96-mph wheelspin spikes out of fused.
local PLAUSIBILITY_BRAKE_GATE_MPS = 0.0

local TICK_RATE_HZ = 200
local TICK_STEP = 1 / TICK_RATE_HZ
local timeAccum = 0

local origBrakeTorque = {}
local wheelToBrakeMap = {3, 4, 1, 2}
local wasBraking = false
local wheelSnapUpCount = 0
local wheelAgreementSnapDownCount = 0

-- Shared sensor state
local fusedSpeed = 0
local fusedPrevWs = {0, 0, 0, 0}
local fusedInitialized = false
local latestWheelSpeed = {0, 0, 0, 0}
local latestSensorY = 0
local latestRawSY = 0

-- Lateral speed integrator. Runs exactly like fusedSpeed but on sensorX.
-- No anchor: starts at 0 on every brake onset, integrates signed sensorX·dtPhys
-- during braking, holds last value when brake released. Signed so direction
-- (left/right drift) is preserved.
-- LATERAL_SETTLE_S delays the integrator for N seconds after brake onset so
-- the sensorX transient at brake kick-in doesn't pollute the integral.
local lateralSpeed = 0
local lateralWasBraking = false
local lateralBrakeElapsed = 0
local LATERAL_SETTLE_S = 1.0

local wheelIsFront = {false, false, false, false}  -- classified at init by body-frame X
local wheelIsLeft  = {false, false, false, false}  -- classified at init by body-frame Y (+Y is left)
local drivetrainType = "UNKNOWN"          -- "FWD"/"RWD"/"AWD"/"UNKNOWN"
local motionDirection = 1                 -- +1 forward, -1 reverse (hysteresis so lockups don't flip it)
-- Yaw telemetry only (recorded into brake events; no active spin prevention).
local lastYawRate = 0
local slipRatios = {0, 0, 0, 0}  -- per-wheel measured slip ratio, refreshed each PID tick

-- Slew-rate limiter state (Bosch MIR / yaw-moment limiter). lastCmd holds the
-- previous-tick brake command per slot so the limiter can cap rate-of-rise on the
-- high-grip side of an L/R imbalance. Decreases unlimited; increases capped.
local USE_SLEW_LIMITER = true
local lastCmd = {0, 0, 0, 0}
local SLEW_MAX_ASYMMETRY = 0.75  -- L/R Δ (in absCoef units) before limiter engages
local SLEW_MAX_STEP_UP   = 0.005 -- per-tick max increase on the high side; 200Hz × 0.005 = 1.0/sec full ramp
local REVERSE_DETECT_THRESHOLD = 1.0      -- m/s — minimum wheel speed to trust direction signal
local REVERSE_LOCKIN_MPS = 0.5            -- m/s — signed avg must exceed this to flip
local throttleSlip = {0, 0, 0, 0}         -- per-wheel wheelspin-slip ratio during throttle events

-- Plausibility ring buffer state
local plausBufSpeed = {}       -- fusedSpeed sampled each tick
local plausBufDeltaV = {}      -- sensorY * dtPhys each tick (velocity change contribution)
local plausIdx = 1             -- next write position (1-indexed, wraps at PLAUSIBILITY_WINDOW_TICKS)
local plausFilled = false      -- true once the buffer has been fully populated
local plausAccelSum = 0        -- running sum of ΔV over the window (O(1) maintenance)
local plausibleSpeed = 0       -- physics-predicted speed from oldest entry + integrated ΔV
local plausibilityActive = false  -- true on ticks where we overrode fusedSpeed
-- Last brake-onset snap result, kept across events so the UI can display it persistently.
-- status: "USED" | "SKIP_BUFFER" | "SKIP_GUARD" | "" (no event yet)
local lastPlausStatus = ""
local lastPlausDeltaMph = 0    -- signed (plaus - fusedBefore) in mph; 0 when SKIP
-- Per-event wheelspin-rejection counter: number of phys ticks the braking-branch
-- gate rejected trustedMax during the current event. Snapshot to lastPlausGateRejects
-- when the event ends, then reset on next brakingJustStarted.
local plausGateRejectTicks = 0
local lastPlausGateRejects = 0
local plausBrakeActive = false  -- mirrors input.brake>0 across detectMu/runTick
local wheelAgreementTicks = 0   -- consecutive 200Hz ticks all 4 wheels agree under no-brake-force
local brakeSimTime = 0

-- wheelAvg estimator (UI + diagnostics)
local wa = {
  speed = 0,
  updated = false,
  speedTwo = 0,
  prevWs = {0, 0, 0, 0},
  locked = {false, false, false, false},
  decelBuf = {{}, {}, {}, {}},
  decelIdx = {0, 0, 0, 0},
  LOCK_WINDOW = 8,
  LOCK_DECEL = -5,
}

local absSpeedSource = { fusedActive = true }

-- Per-wheel PID state
local slipIntegral = {0, 0, 0, 0}
local lastSlipError = {0, 0, 0, 0}
local prevTickWheelSpeed = {0, 0, 0, 0}

local slipTargets = {0.14, 0.14, 0.14, 0.14}
local SLIP_TARGET_MIN = 0.02
local SLIP_TARGET_MAX = 0.30
local TARGET_SMOOTHING = 0.95

-- Active surface probing: every PROBE_INTERVAL_S, briefly force all 4 wheels to
-- full brake (bypassing ABS PID) and compare measured decel to baseline decel.
-- Ratio > 1 → loose surface (sand/gravel/snow), wedge formation beats sliding
-- friction → raise slip target. Ratio < 1 → pavement, peak already found → decay
-- loose bonus. Gated by speed, yaw, pedal, and existing μ confidence.
-- Probe constants live inside CFG (see below) — runTick is near Lua 5.1's
-- 60-upvalue ceiling so read-only consts must not be separate module locals.
local probeTimer            = 0      -- counts ticks between probes
local probeActiveTicks      = 0      -- 0=not probing, >0=probing countdown
local probeDecelSum         = 0      -- sum of sensorY samples inside current probe (averaged at end)
local probeBaselineDecel    = 0      -- EMA of sensorY outside of probes (m/s²)
local looseAdjust           = 0      -- additive bump on top of μ-derived slip target
local lastProbeRatio        = 1.0    -- diagnostic: last probe's decel ratio
local probeLockoutEndTime   = 0      -- brakeSimTime until which wheel anchors stay suppressed

local D_EST_MIN = 0.10
local D_EST_MAX = 1.5
local consensusD = 1.0
local D_SMOOTHING = 0.80
local D_UPDATE_MIN_DECEL = 0.5
local currentBaseTarget = 0.14
local retroResetsTotal = 0

local DECEL_WINDOW_SIZE = 40
local decelWindow = {}
local decelWindowIdx = 0
local stableD = 1.0
local stableDUpdateTicks = 0
local STABLE_D_SETTLE_TICKS = 30
local RETRO_CHANGE_THRESHOLD = 0.40

-- Global D estimator uses chassis sensorY and a single consensusD.
-- Per-wheel μ via wheel-decel is too noisy to be useful in practice — ABS modulation
-- transients dominate the signal. wheelConsensusMu is kept as a 4-entry array for
-- UI compatibility, with all four entries populated from the global consensusD.
local wheelConsensusMu = {1, 1, 1, 1}

local KP = 6.0
local KI = 0.8
local KD = 0.08
local INTEGRAL_MIN = -1.0
local INTEGRAL_MAX = 1.0
local MIN_SPEED = 3.0
local MIN_ADAPT_SPEED = 2.0

local NON_BRAKING_DECEL_FILTER = -5.0
local STANDSTILL_WS_THRESHOLD = 0.3
local STANDSTILL_FUSED_THRESHOLD = 2.0
local STANDSTILL_PHYS_TICKS = 100
local standstillCounter = 0

-- All read-only constants bundled into one upvalue so runTick stays under Lua 5.1's
-- 60-upvalue-per-function ceiling. Mutable state (consensusD, slipTargets, etc.) stays
-- as direct locals because runTick still needs to assign to them.
local CFG = {
  USE_PLAUSIBILITY            = USE_PLAUSIBILITY,
  PLAUSIBILITY_TOLERANCE      = PLAUSIBILITY_TOLERANCE,
  USE_THROTTLE_SLIP           = USE_THROTTLE_SLIP,
  THROTTLE_SLIP_MIN_THROTTLE  = THROTTLE_SLIP_MIN_THROTTLE,
  THROTTLE_SLIP_MIN_REF       = THROTTLE_SLIP_MIN_REF,
  USE_REVERSE_SUPPORT         = USE_REVERSE_SUPPORT,
  REVERSE_KILL_SWITCH         = REVERSE_KILL_SWITCH,
  MIN_SPEED                   = MIN_SPEED,
  MIN_ADAPT_SPEED             = MIN_ADAPT_SPEED,
  MIN_BRAKE_FLOOR             = MIN_BRAKE_FLOOR,
  KP = KP, KI = KI, KD = KD,
  INTEGRAL_MIN                = INTEGRAL_MIN,
  INTEGRAL_MAX                = INTEGRAL_MAX,
  D_UPDATE_MIN_DECEL          = D_UPDATE_MIN_DECEL,
  D_EST_MIN                   = D_EST_MIN,
  D_EST_MAX                   = D_EST_MAX,
  D_SMOOTHING                 = D_SMOOTHING,
  DECEL_WINDOW_SIZE           = DECEL_WINDOW_SIZE,
  STABLE_D_SETTLE_TICKS       = STABLE_D_SETTLE_TICKS,
  RETRO_CHANGE_THRESHOLD      = RETRO_CHANGE_THRESHOLD,
  SLIP_TARGET_MIN             = SLIP_TARGET_MIN,
  SLIP_TARGET_MAX             = SLIP_TARGET_MAX,
  TARGET_SMOOTHING            = TARGET_SMOOTHING,
  USE_PROBE                   = true,
  PROBE_INTERVAL_TICKS        = 200,    -- 1 second at 200Hz between probes
  PROBE_DURATION_TICKS        = 16,     -- 80 ms probe window — long enough for wheels to reach locked steady-state
  PROBE_MIN_SPEED_MPS         = 4.47,   -- 10 mph
  PROBE_MAX_YAW_RATE          = 0.15,   -- rad/s — no probing in corners
  PROBE_SKIP_MU_LO            = 0.80,   -- skip if μ already in [lo,hi] (confident dry)
  PROBE_SKIP_MU_HI            = 1.50,
  PROBE_MIN_BRAKE             = 0.70,   -- only probe under hard brake
  PROBE_BASELINE_ALPHA        = 0.05,   -- EMA α for baseline decel
  PROBE_ADJ_STEP              = 0.15,   -- per-probe adjustment rate (scaled by ratio-1)
  PROBE_ADJ_MAX               = 0.30,   -- cap on looseAdjust added to slip target
  PROBE_RECOVERY_S            = 0.5,    -- after probe ends, wheel anchors stay disabled this long
  USE_SLEW_LIMITER            = USE_SLEW_LIMITER,
  SLEW_MAX_ASYMMETRY          = SLEW_MAX_ASYMMETRY,
  SLEW_MAX_STEP_UP            = SLEW_MAX_STEP_UP,
}

local uiAccum = 0
local function getCondition(slip)
  if slip <= 0.06 then return "Ice"
  elseif slip <= 0.10 then return "Snow"
  elseif slip <= 0.13 then return "Wet"
  else return "Dry Asphalt"
  end
end

-- Map per-wheel μ estimate → surface label. Different scale than slip targets.
local function getMuCondition(mu)
  if mu <= 0.15 then return "Ice"
  elseif mu <= 0.30 then return "Snow"
  elseif mu <= 0.55 then return "Wet"
  else return "Dry Asphalt"
  end
end

local function initDecelWindow()
  decelWindow = {}
  decelWindowIdx = 0
  stableD = 1.0
  stableDUpdateTicks = 0
  consensusD = 1.0
end



local function init(jbeamData)
  origBrakeTorque = {}
  slipIntegral = {0, 0, 0, 0}
  lastSlipError = {0, 0, 0, 0}
  wasBraking = false
  timeAccum = 0
  uiAccum = 0
  standstillCounter = 0

  slipTargets = {0.14, 0.14, 0.14, 0.14}
  prevTickWheelSpeed = {0, 0, 0, 0}
  initDecelWindow()
  -- Reset per-wheel μ display mirror (filled from global consensusD each tick)
  wheelConsensusMu = {1, 1, 1, 1}
  retroResetsTotal = 0
  currentBaseTarget = 0.14

  fusedSpeed = 0
  fusedPrevWs = {0, 0, 0, 0}
  fusedInitialized = false
  latestWheelSpeed = {0, 0, 0, 0}
  latestSensorY = 0
  latestRawSY = 0
  lateralSpeed = 0
  lateralWasBraking = false
  lateralBrakeElapsed = 0
  probeTimer = 0
  probeActiveTicks = 0
  probeDecelSum = 0
  probeBaselineDecel = 0
  probeLockoutEndTime = 0
  looseAdjust = 0
  lastProbeRatio = 1.0

  drivetrainType = "UNKNOWN"
  motionDirection = 1
  brakeSimTime = 0
  throttleSlip = {0, 0, 0, 0}
  plausBufSpeed = {}
  plausBufDeltaV = {}
  plausIdx = 1
  plausFilled = false
  plausAccelSum = 0
  plausibleSpeed = 0
  plausibilityActive = false
  lastPlausStatus = ""
  lastPlausDeltaMph = 0
  plausGateRejectTicks = 0
  lastPlausGateRejects = 0
  plausBrakeActive = false
  wheelAgreementTicks = 0
  lastYawRate = 0

  for i = 0, wheels.wheelRotatorCount - 1 do
    origBrakeTorque[i + 1] = wheels.wheelRotators[i].brakeTorque or 0
  end

  -- Classify wheels by jbeam-frame position. ETK jbeam convention: X is lateral
  -- (+X = left, e.g. engine e1l at x=+0.15 vs e1r at x=-0.15), Y is longitudinal
  -- (-Y = forward, e.g. front wheels at y=-1.40 vs rear at y=+1.41).
  local xs, ys = {0, 0, 0, 0}, {0, 0, 0, 0}
  for i = 0, math.min(3, wheels.wheelRotatorCount - 1) do
    local wheel = wheels.wheelRotators[i]
    local nid = wheel.nodeArm or wheel.node1
    if v and v.data and v.data.nodes and nid and v.data.nodes[nid] then
      local p = v.data.nodes[nid].pos
      if p and p.x then xs[i + 1] = p.x end
      if p and p.y then ys[i + 1] = p.y end
    end
  end
  local avgX = (xs[1] + xs[2] + xs[3] + xs[4]) * 0.25
  local avgY = (ys[1] + ys[2] + ys[3] + ys[4]) * 0.25
  for i = 1, 4 do
    wheelIsFront[i] = ys[i] < avgY  -- forward = smaller Y in jbeam
    wheelIsLeft[i]  = xs[i] > avgX  -- left = larger X in jbeam
  end

  slipRatios = {0, 0, 0, 0}

  wa.speed = 0
  wa.updated = false
  wa.speedTwo = 0
  wa.prevWs = {0, 0, 0, 0}
  wa.locked = {false, false, false, false}
  wa.decelBuf = {{}, {}, {}, {}}
  wa.decelIdx = {0, 0, 0, 0}

  extensions.load('abstelemetry')
end


-- Classify current drivetrain by polling wheel propulsion flags. Cheap, runs every tick
-- so a runtime 4WD-mode switch (Roamer etc.) is picked up immediately.
local function classifyDrivetrain()
  local frontDriven, rearDriven = false, false
  for i = 1, 4 do
    local w = wheels.wheelRotators[i - 1]
    if w and w.isPropulsed then
      if wheelIsFront[i] then frontDriven = true
      else rearDriven = true end
    end
  end
  if frontDriven and rearDriven then return "AWD"
  elseif frontDriven then return "FWD"
  elseif rearDriven then return "RWD"
  else return "UNKNOWN" end
end


-- 2kHz sensor fusion -> fusedSpeed
local function detectMu(dtPhys)
  local rawSY = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorY) or 0
  latestRawSY = rawSY
  -- BeamNG's sensorY is direction-agnostic (positive during decel, negative during accel),
  -- so no motionDirection flip needed here.
  local accForFused = rawSY
  latestSensorY = accForFused

  local signedSum, signedCount = 0, 0
  for i = 0, wheels.wheelRotatorCount - 1 do
    local signedWs = wheels.wheelRotators[i].wheelSpeed or 0
    latestWheelSpeed[i + 1] = math.abs(signedWs)
    if math.abs(signedWs) > REVERSE_DETECT_THRESHOLD then
      signedSum = signedSum + signedWs
      signedCount = signedCount + 1
    end
  end


  -- Motion-direction detection with hysteresis: only flip when signed avg clearly exceeds threshold.
  -- Below REVERSE_DETECT_THRESHOLD (wheels near zero / lockup), keep last direction.
  if USE_REVERSE_SUPPORT and signedCount > 0 then
    local signedAvg = signedSum / signedCount
    if signedAvg > REVERSE_LOCKIN_MPS then motionDirection = 1
    elseif signedAvg < -REVERSE_LOCKIN_MPS then motionDirection = -1 end
  end

  if not fusedInitialized then
    local maxInit = 0
    for i = 1, 4 do
      if latestWheelSpeed[i] > maxInit then maxInit = latestWheelSpeed[i] end
    end
    fusedSpeed = maxInit
    for i = 1, 4 do fusedPrevWs[i] = latestWheelSpeed[i] end
    fusedInitialized = true
  end

  local brakeInput = input.brake or 0
  local isBraking = brakeInput > 0

  -- Integrate fused via accelerometer
  local dv = accForFused * dtPhys
  fusedSpeed = math.max(0, fusedSpeed - dv)

  -- Lateral speed integrator: runs only during brake events, no anchor yet.
  -- Reset to 0 on brake onset, wait LATERAL_SETTLE_S for sensorX transient to
  -- settle, then integrate signed sensorX*dtPhys while braking; hold last
  -- value when brake released.
  local rawSX = (sensors and sensors.ffiSensors and sensors.ffiSensors.sensorX) or 0
  if isBraking then
    if not lateralWasBraking then
      lateralSpeed = 0
      lateralBrakeElapsed = 0
    end
    lateralBrakeElapsed = lateralBrakeElapsed + dtPhys
    if lateralBrakeElapsed >= LATERAL_SETTLE_S then
      lateralSpeed = lateralSpeed + rawSX * dtPhys
    end
  end
  lateralWasBraking = isBraking
  electrics.values.abs_lateralSpeed = lateralSpeed

  local maxWs = 0
  for i = 1, 4 do
    if latestWheelSpeed[i] > maxWs then maxWs = latestWheelSpeed[i] end
  end

  drivetrainType = classifyDrivetrain()

  -- Braking branch: snap fused up to fastest wheel, BUT reject wheel readings more than
  -- PLAUSIBILITY_BRAKE_GATE_MPS above the rolling plausibility prediction — those are
  -- wheelspin contamination at brake-release/onset, not real ground speed.
  if isBraking then
    -- Probe lockout: while probing, and for PROBE_RECOVERY_S afterward, the wheels may
    -- be locked/skidding far below true ground speed. Trusting maxWheel then would drag
    -- fusedSpeed down with them, killing the PID's slip calc and under-braking the rest
    -- of the stop. Disable the snap-up anchor through that window — accelerometer
    -- integrator carries fusedSpeed truthfully in the meantime.
    local inProbeLockout = (probeActiveTicks > 0) or (brakeSimTime < probeLockoutEndTime)
    if not inProbeLockout then
      local maxWheel = 0
      for i = 1, 4 do
        if latestWheelSpeed[i] > maxWheel then maxWheel = latestWheelSpeed[i] end
      end
      if maxWheel > fusedSpeed then
        local gateBlocks = USE_PLAUSIBILITY and plausFilled
                           and (maxWheel > plausibleSpeed + PLAUSIBILITY_BRAKE_GATE_MPS)
        if gateBlocks then
          plausGateRejectTicks = plausGateRejectTicks + 1
        else
          wheelSnapUpCount = wheelSnapUpCount + 1
          fusedSpeed = maxWheel
        end
      end
    end
  else
    -- Non-braking fallback: average wheels that aren't catastrophically decelerating
    local goodSum, goodCount = 0, 0
    for i = 1, 4 do
      local d = (latestWheelSpeed[i] - fusedPrevWs[i]) / dtPhys
      if d > NON_BRAKING_DECEL_FILTER then
        goodSum = goodSum + latestWheelSpeed[i]
        goodCount = goodCount + 1
      end
    end
    if goodCount > 0 then fusedSpeed = goodSum / goodCount end
  end

  for i = 1, 4 do fusedPrevWs[i] = latestWheelSpeed[i] end

  -- Rolling plausibility checker: compare fusedSpeed against physics-predicted speed from
  -- 5000ms ago. Same integration math fusedSpeed itself uses. Runs every tick, always.
  plausibilityActive = false
  if USE_PLAUSIBILITY then
    -- Contribution this tick: reuse the SAME dv fusedSpeed's integrator just used (1:1 match)
    if plausFilled then
      -- Evict the oldest entry's ΔV contribution before overwriting it
      plausAccelSum = plausAccelSum - (plausBufDeltaV[plausIdx] or 0)
    end
    plausBufSpeed[plausIdx] = fusedSpeed
    plausBufDeltaV[plausIdx] = dv
    plausAccelSum = plausAccelSum + dv

    plausIdx = plausIdx + 1
    if plausIdx > PLAUSIBILITY_WINDOW_TICKS then
      plausIdx = 1
      plausFilled = true
    end

    if plausFilled then
      -- After wrap, plausIdx points at the oldest entry (the one we just overwrite next tick)
      local oldestSpeed = plausBufSpeed[plausIdx] or fusedSpeed
      plausibleSpeed = math.max(0, oldestSpeed - plausAccelSum)
      -- NOTE: override of fusedSpeed happens ONLY at brake-event start, in runTick.
      -- This block just maintains the rolling prediction so runTick has it available.
    end

    -- Direct FWD/RWD wheelspin override: when throttle is on and brakes are off,
    -- the non-driven axle is ground truth (it's not being driven, so it can't spin).
    -- Exact comparison, no margin — any disparity at all snaps plausibility to the
    -- non-driven side. AWD not handled here (no clean non-driven reference).
    if (input.throttle or 0) > 0.1 and (input.brake or 0) < 0.1 then
      local frontSum, rearSum, frontN, rearN = 0, 0, 0, 0
      for i = 1, 4 do
        if wheelIsFront[i] then frontSum = frontSum + latestWheelSpeed[i]; frontN = frontN + 1
        else rearSum = rearSum + latestWheelSpeed[i]; rearN = rearN + 1 end
      end
      if frontN > 0 and rearN > 0 then
        local frontAvg = frontSum / frontN
        local rearAvg = rearSum / rearN
        if drivetrainType == "FWD" and frontAvg > rearAvg then
          plausibleSpeed = rearAvg
        elseif drivetrainType == "RWD" and rearAvg > frontAvg then
          plausibleSpeed = frontAvg
        end
      end
    end
  end
  electrics.values.abs_plausibleSpeed = plausibleSpeed
  electrics.values.abs_plausibilityActive = plausibilityActive and 1 or 0

  if maxWs < STANDSTILL_WS_THRESHOLD and fusedSpeed < STANDSTILL_FUSED_THRESHOLD then
    standstillCounter = standstillCounter + 1
    if standstillCounter >= STANDSTILL_PHYS_TICKS then fusedSpeed = 0 end
  else
    standstillCounter = 0
  end
end


-- 200Hz PID + D estimator
local function runTick(dt)
  local abstelem = extensions.abstelemetry
  local haveTelem = abstelem ~= nil and abstelem.setBrakes ~= nil

  -- KILL SWITCH: when gearbox is in reverse (gearIndex == -1), release brake control entirely
  -- and do nothing. BeamNG's stock brake pipeline takes over. Use this to isolate which of
  -- our features is causing the reverse-brake problem by flipping features off/on with this
  -- master toggle flipped off.
  if CFG.REVERSE_KILL_SWITCH and (electrics.values.gearIndex or 0) == -1 then
    if haveTelem and abstelem.releaseBrakes then abstelem.releaseBrakes() end
    return
  end

  local brakeInput = input.brake or 0
  local brakingJustStarted = brakeInput > 0 and not wasBraking
  wasBraking = brakeInput > 0

  -- One-shot plausibility override: at the very start of a brake event, ALWAYS adopt the
  -- physics-predicted speed as our starting MPH. Only fires at brake-event onset; never
  -- continuously. Defensive guards: prediction must be at least walking speed (avoid
  -- garbage), and current fused must not already be near zero (no point snapping at
  -- standstill). PLAUSIBILITY_TOLERANCE no longer gates the snap — kept in CFG for now
  -- in case we want to re-introduce a divergence floor later.
  plausibilityActive = false
  if CFG.USE_PLAUSIBILITY and brakingJustStarted then
    local fusedBefore = fusedSpeed
    -- Reset per-event wheelspin-rejection counter on every brake start
    plausGateRejectTicks = 0
    if not plausFilled then
      lastPlausStatus = "SKIP_BUFFER"
      lastPlausDeltaMph = 0
    elseif plausibleSpeed <= 1.0 or fusedSpeed <= 1.0 then
      lastPlausStatus = "SKIP_GUARD"
      lastPlausDeltaMph = 0
    elseif plausibleSpeed > fusedSpeed then
      -- Plausibility prediction is HIGHER than fused. Normal failure mode is wheels
      -- over-spin pushing fused too high → plaus catches it by being lower. The reverse
      -- (plaus higher than fused) usually means plausibility itself drifted, not that
      -- fused is wrong. Don't snap up; trust the lower fused value.
      lastPlausStatus = "SKIP_HIGHER"
      lastPlausDeltaMph = 0
    else
      fusedSpeed = plausibleSpeed
      plausibilityActive = true
      lastPlausStatus = "USED"
      lastPlausDeltaMph = (plausibleSpeed - fusedBefore) * 2.237
    end
  end

  -- Snapshot per-event wheelspin-rejection count when brake releases, so the UI
  -- and console can show "last event rejected N ticks" until the next brake fires.
  if plausBrakeActive and not (brakeInput > 0) then
    lastPlausGateRejects = plausGateRejectTicks
  end
  plausBrakeActive = brakeInput > 0

  local isBraking = brakeInput > 0

  -- Continuous throttle-slip detection: under throttle (and not braking), each driven wheel's
  -- slip is computed against a ground-truth reference. FWD/RWD: average of non-driven axle.
  -- AWD/UNKNOWN: slowest wheel (least-slipping). Zero when inactive.
  if CFG.USE_THROTTLE_SLIP then
    local throttle = input.throttle or 0
    if throttle > CFG.THROTTLE_SLIP_MIN_THROTTLE and not isBraking then
      local refSpeed = 0
      if drivetrainType == "FWD" then
        local s, n = 0, 0
        for i = 1, 4 do
          if not wheelIsFront[i] then s = s + latestWheelSpeed[i]; n = n + 1 end
        end
        if n > 0 then refSpeed = s / n end
      elseif drivetrainType == "RWD" then
        local s, n = 0, 0
        for i = 1, 4 do
          if wheelIsFront[i] then s = s + latestWheelSpeed[i]; n = n + 1 end
        end
        if n > 0 then refSpeed = s / n end
      else
        refSpeed = latestWheelSpeed[1]
        for i = 2, 4 do
          if latestWheelSpeed[i] < refSpeed then refSpeed = latestWheelSpeed[i] end
        end
      end

      if refSpeed > CFG.THROTTLE_SLIP_MIN_REF then
        for i = 1, 4 do
          local isDriven = (drivetrainType == "FWD" and wheelIsFront[i])
                        or (drivetrainType == "RWD" and not wheelIsFront[i])
                        or (drivetrainType ~= "FWD" and drivetrainType ~= "RWD")
          if isDriven then
            throttleSlip[i] = math.max(0, (latestWheelSpeed[i] - refSpeed) / refSpeed)
          else
            throttleSlip[i] = 0
          end
        end
      else
        for i = 1, 4 do throttleSlip[i] = 0 end
      end
    else
      for i = 1, 4 do throttleSlip[i] = 0 end
    end
  end

  -- Reverse bypass: pass driver brake through with no PID modulation. Handles three arcade
  -- input routings that all surface differently:
  --   (1) driver brakes via gamepad brake axis while NOT in arcade's brake-key-means-throttle
  --       mode → use input.brake directly.
  --   (2) driver presses "forward" while moving backward → arcade writes
  --       electrics.values.brake itself (auto-brake); we must release so the stock pipeline
  --       can apply it — calling setBrakes(0,0,0,0) would zero wd.ref.brakeTorque and
  --       suppress the auto-brake.
  --   (3) driver presses brake axis while in reverse gear → arcade reinterprets as
  --       "more reverse throttle" and writes electrics.values.throttle > 0. In that case
  --       input.brake is still 1, but the driver doesn't actually want to brake.
  if CFG.USE_REVERSE_SUPPORT and motionDirection < 0 then
    -- Detect case (3): arcade has synthesized reverse-throttle, so ignore input.brake.
    local arcadeWantsReverseThrottle = (electrics.values.throttle or 0) > 0.05
    local effectiveBrake = arcadeWantsReverseThrottle and 0 or brakeInput

    for i = 1, 4 do
      slipIntegral[i] = 0
      lastSlipError[i] = 0
      prevTickWheelSpeed[i] = latestWheelSpeed[i]
    end
    if effectiveBrake > 0 then
      local cmd = {0, 0, 0, 0}
      for i = 1, 4 do cmd[wheelToBrakeMap[i]] = effectiveBrake end
      if haveTelem then abstelem.setBrakes(cmd[1], cmd[2], cmd[3], cmd[4]) end
    else
      -- No real brake intent — release so arcade's auto-brake (case 2) or
      -- reverse-throttle (case 3) can reach the wheels via the stock pipeline.
      if haveTelem and abstelem.releaseBrakes then abstelem.releaseBrakes() end
    end
    return
  end

  do
    local goodSpeeds = {}
    local goodSum, goodCount = 0, 0
    for i = 1, 4 do
      local wDecel = (latestWheelSpeed[i] - wa.prevWs[i]) / dt

      wa.decelIdx[i] = (wa.decelIdx[i] % wa.LOCK_WINDOW) + 1
      wa.decelBuf[i][wa.decelIdx[i]] = wDecel

      if wa.locked[i] then
        local latest = wa.decelBuf[i][wa.decelIdx[i]]
        if latest >= 0 then
          local hasPositive = false
          for j = 1, wa.LOCK_WINDOW do
            local v = wa.decelBuf[i][j]
            if v and v > 0 then hasPositive = true; break end
          end
          if hasPositive then wa.locked[i] = false end
        end
      else
        if wDecel < wa.LOCK_DECEL then wa.locked[i] = true end
      end

      if not wa.locked[i] and wDecel > -0.01 then
        goodSum = goodSum + latestWheelSpeed[i]
        goodCount = goodCount + 1
        goodSpeeds[#goodSpeeds + 1] = latestWheelSpeed[i]
      end
    end
    wa.updated = goodCount > 0
    if wa.updated then wa.speed = goodSum / goodCount end

    for a = 1, #goodSpeeds - 1 do
      for b = a + 1, #goodSpeeds do
        if math.abs(goodSpeeds[a] - goodSpeeds[b]) < 0.894 then
          wa.speedTwo = (goodSpeeds[a] + goodSpeeds[b]) / 2
        end
      end
    end

    for i = 1, 4 do wa.prevWs[i] = latestWheelSpeed[i] end
  end

  local carSpeed = fusedSpeed
  absSpeedSource.fusedActive = true

  if brakingJustStarted then
    for i = 1, 4 do
      slipIntegral[i] = 0
      lastSlipError[i] = 0
    end
    wheelAgreementSnapDownCount = 0
    probeTimer = 0
    probeActiveTicks = 0
    probeBaselineDecel = 0
    probeLockoutEndTime = 0
  end

  -- Probe-timer advance + baseline decel EMA (only while braking and not mid-probe)
  if isBraking and probeActiveTicks == 0 then
    probeTimer = probeTimer + 1
    probeBaselineDecel = probeBaselineDecel * (1 - CFG.PROBE_BASELINE_ALPHA)
                       + latestRawSY * CFG.PROBE_BASELINE_ALPHA
  end

  -- Probe-start decision. All gates must pass; don't re-fire while one is active.
  if CFG.USE_PROBE and isBraking and probeActiveTicks == 0
     and probeTimer >= CFG.PROBE_INTERVAL_TICKS
     and brakeInput > CFG.PROBE_MIN_BRAKE
     and fusedSpeed > CFG.PROBE_MIN_SPEED_MPS
     and math.abs(lastYawRate) < CFG.PROBE_MAX_YAW_RATE
     and not (consensusD >= CFG.PROBE_SKIP_MU_LO and consensusD <= CFG.PROBE_SKIP_MU_HI) then
    probeActiveTicks = CFG.PROBE_DURATION_TICKS
    probeDecelSum    = 0
    probeTimer       = 0
  end

  local cmd = {0, 0, 0, 0}

  for i = 1, 4 do
    local slot = wheelToBrakeMap[i]

    if probeActiveTicks > 0 and isBraking then
      -- Probe window: full driver brake, no ABS. PID state frozen (no integrator updates).
      cmd[slot] = brakeInput
      if carSpeed > CFG.MIN_SPEED then
        slipRatios[i] = math.max(0, math.min((carSpeed - latestWheelSpeed[i]) / math.max(carSpeed, 0.1), 1))
      end
    elseif carSpeed > CFG.MIN_SPEED and isBraking then
      local slip = math.max(0, math.min((carSpeed - latestWheelSpeed[i]) / math.max(carSpeed, 0.1), 1))
      local effectiveTarget = math.min(slipTargets[i] + (1 + consensusD) / carSpeed, 1.0)
      local slipError = effectiveTarget - slip

      slipIntegral[i] = math.max(CFG.INTEGRAL_MIN, math.min(CFG.INTEGRAL_MAX, slipIntegral[i] + slipError * dt))

      local slipErrorDerivative = 0
      if lastSlipError[i] ~= 0 then
        slipErrorDerivative = (slipError - lastSlipError[i]) / dt
      end
      lastSlipError[i] = slipError

      local absCoef = math.max(0.0, math.min(1,
        slipError * CFG.KP + slipIntegral[i] * CFG.KI + slipErrorDerivative * CFG.KD))

      if absCoef < CFG.MIN_BRAKE_FLOOR then absCoef = CFG.MIN_BRAKE_FLOOR end

      slipRatios[i] = slip
      cmd[slot] = absCoef * brakeInput
    else
      cmd[slot] = brakeInput
      slipIntegral[i] = 0
      lastSlipError[i] = 0
    end
  end

  -- Probe-end measurement. Accumulate sensorY every tick while probing (same signal
  -- as baseline EMA → apples-to-apples). When countdown reaches 0, compare averaged
  -- probe decel vs baseline. ratio > 1 → loose (locked rubber plows) → bump looseAdjust
  -- up. ratio < 1 → hard surface → adjustment stays at 0 (clamped).
  if probeActiveTicks > 0 then
    probeDecelSum = probeDecelSum + latestRawSY
    probeActiveTicks = probeActiveTicks - 1
    if probeActiveTicks == 0 then
      probeLockoutEndTime = brakeSimTime + CFG.PROBE_RECOVERY_S
      local probeDecel = probeDecelSum / CFG.PROBE_DURATION_TICKS
      if probeBaselineDecel > 0.5 then
        lastProbeRatio = probeDecel / probeBaselineDecel
        looseAdjust = math.max(0, math.min(CFG.PROBE_ADJ_MAX,
          looseAdjust + CFG.PROBE_ADJ_STEP * (lastProbeRatio - 1.0)))
      end
      -- Flush PID transients caused by the wide-open brake burst.
      for j = 1, 4 do
        slipIntegral[j] = 0
        lastSlipError[j] = 0
      end
    end
  end

  for i = 1, 4 do prevTickWheelSpeed[i] = latestWheelSpeed[i] end

  -- Wheel-agreement safety: if driver IS braking but actual brake force is modest
  -- (maxCmd below 0.50) AND all 4 wheels agree within 5 mph for 20 ms, the wheels are
  -- clearly rolling truthfully — snap fused AND plausibility to the lowest wheel.
  -- 5 mph = 2.2352 m/s. 20 ms at 200Hz = 4 consecutive ticks.
  if isBraking then
    local maxCmd = math.max(cmd[1], cmd[2], cmd[3], cmd[4])
    local minWs = math.min(latestWheelSpeed[1], latestWheelSpeed[2], latestWheelSpeed[3], latestWheelSpeed[4])
    local maxWs = math.max(latestWheelSpeed[1], latestWheelSpeed[2], latestWheelSpeed[3], latestWheelSpeed[4])
    local inProbeLockout = (probeActiveTicks > 0) or (brakeSimTime < probeLockoutEndTime)
    if not inProbeLockout and maxCmd < 0.50 and (maxWs - minWs) < 2.2352 then
      wheelAgreementTicks = wheelAgreementTicks + 1
      if wheelAgreementTicks >= 4 then
        wheelAgreementSnapDownCount = wheelAgreementSnapDownCount + 1
        fusedSpeed = minWs
        plausibleSpeed = minWs
      end
    else
      wheelAgreementTicks = 0
    end
  else
    wheelAgreementTicks = 0
  end

  -- Global D estimator. sensorY = chassis decel, naturally bounded.
  -- All 4 slipTargets smoothed toward the same currentBaseTarget. wheelConsensusMu mirrors
  -- consensusD into a 4-entry array purely for UI display compatibility.
  if isBraking and carSpeed > CFG.MIN_ADAPT_SPEED then
    local measuredDecel = latestRawSY

    if measuredDecel > CFG.D_UPDATE_MIN_DECEL then
      decelWindowIdx = (decelWindowIdx % CFG.DECEL_WINDOW_SIZE) + 1
      decelWindow[decelWindowIdx] = measuredDecel

      local peakDecel = 0
      for j = 1, CFG.DECEL_WINDOW_SIZE do
        if decelWindow[j] and decelWindow[j] > peakDecel then peakDecel = decelWindow[j] end
      end

      local instantD = math.max(CFG.D_EST_MIN, math.min(CFG.D_EST_MAX, peakDecel / 9.81 * 0.91))

      stableDUpdateTicks = stableDUpdateTicks + 1
      if stableDUpdateTicks > CFG.STABLE_D_SETTLE_TICKS then
        local dChange = math.abs(instantD - stableD) / math.max(stableD, 0.1)
        if dChange > CFG.RETRO_CHANGE_THRESHOLD then
          decelWindow = {}
          decelWindowIdx = 1
          decelWindow[1] = measuredDecel
          retroResetsTotal = retroResetsTotal + 1
          stableDUpdateTicks = 0
          consensusD = instantD
        end
        stableD = consensusD
      end

      consensusD = consensusD * CFG.D_SMOOTHING + instantD * (1.0 - CFG.D_SMOOTHING)

      currentBaseTarget = math.max(CFG.SLIP_TARGET_MIN, math.min(CFG.SLIP_TARGET_MAX,
        0.04 + consensusD * 0.10))
      -- Off-road probe bump: adds on top of μ-derived target, capped separately at 1.0.
      currentBaseTarget = math.min(1.0, currentBaseTarget + looseAdjust)

      for j = 1, 4 do
        slipTargets[j] = slipTargets[j] * CFG.TARGET_SMOOTHING + currentBaseTarget * (1.0 - CFG.TARGET_SMOOTHING)
        wheelConsensusMu[j] = consensusD  -- mirror to per-wheel array for UI display
      end
    end
  end

  -- Yaw rate: read once per tick for the probe gate (no active spin prevention).
  lastYawRate = (obj and obj.getYawAngularVelocity and obj:getYawAngularVelocity()) or 0

  -- Slew-rate limiter (Bosch MIR / yaw-moment limiter). Per axle, if one side's PID
  -- output is significantly higher than the other, cap the high side's per-tick
  -- INCREASE so it ramps in slowly. Decreases pass through unchanged (fast release,
  -- slow re-apply). On uniform surfaces the asymmetry stays small and the limiter
  -- never engages, so no straight-line stopping cost.
  if CFG.USE_SLEW_LIMITER then
    for axle = 1, 2 do
      local leftIdx, rightIdx = nil, nil
      for i = 1, 4 do
        local isThisAxle = (axle == 1 and wheelIsFront[i]) or (axle == 2 and not wheelIsFront[i])
        if isThisAxle then
          if wheelIsLeft[i] then leftIdx = i else rightIdx = i end
        end
      end
      if leftIdx and rightIdx then
        local leftSlot  = wheelToBrakeMap[leftIdx]
        local rightSlot = wheelToBrakeMap[rightIdx]
        local leftRaw   = cmd[leftSlot]  or 0
        local rightRaw  = cmd[rightSlot] or 0
        if rightRaw > leftRaw + CFG.SLEW_MAX_ASYMMETRY then
          cmd[rightSlot] = math.min(rightRaw, (lastCmd[rightSlot] or 0) + CFG.SLEW_MAX_STEP_UP)
        elseif leftRaw > rightRaw + CFG.SLEW_MAX_ASYMMETRY then
          cmd[leftSlot] = math.min(leftRaw, (lastCmd[leftSlot] or 0) + CFG.SLEW_MAX_STEP_UP)
        end
      end
    end
  end
  for i = 1, 4 do lastCmd[i] = cmd[i] or 0 end

  if haveTelem then
    abstelem.setBrakes(cmd[1], cmd[2], cmd[3], cmd[4])
  end

end


local function update(dtPhys)
  detectMu(dtPhys)
  brakeSimTime = brakeSimTime + dtPhys

  timeAccum = timeAccum + dtPhys
  while timeAccum >= TICK_STEP do
    runTick(TICK_STEP)
    timeAccum = timeAccum - TICK_STEP
  end

  electrics.values.abs_consensusD = consensusD
  electrics.values.abs_baseTarget = currentBaseTarget
  electrics.values.abs_retroResets = retroResetsTotal
  electrics.values.abs_surface = getCondition(slipTargets[1])
  electrics.values.abs_wheelAvgSpeed = wa.speed
  electrics.values.abs_wheelAvgSpeedTwo = wa.speedTwo
  electrics.values.abs_drivetrain = drivetrainType
  electrics.values.abs_motionDirection = motionDirection
  electrics.values.abs_wheelMu_RR = wheelConsensusMu[1]
  electrics.values.abs_wheelMu_RL = wheelConsensusMu[2]
  electrics.values.abs_wheelMu_FR = wheelConsensusMu[3]
  electrics.values.abs_wheelMu_FL = wheelConsensusMu[4]
  electrics.values.abs_throttleSlip_RR = throttleSlip[1]
  electrics.values.abs_throttleSlip_RL = throttleSlip[2]
  electrics.values.abs_throttleSlip_FR = throttleSlip[3]
  electrics.values.abs_throttleSlip_FL = throttleSlip[4]

  uiAccum = uiAccum + dtPhys
  if uiAccum >= 0.2 then
    uiAccum = 0
    if guihooks then
      guihooks.trigger('updateABSGrip', {
        RR = { surfaceMu = string.format("%.2f", wheelConsensusMu[1]), slipMu = string.format("%.2f", slipRatios[1]) },
        RL = { surfaceMu = string.format("%.2f", wheelConsensusMu[2]), slipMu = string.format("%.2f", slipRatios[2]) },
        FR = { surfaceMu = string.format("%.2f", wheelConsensusMu[3]), slipMu = string.format("%.2f", slipRatios[3]) },
        FL = { surfaceMu = string.format("%.2f", wheelConsensusMu[4]), slipMu = string.format("%.2f", slipRatios[4]) },
        speeds = {
          airspeed       = string.format("%.1f", electrics.values.airspeed or 0),
          fusedSpeed     = string.format("%.1f", fusedSpeed or 0),
          plausibleSpeed = string.format("%.1f", plausibleSpeed or 0),
          virtualAirspeed = string.format("%.1f", electrics.values.virtualAirspeed or 0),
          fusedActive    = absSpeedSource.fusedActive,
          snapUpCount    = wheelSnapUpCount,
          snapDnCount    = wheelAgreementSnapDownCount,
}
      })
    end
  end
end


local function reset(jbeamData)
  init(jbeamData)
end


M.init = init
M.update = update
M.reset = reset

return M
