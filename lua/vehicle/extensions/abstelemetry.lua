-- abstelemetry.lua — per-wheel brake application for Blake ABS-2F
--
-- Keeps per-wheel brake commands set by setBrakes() alive at 2000Hz physics rate.
-- BeamNG's stock pipeline recomputes wd.ref.brakeTorque every physics substep,
-- so without this re-application the per-wheel split gets clobbered back to a
-- single axle-matched value.

local M = {}

local wheelData = {}
local initialized = false
local perWheelMode = false
local brakeCmd = {0, 0, 0, 0}         -- FR, FL, RR, RL
local origBrakeTorque = {}
-- wheelData order: 1=RR, 2=RL, 3=FR, 4=FL
-- brakeCmd  order: 1=FR, 2=FL, 3=RR, 4=RL
local wheelToBrakeMap = {3, 4, 1, 2}


local function tryInitWheels()
  wheelData = {}
  pcall(function()
    if wheels and wheels.wheelRotators and wheels.wheelRotatorCount and wheels.wheelRotatorCount > 0 then
      for i = 0, wheels.wheelRotatorCount - 1 do
        local w = wheels.wheelRotators[i]
        if w then
          table.insert(wheelData, { name = w.name or ("rot_" .. i), ref = w })
        end
      end
    end
  end)

  if #wheelData > 0 then
    initialized = true
    for i, wd in ipairs(wheelData) do
      origBrakeTorque[i] = wd.ref.brakeTorque or 0
      -- Take over from the stock per-wheel ABS if the wheel has one.
      pcall(function()
        if wd.ref.absSlipRatioTarget ~= nil then wd.ref.absSlipRatioTarget = 0 end
        if wd.ref.absEnabled ~= nil then wd.ref.absEnabled = false end
        if wd.ref.absActive ~= nil then wd.ref.absActive = false end
      end)
    end
  end
end


local function applyPerWheelBrakes()
  if not perWheelMode or not initialized or #wheelData < 4 then return end
  local maxBrake = math.max(brakeCmd[1], brakeCmd[2], brakeCmd[3], brakeCmd[4])
  for i, wd in ipairs(wheelData) do
    local cmdIdx = wheelToBrakeMap[i]
    if cmdIdx and origBrakeTorque[i] and origBrakeTorque[i] > 0 then
      if maxBrake > 0.001 then
        wd.ref.brakeTorque = origBrakeTorque[i] * (brakeCmd[cmdIdx] or 0) / maxBrake
      else
        wd.ref.brakeTorque = 0
      end
    end
  end
end


local function setBrakes(fr, fl, rr, rl)
  fr = math.max(0, math.min(1, fr or 0))
  fl = math.max(0, math.min(1, fl or 0))
  rr = math.max(0, math.min(1, rr or 0))
  rl = math.max(0, math.min(1, rl or 0))

  brakeCmd = {fr, fl, rr, rl}
  perWheelMode = true
  electrics.values.brake = math.max(fr, fl, rr, rl)
  applyPerWheelBrakes()
end


local function releaseBrakes()
  perWheelMode = false
  brakeCmd = {0, 0, 0, 0}
  for i, wd in ipairs(wheelData) do
    if origBrakeTorque[i] then wd.ref.brakeTorque = origBrakeTorque[i] end
  end
end


local function onPhysicsStep(dtPhys)
  if not initialized then return end
  applyPerWheelBrakes()
end


local function onGraphicsStep(dtSim)
  if not initialized then tryInitWheels() end
end


local function onExtensionLoaded()
  enablePhysicsStepHook()
end


local function onReset()
  initialized = false
  perWheelMode = false
  brakeCmd = {0, 0, 0, 0}
  origBrakeTorque = {}
end


M.onExtensionLoaded = onExtensionLoaded
M.updateGFX         = onGraphicsStep
M.onPhysicsStep     = onPhysicsStep
M.onReset           = onReset
M.setBrakes         = setBrakes
M.releaseBrakes     = releaseBrakes

return M
