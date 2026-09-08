# BeamNG Dynamic ABS

An anti lock braking controller for BeamNG.drive. It replaces the stock ABS on the vanilla vehicles with a per wheel slip controller that runs at 200 hertz on a fused speed estimate, regulating each wheel to the slip target the vehicle itself declares.

## How it works

- Fused ground speed: wheel speeds are combined with the longitudinal accelerometer, so the controller still knows the road speed when every wheel is locked, in the air, or landing.
- Vehicle declared slip target: the target comes from the slip ratio the car publishes in its own brakes file, plus a speed term, rather than from an estimate. Every car gets the number its tyres and brakes were designed around.
- Grip aware trimming: a per wheel grip estimator scales the target down and raises loop gain when grip falls, so ice and wet get a shallower target and a faster loop, while dry asphalt runs the declared target untouched.
- Cornering gain: loop gain rises with lateral load, so the controller stays responsive while the tyres are already working sideways.
- Low speed re anchor: below a threshold the fused speed becomes a blend of the accelerometer integral, which over reads, and the fastest wheel, which under reads, so the two errors cancel.
- Own sensors only: wheel speeds, the IMU, and the driver inputs. The controller never reads true vehicle velocity from the engine.

## Install

1. Copy the repository folder into the BeamNG.drive user folder under mods, unpacked, Dynamic_ABS, or zip it and drop the zip into mods.
2. In the vehicle configurator choose Dynamic ABS in the ABS slot.
3. Optional: add the ABS Grip Gauges app from the app menu to watch per wheel grip and fused speed while driving.

## Measured against the stock ABS

Every number comes from the same measurement: braking at full pedal, distance taken as the chord from the moment true speed crosses the target down to 1 metre per second, average g derived from speed and distance. Speed and position are sampled in the physics step at 2000 hertz, so frame rate cannot affect a result.

Every stop starts from a freshly spawned vehicle, so brake temperature, damage and drivetrain state cannot carry from one measurement into the next, and the two controllers swap which one runs first on alternate repetitions so run order cannot favour either. Brake surface temperature is recorded at the start and the end of every stop and is published with the raw data.

A campaign is published as it fills rather than only when it is complete, so these figures gain precision over successive updates. Every published set is balanced: a repetition covers every car, speed and controller before the next one begins, so no cell carries more stops than another. The run count behind each cell is on the wiki pages.

The figures below are averaged across every car tested, so they describe the controller rather than any one vehicle. Per car tables and charts are in the wiki.

Tested on BeamNG.drive 0.39.4.0.20972 for the surface areas and BeamNG.tech 0.37.6.0.18775 for straight line and cornering. Four cars, each in a custom configuration rather than a factory trim: an ETK 800 sedan on sport plus tyres, a Hirochi SBR4 on race brakes and race tyres, a Hirochi Scintilla on sport plus tyres, and a Gavril Roamer pickup on all terrain tyres with rear drum brakes. The parts are listed in the wiki.

### Straight line

| Speed | Average g against stock | Cars |
|---|---|---|
| 60 miles per hour | +0.0106 | 4 |
| 80 miles per hour | +0.0054 | 4 |
| 120 miles per hour | +0.0126 | 4 |
| 160 miles per hour | +0.0442 | 2 |
| 180 miles per hour | +0.0417 | 2 |

### Cornering

Sweeping left corners at 60 miles per hour with full brake, at the steer angles where the controllers separate most.

These cornering figures are provisional. They were collected before the fresh spawn rule described above, so brake state carries between stops. They are being re measured and this table will be replaced when that finishes.

| Steer | Stopping distance against stock | Cars |
|---|---|---|
| 0.20 | -5.32 metres | 4 |
| 0.50 | -6.12 metres | 4 |
| 1.00 | -5.98 metres | 4 |

### Surfaces

Recorded areas on gridmap: ice, grass, sand, inclines, a jump and three bump sections.

These surface figures were collected before the fresh spawn change above, so they carry brake heat from one test point to the next and are being re measured. Read them as provisional. The straight line and cornering figures on this page do not have that problem.

These averages hide a real split, so read the per car page before drawing a conclusion from them. Results on ice and on the bump sections depend strongly on the car. One vehicle loses about a third of its braking on the bump sections while the other two gain there, and that single result pulls every bump average below zero.

| Area | Average g against stock | Cars |
|---|---|---|
| Bumps 1 | -0.0779 | 3 |
| Bumps 2 | -0.1207 | 3 |
| Bumps 3 | -0.1414 | 3 |
| Grass | -0.0728 | 2 |
| Ice | -0.0062 | 3 |
| Incline 15 deg | -0.0116 | 1 |
| Incline 35 deg | -0.0707 | 3 |
| Sand | +0.0141 | 3 |
| Small jump | +0.0039 | 3 |

## More detail

The wiki carries the per car and per condition breakdown, the full version history, and the testing method in detail: https://github.com/ITakeCake/DynamicABS/wiki

Published controller build: V4.00.

