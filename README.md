# BeamNG Dynamic ABS

An advanced, adaptive anti-lock braking controller for BeamNG.drive that actively responds to changing conditions. 

### What makes it Dynamic?
Unlike standard static ABS controllers, this system:
- **Fuses Telemetry Data:** Combines wheel speeds with IMU (accelerometer) data to calculate the true ground speed of the vehicle, even when all four wheels are locked or airborne.
- **Adapts to Surface Grip:** Continuously estimates the current surface friction (D-Estimator) and dynamically adjusts the target wheel slip. It knows the difference between asphalt and ice and changes its braking strategy accordingly.
- **Per-Wheel Optimization:** Each wheel runs its own independent PID control loop and grip estimator, meaning you can brake safely even with two wheels on tarmac and two wheels on wet grass.
- **Dynamic Vehicle Geometry:** Uses yaw rates and vehicle width/length to calculate the exact speed of *each individual wheel hub* during turns, rather than relying on a single center-of-mass speed.

## Results: 10-run straight-line and 3-run cornering campaign (2026-09-05)

Full tables, charts and the raw CSVs are in [results/RESULTS.md](results/RESULTS.md); spreadsheet layout in
[results/RESULTS_SHEET.md](results/RESULTS_SHEET.md) and [results/results.xlsx](results/results.xlsx). Same etk800, same wheels,
tyres and brakes, only the ABS part differs. Every stop is measured by the BrakeTest mod's 2 kHz state machine.

**Straight line, smallgrid, 10 runs per cell (mean g, mean distance):**

| Speed | DynamicABS | Stock ABS | Δ g |
|---|---|---|---|
| 60 mph | 1.196 g, 30.6 m | 1.170 g, 31.3 m | +0.025 |
| 80 mph | 1.207 g, 54.0 m | 1.192 g, 54.7 m | +0.016 |
| 120 mph | 1.204 g, 121.7 m | 1.200 g, 122.2 m | +0.005 |
| 160 mph | 1.212 g, 215.1 m | 1.203 g, 216.8 m | +0.009 |

**Braking while cornering at 60 mph**, brake 0.75 to 1.00 in 0.05 steps, steer 0.05 to 1.00, single corner and
left-then-right double corner, 3 runs per cell, 288 paired stops:

| Metric (paired means) | DynamicABS | Stock ABS | DynamicABS better in |
|---|---|---|---|
| Stopping distance (chord) | 40.5 m | 41.8 m | 197 / 288 |
| Path length | 42.7 m | 43.7 m | 191 / 288 |
| Time to stop | 2.92 s | 2.97 s | 192 / 288 |

DynamicABS wins every paired stop from 0.10 to 0.25 steer (shorter path and shorter time, so it is braking harder,
not just turning more). At 0.50 and 0.75 steer the stock ABS stops about 1 to 3 m shorter and DynamicABS carries
15 to 19° more yaw. Adding traction and stability control to the stock car changed nothing measurable.

![straight](results/straight.png)
![corner single](results/corner_single.png)

## Results: DynamicABS vs stock ABS (2026-09-04)

25 recorded stops on gridmap_v2 with the etk800: asphalt at 30 to 120 mph, ice, grass, sand, 15° and 35° inclines,
a small jump, a bump, and two rough-road sections. Same start point and stop line for both controllers, one run per
stop, standard 2 kHz brake metric (average deceleration from the target speed down to 1 m/s). Bump and jump stops
vary about ±0.05 g from run to run; flat stops repeat within 0.005 g.

| Controller | Mean g (25 stops) | Total stopping distance (m) | Stops shorter than stock | Worst single stop vs stock (g) |
|---|---|---|---|---|
| **DynamicABS, current build (2026-09-04)** | 1.0075 | 888.2 | 13 / 25 | -0.036 |
| Stock ABS (BeamNG built-in) | 0.9965 | 910.8 | 0 / 25 | +0.000 |
| DynamicABS before the bump work (2026-08-31) | 0.9869 | 898.2 | 10 / 25 | -0.145 |

The current build adds fused-speed re-anchoring after pitch events and landings on top of the released controller;
those changes are not yet in this repository.

| Stop | Speed | Stock ABS (g) | DynamicABS (g) | Δ (g) | Stock (m) | DynamicABS (m) |
|---|---|---|---|---|---|---|
| Asphalt A S1 | 60 mph | 1.304 | 1.307 | +0.003 | 28.1 | 28.0 |
| Asphalt A S1 | 90 mph | 1.461 | 1.454 | -0.006 | 56.5 | 56.7 |
| Asphalt A S2 | 60 mph | 1.400 | 1.398 | -0.002 | 26.2 | 26.2 |
| Asphalt A S2 | 90 mph | 1.512 | 1.497 | -0.015 | 54.5 | 55.1 |
| Asphalt B S1 | 30 mph | 1.354 | 1.356 | +0.002 | 6.7 | 6.7 |
| Ice S1 | 40 mph | 0.252 | 0.285 | +0.033 | 64.5 | 57.1 |
| Ice S2 | 40 mph | 0.241 | 0.278 | +0.037 | 67.4 | 58.4 |
| Grass S1 | 60 mph | 0.666 | 0.692 | +0.025 | 55.0 | 52.9 |
| Grass S2 | 60 mph | 0.637 | 0.634 | -0.003 | 57.5 | 57.7 |
| Sand S1 | 60 mph | 0.698 | 0.706 | +0.008 | 52.4 | 51.8 |
| Sand S2 | 60 mph | 0.695 | 0.679 | -0.016 | 52.7 | 53.9 |
| Incline 15° S1 | 45 mph | 1.560 | 1.588 | +0.028 | 13.2 | 13.0 |
| Incline 35° S1 | 45 mph | 1.626 | 1.605 | -0.022 | 12.7 | 12.8 |
| Small jump S1 | 30 mph | 0.572 | 0.614 | +0.041 | 15.9 | 14.9 |
| Small jump S2 | 30 mph | 0.962 | 0.928 | -0.033 | 9.5 | 9.8 |
| Small jump S3 | 30 mph | 0.585 | 0.715 | +0.130 | 15.6 | 12.8 |
| Bump S1 | 40 mph | 0.752 | 0.727 | -0.025 | 21.6 | 22.4 |
| Bump S2 | 40 mph | 1.047 | 1.219 | +0.172 | 15.5 | 13.3 |
| Rough road 1 S1 | 30 mph | 1.115 | 1.130 | +0.014 | 8.2 | 8.1 |
| Rough road 1 S2 | 30 mph | 1.130 | 1.111 | -0.018 | 8.1 | 8.2 |
| Rough road 2 S1 | 25 mph | 0.894 | 0.863 | -0.031 | 7.1 | 7.3 |
| Rough road 2 S2 | 25 mph | 1.219 | 1.183 | -0.036 | 5.2 | 5.3 |
| Rough road 2 S3 | 25 mph | 0.881 | 0.857 | -0.024 | 7.2 | 7.4 |
| Straight S2 | 120 mph | 1.173 | 1.179 | +0.007 | 125.0 | 124.3 |
| Straight S3 | 120 mph | 1.176 | 1.181 | +0.005 | 124.7 | 124.1 |
