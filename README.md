# ABS-2F

A per-wheel PID anti-lock braking controller for BeamNG.drive.

ABS-2F slots into 21 vanilla vehicles automatically. Enable it from the Parts menu under the vehicle's ABS slot.

## Install

1. Download `ABS-2F.zip` from the [Releases](../../releases) page.
2. Drop the zip into your BeamNG mods folder:
   ```
   %LOCALAPPDATA%\BeamNG.drive\<version>\mods\
   ```
   (e.g. `C:\Users\YourName\AppData\Local\BeamNG.drive\0.36\mods\`)
3. Launch BeamNG, spawn a supported vehicle, open the Parts menu, and select **ABS-2F** under the vehicle's Anti-Lock Braking System slot.
4. Optional: enable the **ABS Grip Gauges** UI app from the in-game app manager to see live per-wheel surface μ, slip ratios, and fused speeds.

## Supported vehicles (vanilla BeamNG 0.36)

| Category | Vehicle |
|---|---|
| Sedan / Coupe | Ibishu 200BX, Ibishu Covet, Ibishu Pessima, ETK 800-Series, ETK I-Series, ETK K-Series, Gavril Grand Marshal, Bruckell LeGran, Bruckell Moonhawk / Nine, Hirochi Sunburst 2, Soliad Lansdale, Soliad Wendover |
| Sport | Hirochi Bastion, Hirochi SBR4, Hirochi Scintilla, Hirochi Sunburst 2 DSE, Soliad Vivace |
| SUV / Truck | Ibishu Hopper, Gavril D-Series, Rock Bouncer, Gavril H-Series |

Because slot targeting is handled entirely through BeamNG's slot system, this mod does **not** overwrite any vanilla files and will not conflict with other vehicle mods.

## How it works

- **Per-wheel PID** on measured slip ratio. Each wheel runs its own KP/KI/KD loop against a shared slip target.
- **Adaptive surface μ estimator** (`consensusD`) derives the peak achievable deceleration from a rolling 40-sample chassis decel window, retro-resets on large step changes, and adjusts the slip target accordingly.
- **Active surface probing** briefly forces full brake on all four wheels every ~1 second under hard braking. If measured decel exceeds baseline (loose surface: locked rubber plows better than it slides), it bumps the slip target up.
- **Bosch MIR / yaw-moment limiter** caps the per-tick rate-of-rise on the high-grip side of a split-μ surface, reducing yaw moment without sacrificing stopping distance on uniform surfaces.
- **Fused-speed estimation** combines 2 kHz accelerometer integration, wheel-speed consensus, and a 5-second physics-based plausibility check that rejects wheelspin spikes from ever reaching the speed estimate.
- **Reverse-aware + throttle-slip aware.** Handles BeamNG's arcade brake-key-means-throttle routing and computes wheelspin slip continuously for instrumentation.

## Caveat on tuning

PID gains (`KP=6.0`, `KI=0.8`, `KD=0.08`) and the slip-target formula (`0.04 + μ × 0.10`) were originally tuned on the ETK 800-Series. All 21 supported vehicles will function — per-wheel PID is robust — but heavier vehicles (Grand Marshal, Pessima) or race-tuned ones (Scintilla, SBR4) may not achieve peak braking efficiency. Not a bug, just physics.

## Credits

- Brake-torque application via the `abstelemetry.lua` vehicle extension (included).
- ABS Grip Gauges UI app (included).

## License

MIT. See [LICENSE](LICENSE).
