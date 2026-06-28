# BeamNG Dynamic ABS

An advanced, adaptive anti-lock braking controller for BeamNG.drive that actively responds to changing conditions. 

### What makes it Dynamic?
Unlike standard static ABS controllers, this system:
- **Fuses Telemetry Data:** Combines wheel speeds with IMU (accelerometer) data to calculate the true ground speed of the vehicle, even when all four wheels are locked or airborne.
- **Adapts to Surface Grip:** Continuously estimates the current surface friction (D-Estimator) and dynamically adjusts the target wheel slip. It knows the difference between asphalt and ice and changes its braking strategy accordingly.
- **Per-Wheel Optimization:** Each wheel runs its own independent PID control loop and grip estimator, meaning you can brake safely even with two wheels on tarmac and two wheels on wet grass.
- **Dynamic Vehicle Geometry:** Uses yaw rates and vehicle width/length to calculate the exact speed of *each individual wheel hub* during turns, rather than relying on a single center-of-mass speed.
