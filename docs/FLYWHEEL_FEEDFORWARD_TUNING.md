# Flywheel Feedforward Control Tuning Guide

This guide explains how to characterize and tune the feedforward control mode for launcher flywheels.

## What is Feedforward Control?

Feedforward control applies a fixed power level based on the desired velocity, without using feedback from the current velocity. The key advantage is **predictability** - the same target velocity always gets the same power, making behavior consistent and easy to understand.

The feedforward model is a simple linear equation:
```
power = kS + kV × targetRPM
```

Where:
- **kS** (static friction): Minimum power needed to overcome friction and start the motor spinning
- **kV** (velocity gain): Additional power needed per RPM

## When to Use Feedforward Control

**Best for:**
- Flywheels that run at consistent speeds (e.g., always 3000 RPM for launching)
- Systems where you want predictable, repeatable behavior
- Situations where bang-bang oscillation is unacceptable

**Not ideal for:**
- Rapidly changing target velocities
- High precision requirements at very low speeds
- Systems with high friction variability

## Characterization Process

### Step 1: Find kS (Static Friction)

**Goal:** Determine the minimum power needed to overcome friction.

**Method:**
1. Set control mode to `FEEDFORWARD`
2. Create a simple test OpMode that gradually increases power
3. Find the minimum power where the motor just starts to spin consistently
4. Set `kS` to this value (typically 0.05 - 0.15)

**Example test code:**
```java
// In test OpMode
launcher.setDebugOverrideEnabled(true);
double testPower = 0.0;

while (opModeIsActive()) {
    testPower += gamepad1.left_stick_y * 0.01;  // Adjust power
    testPower = Range.clip(testPower, 0.0, 1.0);

    // Apply power directly to motor
    launcher.debugSetLaneTargetRpm(LauncherLane.CENTER, testPower * 6000);

    telemetry.addData("Test Power", "%.3f", testPower);
    telemetry.addData("Current RPM", launcher.getCurrentRpm(LauncherLane.CENTER));
    telemetry.update();
}
```

**What to look for:**
- Motor should just barely start spinning at kS
- Below kS: motor doesn't move
- At kS: motor spins slowly but consistently

### Step 2: Collect Power-to-RPM Data

**Goal:** Measure steady-state RPM at different power levels.

**Method:**
1. Test at 5-7 different power levels (e.g., 0.2, 0.4, 0.6, 0.8, 1.0)
2. For each power level:
   - Apply the power
   - Wait 2-3 seconds for velocity to stabilize
   - Record the steady-state RPM
3. Create a table of (power, RPM) pairs

**Example data collection:**
```
Power | RPM (steady-state)
------|------------------
0.20  | 500
0.40  | 1500
0.60  | 2500
0.80  | 3500
1.00  | 4500
```

### Step 3: Calculate kV (Velocity Gain)

**Goal:** Find the linear relationship between power and RPM.

**Method:**
For each data point, calculate:
```
kV = (power - kS) / rpm
```

Then take the average of all kV values.

**Example calculation:**
```
Assuming kS = 0.10

Data point 1: kV = (0.20 - 0.10) / 500  = 0.0002
Data point 2: kV = (0.40 - 0.10) / 1500 = 0.0002
Data point 3: kV = (0.60 - 0.10) / 2500 = 0.0002
Data point 4: kV = (0.80 - 0.10) / 3500 = 0.0002
Data point 5: kV = (1.00 - 0.10) / 4500 = 0.0002

Average kV = 0.0002
```

**Alternative (linear regression):**
Plot power vs RPM and find the best-fit line. The slope is kV, and the y-intercept is kS.

### Step 4: Validate the Model

**Goal:** Verify that the model produces expected velocities.

**Method:**
1. Set the calculated kS and kV values in FTC Dashboard
2. Test at several target RPMs (e.g., 1000, 2000, 3000, 4000)
3. For each target:
   - Command the target RPM
   - Wait for velocity to stabilize
   - Compare actual RPM to target RPM

**Expected results:**
- Actual RPM should be within 5-10% of target
- If actual is consistently higher: reduce kV slightly
- If actual is consistently lower: increase kV slightly
- If low RPMs are too low: increase kS
- If high RPMs are too high: reduce kS

## Configuration in FTC Dashboard

Once you have kS and kV values:

1. Connect to FTC Dashboard: `http://192.168.49.1:8080/dash`
2. Navigate to **Config** tab
3. Find **LauncherFlywheelConfig** → **modeConfig** → **feedforward**
4. Set your values:
   ```
   kS: 0.10          // Your characterized static friction
   kV: 0.0002        // Your characterized velocity gain
   maxPower: 1.0     // Maximum power limit
   minPower: 0.0     // Minimum power limit
   ```
5. Change **mode** to **FEEDFORWARD**
6. Save configuration

## Fine-Tuning Tips

### If velocity oscillates:
- Feedforward doesn't use feedback, so it shouldn't oscillate
- Check for mechanical issues (bearing friction, belt tension)

### If velocity is too low at all speeds:
- Increase kS slightly (add more base power)
- Or increase kV (steeper power-to-RPM slope)

### If velocity is too high at all speeds:
- Decrease kS (less base power)
- Or decrease kV (shallower slope)

### If low RPMs are good but high RPMs are off:
- Adjust kV only
- If high RPMs too high: reduce kV
- If high RPMs too low: increase kV

### If high RPMs are good but low RPMs are off:
- Adjust kS only
- If low RPMs too low: increase kS
- If low RPMs too high: decrease kS

## Advanced: Non-Linear Models

If the power-to-RPM relationship is not linear (curve in your data), you can:

1. **Use different kV for different RPM ranges:**
   - Low RPM (0-2000): kV = 0.00025
   - High RPM (2000+): kV = 0.00018

2. **Add a quadratic term:**
   - `power = kS + kV × rpm + kV2 × rpm²`
   - Requires code modification

3. **Use a lookup table with interpolation:**
   - Store (RPM, power) pairs
   - Interpolate between nearest neighbors
   - Most accurate but more complex

For most FTC applications, the simple linear model works well!

## Troubleshooting

### Motor doesn't spin at all
- Check kS is high enough (try 0.15)
- Verify motor is not mechanically jammed
- Check motor direction is correct

### Velocity is very unstable
- Battery voltage may be low (feedforward uses voltage compensation)
- Check for loose mechanical connections
- Verify encoder is working properly

### Different velocities at same power on different runs
- Battery voltage varying (check voltage compensation is enabled)
- Motor warming up (run for 30s before characterization)
- Friction changing (check bearings and belts)

### Model doesn't match reality
- Re-characterize with fresh battery (>12.5V)
- Collect more data points (10+ instead of 5)
- Check for measurement errors in RPM readings
- Verify you're measuring steady-state RPM (wait 2-3s)

## Comparison to Other Control Modes

| Mode | Pros | Cons |
|------|------|------|
| **FEEDFORWARD** | Predictable, no oscillation, simple | No correction for disturbances |
| **PURE_BANG_BANG** | Fast response, no tuning needed | Oscillates around target |
| **HYBRID** | Good accuracy, handles disturbances | Needs PID tuning, more complex |
| **BANG_BANG_HOLD** | Fast + stable, moderate complexity | Needs hold power tuning |

## Next Steps

After characterization:
1. Test in real match scenarios
2. Verify consistency across multiple battery charges
3. Check performance with game pieces being launched
4. Consider adding a small feedback term if needed (hybrid feedforward + P control)
