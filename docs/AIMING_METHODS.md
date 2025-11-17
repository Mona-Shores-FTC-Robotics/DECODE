# Aiming Methods - DECODE 2025

## Overview

Three aiming methods are available for testing and comparison. Each approach has different strengths and weaknesses.

## Current Button Configuration

- **B button (hold)**: Geometry-based aiming (default)
- **A button**: Vision relocalization

## Three Aiming Methods

### 1. Geometry-Based Aiming (Current Default)

**Command:** `AimAndDriveCommand`
**Method:** `DriveSubsystem.aimAndDrive()`
**Button:** B (hold)

**How it works:**
- Calculates angle from robot pose to basket centroid using `atan2(dy, dx)`
- Uses odometry pose and tunable basket target coordinates
- Continuously tracks target every loop

**Basket coordinates (tunable via FTC Dashboard → FieldConstants → BasketTargets):**
- Blue: (8.33, 134.33) - centroid of triangular basket
- Red: (135.67, 134.33)

**Pros:**
- Can aim from anywhere on field
- Smooth continuous tracking as you move
- Works without AprilTag visibility (uses odometry)

**Cons:**
- Requires accurate odometry
- Requires accurate field coordinates
- Coordinate frame issues can cause wrong angles
- Sensitive to odometry drift

**Tuning parameters (FTC Dashboard → DriveSubsystem → AimAssistConfig):**
- `kP`: 0.5 (proportional gain)
- `kMaxTurn`: 0.7 (max turn speed)

---

### 2. Vision-Centered Aiming (NEW)

**Command:** `AimAndDriveVisionCenteredCommand`
**Method:** `DriveSubsystem.aimAndDriveVisionCentered()`
**Button:** Not yet bound (see binding instructions below)

**How it works:**
- Uses Limelight `tx` (horizontal offset in degrees) to center AprilTag in camera view
- When tx = 0, target is centered
- tx > 0 means target is to the right → turn clockwise
- tx < 0 means target is to the left → turn counter-clockwise
- Similar to original FTC `RobotAutoDriveToAprilTagOmni` example

**Pros:**
- Self-correcting (always aims at what camera sees)
- No coordinate frame issues
- No odometry dependency
- Simple and intuitive

**Cons:**
- Requires AprilTag visibility
- If tag not visible, robot holds current heading (no turn)
- Camera must be properly aligned with robot

**Tuning parameters (FTC Dashboard → DriveSubsystem → VisionCenteredAimConfig):**
- `kP`: 0.03 (turn per degree of tx offset)
- `kMaxTurn`: 0.7 (max turn speed)
- `deadbandDeg`: 1.0 (stop turning when tx < 1 degree)

---

### 3. Fixed-Angle Aiming (NEW)

**Command:** `AimAndDriveFixedAngleCommand`
**Method:** `DriveSubsystem.aimAndDriveFixedAngle()`
**Button:** Not yet bound (see binding instructions below)

**How it works:**
- Rotates to a fixed heading based on alliance
- Default: 60° for blue, 120° for red
- Driver positions robot on launch line, presses button, robot aims to fixed angle
- No vision or pose calculations needed

**Pros:**
- Simplest method
- Predictable and consistent
- Easy for drivers to understand
- No vision or odometry needed

**Cons:**
- Only works from specific field positions (launch line)
- No dynamic tracking - fixed angle only
- If robot moves, angle doesn't update

**Tuning parameters (FTC Dashboard → DriveSubsystem → FixedAngleAimConfig):**
- `blueHeadingDeg`: 60.0 (blue alliance fixed angle)
- `redHeadingDeg`: 120.0 (red alliance fixed angle)
- `kP`: 0.5 (proportional gain)
- `kMaxTurn`: 0.7 (max turn speed)

---

## Testing Instructions

### Option 1: Quick Test (Change Default Binding)

Edit `DriverBindings.java` to test different methods on B button:

```java
// Test geometry-based (current default)
aimAndDrive = new AimAndDriveCommand(
    fieldX::get,
    fieldY::get,
    slowHold::get,
    robot.drive
);

// OR test vision-centered
aimAndDrive = new AimAndDriveVisionCenteredCommand(
    fieldX::get,
    fieldY::get,
    slowHold::get,
    robot.drive
);

// OR test fixed-angle
aimAndDrive = new AimAndDriveFixedAngleCommand(
    fieldX::get,
    fieldY::get,
    slowHold::get,
    robot.drive
);
```

### Option 2: Bind All Three for Comparison

Add additional buttons to `DriverBindings.java`:

```java
// In configureTeleopBindings():

// B button: Geometry-based (current)
aimAndDrive = new AimAndDriveCommand(
    fieldX::get,
    fieldY::get,
    slowHold::get,
    robot.drive
);
aimHold.whenBecomesTrue(aimAndDrive)
        .whenBecomesFalse(aimAndDrive::cancel);

// X button: Vision-centered
Command aimVisionCentered = new AimAndDriveVisionCenteredCommand(
    fieldX::get,
    fieldY::get,
    slowHold::get,
    robot.drive
);
driver.x().whenBecomesTrue(aimVisionCentered)
         .whenBecomesFalse(aimVisionCentered::cancel);

// Y button: Fixed-angle
Command aimFixedAngle = new AimAndDriveFixedAngleCommand(
    fieldX::get,
    fieldY::get,
    slowHold::get,
    robot.drive
);
driver.y().whenBecomesTrue(aimFixedAngle)
         .whenBecomesFalse(aimFixedAngle::cancel);
```

---

## Tuning via FTC Dashboard

All parameters are tunable live during matches via FTC Dashboard:

1. Connect to robot WiFi
2. Open browser: `http://192.168.49.1:8080/dash`
3. Go to **Config** tab
4. Find relevant sections:
   - **FieldConstants → BasketTargets**: Adjust basket coordinates
   - **DriveSubsystem → AimAssistConfig**: Geometry-based gains
   - **DriveSubsystem → VisionCenteredAimConfig**: Vision-centered gains
   - **DriveSubsystem → FixedAngleAimConfig**: Fixed angles

Changes take effect immediately (no code recompile needed).

---

## Troubleshooting

### Geometry-Based Issues

**Problem:** Robot aims at wrong angle consistently
**Fix:** Adjust basket Y coordinate in `FieldConstants.BasketTargets.blueY` / `redY`
- If aiming too far back: Decrease Y (try 130-132)
- If aiming too far forward: Increase Y (try 136-138)
- Current centroid: Y = 134.33

**Problem:** Robot oscillates around target
**Fix:** Decrease `AimAssistConfig.kP` (try 0.3-0.4)

**Problem:** Robot turns too slowly
**Fix:** Increase `AimAssistConfig.kP` (try 0.6-0.8)

### Vision-Centered Issues

**Problem:** Robot doesn't turn at all
**Check:**
- AprilTag visible? (check telemetry for tx value)
- Limelight connected? (check telemetry)
- Try increasing `VisionCenteredAimConfig.kP` to 0.05

**Problem:** Robot turns wrong direction
**Fix:** The sign might need flipping - edit DriveSubsystem.java line 490:
```java
// Try changing from:
double turn = Range.clip(-txDegrees * visionCenteredAimConfig.kP, -maxTurn, maxTurn);
// To:
double turn = Range.clip(txDegrees * visionCenteredAimConfig.kP, -maxTurn, maxTurn);
```

**Problem:** Robot oscillates
**Fix:**
- Decrease `VisionCenteredAimConfig.kP` (try 0.02)
- Increase `deadbandDeg` (try 2.0-3.0)

### Fixed-Angle Issues

**Problem:** Wrong angle for launch line
**Fix:** Adjust `FixedAngleAimConfig.blueHeadingDeg` / `redHeadingDeg` via FTC Dashboard
- Measure the correct angle from field
- Update config values
- Test and iterate

---

## Recommendations

1. **Start with fixed-angle** - simplest, easiest to verify correct angle
2. **Then test vision-centered** - should work well if AprilTag visible
3. **Finally test geometry-based** - most complex, may need coordinate tuning

**For matches:**
- Use whichever method proves most reliable in testing
- Keep fixed-angle as emergency backup
- Consider having multiple methods available on different buttons

---

## Projectile Arc Considerations

Your launcher shoots with an arc. The basket centroid (134.33) may not be optimal:

- **High arc:** Aim might need to be further back (Y > 134.33)
- **Low/flat arc:** Aim might need to be at front edge (Y < 134.33)

**Testing approach:**
1. Use fixed-angle method to find best angle from launch line
2. Once you know the correct angle, reverse-calculate what Y coordinate produces that angle
3. Update `BasketTargets.blueY` / `redY` accordingly
4. Then geometry-based should work correctly

**Example calculation:**
If you're at position (72, 100) and need to aim at 60°:
- tan(60°) = dy / dx
- dy = dx * tan(60°)
- targetY = robotY + dy
- targetY = 100 + (targetX - 72) * tan(60°)
- Work backwards to find targetY that gives desired angle
