# Launcher RPM Tuning Roadmap

This document outlines the plan for developing distance-based launcher RPM targeting.

## Overview

**Goal:** Achieve accurate autonomous shooting from any distance without manual RPM configuration.

**Strategy:** Three-phase iterative approach that lets us compete now while building toward fully autonomous shooting.

---

## Phase 1: Position-Based Tunable RPM (READY NOW)

**Timeline:** Practice 1 (TODAY)
**Effort:** 0 minutes (already implemented)
**Competition Ready:** YES

### What It Does
- Far launch position uses `LaunchAtPositionCommand.PositionRpmConfig.farLaunchRpm` (default: 4200)
- Close launch position uses `LaunchAtPositionCommand.PositionRpmConfig.closeLaunchRpm` (default: 2700)
- Tune these values live in FTC Dashboard without code pushes

### How to Use

**During Practice:**
1. Connect to robot WiFi: `192.168.49.1`
2. Open FTC Dashboard: `http://192.168.49.1:8080/dash`
3. Go to **Config** tab
4. Find `LaunchAtPositionCommand` → `PositionRpmConfig`
5. Adjust `farLaunchRpm` and `closeLaunchRpm` in real-time
6. Run autonomous, observe results, repeat

**Recording Your Findings:**
```
Practice Date: ___________

LAUNCH_FAR Position:
- Starting RPM: 4200
- Final RPM: ______
- Shot Accuracy: ___/10
- Notes: ________________________________

LAUNCH_CLOSE Position:
- Starting RPM: 2700
- Final RPM: ______
- Shot Accuracy: ___/10
- Notes: ________________________________
```

### Quick Test TeleOp (optional)
To test RPMs manually during practice, you can use:
```java
// In DecodeTeleOp, add a test button:
driverPad.dpadUp().whenBecomesTrue(() -> {
    launcherCommands.spinUpToRpm(4200.0); // Test specific RPM
});
```

---

## Phase 2: Data Collection (START Practice 1)

**Timeline:** Practices 1-3 (run during Phase 1 tuning)
**Effort:** 15 min setup + passive collection
**Competition Ready:** N/A (background task)

### What It Does
- Logs every shot: distance, RPM, robot pose, timestamp
- Saves to CSV for offline analysis
- Builds dataset to derive distance→RPM formula

### How to Enable (Optional for Practice 1)

**Option A: Manual Logging After Each Shot**
Just record by hand during practice:
```
Distance (inches) | RPM Used | Made/Missed | Notes
------------------|----------|-------------|------
96"              | 4200     | 8/10        | Consistent high
48"              | 2700     | 9/10        | Good
72"              | 3500     | 6/10        | Test shot
```

**Option B: Automated Logging (15 min setup)**

1. Add `ShotDataLogger` to `Robot.java`:
```java
public final ShotDataLogger shotLogger;

public Robot(HardwareMap hardwareMap) {
    // ... existing init ...
    shotLogger = new ShotDataLogger(logger);
}
```

2. Log shots in autonomous (already partially done):
```java
// After each shot in scoreSequence(), add:
robot.shotLogger.logShotAtPosition(
    "LAUNCH_FAR",
    4200.0, // commanded RPM
    robot.launcher.getCurrentRpm(LauncherLane.LEFT),
    robot.launcher.getCurrentRpm(LauncherLane.RIGHT),
    robot.drive.getFollower().getPose()
);
```

3. Collect CSV files from robot storage after practice

### Data Analysis (After Practice 2-3)

1. Open CSV in Excel/Google Sheets
2. Plot: `Distance (X-axis)` vs `RPM (Y-axis)` vs `Success Rate (color)`
3. Look for relationship:
   - Linear? `RPM = a * distance + b`
   - Quadratic? `RPM = a * distance² + b * distance + c`
4. Use trendline to find formula coefficients

---

## Phase 3: Distance-Based Formula (Practice 3-4)

**Timeline:** After collecting 20-30 shots of data
**Effort:** 1-2 hours implementation
**Competition Ready:** After validation practice

### What It Does
- Calculates optimal RPM from distance to goal
- Uses AprilTag vision for real-time distance
- Falls back to position-based if vision unavailable

### Implementation (Future)

**Template Command (not implemented yet):**
```java
public class LaunchAtDistanceCommand extends Command {

    @Configurable
    public static class FormulaConfig {
        // Fitted from data:
        public static double rpmSlope = 25.0;      // RPM per inch
        public static double rpmIntercept = 1500.0; // Base RPM

        // Safety limits:
        public static double minRpm = 2000.0;
        public static double maxRpm = 5000.0;
    }

    public double calculateRpm(double distanceInches) {
        double rpm = FormulaConfig.rpmSlope * distanceInches
                   + FormulaConfig.rpmIntercept;
        return Range.clip(rpm, FormulaConfig.minRpm, FormulaConfig.maxRpm);
    }
}
```

**Integration Points:**
- Get distance from AprilTag: `VisionSubsystem.getDistanceToTarget()`
- Calculate RPM: `calculateRpm(distance)`
- Set and spin up: `launcher.setLaunchRpm(lane, rpm)`

### Validation Testing
Before using in competition:
- ✅ Test at 5 different distances
- ✅ Compare formula RPM vs manually tuned RPM
- ✅ Verify shot accuracy remains high
- ✅ Ensure fallback to position-based works

---

## Practice Schedule

### **Practice 1 (TODAY)** - 90 minutes
**Goal:** Tune far and close launch RPMs, start data collection

- [ ] **0:00-0:15** - Deploy code, test autonomous runs
- [ ] **0:15-0:45** - Tune `farLaunchRpm` via Dashboard
  - Start at 4200, adjust in 100 RPM increments
  - Record accuracy at each setting
  - Find sweet spot
- [ ] **0:45-1:15** - Tune `closeLaunchRpm` via Dashboard
  - Start at 2700, adjust similarly
- [ ] **1:15-1:30** - Run full autonomous 3x, verify consistency

**Deliverables:**
- Final RPM values for far and close positions
- Hand-written shot log (distance, RPM, accuracy)

---

### **Practice 2** - 90 minutes
**Goal:** Refine tuning, test intermediate distances, expand dataset

- [ ] **0:00-0:30** - Verify Practice 1 settings still work
- [ ] **0:30-1:00** - Test shooting from 3-4 new distances
  - Use `launcherCommands.spinUpToRpm(testRpm)`
  - Try: 60", 72", 84" from goal
- [ ] **1:00-1:30** - Collect 15-20 shots for data analysis

**Deliverables:**
- Expanded shot log with 20+ data points
- Initial distance vs RPM trend observations

---

### **Practice 3** - 90 minutes
**Goal:** Fit formula, implement Phase 3 prototype

- [ ] **Before Practice** - Analyze data, fit trendline
  - Calculate `rpmSlope` and `rpmIntercept`
- [ ] **0:00-0:30** - Implement `LaunchAtDistanceCommand`
- [ ] **0:30-1:00** - Test formula at known distances
- [ ] **1:00-1:30** - Validate vs manual tuning

**Deliverables:**
- Working distance-based formula
- Validation data showing formula accuracy

---

### **Practice 4** - 90 minutes
**Goal:** Integrate with AprilTag vision, polish for competition

- [ ] **0:00-0:30** - Add AprilTag distance calculation
- [ ] **0:30-1:00** - Test shooting from arbitrary positions
- [ ] **1:00-1:30** - Competition practice runs

**Deliverables:**
- Fully autonomous distance-based shooting
- Competition-ready code

---

## Quick Reference

### Current Code Status (Practice 1)
✅ `LaunchAtPositionCommand` - position-based RPM with Dashboard tuning
✅ `SpinUpUntilReadyCommand` - waits for launchers to reach target
✅ `ShotDataLogger` - data collection infrastructure (optional)
⬜ `LaunchAtDistanceCommand` - Phase 3 (not implemented)

### FTC Dashboard Config Locations
- **Phase 1 RPMs:** `LaunchAtPositionCommand` → `PositionRpmConfig`
  - `farLaunchRpm` (default: 4200)
  - `closeLaunchRpm` (default: 2700)
  - `timeoutSeconds` (default: 3.0)
- **Data Logging:** `ShotDataLogger` → `ShotLoggingConfig`
  - `enableShotLogging` (default: true)

### Autonomous Command Usage
```java
// Far autonomous uses:
launcherCommands.spinUpForPosition(FieldPoint.LAUNCH_FAR);

// Close autonomous uses:
launcherCommands.spinUpForPosition(FieldPoint.LAUNCH_CLOSE);

// Manual RPM testing:
launcherCommands.spinUpToRpm(3800.0);
```

---

## Troubleshooting

**Q: Launchers timeout before reaching RPM**
A: Increase `LaunchAtPositionCommand.PositionRpmConfig.timeoutSeconds` in Dashboard

**Q: Shots are inconsistent at same RPM**
A: Check:
- Battery voltage (affects motor power)
- Flywheel wear/debris
- Feeder timing (`LauncherSubsystem.Timing.recoveryMs`)

**Q: Dashboard changes don't persist**
A: Dashboard config is runtime-only. Once you find good values, update the code defaults

**Q: One launcher spins, others don't**
A: Check `LauncherSubsystem` configs - ensure non-zero RPMs for enabled launchers

---

## Future Enhancements (Post-Season)

- **Auto-aim with vision:** Combine distance formula with heading adjustment
- **Trajectory compensation:** Account for robot motion while shooting
- **Multi-point calibration:** Different formulas for different field zones
- **Machine learning:** Neural network for RPM prediction (overkill but cool!)

---

## Questions?

- Check FTC Dashboard: `http://192.168.49.1:8080/dash`
- Review telemetry logs from robot
- Test commands in `DecodeTeleOp` before autonomous

**Good luck at competition! 🚀**
