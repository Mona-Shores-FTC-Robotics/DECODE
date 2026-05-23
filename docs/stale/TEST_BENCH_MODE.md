# Test Bench Mode

Quick guide for testing autonomous paths without expansion hub hardware.

## How to Enable

**Option 1: Via FTC Dashboard** (Easiest - no code push needed)
1. Connect to robot/test bench: `192.168.49.1`
2. Open FTC Dashboard: `http://192.168.49.1:8080/dash`
3. Go to **Config** tab
4. Find `Robot` → `TestBenchConfig`
5. Set `testBenchMode` to `true`
6. Click **Save Config**

**Option 2: In Code** (Permanent until changed)
```java
// In Robot.java, change:
public static boolean testBenchMode = true;  // Enable test bench mode
```

## What Test Bench Mode Does

**Skips initialization of:**
- ✅ Launcher subsystem (3 motors + 3 servos on expansion hub)
- ✅ Intake subsystem (motors on expansion hub)
- ✅ Lighting subsystem (LEDs)

**Still initializes:**
- ✅ Drive subsystem (for path building and visualization)
- ✅ Vision subsystem (for AprilTag detection)
- ✅ Pedro Pathing (for autonomous paths)

## What Works in Test Bench Mode

✅ **Autonomous Path Visualization**
- Run `DecodeAutonomousFarCommand` or `DecodeAutonomousCloseCommand`
- See paths on FTC Dashboard: `http://192.168.49.1:8080/dash`
- Test alliance selection (Red/Blue)
- Verify start poses and waypoints

✅ **Alliance Detection**
- AprilTag-based alliance detection via Limelight
- Manual alliance selection with dpad left/right

✅ **Path Previews**
- Press Left Bumper: Preview Blue alliance paths
- Press Right Bumper: Preview Red alliance paths

## What Doesn't Work

❌ Launcher commands (will be null)
❌ Intake commands (will be null)
❌ Lighting patterns
❌ Actual robot movement (no motors)

## Typical Test Bench Workflow

1. **Enable test bench mode** via Dashboard
2. **Select autonomous opmode** (e.g., "Decode Auto Far (Command)")
3. **Init phase:**
   - Select alliance (dpad left/right or let vision detect)
   - Press Y to apply alliance
   - Press Left/Right Bumper to preview paths
   - Verify start pose is correct
4. **Start:**
   - Autonomous will build paths but won't execute movement
   - Watch FTC Dashboard for path visualization
   - Check telemetry for pose updates

## Troubleshooting

**Q: "ERROR: Could not find motor/servo"**
A: Make sure `testBenchMode = true` in Dashboard Config

**Q: "NullPointerException in autonomous"**
A: Code is trying to use launcher/intake. These are null in test bench mode.
   - Check if your autonomous code has null checks for test commands

**Q: "Can't see paths on Dashboard"**
A:
- Verify connected to `192.168.49.1:8080/dash`
- Check that **Field** view is enabled in Dashboard
- Ensure autonomous opmode is running

**Q: "Want to go back to full robot mode"**
A: Set `testBenchMode = false` in Dashboard Config, or change in code and rebuild

## Hardware Config for Test Bench

**Minimum hardware needed:**
- Control Hub (with WiFi)
- REV Hub (for drive motors if you want to test drive)
- Optional: Limelight for vision testing
- Optional: Pinpoint odometry for pose tracking

**NOT needed:**
- Expansion Hub (launcher, intake, lighting live here)

## Path Testing Checklist

When testing autonomous paths on bench:

- [ ] Test bench mode enabled (`testBenchMode = true`)
- [ ] OpMode initializes without errors
- [ ] Can select alliance (Red/Blue)
- [ ] Can preview paths with bumpers
- [ ] Dashboard shows field visualization at `192.168.49.1:8080/dash`
- [ ] Start poses correct for both alliances
- [ ] Path waypoints visible and reasonable
- [ ] No NullPointerExceptions during path building

## Disabling Test Bench Mode

**Before deploying to robot:**
1. Go to FTC Dashboard Config
2. Set `testBenchMode = false`
3. Or rebuild code with `testBenchMode = false` in Robot.java
4. Verify launcher, intake, lighting initialize correctly

---

**Remember:** Test bench mode is ONLY for path visualization and testing. Always disable it before competition!
