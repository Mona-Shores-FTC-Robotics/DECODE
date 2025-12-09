# DECODE Todo List - Analysis & Organization
**Generated**: 2025-12-09
**Branch Analysis**: Last 48 hours of commits on current branch

## Executive Summary

Recent development has been highly productive, with **7 out of 10** items either completed or significantly progressed. The team has focused on autonomous improvements, sensor tuning, telemetry enhancements, and launcher configuration.

---

## ✅ Recently Implemented Features (Last 48 Hours)

### 1. **Visual Indicator for DECODE Mode Switch** ✅ COMPLETE
- **Implementation**: `LightingSubsystem.showDecodeModeSwitchNotification()` (line 348)
- **Trigger**: Auto-switches at 50 seconds remaining in TeleOp (line 192 in DecodeTeleOp.java)
- **Visual**: 2-second rainbow flash pattern
- **Commits**:
  - `e78f5af` - Fix TeleOp duration and DECODE switch at 50s
  - `5a8a259` - Fix DECODE mode not working during TeleOp
- **Status**: Fully functional, properly integrated

### 2. **Motif Pattern Telemetry & D-Pad Detection** ✅ COMPLETE
- **Implementation**: Motif pattern added to MATCH-level telemetry
- **Detection**: D-pad right button (`detectMotifButton`) for manual detection when auto fails
- **Location**: `OperatorBindings.java:85` (dpad right binding)
- **Commits**:
  - `4823620` - Add motif pattern to MATCH telemetry and dpad-right detection
  - `0259feb` - Add launcher mode and motif tail to MATCH telemetry
- **Status**: Complete with haptic feedback

### 3. **Motif Tail Control Training** ✅ IMPLEMENTED (Training Pending)
- **Implementation**: Left stick quick-set controls (left=0, up=1, right=2)
- **Location**: `OperatorBindings.java:101-103`
- **Visual Feedback**: Uses lighting system
- **Status**: Code complete, students need training (non-code task)

### 4. **Auto Eject Timing** ✅ SIGNIFICANTLY IMPROVED
- **New Command**: `TimedEjectCommand.java` - timed eject sequence (default 1200ms)
- **Pattern**: Strategic eject+intake sequence applied across autonomous commands
- **Implementation**: Used in `CloseThreeAtOnceCommand` and propagated to other autos
- **Configuration**: Per-auto tunable eject times (e.g., `Config.ejectTime = 1200`)
- **Commits**:
  - `aaf4fb1` - Add TimedEjectCommand for timed eject sequence
  - `5cfc877` - Apply strategic eject+intake pattern from CloseThreeAtOnce to other autos
- **Status**: Working well in practice, may need minor timing adjustments

### 5. **Lane Sensor Distance Threshold Retuning** ✅ COMPLETE
- **Implementation**: Per-robot, per-lane threshold configuration
- **Configs**:
  - Robot 19429: Left=9.0cm, Center=7.0cm, Right=5.0cm
  - Robot 20245: Left=4.0cm, Center=4.0cm, Right=4.5cm
- **Location**: `IntakeLaneSensorConfig.java:184-198`
- **Additional**: Hue filter window increased to 10 samples for better stability
- **Commits**:
  - `127a00e` - practice 12-8-25 (sensor config updates)
  - `77a89e2` - hue smoothing to make color detection more reliable
  - `13f2e00` - Add circular moving average filter for hue values
- **Status**: Tuned and stable

### 6. **Hood and RPM Tuning** 🔄 IN PROGRESS
- **Files Modified**:
  - `LauncherFlywheelConfig.java`
  - `LauncherHoodConfig.java`
  - `LauncherTimingConfig.java`
  - `LauncherReverseIntakeConfig.java`
- **Recent Work**: Active tuning for new hood extension
- **Commit**: `b495b04` - working on launcher (12 hours ago)
- **Status**: Ongoing - needs final decisions on arc vs drill-shot style

### 7. **Speed-Gated Per-Lane Hood Timing** ✅ COMPLETE
- **Feature**: Hood retracts independently per lane when flywheel reaches threshold RPM
- **Purpose**: Optimized for human loading
- **Commit**: `489788b` - Add speed-gated per-lane hood timing for human loading
- **Status**: Implemented and functional

### 8. **Relocalization Confidence Indicator** ✅ ALREADY EXISTS
- **Implementation**: `DriveSubsystem.visionRelocalizeStatus` (line 138)
- **Display**: Shows status for 2 seconds after relocalization event
- **Location**: `DecodeTeleOp.java:274-281` (relocalizeTelemetry method)
- **Format**: Displays text status like "Vision re-localize: [status]"
- **Status**: Already implemented, may need enhancement for "confidence" vs "status"

---

## 📋 Outstanding TODO Items

### HIGH PRIORITY

#### 1. **Finalize Hood & RPM Values After New Extension** 🔴 CRITICAL
- **Reason**: Recent hardware changes require tuning decisions
- **Action Items**:
  - [ ] Test arc-shot trajectory vs drill-shot at competition distances
  - [ ] Measure consistency at SHORT/MID/LONG ranges
  - [ ] Lock in final `LauncherFlywheelConfig` RPM values
  - [ ] Lock in final `LauncherHoodConfig` positions
  - [ ] Document final choices in config comments
- **Blocking**: Competition readiness
- **Estimated Effort**: 2-3 practice sessions

#### 2. **Enhance Relocalization Confidence Indicator** 🟡 MEDIUM
- **Current State**: Text-only status message (2-second timeout)
- **Enhancement Options**:
  - Add numerical confidence score (0-100%)
  - Add visual LED indicator (green=high, yellow=medium, red=low)
  - Show distance/angle to detected tag
  - Display tag count in view
- **Action Items**:
  - [ ] Define confidence metric (decision margin? tag count?)
  - [ ] Expose confidence from `VisionSubsystemLimelight`
  - [ ] Add to telemetry output
  - [ ] Optional: Add lighting pattern for low confidence
- **Estimated Effort**: 2-4 hours

### MEDIUM PRIORITY

#### 3. **Early Detection for Smart Intake in Auto** 🟡 PARTIALLY DONE
- **Current State**: `fullCountThreshold = 3` in `AutoSmartIntakeCommand`
- **Original Goal**: "Treat one artifact as full in auto"
- **Gap**: Threshold is still 3, not 1
- **Considerations**:
  - Threshold of 1 may cause premature reversals
  - Current threshold of 3 working well in practice
  - May need separate auto vs teleop thresholds
- **Action Items**:
  - [ ] Test with `fullCountThreshold = 1` in autonomous
  - [ ] Measure false positive rate and jamming incidents
  - [ ] Consider separate `AutoSmartIntakeConfig` for auto vs teleop
  - [ ] If threshold=1 works, document the reasoning
- **Estimated Effort**: 1-2 hours testing + tuning

#### 4. **Fix Auto Eject Timing to Stop Before New Pickups** 🟢 MOSTLY DONE
- **Current State**: `TimedEjectCommand` stops intake after configured duration
- **Implementation**: Used in autonomous sequences with strategic timing
- **Remaining Work**:
  - [ ] Verify eject completes before next `AutoSmartIntakeCommand` starts
  - [ ] Check for any overlap in `CloseThreeAtOnceCommand` sequences
  - [ ] Test at competition with full battery (faster movement)
- **Estimated Effort**: 30 minutes verification

### LOW PRIORITY (Non-Code Tasks)

#### 5. **Train Students on Motif Tail Controls** 📚 TRAINING
- **Implementation**: Complete (left stick: left=0, up=1, right=2)
- **Action Items**:
  - [ ] Create driver cheat sheet
  - [ ] Practice sessions with operators
  - [ ] Document in team playbook
- **Estimated Effort**: 1 practice session

#### 6. **Train Students on D-Pad Motif Viewer** 📚 TRAINING
- **Implementation**: Complete (D-pad right to detect motif when auto fails)
- **Action Items**:
  - [ ] Add to driver troubleshooting guide
  - [ ] Practice failure recovery scenarios
  - [ ] Document when to use (after auto crash/timeout)
- **Estimated Effort**: 1 practice session

### OPTIMIZATION (Future Consideration)

#### 7. **Staggered Sensor Polling to Reduce Control Hub Load** 🔵 FUTURE
- **Current State**: Sensors polled at fixed intervals
  - Color sensors: 100ms (`IntakeLaneSensorConfig.Polling.samplePeriodMs`)
  - Limelight: 50ms throttle (I2C)
- **Potential Improvement**: Offset polling windows to reduce instantaneous load
- **Priority**: Low - current loop times are acceptable (<25ms target)
- **Action Items**:
  - [ ] Profile current loop timing under load (use RT diagnostic mode)
  - [ ] Only implement if loop times consistently exceed 20ms
  - [ ] Consider asynchronous polling with cached results
- **Estimated Effort**: 4-6 hours (if needed)

---

## 🎯 Recommended Priority Order

### **Before Next Competition** (This Week)
1. **Finalize hood & RPM values** 🔴 - Blocking competition readiness
2. **Verify auto eject timing** 🟢 - Quick safety check
3. **Test early detection threshold** 🟡 - May improve auto consistency
4. **Train students on controls** 📚 - Essential for operators

### **Nice to Have** (Next Week)
5. **Enhance relocalization confidence display** 🟡 - Better driver awareness
6. **Profile sensor polling** 🔵 - Only if performance issues arise

---

## 📊 Progress Metrics

- **Completed**: 7/10 items (70%)
- **In Progress**: 1/10 items (10%)
- **Outstanding**: 2/10 items (20%)
- **Code Complete**: 8/10 items (80%)
- **Training Required**: 2/10 items (20%)

---

## 🔍 Missing Items Analysis

**Were there any important changes you might have missed?**

After reviewing all commits in the last 48 hours, I noticed several related improvements that weren't on your original list:

1. **Telemetry System Simplification** ✅ (commits: `60d0b00`, `59b6bbd`, `50148ad`)
   - Reduced to MATCH/DEBUG only (removed PRACTICE level)
   - Safe-by-default to prevent disconnects
   - Compile-time `TelemetryLevel` setting

2. **Expansion Hub Stop Error Fix** ✅ (commits: `2003329`, `e08eb7b`, `8575525`)
   - Wrapped hardware calls in try-catch during shutdown
   - Pose handoff moved to beginning of `onStop()`
   - Applied across all autonomous OpModes

3. **Alliance Default Improvement** ✅ (commit: `62a2698`)
   - TeleOp now defaults to BLUE instead of UNKNOWN
   - Prevents "can't shoot" scenarios when auto handoff fails

4. **Intake Control Remapping** ✅ (commit: `8135c01`)
   - Smart intake: right bumper + right trigger
   - Regular intake: left trigger only
   - Better ergonomics for operators

5. **Eject Button Addition** ✅ (commits: `64932cf`, `077307f`)
   - Operator square button now triggers eject
   - Manual control for stuck artifacts

These are all solid improvements that support your main objectives. No critical gaps identified.

---

## 📝 Notes

- The codebase is in excellent shape with recent stability improvements
- Hood/RPM tuning is the only blocking item before competition
- Most "TODOs" are actually training/documentation tasks at this point
- Consider this list as living documentation - update after each practice session

---

## 🔗 Key Files for Reference

- **Launcher Config**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/launcher/config/`
- **Intake Config**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/intake/config/IntakeLaneSensorConfig.java`
- **TeleOp**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/DecodeTeleOp.java`
- **Operator Bindings**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/bindings/OperatorBindings.java`
- **Auto Commands**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/Autos/Commands/`
