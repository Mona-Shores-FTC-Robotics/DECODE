# @AutoLogOutput Audit

**Date:** 2025-11-14
**Current Count:** 64 @AutoLogOutput methods in production subsystems (excluding demos/tests)

## Summary

The current telemetry logging includes significant redundancy and low-value fields that contribute to loop time overhead. This audit identifies essential fields vs debug-only fields.

## Breakdown by Subsystem

### IntakeSubsystem (28 methods → recommend 10)

**KEEP (Essential - 7 methods):**
- `getLeftColor()` - Artifact color in left lane (critical for drivers)
- `getCenterColor()` - Artifact color in center lane (critical for drivers)
- `getRightColor()` - Artifact color in right lane (critical for drivers)
- `getCommandedMode()` - Requested intake mode (critical state)
- `getAppliedMode()` - Actual intake mode (critical state)
- `getMotorPower()` - Intake motor power (useful diagnostics)
- `isLoggedRollerActive()` - Roller active state (critical)

**KEEP (Useful for tuning - 3 methods):**
- `getLeftDetectedColor()` - Raw sensor reading (useful for color sensor debugging)
- `getCenterDetectedColor()` - Raw sensor reading (useful for color sensor debugging)
- `getRightDetectedColor()` - Raw sensor reading (useful for color sensor debugging)

**REMOVE (Low value - 18 methods):**
- `isRollerPresent()` - Hardware presence check (static, not useful during match)
- `getAllianceString()` - Redundant (alliance logged at OpMode level)
- `isAnyLaneSensorsPresent()` - Hardware check (not useful during match)
- `isSensorPollingEnabled()` - Config check (always enabled)
- `getSensorSamplePeriodMs()` - Config value (doesn't change)
- `isLeftSensorPresent()` - Hardware check (static)
- `isLeftWithinDistance()` - Low-level sensor detail (noisy)
- `isCenterSensorPresent()` - Hardware check (static)
- `isCenterWithinDistance()` - Low-level sensor detail (noisy)
- `isRightSensorPresent()` - Hardware check (static)
- `isRightWithinDistance()` - Low-level sensor detail (noisy)
- `isModeOverrideEnabled()` - Debug flag (rarely used)
- `getModeOverride()` - Debug value (rarely used)
- `getLoggedRollerPosition()` - Servo position (not actionable)
- `getModeResolveMs()` - Timing breakdown (now captured in main loop timing)
- `getSensorPollMs()` - Timing breakdown (now captured in main loop timing)
- `getServoUpdateMs()` - Timing breakdown (now captured in main loop timing)
- `getPeriodicTotalMs()` - Timing breakdown (now captured in main loop timing)

### LauncherSubsystem (16 methods → recommend 7)

**KEEP (Essential - 4 methods):**
- `getLogState()` - Launcher state machine state (critical)
- `getLogBusy()` - Whether launcher is executing (critical)
- `getLogQueuedShots()` - Number of queued shots (critical)
- `getLogLeftReady()` + `getLogCenterReady()` + `getLogRightReady()` - Per-lane ready status (drivers need this)
  → Actually 3 methods, so 6 essential total

**KEEP (Useful for tuning - 1 method):**
- `getLogAverageTargetRpm()` - Target RPM across all lanes (useful for verifying setpoints)

**REMOVE or make DEBUG-only (9 methods):**
- `getLogRequestedSpinMode()` - Spin mode (rarely changes, debug only)
- `getLogEffectiveSpinMode()` - Effective spin mode (debug only)
- `getLogControlMode()` - Control mode (rarely changes)
- `getLogStateElapsedSec()` - Time in state (low value)
- `getLogActiveShotLane()` - Which lane shooting (transient, low value)
- `getLogActiveShotAgeMs()` - Shot age (transient, low value)
- `getLogLastShotCompletionMs()` - Last shot timestamp (low value)
- `getLogAverageCurrentRpm()` - Current RPM (useful for tuning but noisy)
- `getLogAveragePower()` - Average power (useful for tuning but noisy)

### LauncherCoordinator (14 methods → recommend 2)

**KEEP (Essential - 2 methods):**
- `getArtifactStateString()` - Artifact state (NONE/PARTIAL/FULL)
- `getArtifactCountLogged()` - Total artifact count (critical for drivers)

**REMOVE (Redundant - 12 methods):**
- `getLeftColor()` - **Duplicate** of IntakeSubsystem.getLeftColor()
- `getCenterColor()` - **Duplicate** of IntakeSubsystem.getCenterColor()
- `getRightColor()` - **Duplicate** of IntakeSubsystem.getRightColor()
- `getLauncherState()` - **Duplicate** of LauncherSubsystem.getLogState()
- `getLauncherSpinMode()` - **Duplicate** of LauncherSubsystem spin mode methods
- `getLauncherQueuedShots()` - **Duplicate** of LauncherSubsystem.getLogQueuedShots()
- `getIntakeRequestedMode()` - **Duplicate** of IntakeSubsystem.getCommandedMode()
- `getIntakeAppliedModeString()` - **Duplicate** of IntakeSubsystem.getAppliedMode()
- `getAnyActiveLanes()` - Derivable from artifact count
- `getLightingRegistered()` - Internal debug flag (not useful)
- `getIntakeAutomationEnabled()` - Always true after RobotMode removal
- `getIntakeOverrideActive()` - Debug flag (low value)

### VisionSubsystemLimelight (4 methods → keep all 4)

**KEEP (All essential):**
- `getVisionState()` - Vision state machine (critical)
- `getHasValidTag()` - Tag detection status (critical)
- `getLoggedCurrentTagId()` - Which AprilTag detected (critical)
- `getTimeSinceLastSeenMs()` - Staleness indicator (useful)

Vision is already minimal and focused.

## Proposed Reduction

| Subsystem | Current | Proposed | Reduction |
|-----------|---------|----------|-----------|
| IntakeSubsystem | 28 | 10 | -18 (64%) |
| LauncherSubsystem | 16 | 7 | -9 (56%) |
| LauncherCoordinator | 14 | 2 | -12 (86%) |
| VisionSubsystemLimelight | 4 | 4 | 0 (0%) |
| **Total** | **64** | **23** | **-39 (61%)** |

**Expected Impact:**
- 61% reduction in @AutoLogOutput method calls (64 → 23)
- Estimated 30-40% reduction in AutoLogManager overhead
- Combined with tiered telemetry, should achieve <15ms MATCH mode loop times

## Implementation Priority

### Phase 1 (High Impact): Remove Duplicates in LauncherCoordinator
- Remove 12 duplicate logging methods
- Immediate 19% reduction (64 → 52)
- Zero risk (data still available from other subsystems)

### Phase 2 (Medium Impact): Remove Low-Value IntakeSubsystem Fields
- Remove 18 low-value fields (hardware checks, timing breakdowns, static config)
- Keep essential colors and state
- 28% reduction (52 → 34)

### Phase 3 (Low Impact): Trim LauncherSubsystem Debug Fields
- Remove 9 debug/tuning fields
- 14% reduction (34 → 25)

### Phase 4 (Final Polish): Conditional Logging
- Consider making certain "useful for tuning" fields only log in PRACTICE/DEBUG modes
- Implement DEBUG-only @AutoLogOutput via custom annotation or runtime checks

## Alternative: Tiered @AutoLogOutput

Instead of removing methods entirely, implement logging levels in AutoLogManager:

```java
@AutoLogOutput(level = TelemetryLevel.DEBUG)
public double getLogAverageCurrentRpm() { ... }

@AutoLogOutput(level = TelemetryLevel.PRACTICE)
public double getLogAverageTargetRpm() { ... }

@AutoLogOutput // defaults to MATCH level (always logged)
public boolean getLogBusy() { ... }
```

This preserves all diagnostic capability while reducing MATCH mode overhead.

## Recommendations

1. **Start with Phase 1** (LauncherCoordinator duplicates) - zero risk, immediate 19% gain
2. **Validate on robot** - confirm loop time improvement before proceeding
3. **Implement tiered @AutoLogOutput** - more flexible than removing methods entirely
4. **Document essential fields** - update CLAUDE.md with "what to log" guidelines
5. **Consider structured events** - replace scattered @AutoLogOutput with targeted event logging for critical state changes

## Testing Checklist

- [ ] Build passes after removals
- [ ] All OpModes start without errors
- [ ] WPILOG files still contain essential data (pose, artifacts, launcher ready)
- [ ] FTC Dashboard shows critical driver info (pose, alliance, ready status)
- [ ] Loop times in MATCH mode <15ms consistently
- [ ] AdvantageScope replay still works for post-match analysis
