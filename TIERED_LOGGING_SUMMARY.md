# Tiered Logging Implementation Summary

## What We've Accomplished

### ✅ Created `LoggingConfig.java`
- Central configuration for all logging tiers
- Three tiers: MINIMAL, MATCH (default), DIAGNOSTIC
- Individual feature toggles for fine-grained control
- Configurable via FTC Dashboard during init

### ✅ Updated `DriveSubsystem.java`
- Implemented tiered logging pattern
- Motor powers: MATCH tier (~8 fields)
- Motor currents: DIAGNOSTIC tier (expensive, ~4 I2C reads)
- Motor velocities: DIAGNOSTIC tier (expensive, ~4 encoder reads)
- PoseFusion data: DIAGNOSTIC tier

### ✅ Created Implementation Guide
- Complete pattern documentation in `LOGGING_IMPLEMENTATION_GUIDE.md`
- Step-by-step examples for each subsystem
- Performance expectations for each tier

---

## What's Confirmed

✅ **Pedro Pathing is fast:** Only 5-10ms
✅ **Base loop time is excellent:** 10-30ms without logging
✅ **Problem was AutoLog overhead:** Sampling 181 methods was killing performance
✅ **Per-lane launcher data is critical:** Must be in MATCH tier, not DIAGNOSTIC

---

## What You Need to Do

### Priority 1: Apply Pattern to Remaining Subsystems

Use `LOGGING_IMPLEMENTATION_GUIDE.md` as your reference. Follow the DriveSubsystem pattern:

1. **LauncherSubsystem** (~20 min)
   - Critical: State, spin mode, queued shots
   - Match: **All per-lane RPM/power/ready** (12 fields)
   - Diagnostic: Control phases, feeders, recovery timers

2. **IntakeSubsystem** (~15 min)
   - Critical: Current mode
   - Match: Per-lane colors, artifact count, presence detection
   - Diagnostic: HSV values, distances, timing

3. **VisionSubsystemLimelight** (~10 min)
   - Critical: Has tag
   - Match: Tag ID, basic pose, odometry pending
   - Diagnostic: tx/ty/ta, range/bearing/yaw, timing

4. **LauncherCoordinator** (~10 min)
   - Critical: Artifact state
   - Match: Launcher state/spin, intake mode, lane colors
   - Diagnostic: Automation flags, overrides

### Priority 2: Remove Duplicate Logging in TelemetryService

**File:** `TelemetryService.java` (lines 376-394)

**Change this:**
```java
private void log(TelemetryPacket packet, String key, Double value) {
    packet.put(key, value);
    KoalaLog.log(key, value, true);  // REMOVE THIS
}
```

**To this:**
```java
private void log(TelemetryPacket packet, String key, Double value) {
    packet.put(key, value);  // Keep dashboard display, remove duplicate logging
}
```

Do this for ALL `log()` helper method variants (Boolean, String, int).

**Expected savings:** ~15ms

---

## Expected Results

### MINIMAL Mode (Tier 0)
**Loop Time:** 10-15ms
**Logs:** ~15 fields (pose, states only)
**Use Case:** Performance testing, not useful for match analysis

### MATCH Mode (Tier 1) - DEFAULT
**Loop Time:** 18-28ms
**Logs:** ~80 fields including:
- Robot pose (x, y, heading)
- Drive: Motor powers, commands, modes
- Launcher: **All per-lane RPM, power, ready states**
- Intake: Per-lane colors, artifact states
- Vision: Tag detection, basic pose
- Coordinator: Artifact count, modes

**Use Case:** Competition matches - full shot analysis capability

### DIAGNOSTIC Mode (Tier 2)
**Loop Time:** 40-60ms
**Logs:** All 181 fields including:
- Motor currents (expensive I2C reads)
- Motor velocities (expensive encoder reads)
- Control phases, counters
- Detailed vision data (tx, ty, ta, range, bearing)
- HSV color values
- All timing breakdowns

**Use Case:** Tuning sessions, hardware debugging

---

## How to Use

### During Init (via FTC Dashboard):
1. Connect: `http://192.168.49.1:8080/dash`
2. Go to **Config** tab
3. Find `LoggingConfig` → `ActiveTier` → `tier`
4. Select: `MINIMAL`, `MATCH`, or `DIAGNOSTIC`
5. Start OpMode

### In Code (for testing):
```java
// Set in DecodeTeleOp.onInit()
LoggingConfig.ActiveTier.tier = LoggingConfig.LoggingTier.DIAGNOSTIC;
```

### Enable Individual Features:
```java
// Turn on motor currents even in MATCH mode
LoggingConfig.Features.logMotorCurrents = true;
```

---

## Testing Checklist

After implementing:

- [ ] Test MINIMAL mode: Verify robot drives, loop <15ms
- [ ] Test MATCH mode: Verify essential data logged, loop <28ms
- [ ] Test DIAGNOSTIC mode: Verify all data logged, loop <60ms
- [ ] Test feature toggles: Enable `logMotorCurrents` in MATCH, verify works
- [ ] Open WPILOG in AdvantageScope: Verify all expected fields present
- [ ] Run a real match: Confirm MATCH tier gives useful analysis data
- [ ] Check per-lane launcher data: Verify RPM tracking for all 3 lanes

---

## Key Design Decisions

1. **Per-lane launcher data in MATCH tier**
   - You need this for shot performance analysis
   - Worth the extra ~5ms overhead

2. **Motor currents DIAGNOSTIC-only**
   - Very expensive (I2C reads, ~20ms total)
   - Only needed for hardware debugging, not match analysis

3. **Keep TelemetryService display**
   - Looks good, teams like it
   - Just remove duplicate KoalaLog calls

4. **Default to MATCH tier**
   - Good balance of performance and data
   - 18-28ms is well within acceptable range (<50ms)

---

## Questions?

- **"Loop time still high in MATCH?"** → Check if you removed duplicate KoalaLog calls in TelemetryService
- **"Missing launcher data in logs?"** → Verify per-lane methods use `isMatchOrHigher()`, not `isDiagnostic()`
- **"Can't find LoggingConfig in dashboard?"** → Make sure class is annotated with `@Configurable`
- **"Want even faster loops?"** → Use MINIMAL tier (but you'll lose critical match data)

---

## Estimated Time to Complete

- **Subsystem updates:** 45-60 minutes
- **TelemetryService cleanup:** 5 minutes
- **Testing:** 15-20 minutes
- **Total:** ~70-85 minutes

---

## Files Modified

1. ✅ `LoggingConfig.java` (created)
2. ✅ `DriveSubsystem.java` (updated)
3. ⬜ `LauncherSubsystem.java` (needs update)
4. ⬜ `IntakeSubsystem.java` (needs update)
5. ⬜ `VisionSubsystemLimelight.java` (needs update)
6. ⬜ `LauncherCoordinator.java` (needs update)
7. ⬜ `TelemetryService.java` (remove duplicate logging)

---

## Next Action

Start with **LauncherSubsystem** - it's the most important one since you need per-lane tracking. Use the pattern in `LOGGING_IMPLEMENTATION_GUIDE.md` and the DriveSubsystem example as your reference.

Good luck! This should give you fast loops (~20ms) with all the critical data you need for match analysis.
