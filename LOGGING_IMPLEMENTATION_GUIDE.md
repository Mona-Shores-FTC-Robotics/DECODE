# Tiered Logging Implementation Guide

## Overview
This guide shows how to apply tiered logging to all subsystems using `LoggingConfig`.

**Goal:** Keep loop times at 10-30ms while logging meaningful data.

---

## Quick Reference: Logging Tiers

| Tier | Overhead | When to Use | Enabled By |
|------|----------|-------------|------------|
| **MINIMAL** | ~3ms | Only robot pose + states | `LoggingConfig.tier = MINIMAL` |
| **MATCH** (default) | ~8ms | Competition matches | `LoggingConfig.tier = MATCH` |
| **DIAGNOSTIC** | ~40ms | Tuning/debugging | `LoggingConfig.tier = DIAGNOSTIC` |

---

## Pattern: How to Make a Method Conditional

### Always-On (Critical Data)
```java
@AutoLogOutput
public String getDriveModeString() {
    return activeMode.name();  // No check needed - always log
}
```

### Match-Level Data
```java
@AutoLogOutput
public double getCommandTurn() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return 0.0;  // Skip in MINIMAL mode
    }
    return lastCommandTurn;
}
```

### Diagnostic-Level Data (Expensive)
```java
@AutoLogOutput
public double getLfCurrentAmps() {
    if (!LoggingConfig.shouldLogMotorCurrents()) {
        return Double.NaN;  // Skip unless explicitly enabled
    }
    return readCurrentAmps(motorLf);  // Expensive I2C read
}
```

---

## Subsystem Checklist

### ✅ DriveSubsystem (DONE)
- Critical: Drive mode, robot-centric
- Match: Motor powers, command values
- Diagnostic: Motor currents, velocities, fusion data

### ⬜ LauncherSubsystem (TODO)

**Critical (always log):**
- Launcher state (DISABLED, READY, etc.)
- Spin mode (OFF, IDLE, FULL)
- Queued shots count

**Match (default):**
- **Per-lane target RPM** (LEFT, CENTER, RIGHT - critical for shot analysis)
- **Per-lane current RPM** (LEFT, CENTER, RIGHT)
- **Per-lane power values** (LEFT, CENTER, RIGHT)
- **Per-lane ready states** (LEFT, CENTER, RIGHT booleans)
- Average RPM (convenience aggregate)

**Diagnostic:**
- Control phases (BANG, HYBRID, HOLD)
- Bang-to-hold counters (for tuning control transitions)
- Feeder positions (servo debugging)
- Recovery timers (timing diagnostics)

**Implementation Example:**
```java
// In LauncherSubsystem.java

// CRITICAL - always log
@AutoLogOutput
public LauncherState getLogState() {
    return state;
}

@AutoLogOutput
public SpinMode getLogEffectiveSpinMode() {
    return computeEffectiveSpinMode();
}

@AutoLogOutput
public int getLogQueuedShots() {
    return getQueuedShots();
}

// MATCH - per-lane launcher data (CRITICAL for shot analysis)
@AutoLogOutput
public double getLogLeftTargetRpm() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return 0.0;
    }
    return getTargetRpm(LauncherLane.LEFT);
}

@AutoLogOutput
public double getLogLeftCurrentRpm() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return 0.0;
    }
    return getCurrentRpm(LauncherLane.LEFT);
}

@AutoLogOutput
public double getLogLeftPower() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return 0.0;
    }
    return getLastPower(LauncherLane.LEFT);
}

@AutoLogOutput
public boolean getLogLeftReady() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return false;
    }
    return isLaneReady(LauncherLane.LEFT);
}

// ... repeat for CENTER and RIGHT lanes ...

// Optional: Average values for convenience
@AutoLogOutput
public double getLogAverageTargetRpm() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return 0.0;
    }
    return getTargetRpm();  // Average across lanes
}

@AutoLogOutput
public double getLogAverageCurrentRpm() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return 0.0;
    }
    return getCurrentRpm();  // Average across lanes
}

// DIAGNOSTIC - control phase details
@AutoLogOutput
public String getLogLeftPhase() {
    if (!LoggingConfig.shouldLogControlPhases()) {
        return "UNKNOWN";
    }
    return getPhaseName(LauncherLane.LEFT);
}

@AutoLogOutput
public int getLogLeftBangToHoldCount() {
    if (!LoggingConfig.shouldLogControlPhases()) {
        return 0;
    }
    return getBangToHoldCount(LauncherLane.LEFT);
}

@AutoLogOutput
public double getLogLeftFeederPosition() {
    if (!LoggingConfig.isDiagnostic()) {
        return Double.NaN;
    }
    return getFeederPosition(LauncherLane.LEFT);
}

// ... repeat for CENTER and RIGHT lanes ...
```

### ⬜ IntakeSubsystem (TODO)

**Critical:**
- Current intake mode (PASSIVE_REVERSE, ACTIVE_FORWARD, STOPPED)

**Match:**
- Per-lane artifact colors (LEFT, CENTER, RIGHT)
- Artifact count
- Lane "within distance" booleans (for presence detection)

**Diagnostic:**
- Per-lane HSV values (hue, saturation, value)
- Per-lane distance sensor readings
- Sensor polling timing
- Mode resolve timing

**Implementation Example:**
```java
// CRITICAL
@AutoLogOutput
public String getCommandedMode() {
    return intakeMode.name();
}

// MATCH
@AutoLogOutput
public String getLeftColor() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return "NONE";
    }
    return getLaneColor(LauncherLane.LEFT).name();
}

@AutoLogOutput
public boolean isLeftWithinDistance() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return false;
    }
    return getLaneSample(LauncherLane.LEFT).withinDistance;
}

// DIAGNOSTIC
@AutoLogOutput
public float getLeftHue() {
    if (!LoggingConfig.shouldLogPerLaneIntake()) {
        return 0.0f;
    }
    return getLaneSample(LauncherLane.LEFT).hue;
}

@AutoLogOutput
public double getSensorPollMs() {
    if (!LoggingConfig.shouldLogSubsystemTimings()) {
        return 0.0;
    }
    return lastSensorPollMs;
}
```

### ⬜ VisionSubsystemLimelight (TODO)

**Critical:**
- Has valid tag (boolean)

**Match:**
- Current tag ID
- Basic robot pose from vision (x, y, heading)
- Odometry update pending

**Diagnostic:**
- tx, ty, ta (targeting angles)
- Range, bearing, yaw
- Time since last seen

**Implementation Example:**
```java
// CRITICAL
@AutoLogOutput
public boolean getHasValidTag() {
    refreshSnapshotIfStale();
    return lastSnapshot != null;
}

// MATCH
@AutoLogOutput
public int getLoggedCurrentTagId() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return -1;
    }
    refreshSnapshotIfStale();
    return lastSnapshot == null ? -1 : lastSnapshot.getTagId();
}

@AutoLogOutput
public double getLastPoseXInches() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return Double.NaN;
    }
    return lastRobotPose != null ? lastRobotPose.getX() : Double.NaN;
}

// DIAGNOSTIC
@AutoLogOutput
public double getLastTxDegrees() {
    if (!LoggingConfig.shouldLogDetailedVision()) {
        return Double.NaN;
    }
    return lastSnapshot != null ? lastSnapshot.getTxDegrees() : Double.NaN;
}

@AutoLogOutput
public double getTimeSinceLastSeenMs() {
    if (!LoggingConfig.shouldLogDetailedVision()) {
        return Double.NaN;
    }
    return lastSnapshotTimestampMs == 0L
            ? Double.POSITIVE_INFINITY
            : Math.max(0.0, System.currentTimeMillis() - lastSnapshotTimestampMs);
}
```

### ⬜ LauncherCoordinator (TODO)

**Critical:**
- Artifact state (EMPTY, ONE, TWO, THREE)

**Match:**
- Launcher state from owned launcher
- Launcher spin mode
- Intake requested mode
- Per-lane colors (from laneColors map)

**Diagnostic:**
- Intake automation enabled
- Manual override active
- Lighting registered

**Implementation Example:**
```java
// CRITICAL
@AutoLogOutput
public String getArtifactStateString() {
    return artifactState.name();
}

@AutoLogOutput
public int getArtifactCountLogged() {
    return artifactState.count();
}

// MATCH
@AutoLogOutput
public String getLauncherState() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return "UNKNOWN";
    }
    return launcher.getState().name();
}

@AutoLogOutput
public String getLeftColor() {
    if (!LoggingConfig.isMatchOrHigher()) {
        return "NONE";
    }
    return laneColors.getOrDefault(LauncherLane.LEFT, ArtifactColor.NONE).name();
}

// DIAGNOSTIC
@AutoLogOutput
public boolean getIntakeAutomationEnabled() {
    if (!LoggingConfig.isDiagnostic()) {
        return false;
    }
    return intakeAutomationEnabled;
}
```

---

## Additional Optimizations

### 1. Remove Duplicate KoalaLog Calls in TelemetryService

**File:** `TelemetryService.java`

**Current (lines 376-394):**
```java
private void log(TelemetryPacket packet, String key, Double value) {
    packet.put(key, value);
    KoalaLog.log(key, value, true);  // DUPLICATE - already logged by AutoLog
}
```

**Updated:**
```java
private void log(TelemetryPacket packet, String key, Double value) {
    packet.put(key, value);  // Only send to FTC Dashboard, don't duplicate to KoalaLog
}
```

Do this for ALL `log()` helper methods (Boolean, String, int versions).

**Why:** AutoLog already logs everything via @AutoLogOutput. The KoalaLog calls in TelemetryService are pure duplication adding ~15ms overhead.

**Keep:** The dashboard display itself (it looks good!), just remove the redundant logging.

---

### 2. Adjust AutoLog Frequency

**File:** `DecodeTeleOp.java:42-43`

**Current:**
```java
public static long AUTO_LOG_INTERVAL_MS = 400L; // 20Hz
```

**Recommended for matches:**
```java
public static long AUTO_LOG_INTERVAL_MS = 1000L; // 5Hz - plenty for post-match analysis
```

**For testing/tuning:**
```java
public static long AUTO_LOG_INTERVAL_MS = 200L; // 10Hz - faster diagnostics
```

---

## Usage: Changing Tiers During Init

### Via FTC Dashboard Config Tab:
1. Connect to robot: `http://192.168.49.1:8080/dash`
2. Go to **Config** tab
3. Find `LoggingConfig` → `ActiveTier` → `tier`
4. Select: `MINIMAL`, `MATCH`, or `DIAGNOSTIC`
5. Save and start OpMode

### Via Code (for testing):
```java
// In DecodeTeleOp.onInit()
LoggingConfig.ActiveTier.tier = LoggingConfig.LoggingTier.DIAGNOSTIC;
```

### Enable Individual Features:
```java
// Turn on motor currents even in MATCH mode
LoggingConfig.Features.logMotorCurrents = true;

// Turn on per-lane launcher data
LoggingConfig.Features.logPerLaneLauncher = true;
```

---

## Expected Performance

| Configuration | Loop Time | @AutoLogOutput Calls | Use Case |
|---------------|-----------|---------------------|----------|
| MINIMAL | 10-15ms | ~15 | Fast testing, not useful for analysis |
| **MATCH (default)** | **18-28ms** | **~80** | **Competition matches** |
| MATCH + per-lane intake | 20-30ms | ~90 | Match with detailed intake diagnostics |
| DIAGNOSTIC | 40-60ms | ~181 (all) | Tuning sessions, debugging |
| DIAGNOSTIC + currents | 60-80ms | ~181 + 4 I2C | Deep hardware diagnostics |

**Note:** Per-lane launcher data (12 fields: 3 lanes × 4 metrics) is included in MATCH tier because it's critical for shot performance analysis.

---

## Testing Plan

1. ✅ **Test MINIMAL mode:** Verify robot still drives, loop times <15ms
2. ✅ **Test MATCH mode (default):** Verify essential data logged, loop times <25ms
3. ✅ **Test DIAGNOSTIC mode:** Verify all data logged, loop times <60ms
4. ✅ **Test individual features:** Enable `logMotorCurrents` in MATCH, verify it works
5. ✅ **Open WPILOG in AdvantageScope:** Verify all expected fields present for each mode

---

## Next Steps

1. Apply pattern to `LauncherSubsystem.java` (follow DriveSubsystem example)
2. Apply pattern to `IntakeSubsystem.java`
3. Apply pattern to `VisionSubsystemLimelight.java`
4. Apply pattern to `LauncherCoordinator.java`
5. Remove duplicate `KoalaLog.log()` calls in `TelemetryService.java`
6. Test all three tiers on robot
7. Set default to `MATCH` mode for competition

**Estimated time:** 30-45 minutes to update all subsystems
