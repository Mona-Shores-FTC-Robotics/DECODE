# Whiffle Ball Detection: Handling Holes

## The Problem

Whiffle balls (game artifacts) have **large holes** that cause intermittent sensor readings:

1. Sensor beam hits **solid part of ball** → Good reading (close distance, artifact color)
2. Ball rotates/vibrates slightly
3. Sensor beam hits **hole** → Bad reading (far distance, background color)
4. Result: Detection flickers on/off rapidly

This causes:
- False negatives (artifact present but not detected)
- Noisy distance readings (jumps between close and far)
- Color flickering (artifact color vs background)
- Unstable lane counts

## Solutions

### Solution 1: Temporal Hysteresis (Keep-Alive Timer)

**Idea:** Once we detect an artifact, keep it "alive" for a short time even if readings go bad (likely just hit a hole).

**Implementation:** Add a keep-alive timer that prevents clearing detection for X milliseconds after last good reading.

This is the **most effective solution** for whiffle balls.

### Solution 2: Require Multiple Consecutive Bad Samples to Clear

**Idea:** Don't immediately clear artifact on one bad sample - require 2-3 consecutive bad samples.

This is already partially implemented via the debounce system, but it's asymmetric (requires confirmations to SET but clears immediately on non-artifact).

### Solution 3: Distance Variance as Presence Indicator

**Idea:** High distance variance = whiffle ball present (holes causing jumps). Use variance as a positive signal.

If distance readings are jumping around (e.g., 4cm → 15cm → 5cm → 20cm), that's actually GOOD evidence that a holey artifact is present.

### Solution 4: Relaxed Presence Scoring for Known Artifacts

**Idea:** Once we've confidently detected an artifact color, stay locked on even with weaker presence signals.

### Solution 5: Physical/Optical Solutions

- **Increase sensor gain** - Get stronger reflections from ball material
- **Adjust sensor angle** - Reduce probability of beam going straight through hole
- **Use wider beam pattern** - If possible, detect ball edge rather than center
- **Add multiple sensors per lane** - Redundancy (expensive)

---

## Implementation Status

✅ **Solution 1 (Temporal Hysteresis + Distance-Based Clearing) - IMPLEMENTED**

### What Changed

**Config Parameters** (in `IntakeLaneSensorConfig.Gating`):
- `consecutiveConfirmationsRequired` (default: 1) - Number of consecutive good samples required to detect
- `consecutiveClearConfirmationsRequired` (default: 2) - Number of consecutive bad samples required to clear detection

**IntakeSubsystem Changes:**
- Added per-lane clear counters (`laneClearCandidateCount`)
- Simplified `applyGatedLaneColor()` with debounce-based clearing:
  - Presence detection determines if artifact is there
  - Debounce prevents single-sample flicker
  - Moving average filters smooth sensor values before threshold checks
  - Bad reading outside keep-alive window → count consecutive bad samples
  - Clear after N consecutive bad samples
  - **Once clearing starts, don't reset timer** (prevents "reviving" from intermittent holes)

**How It Works:**
1. **Detection Phase:**
   - Sensor hits solid part → Detects GREEN/PURPLE, records timestamp
   - Ball rotates, sensor hits hole → Sees NONE but within keep-alive → Keep alive
   - Sensor hits solid part again → Updates timestamp (if not clearing)
   - Detection stable despite holes!

2. **Clearing Phase (Artifact Removed):**
   - **Fast path:** Distance sensor reads >7cm → Clear immediately (ignores keep-alive)
   - **Slow path:** Distance invalid/noisy → Wait for keep-alive to expire (400ms) → Count bad samples (2×200ms) → Clear after ~800ms

This creates "sticky" detection during operation while ensuring prompt clearing when artifact is removed.

---

## Recommended Settings for Whiffle Balls

### Increase Keep-Alive Duration
```
Path: IntakeSubsystem → laneSensorConfig → gating
New parameter: keepAliveMs = 500.0  (0.5 seconds)
```

### Relax Distance Thresholds
Whiffle balls may wobble, so exit threshold should be much larger:
```
Path: IntakeSubsystem → laneSensorConfig → lanePresenceConfig20245
exitDistanceCm = enterDistanceCm + 3.0  (larger hysteresis gap)
```

### Increase Debounce for Entry, Not Exit
```
Path: IntakeSubsystem → laneSensorConfig → gating
consecutiveConfirmationsRequired = 3-4  (to detect)
clearConfirmationsRequired = 3-4  (NEW: to clear)
```

### Lower Presence Score Threshold
Accept noisier presence signals:
```
Path: IntakeSubsystem → laneSensorConfig → presence
minPresenceScore = 0.15-0.20  (was 0.25)
```

### Increase Sensor Gain
Get stronger reflections from ball material:
```
Path: IntakeSubsystem → laneSensorConfig → hardware
sensorGain = 30-35  (was 20)
```

### Disable Background Detection (Optional)
Background detection may be triggering when beam hits holes:
```
Path: IntakeSubsystem → laneSensorConfig → background
enableBackgroundDetection = false
```

---

## Testing Procedure

1. **Place whiffle ball in lane** - rotate it slowly by hand
2. **Watch telemetry** - `intake/sample/{lane}/distance_cm` should jump around
3. **Check stability** - Run diagnostics OpMode, should see >80% stability with keep-alive
4. **Test rapid movement** - Shake intake, verify detection stays locked
5. **Test clearing** - Remove ball, verify it clears within 1-2 seconds

---

## Advanced: Distance Variance Detection

If temporal hysteresis alone isn't enough, we can use distance variance as a positive signal.

**Logic:**
- Stable distance + good color = solid object (artifact)
- Jumping distance + color flickering = holey object (also artifact!)
- Stable distance + background color = empty space (no artifact)

This inverts the problem: holes become a FEATURE, not a bug.

**Implementation sketch:**
```java
// Track last N distance readings
if (distanceStddev > 2.0 && colorFlickeringBetweenArtifactAndBackground) {
    // This is likely a whiffle ball!
    presenceScore += 0.3; // Bonus for hole pattern
}
```
