# Whiffle Ball Detection Fix - Version 2

## The Problem You Reported

> "its kinda better but worse. they never turn off now but they definitely dont flicker"

**Root cause:** The keep-alive timer was resetting every time the sensor briefly hit a solid part of the whiffle ball. Since whiffle balls are stationary (or slowly rotating) in the lanes, they constantly trigger brief "good" readings that reset the timer, preventing the detection from ever clearing.

## The Fix

Added **dual clearing logic** with two independent paths:

### Path 1: Distance-Based Instant Clear (NEW)
- **When:** Distance sensor reads >exitThreshold + 2cm margin
- **Action:** Clear immediately (ignores keep-alive timer)
- **Why:** When you remove the artifact, distance jumps to 10-20cm+ → Instant clear
- **Benefit:** No more "never clears" issue!

### Path 2: Temporal Hysteresis (Original, with Fix)
- **When:** Distance is invalid OR within range
- **Action:** Use keep-alive timer to prevent flicker
- **NEW:** Once clearing starts (bad samples accumulating), don't reset timer on new good readings
- **Why:** Prevents whiffle ball from "reviving" detection by hitting solid parts during clearing process

## What Changed in Code

**Files Modified:**
1. `IntakeLaneSensorConfig.java` - Added `distanceClearanceMarginCm` parameter (default: 2.0cm)
2. `IntakeSubsystem.java` - Implemented dual clearing logic in `applyGatedLaneColor()`:
   - Added distance-based override at start of function
   - Modified timer reset logic - only reset if NOT currently clearing
   - Clear counter reset moved to detection confirmation path

## How to Test

### Step 1: Build and Deploy
```bash
./gradlew :TeamCode:assembleDebug
./gradlew :FtcRobotController:installDebug
```

### Step 2: Test Stability (Should NOT Flicker)
1. Place whiffle ball in a lane
2. Slowly rotate it by hand
3. **Expected:** Detection stays GREEN/PURPLE (no flicker)
4. Watch FTC Dashboard `intake/sample/{lane}/distance_cm` - will jump (4cm → 15cm → 5cm) but color stays stable

### Step 3: Test Clearing (Should Clear Quickly)
1. With ball in lane and detected
2. **Remove the ball** (pull it out of the intake)
3. **Expected:** Detection clears within 0.5-1 second
4. Watch FTC Dashboard `intake/sample/{lane}/distance_cm` - should jump to >10cm and detection clears immediately

### Step 4: Test Edge Case (Ball at Range Limit)
1. Place ball at the very edge of detection range (4-6cm)
2. Slowly pull it away
3. **Expected:** As soon as distance exceeds exit threshold + 2cm, detection clears
4. Example: If `exitDistanceCm = 5.5`, detection should clear at ~7.5cm

## If Clearing Still Doesn't Work

### Scenario 1: Ball removed but detection doesn't clear

**Diagnosis:** Check FTC Dashboard `intake/sample/{lane}/distance_cm` when ball is removed
- If distance jumps to >10cm but still doesn't clear → Bug, report to me
- If distance stays <7cm after removal → Ball is still in detection range (check physical setup)
- If distance is NaN or invalid → Distance sensor broken, will use slow clearing path

**Fix:** Decrease clearance margin to make clearing more aggressive
1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
2. Set `distanceClearanceMarginCm` to 1.0 (was 2.0)
3. Test again

### Scenario 2: Detection flickers when ball is present

**Diagnosis:** Keep-alive window too short for your ball's hole pattern

**Fix:** Increase keep-alive duration
1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
2. Increase `keepAliveMs` from 400 to 600-800
3. Test again - should be more stable

### Scenario 3: Clearing is too slow

**Diagnosis:** Need to reduce debounce on clearing

**Fix:** Reduce clear confirmations required
1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
2. Set `consecutiveClearConfirmationsRequired` to 1 (was 2)
3. Test again - should clear faster

## Config Parameters Reference

**Path:** FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating

| Parameter | Default | What It Does | Tune Up | Tune Down |
|-----------|---------|--------------|---------|-----------|
| `keepAliveMs` | 400 | How long to keep detection alive after last good reading | More stable, slower clear | Less stable, faster clear |
| `consecutiveClearConfirmationsRequired` | 2 | How many bad samples needed to clear | Harder to clear (stable) | Easier to clear (faster) |
| `distanceClearanceMarginCm` | 2.0 | Distance beyond exit threshold for instant clear | Harder to clear (needs farther) | Easier to clear (clears sooner) |

## Expected Behavior Summary

| Action | Old Behavior | New Behavior |
|--------|--------------|--------------|
| **Ball in lane, rotating** | Flickered GREEN→NONE→GREEN | Stable GREEN |
| **Ball removed** | Stayed GREEN (never cleared!) | Clears in <1 second |
| **Ball at edge, pulled away** | Flickered or stuck | Clears as soon as distance >7cm |
| **Rapid intake/outtake** | Flickered badly | Stable during vibration |

## State Machine Diagram

```
[NO ARTIFACT DETECTED]
        ↓ (good color + within distance × N times)
[ARTIFACT DETECTED] ← ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
        ↓                                 │ (keep-alive timer active)
   (bad reading)                          │ (brief hole reading)
        ↓                                 │
   distance > exit + margin? ────────────┘
        ↓ YES
   [CLEAR IMMEDIATELY]

        ↓ NO
   time since good < keepAlive? ─────────┐
        ↓ NO                             │ YES (keep alive)
   [CLEARING IN PROGRESS]                │
        ↓                                 │
   (don't reset timer on new readings) ← ┘
        ↓
   (accumulate bad samples × N)
        ↓
   [CLEAR DETECTION]
```

## Key Insights

1. **Whiffle balls are stationary but holey** - They give intermittent good readings, so timer-only approaches fail
2. **Distance is the ground truth** - When artifact is removed, distance WILL jump (unless sensor broken)
3. **Use distance as override** - Don't wait for timer when distance proves artifact is gone
4. **Prevent "reviving"** - Once we start clearing, commit to it (don't reset timer)

## Debugging Tips

**Enable detailed telemetry:**
1. Connect to FTC Dashboard
2. Watch these keys during testing:
   - `intake/sample/{lane}/distance_cm` - Physical distance reading
   - `intake/sample/{lane}/within_distance` - Hysteresis state
   - `intake/classifier/{lane}/reason` - Why this classification?
   - Your lane color telemetry - Final detection state

**Look for patterns:**
- Distance jumping 4→15→5→12→4cm = Whiffle ball holes (GOOD - detection should stay stable)
- Distance steady at 4-5cm, then jumps to >10cm = Ball removed (detection should clear immediately)
- Distance steady at 4-5cm, color flickers = Tune presence detection thresholds

## Success Criteria

✅ **Test 1:** Ball in lane, rotated slowly → Detection stable (>90% stability in diagnostics)
✅ **Test 2:** Ball removed → Detection clears in <1 second
✅ **Test 3:** Ball at edge, pulled away gradually → Clears as soon as distance exceeds threshold
✅ **Test 4:** Rapid intake shaking → Detection survives vibration
✅ **Test 5:** 3 balls in 3 lanes → All detect and clear independently

Once all 5 tests pass, your whiffle ball detection is SOLID!

## Next Steps if Issues Persist

If you've tried all the above and still have problems:

1. **Check distance sensor hardware:**
   - Run diagnostics OpMode and check `distance_cm` readings
   - Should read 4-6cm with ball present, >10cm with ball removed
   - If always NaN or >100cm, sensor may be broken

2. **Check exit thresholds are set correctly:**
   - FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → lanePresenceConfig20245
   - Each lane should have `exitDistanceCm` > typical artifact distance
   - Example: If ball is at 4-5cm, set exit to 6-7cm (gives margin for instant clearing)

3. **Capture telemetry and share:**
   - Run for 30 seconds with ball in lane
   - Remove ball
   - Export FTC Dashboard logs showing distance and detection state

4. **Consider physical fixes:**
   - Angle sensor to hit ball rim instead of center (reduces hole hits)
   - Increase sensor gain to get stronger reflections
   - Add mechanical guides to control ball orientation
