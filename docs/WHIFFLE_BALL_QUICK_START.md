# Whiffle Ball Detection - Quick Start

## What Was Fixed

Your artifact detection was **clearing immediately** when the sensor beam hit a whiffle ball hole. Now it uses **temporal hysteresis** (keep-alive timer) to maintain detection through brief interruptions.

**Before:**
```
Sensor: SOLID → HOLE → SOLID → HOLE → SOLID
Detection: GREEN → NONE → GREEN → NONE → GREEN  (flickers!)
```

**After (with keep-alive + distance-based clearing):**
```
Sensor: SOLID → HOLE → SOLID → HOLE → SOLID
Detection: GREEN → GREEN → GREEN → GREEN → GREEN  (stable!)

When ball removed (distance jumps to 15cm+):
Detection: GREEN → NONE  (clears immediately!)
```

---

## How to Test (5 minutes)

### Step 1: Build and Deploy
```bash
./gradlew :TeamCode:assembleDebug
./gradlew :FtcRobotController:installDebug
```

### Step 2: Run Your Existing TeleOp
- No code changes needed - the fix is already active
- Default settings should work for most robots

### Step 3: Test Basic Detection
1. **Place whiffle ball in a lane** (LEFT, CENTER, or RIGHT)
2. **Slowly rotate the ball** while watching FTC Dashboard
3. **Expected:** Detection should stay stable (GREEN or PURPLE)
4. **Remove the ball** - should clear within ~1 second

### Step 4: Check FTC Dashboard Telemetry
Connect to `http://192.168.49.1:8080/dash` and watch:
- `intake/sample/{lane}/distance_cm` - Will jump around (4cm → 15cm → 5cm) due to holes
- Lane color in your telemetry - Should stay stable despite distance jumps

---

## If Detection Is Still Unstable

### Issue 1: Detection still flickers (unstable)

**Fix:** Increase Keep-Alive Duration

If the ball rotates fast or has many holes:

1. Open FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
2. Increase `keepAliveMs` from 400 to 600-800
3. Test again - detection should survive longer gaps between good readings

### Issue 2: Detection never clears when ball is removed

**Fix:** Adjust Distance Clearance Margin

If the ball sits at the edge of detection range and doesn't trigger distance-based clearing:

1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
2. **Decrease** `distanceClearanceMarginCm` from 2.0 to 1.0-1.5
   - This makes clearing more sensitive - artifact clears as soon as distance exceeds exit threshold by this margin
3. Check your exit thresholds in `lanePresenceConfig20245` - they should be larger than typical artifact distance
   - Example: If artifact is at 4-5cm, set `exitDistanceCm` to 6-7cm
4. Test: Remove ball and watch FTC Dashboard `intake/sample/{lane}/distance_cm` - should jump to >10cm and clear instantly

### Issue 3: Detection clears too slowly after removing ball

**Fix:** Reduce Clear Confirmations

1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
2. **Decrease** `consecutiveClearConfirmationsRequired` from 2 to 1
   - This makes clearing faster but may cause more false clears if sensor is noisy
3. **Or decrease** `keepAliveMs` from 400 to 200-300
   - Shorter keep-alive = faster clearing, but less tolerance for holes

### Option 3: Relax Distance Thresholds

If the ball wobbles in the lane:

1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → lanePresenceConfig20245
2. For each lane, increase the **exit** threshold by 2-3 cm:
   - Example: `centerExitDistanceCm` from 5.5 to 7.5
3. This gives more distance tolerance for wobbling artifacts

### Option 4: Increase Sensor Gain

If reflections from the ball are weak:

1. FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → hardware
2. Increase `sensorGain` from 20 to 30-35
3. Stronger reflections = better detection through holes

---

## Recommended Settings by Ball Type

### Small Holes (Good Coverage)
```
keepAliveMs = 300-400
consecutiveClearConfirmationsRequired = 2
sensorGain = 20-25
```

### Medium Holes (Moderate Coverage)
```
keepAliveMs = 400-600
consecutiveClearConfirmationsRequired = 2-3
sensorGain = 25-30
```

### Large Holes (Sparse Coverage) - Whiffle Balls
```
keepAliveMs = 600-800
consecutiveClearConfirmationsRequired = 3-4
sensorGain = 30-35
exitDistanceCm = enterDistanceCm + 3.0 (larger hysteresis)
```

---

## Understanding the Keep-Alive Window

**Keep-Alive Duration** is how long to "trust" the last good reading:

- **Too short** (100-200ms): Detection flickers when hitting holes
- **Just right** (400-600ms): Stable detection, clears quickly when removed
- **Too long** (1000+ ms): Detection "sticks" after ball is removed

**Rule of thumb:** Set to 2x your sensor polling period (default 200ms → 400ms keep-alive)

---

## Testing Checklist

✅ **Place ball in lane** - Should detect within 0.5 seconds
✅ **Rotate ball slowly** - Detection stays stable (no flickering)
✅ **Shake intake rapidly** - Detection survives vibration
✅ **Remove ball** - Detection clears within 1-2 seconds
✅ **Place multiple balls** - All lanes detect independently
✅ **Test with field lighting** - Works in match conditions

---

## Advanced: Visualizing Detection Stability

Run the **"Artifact Detection Diagnostics"** OpMode (created earlier):

1. Select your OpMode list → "Artifact Detection Diagnostics"
2. Place ball in a lane
3. Let it run for 30 seconds
4. Check **Stability Percentage**:
   - **>90%**: Excellent - detection is rock solid
   - **70-90%**: Good - minor noise acceptable
   - **<70%**: Needs tuning - increase keep-alive or gain

---

## What If Keep-Alive Isn't Enough?

If you've tried all the above and still have issues, you may need **distance variance detection** (advanced solution):

The idea: Use the PRESENCE of noise as a positive signal - if distance is jumping around, that's evidence of a holey artifact.

See `docs/WHIFFLE_BALL_DETECTION.md` for implementation details.

---

## FAQ

**Q: Will this make detection slower to respond?**

A: Slightly - clearing now takes `keepAliveMs + (clearConfirmations * pollPeriod)` instead of instant. With defaults (400ms + 2*200ms = 800ms), artifacts clear in <1 second, which is acceptable for most use cases.

**Q: Does this affect color detection?**

A: No - color classification is unchanged. This only affects the **presence detection** (artifact vs nothing).

**Q: Can I disable keep-alive?**

A: Yes - set `keepAliveMs = 0` to revert to immediate clearing (not recommended for whiffle balls).

**Q: What if the ball is completely hollow (no solid parts)?**

A: Temporal hysteresis won't help if there are no good readings. You'd need:
1. Physical solution (angled sensor to hit rim)
2. Multiple sensors per lane (redundancy)
3. Mechanical guides to ensure ball orientation

**Q: Does this work for other holey objects?**

A: Yes! This works for any object with intermittent sensor readings - holey balls, mesh fabrics, perforated materials, etc.

---

## Summary

1. ✅ **Implementation complete** - Temporal hysteresis is now active
2. 🚀 **Test immediately** - Should work with default settings
3. 🎛️ **Tune if needed** - Increase `keepAliveMs` for more stability
4. 📊 **Verify with diagnostics** - Check stability percentage

The key insight: **Whiffle ball holes are PREDICTABLE** - the ball can't disappear instantly. By keeping detection alive for a brief window (400ms), we survive the holes and maintain stable tracking.
