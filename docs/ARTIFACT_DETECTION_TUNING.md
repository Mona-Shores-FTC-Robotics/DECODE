# Artifact Detection Tuning Guide

This guide helps diagnose and fix artifact presence detection issues (false positives/negatives, noise, flickering).

**🎯 QUICK START FOR WHIFFLE BALLS:** If your artifacts have holes and detection flickers, see `WHIFFLE_BALL_QUICK_START.md` for an immediate fix using temporal hysteresis (keep-alive timer).

## Quick Diagnosis

Run the **"Artifact Detection Diagnostics"** OpMode and observe the stability percentage:

- **90%+ stability**: Detection is working well
- **70-90% stability**: Some noise, tune thresholds
- **50-70% stability**: Significant noise, check configuration
- **<50% stability**: Critical issues, sensor may be broken or severely misconfigured

## Common Problems and Fixes

### Problem 1: "Artifacts not detected even when present"

**Symptoms:** Stability is good but color stays NONE when artifact is in lane

**Diagnosis:** Check FTC Dashboard telemetry for `intake/classifier/{lane}/reason`:
- If `distance_out`: Distance thresholds too strict
- If `no_signal`: Sensor gain too low or poor lighting
- If `background`: Background detection rejecting artifacts

**Fix:**

1. **Distance thresholds** (most common):
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig
   - Navigate to `lanePresenceConfig20245` (or 19429 for your robot)
   - Increase the enter thresholds for each lane:
     - `leftEnterDistanceCm`: Try 5.0-6.0 cm
     - `centerEnterDistanceCm`: Try 5.0-7.0 cm
     - `rightEnterDistanceCm`: Try 5.0-6.0 cm
   - Keep exit thresholds 0.5-1.0 cm higher than enter thresholds

2. **Sensor gain too low**:
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → hardware
   - Increase `sensorGain` from 20.0 to 25-30
   - Verify `enableSensorLight` is `true`

3. **Background detection too aggressive**:
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → background
   - Increase `maxBackgroundDistance` from 40.0 to 50-60
   - Or disable: set `enableBackgroundDetection` to `false`

4. **Quality thresholds too strict**:
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → quality
   - Decrease `minSaturation` from 0.15 to 0.10

---

### Problem 2: "Detection flickers between colors / NONE"

**Symptoms:** Stability <70%, high transition count, color changes rapidly

**Diagnosis:** Check diagnostics OpMode "Noise Analysis" section

**Fix:**

1. **Increase debouncing** (first line of defense):
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → gating
   - Increase `consecutiveConfirmationsRequired` from 2 to 3-4

2. **Distance hysteresis too small**:
   - Make exit thresholds significantly larger than enter thresholds (1.0-2.0 cm gap)
   - Example: enter=4.0 cm, exit=5.5 cm

3. **Sensor noise** (hardware issue):
   - Check sensor mounting - vibration causes distance noise
   - Check I2C cable connection
   - Try different sensor gain (both higher and lower)

---

### Problem 3: "False positives - detects artifacts when lane is empty"

**Symptoms:** Detection shows GREEN/PURPLE when pointing at field mat or empty space

**Diagnosis:** Run "Artifact Color Calibration" OpMode and capture BACKGROUND samples

**Fix:**

1. **Enable background detection**:
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → background
   - Set `enableBackgroundDetection` to `true`
   - Run calibration OpMode:
     - Point sensor at empty space / field mat
     - Press LEFT_BUMPER to capture background samples (get 10-20 samples)
     - Note the recommended `backgroundHue`, `backgroundSaturation`, `backgroundValue`
   - Update background config with recommended values
   - Set `maxBackgroundDistance` to recommended value (usually 30-40)

2. **Distance gating**:
   - Ensure `useDistance` is `true` in presence config
   - Tighten enter thresholds (lower values = closer range required)

3. **Multi-factor presence scoring**:
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → presence
   - Enable `enablePresenceScoring` (set to `true`)
   - Increase `minPresenceScore` from 0.25 to 0.35-0.40
   - Increase `minTotalIntensity` from 10 to 15-20

---

### Problem 4: "Wrong color detected (GREEN shows as PURPLE or vice versa)"

**Symptoms:** Stability is good but color is consistently wrong

**Diagnosis:** Run "Artifact Color Calibration" OpMode to check color separation

**Fix:**

1. **Re-calibrate color classifier**:
   - Run calibration OpMode
   - Capture 10-20 GREEN samples (press A)
   - Capture 10-20 PURPLE samples (press B)
   - Check "Separation Analysis":
     - Hue separation should be >60° for reliable detection
     - Green ratio separation should be >0.2
   - Apply recommended parameters to FTC Dashboard config

2. **Adjust decision boundary** (DECISION_BOUNDARY mode):
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → classifier → decision
   - Set `hueDecisionBoundary` to recommended value from calibration (typically 170-180°)

3. **Check lighting conditions**:
   - Artifacts should be well-lit (sensor LED + ambient light)
   - Avoid extreme ambient colors (red/blue field lights)

---

### Problem 5: "Slow response - takes too long to detect artifact"

**Symptoms:** Detection is accurate but laggy

**Fix:**

1. **Reduce debouncing**:
   - Decrease `consecutiveConfirmationsRequired` to 1-2 (trade-off: more noise)

2. **Reduce polling period** (use with caution - impacts loop time):
   - Go to FTC Dashboard → Config → IntakeSubsystem → laneSensorConfig → polling
   - Decrease `samplePeriodMs` from 200 to 150 (minimum 100 to avoid I2C overload)

---

## Systematic Tuning Procedure

Follow these steps in order for best results:

### Step 1: Run Diagnostics OpMode
- Deploy "Artifact Detection Diagnostics" OpMode
- Place artifacts in each lane one at a time
- Check stability percentage for each lane
- Identify which lanes have issues

### Step 2: Run Calibration OpMode
- Deploy "Artifact Color Calibration" OpMode
- For each problematic lane:
  1. Place GREEN artifact, press A to capture 10-20 samples
  2. Place PURPLE artifact, press B to capture 10-20 samples
  3. Point at empty space, press LEFT_BUMPER to capture background
- Review recommended parameters on telemetry

### Step 3: Apply Calibrated Parameters
- Open FTC Dashboard: `http://192.168.49.1:8080/dash`
- Go to Config tab → IntakeSubsystem → laneSensorConfig
- Apply recommended values from calibration OpMode:
  - Classifier parameters (decision boundary, distance targets)
  - Quality thresholds (minSaturation)
  - Background detection (backgroundHue, maxBackgroundDistance)

### Step 4: Tune Distance Thresholds
Each lane has different geometry, so per-lane tuning is critical:

1. Place artifact in LEFT lane
2. Watch `intake/sample/left/distance_cm` in FTC Dashboard
3. Note the distance when artifact is reliably in position (e.g., 4.2 cm)
4. Set `leftEnterDistanceCm` to this value
5. Set `leftExitDistanceCm` to 0.5-1.0 cm higher
6. Repeat for CENTER and RIGHT lanes

### Step 5: Tune Debounce
Based on stability from diagnostics:

- Stability >90%: No changes needed
- Stability 70-90%: Increase `consecutiveConfirmationsRequired` to 3
- Stability 50-70%: Increase to 4, tune presence thresholds
- Stability <50%: Check hardware (sensor mount, cables, gain)

### Step 6: Verify in Real Conditions
- Run your TeleOp or Auto
- Test with rapid intake/outtake
- Test with field lighting conditions
- Verify no false positives when lanes are empty

---

## Advanced: Understanding the Detection Pipeline

Your artifact detection uses a **multi-stage pipeline**:

```
1. Distance Hysteresis (per-lane enter/exit thresholds)
   ↓ (if within distance)
2. Quality Check (min saturation, min value, min intensity)
   ↓ (if quality OK)
3. Background Detection (HSV distance to field mat)
   ↓ (if not background)
4. Color Classification (hue decision boundary)
   ↓ (returns color)
5. Debouncing (consecutive confirmations required)
   ↓ (if stable)
6. Lane Color Update (GREEN / PURPLE accepted)
```

Each stage can reject the sample:
- Stage 1 failure → `distance_out`
- Stage 2 failure → `no_signal`
- Stage 3 failure → `background`
- Stages 4-6 failure → returns NONE (artifact present but too uncertain)

Check `intake/classifier/{lane}/reason` in telemetry to see which stage is rejecting.

---

## Telemetry Keys Reference

### Per-lane sensor readings:
```
intake/sample/{lane}/distance_cm          - Distance sensor reading
intake/sample/{lane}/within_distance      - Boolean: within hysteresis threshold
intake/sample/{lane}/hue                  - Hue (0-360°)
intake/sample/{lane}/sat                  - Saturation (0-1)
intake/sample/{lane}/val                  - Value/brightness (0-1)
intake/sample/{lane}/total_intensity      - R+G+B sum (0-765)
intake/sample/{lane}/scaled_r/g/b         - RGB values (0-255)
```

### Per-lane classification diagnostics:
```
intake/classifier/{lane}/reason                     - Why this classification? (no_presence, no_signal, GREEN, PURPLE, etc.)
intake/classifier/{lane}/hue_unwrapped              - Hue after purple wrap handling
intake/classifier/{lane}/hue_boundary               - Current decision boundary
intake/classifier/{lane}/hue_distance_from_boundary - How far hue is from boundary (degrees)
```

### Lane color state (after debouncing):
```
FTC Dashboard will show final detected colors per lane in the main telemetry
```

---

## Config Parameter Quick Reference

### Distance Thresholds (per robot)
**Path:** `IntakeSubsystem → laneSensorConfig → lanePresenceConfig20245`

- `leftEnterDistanceCm` - Enter threshold for LEFT lane
- `leftExitDistanceCm` - Exit threshold for LEFT lane (hysteresis)
- `centerEnterDistanceCm` - Enter threshold for CENTER lane
- `centerExitDistanceCm` - Exit threshold for CENTER lane
- `rightEnterDistanceCm` - Enter threshold for RIGHT lane
- `rightExitDistanceCm` - Exit threshold for RIGHT lane

**Typical values:** 3.5-6.0 cm (depends on robot geometry)
**Hysteresis gap:** 0.5-1.5 cm

### Sensor Hardware
**Path:** `IntakeSubsystem → laneSensorConfig → hardware`

- `enableSensorLight` - Turn on REV sensor white LED (usually true)
- `overrideSensorGain` - Set custom gain (usually true)
- `sensorGain` - Gain multiplier (typical: 20-30, max ~50)

### Quality Thresholds
**Path:** `IntakeSubsystem → laneSensorConfig → quality`

- `minSaturation` - Minimum color saturation (typical: 0.10-0.15)

### Debounce Gating
**Path:** `IntakeSubsystem → laneSensorConfig → gating`

- `consecutiveConfirmationsRequired` - Samples needed to confirm detection (typical: 1-3)
- `consecutiveClearConfirmationsRequired` - Samples needed to confirm clear (typical: 2-4)

### Background Detection
**Path:** `IntakeSubsystem → laneSensorConfig → background`

- `enableBackgroundDetection` - Enable field mat rejection (recommended: true)
- `backgroundHue` - Hue of empty space (calibrate with OpMode)
- `backgroundSaturation` - Saturation of empty space
- `backgroundValue` - Brightness of empty space
- `maxBackgroundDistance` - Max HSV distance to classify as background (typical: 30-40)

### Classifier Mode
**Path:** `IntakeSubsystem → laneSensorConfig → classifier`

- `mode` - "DECISION_BOUNDARY" (recommended), "RANGE_BASED", or "DISTANCE_BASED"

**Decision Boundary Parameters** (recommended):
- `hueDecisionBoundary` - Single threshold to split green/purple (typical: 165°)

---

## FAQ

**Q: Should I disable background detection to reduce false negatives?**

A: Only as a last resort. Background detection prevents false positives when intake is empty. Instead, calibrate background parameters properly using the calibration OpMode.

**Q: Which classifier mode should I use?**

A: DECISION_BOUNDARY is recommended - it's simplest and most robust. It treats green vs purple as a two-class problem with a single decision boundary in hue space.

**Q: My distance readings are very noisy (stddev >1.0 cm). What's wrong?**

A: This indicates hardware issues:
1. Check sensor mounting - vibration causes distance noise
2. Verify I2C cable is secure
3. Check for mechanical interference (artifact bouncing in lane)

**Q: How do I know if my sensor gain is too high?**

A: Watch the scaled RGB values in diagnostics. If all three are maxed at 255 when artifact is present, gain is too high (sensor saturated). Reduce gain until you see reasonable values (50-200 range).

**Q: Can I tune parameters during a match?**

A: Yes! All `@Configurable` parameters can be changed via FTC Dashboard during init or even while OpMode is running (depending on when the value is read). Changes take effect immediately.

---

## Example: Fixing High Noise (Stability <70%)

**Scenario:** CENTER lane shows 55% stability, 23 transitions in 50 samples

**Step 1 - Check reason codes:**
- Most samples show `GREEN` or `NONE` alternating
- A few show `distance_out`

**Step 2 - Increase distance hysteresis:**
```
centerEnterDistanceCm: 4.5 → 5.0
centerExitDistanceCm: 5.5 → 6.5  (larger gap)
```

**Step 3 - Increase debouncing:**
```
consecutiveConfirmationsRequired: 2 → 3
```

**Result:** Stability improves to 85%, transitions drop to 8

**If still unstable:** Check hardware (sensor mount, vibration, cables)

---

## Getting Help

If you've followed this guide and still have issues:

1. Capture diagnostics:
   - Run Diagnostics OpMode for 30 seconds with artifacts in lanes
   - Note stability percentages and transition counts
   - Screenshot telemetry showing noise analysis

2. Capture telemetry:
   - Connect to FTC Dashboard
   - Export telemetry showing `intake/sample/*` and `intake/classifier/*` keys

3. Check hardware:
   - Verify sensor is REV Color Sensor V3 (with distance)
   - Check I2C address conflicts (shouldn't have multiple sensors on same address)
   - Test sensor on bench with known-good artifact

4. Review config:
   - Export your current IntakeSubsystem config from FTC Dashboard
   - Compare to recommended values in this guide
