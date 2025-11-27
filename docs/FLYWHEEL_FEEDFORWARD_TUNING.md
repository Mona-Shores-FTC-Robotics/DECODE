# Flywheel Feedforward Control Tuning Guide

This guide explains how to characterize and tune the feedforward + proportional feedback control mode for launcher flywheels.

## What is Feedforward + Feedback Control?

The control system combines two strategies:

1. **Feedforward**: Predicts the power needed based on target velocity (no feedback)
2. **Proportional Feedback**: Corrects for errors based on actual velocity (optional)

The complete model is:
```
power = (kS + kV × targetRPM) + kP × (targetRPM - currentRPM)
         \_____feedforward_____/    \________feedback_________/
```

Where:
- **kS** (static friction): Minimum power needed to overcome friction and start spinning
- **kV** (velocity gain): Additional power per RPM (feedforward term)
- **kP** (proportional gain): Error correction strength (0 = pure feedforward, >0 = adds feedback)

## Per-Lane Tuning

**Each flywheel (LEFT, CENTER, RIGHT) has its own kS, kV, and kP values!**

This allows you to account for:
- Different motor wear/friction
- Mechanical differences (bearing quality, alignment)
- Weight imbalances in game pieces
- Different launch trajectories per lane

Configure in FTC Dashboard under:
- `LauncherFlywheelConfig` → `flywheelLeft` → `kS`, `kV`, `kP`
- `LauncherFlywheelConfig` → `flywheelCenter` → `kS`, `kV`, `kP`
- `LauncherFlywheelConfig` → `flywheelRight` → `kS`, `kV`, `kP`

## When to Use This Control Mode

**Best for:**
- Flywheels that run at consistent speeds (e.g., always 3000 RPM for launching)
- Systems where you want predictable, repeatable behavior
- Situations where bang-bang oscillation is unacceptable
- When different flywheels need different tuning

**Advantages:**
- Simple and predictable (feedforward)
- Corrects for disturbances (proportional feedback)
- Per-lane tuning for optimal performance
- Easy to characterize and tune

## Tuning Strategy: Start Simple, Add Complexity

### Phase 1: Pure Feedforward (Recommended Starting Point)
**Set kP = 0 for all lanes**

Tune kS and kV for each lane individually. This gives you predictable, stable control without oscillation.

### Phase 2: Add Feedback (If Needed)
**Set kP > 0 to add error correction**

If pure feedforward isn't accurate enough (e.g., RPM drifts under load), add a small kP value (start with 0.001-0.005).

## Characterization Process

### Step 1: Find kS (Static Friction) Per Lane

**Goal:** Determine the minimum power needed to overcome friction for each flywheel.

**Method:**
1. Set control mode to `FEEDFORWARD`
2. Test each lane individually
3. Gradually increase power until motor just starts spinning
4. Record kS for that lane (typically 0.05 - 0.15)

**What to look for:**
- Each lane may have different kS values
- LEFT might need 0.08, CENTER 0.10, RIGHT 0.12 (example)
- This is normal! It reflects mechanical differences

### Step 2: Collect Power-to-RPM Data Per Lane

**Goal:** Measure steady-state RPM at different power levels for each flywheel.

**Method:**
1. Test one lane at a time (disable others or test separately)
2. Apply 5-7 different power levels (e.g., 0.2, 0.4, 0.6, 0.8, 1.0)
3. For each power level:
   - Wait 2-3 seconds for velocity to stabilize
   - Record the steady-state RPM
4. Create a table of (power, RPM) pairs for that lane

**Example data for CENTER lane:**
```
Power | RPM (steady-state)
------|------------------
0.20  | 450
0.40  | 1400
0.60  | 2400
0.80  | 3400
1.00  | 4400
```

**Example data for LEFT lane (might be different!):**
```
Power | RPM (steady-state)
------|------------------
0.20  | 520
0.40  | 1520
0.60  | 2520
0.80  | 3520
1.00  | 4520
```

### Step 3: Calculate kV Per Lane

**Goal:** Find the linear relationship between power and RPM for each flywheel.

**Method:**
For each data point, calculate:
```
kV = (power - kS) / rpm
```

Then take the average of all kV values for that lane.

**Example for CENTER lane (kS = 0.10):**
```
Data point 1: kV = (0.20 - 0.10) / 450  = 0.000222
Data point 2: kV = (0.40 - 0.10) / 1400 = 0.000214
Data point 3: kV = (0.60 - 0.10) / 2400 = 0.000208
Data point 4: kV = (0.80 - 0.10) / 3400 = 0.000206
Data point 5: kV = (1.00 - 0.10) / 4400 = 0.000205

Average kV = 0.000211 for CENTER
```

Repeat for LEFT and RIGHT lanes!

### Step 4: Configure in FTC Dashboard

1. Connect to FTC Dashboard: `http://192.168.49.1:8080/dash`
2. Navigate to **Config** tab
3. Set values for each lane:

**For LEFT lane:**
```
LauncherFlywheelConfig → flywheelLeft
  kS: 0.08     (your measured value)
  kV: 0.000205 (your calculated value)
  kP: 0.0      (start with pure feedforward)
```

**For CENTER lane:**
```
LauncherFlywheelConfig → flywheelCenter
  kS: 0.10
  kV: 0.000211
  kP: 0.0
```

**For RIGHT lane:**
```
LauncherFlywheelConfig → flywheelRight
  kS: 0.12
  kV: 0.000218
  kP: 0.0
```

4. Set control mode:
```
LauncherFlywheelConfig → modeConfig → mode: FEEDFORWARD
```

### Step 5: Validate and Tune kP (Optional)

**Test with pure feedforward first (kP = 0):**
1. Command each lane to several target RPMs (1000, 2000, 3000, 4000)
2. Observe steady-state RPM for each lane
3. Check FTC Dashboard telemetry:
   - `velocity_rpm_LEFT/CENTER/RIGHT` - actual RPM
   - `ff_error_LEFT/CENTER/RIGHT` - velocity error
   - `ff_feedforward_LEFT/CENTER/RIGHT` - feedforward power component

**If errors are consistently too large, add proportional feedback:**
1. Start with small kP (0.001 - 0.005) for the problematic lane
2. Increase gradually if needed
3. Watch for oscillation (if RPM bounces, reduce kP)

**Typical kP values:**
- kP = 0: Pure feedforward (no correction)
- kP = 0.001-0.005: Gentle correction (recommended starting point)
- kP = 0.01-0.02: Aggressive correction (may oscillate)

## Using Telemetry for Tuning

When FEEDFORWARD mode is active, the following telemetry is logged for each lane:

| Telemetry Key | Description |
|---------------|-------------|
| `velocity_rpm_[LANE]` | Actual motor velocity in RPM |
| `ff_error_[LANE]` | Velocity error (target - actual) |
| `ff_feedforward_[LANE]` | Feedforward power component (kS + kV*rpm) |
| `ff_feedback_[LANE]` | Feedback power component (kP*error) |
| `ff_power_[LANE]` | Total applied motor power |

**Tuning with telemetry:**
1. **If ff_error is large and consistent** → Adjust kS or kV for that lane
2. **If ff_error oscillates** → Reduce kP for that lane
3. **If ff_feedforward looks wrong** → Re-characterize kS and kV
4. **If one lane differs significantly** → That's expected! Tune it independently

## Fine-Tuning Tips

### If one lane's velocity is too low:
- Increase kS (more base power)
- OR increase kV (steeper power-RPM slope)
- OR add small kP for correction

### If one lane's velocity is too high:
- Decrease kS (less base power)
- OR decrease kV (shallower slope)

### If one lane oscillates:
- Reduce kP for that lane
- Check for mechanical issues (loose belt, bad bearing)

### If all lanes are off by the same percentage:
- Check battery voltage (feedforward uses voltage compensation)
- Re-characterize with fresh battery (>12.5V)

### If behavior changes over time:
- Motor warming up (wait 30s before final characterization)
- Battery draining (voltage compensation should handle this)
- Mechanical wear (may need to re-tune periodically)

## Comparison to Other Control Modes

| Mode | Pros | Cons |
|------|------|------|
| **FEEDFORWARD** (kP=0) | Predictable, no oscillation, simple | No disturbance correction |
| **FEEDFORWARD** (kP>0) | Predictable + corrects errors | Needs kP tuning, may oscillate |
| **PURE_BANG_BANG** | Fast response, no tuning | Oscillates at target |
| **HYBRID** | Good accuracy | Complex PID tuning |
| **BANG_BANG_HOLD** | Fast + stable | Needs hold power tuning |

## Troubleshooting

### One lane behaves very differently from others
- **Normal!** This is why per-lane tuning exists
- Check mechanical differences (belt tension, bearing wear)
- Characterize that lane independently

### Feedforward power looks correct but RPM is wrong
- Mechanical issue (friction, binding)
- Wrong ticksPerRev or gearRatio in config
- Motor direction reversed

### Feedback term (kP) doesn't seem to help
- kP too small (try increasing)
- Feedback fighting feedforward (check signs)
- Need better feedforward tuning first

### All lanes need very different kS/kV values
- Check for mechanical issues
- Verify motors are same model
- May indicate worn/damaged components

### Velocity drifts over time despite good initial tuning
- Battery voltage changing (check voltage compensation is enabled)
- Motors heating up (characterize when warm)
- Mechanical wear (may need maintenance)

## Advanced: Non-Linear Models

If power-to-RPM relationship is not linear for a lane:

1. **Use different kV for different RPM ranges**
2. **Add quadratic term** (requires code modification)
3. **Use lookup table** (most accurate, more complex)

For most FTC applications, the simple linear model works well per-lane!

## Quick Reference

**Recommended tuning order:**
1. Characterize kS for each lane (find minimum power to spin)
2. Characterize kV for each lane (measure power vs RPM)
3. Start with kP = 0 (pure feedforward)
4. If needed, add small kP (0.001-0.005) for problem lanes only
5. Adjust kS/kV first, kP second

**Typical values:**
- kS: 0.05 - 0.15 (varies per lane)
- kV: 0.00015 - 0.00025 (varies per lane)
- kP: 0.0 (pure FF) or 0.001 - 0.005 (with feedback)

**Dashboard path:**
- Config → LauncherFlywheelConfig → flywheelLeft/Center/Right → kS, kV, kP
- Config → LauncherFlywheelConfig → modeConfig → mode → FEEDFORWARD
