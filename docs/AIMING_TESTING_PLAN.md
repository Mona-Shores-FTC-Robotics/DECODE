# Aiming Methods - Testing Plan

## Overview

You have **FOUR** aiming approaches available for testing. This document explains each method, when to use it, and provides a systematic testing plan.

---

## 🎯 THE FOUR AIMING METHODS

### Method 1: Geometry-Based (Continuous Tracking)

**Command:** `AimAndDriveCommand`
**Method:** `DriveSubsystem.aimAndDrive()`
**Button:** B (hold)

**How it works:**
- Calculates angle from robot pose to **basket incenter** using `atan2(dy, dx)`
- Uses odometry pose and inscribed circle center as target
- **Incenter ≈ (5.95, 131.82) for Blue, (138.05, 131.82) for Red**
- Continuously tracks target every loop while you drive

**Pros:**
- ✅ Can aim from anywhere on field
- ✅ Smooth continuous tracking as you move
- ✅ Works without AprilTag visibility (uses odometry)
- ✅ Incenter is better than centroid (more centered in basket opening)

**Cons:**
- ❌ Requires accurate odometry
- ❌ Sensitive to odometry drift
- ❌ Coordinate frame issues can cause wrong angles

**Tuning (FTC Dashboard → DriveSubsystem → AimAssistConfig):**
- `kP`: 0.5 (proportional gain)
- `kMaxTurn`: 0.7 (max turn speed)

**Fine-tuning for projectile arc (FTC Dashboard → FieldConstants → BasketTargets):**
- `deltaY`: 0.0 (offset from incenter)
  - If shots land short: Increase deltaY (try +2 to +5)
  - If shots overshoot: Decrease deltaY (try -2 to -5)
- `deltaX`: 0.0 (typically leave at 0)

---

### Method 2: Vision-Centered (Continuous Tracking)

**Command:** `AimAndDriveVisionCenteredCommand`
**Method:** `DriveSubsystem.aimAndDriveVisionCentered()`
**Button:** Not yet bound

**How it works:**
- Uses Limelight `tx` (horizontal offset) to center AprilTag in camera view
- When error = 0, target is centered
- **No coordinate math** - just direct camera feedback
- Similar to original FTC `RobotAutoDriveToAprilTagOmni` example

**Pros:**
- ✅ Self-correcting (always aims at what camera sees)
- ✅ No coordinate frame issues
- ✅ No odometry dependency
- ✅ Simple and intuitive

**Cons:**
- ❌ Requires AprilTag visibility
- ❌ If tag not visible, robot holds heading (no turn)
- ❌ Camera must be properly aligned with robot

**Tuning (FTC Dashboard → DriveSubsystem → VisionCenteredAimConfig):**
- `kP`: 1.7 (motor power per radian, equiv. to 0.03 per degree)
- `kMaxTurn`: 0.7 (max turn speed)
- `deadbandDeg`: 1.0 (stop turning when tx < 1°)

---

### Method 3: Fixed-Angle (Continuous Tracking)

**Command:** `AimAndDriveFixedAngleCommand`
**Method:** `DriveSubsystem.aimAndDriveFixedAngle()`
**Button:** Not yet bound

**How it works:**
- Rotates to a fixed heading based on alliance
- Default: 60° for blue, 120° for red
- Driver positions on launch line, presses button, robot aims to fixed angle
- **No vision or pose calculations needed**

**Pros:**
- ✅ Simplest method
- ✅ Predictable and consistent
- ✅ Easy for drivers to understand
- ✅ No vision or odometry needed

**Cons:**
- ❌ Only works from specific field positions (launch line)
- ❌ No dynamic tracking - fixed angle only
- ❌ If robot moves, angle doesn't update

**Tuning (FTC Dashboard → DriveSubsystem → FixedAngleAimConfig):**
- `blueHeadingDeg`: 60.0 (blue alliance fixed angle)
- `redHeadingDeg`: 120.0 (red alliance fixed angle)
- `kP`: 0.5 (proportional gain)
- `kMaxTurn`: 0.7 (max turn speed)

---

### Method 4: Capture-and-Aim (Snap Turn)

**Command:** `CaptureAndAimCommand`
**Method:** Uses `Follower.turnTo()` (Pedro's PIDF)
**Button:** X (in VisionRedo)

**How it works:**
1. **Sampling phase:** Captures target heading over multiple frames (default 3)
2. **Turn phase:** Uses Pedro Follower's `turnTo()` for discrete rotation
3. **Driver lockout:** Robot completes turn autonomously, then command ends

**Key differences from continuous methods:**
- **One-time snap:** Captures heading once, then turns to it
- **Autonomous turn:** Uses Pedro's built-in PIDF control
- **Locks out driver:** Can't translate during turn
- **Better for autonomous:** Clean, discrete turn action

**Pros:**
- ✅ Clean discrete action (good for autonomous sequences)
- ✅ Uses Pedro's tuned PIDF control
- ✅ Filters noise by averaging multiple frames
- ✅ Deterministic - you know when it's done

**Cons:**
- ❌ Locks out driver during turn
- ❌ Can't strafe while aiming
- ❌ Less natural for teleop play
- ❌ Target can move while turning (stale capture)

**Tuning (FTC Dashboard → CaptureAndAimCommand → Config):**
- `sampleFrames`: 3 (number of vision samples to average)
- `frameSampleIntervalMs`: 50.0 (time between samples)
- `samplingTimeoutMs`: 500.0 (max time for sampling before giving up)

**When to use:**
- ✅ Autonomous sequences
- ✅ When you want a clean "snap and shoot" action
- ❌ NOT for dynamic teleop driving

---

## 📋 SYSTEMATIC TESTING PLAN

### Phase 1: Fixed-Angle Validation (Simplest First)

**Goal:** Establish a known-good baseline angle from the launch line.

**Steps:**
1. **Bind fixed-angle to Y button** (or replace B button temporarily)
2. **Position robot on launch line** (middle of field, specific distance from basket)
3. **Press aim button** - robot should rotate to fixed angle
4. **Shoot and observe:**
   - Does ball go in basket?
   - Consistently left/right/short/long?
5. **Tune fixed angle via FTC Dashboard:**
   - Adjust `blueHeadingDeg` / `redHeadingDeg`
   - Find the angle that consistently scores
6. **Record the winning angle** (e.g., "65° blue, 115° red")

**Success criteria:** Can score 8/10 shots from launch line with fixed angle.

---

### Phase 2: Vision-Centered Testing

**Goal:** Test if camera-based aiming works better than fixed angle.

**Steps:**
1. **Bind vision-centered to X button**
2. **Position robot on launch line** (same spot as Phase 1)
3. **Press aim button while holding position**
   - Robot should rotate to center AprilTag in camera
4. **Check telemetry:**
   - Is tx error close to zero when aimed?
   - Is heading stable (not oscillating)?
5. **Shoot and observe:**
   - Does it score better/worse than fixed angle?
6. **Test from multiple field positions:**
   - Mid-field left
   - Mid-field right
   - Close to basket
   - Far from basket
7. **Tune if needed:**
   - If oscillates: Decrease `kP` (try 1.0-1.5)
   - If too slow: Increase `kP` (try 2.0-2.5)
   - If chatters: Increase `deadbandDeg` (try 2.0-3.0)

**Success criteria:** Can score from multiple field positions with vision tracking.

---

### Phase 3: Geometry-Based Testing

**Goal:** Test if pose-based aiming works with inscribed circle.

**Steps:**
1. **Ensure robot pose is accurate:**
   - Start from known position
   - OR use vision relocalization (A button) to set pose
2. **Bind geometry-based to B button** (current default)
3. **Drive around field and test aiming:**
   - Mid-field
   - Corners
   - Close/far from basket
4. **Check telemetry for aim error:**
   - Is robot pointing at incenter coordinates?
   - Does angle look correct visually?
5. **Shoot from various positions**
6. **If shots are consistently off (e.g., always left):**
   - Adjust `BasketTargets.deltaY` in FTC Dashboard
   - Positive deltaY = aim further back
   - Negative deltaY = aim more forward
7. **Compare to vision-centered:**
   - Which is more accurate?
   - Which feels better for driver?

**Success criteria:** Geometry matches vision-centered accuracy from same positions.

---

### Phase 4: Capture-and-Aim Testing

**Goal:** Evaluate snap-turn approach for specific use cases.

**Steps:**
1. **Bind capture-and-aim to X button** (or test separately)
2. **Test in stationary scenarios:**
   - Robot stopped at launch line
   - Press X, robot snaps to aim angle
   - Shoot
3. **Observe:**
   - Is turn smooth and accurate?
   - Does averaging help with vision noise?
   - Is driver lockout acceptable?
4. **Compare to continuous methods:**
   - B button (geometry): Can strafe while aiming
   - X button (capture): Snap turn, then shoot
   - Which do drivers prefer?

**Success criteria:** Understand when capture-and-aim is better/worse than continuous.

---

## 🏆 RECOMMENDED STRATEGY

Based on the analysis, here's my recommended testing order and strategy:

### Recommended Testing Sequence

1. **Start with Fixed-Angle (Method 3)**
   - Simplest to validate
   - Establishes baseline performance
   - Proves your launcher mechanics work
   - Record the winning angle for reference

2. **Then Vision-Centered (Method 2)**
   - Should work like your original FTC example
   - Self-correcting, no coordinate issues
   - Test from multiple positions
   - This is likely your **best teleop option**

3. **Then Geometry-Based (Method 1)**
   - Now uses inscribed circle (better than centroid)
   - Good for when AprilTag not visible
   - Requires accurate odometry
   - Use `deltaY` tuning for projectile arc

4. **Finally Capture-and-Aim (Method 4)**
   - Evaluate for autonomous sequences
   - Likely NOT your primary teleop method
   - Useful for "snap and shoot" scenarios

### Final Button Binding Recommendation

For **matches**, I recommend:

**Option A: Vision + Geometry (Self-Adaptive)**
- **B button (hold):** Geometry-based continuous tracking
  - Works everywhere, even without AprilTag visibility
  - Falls back gracefully
- **X button (hold):** Vision-centered continuous tracking
  - Use when AprilTag visible for best accuracy
  - Driver chooses which to use based on situation
- **Y button:** Fixed-angle (emergency backup from launch line)
- **A button:** Vision relocalization

**Option B: Vision Only (Simpler)**
- **B button (hold):** Vision-centered continuous tracking
  - Primary aiming method
  - Requires AprilTag visibility
- **Y button:** Fixed-angle (backup when vision fails)
- **A button:** Vision relocalization

### Why NOT Capture-and-Aim for TeleOp?

**Capture-and-aim is great for:**
- ✅ Autonomous sequences (clean, discrete actions)
- ✅ When driver can pause and wait for turn
- ✅ Reducing vision noise through averaging

**But for teleop it has issues:**
- ❌ Locks out driver control during turn
- ❌ Can't strafe while aiming (reduces mobility)
- ❌ Target capture can go stale while turning
- ❌ Feels unnatural compared to continuous tracking

**Continuous methods (B button) are better because:**
- ✅ Driver keeps full translation control
- ✅ Can position while aiming
- ✅ More natural feel
- ✅ Real-time tracking (never stale)

---

## 🔧 TUNING GUIDE

### Inscribed Circle vs Centroid

Your VisionRedo correctly identified that **inscribed circle (incenter)** is better than centroid:

- **Centroid:** Simple average of triangle vertices
  - Blue: (8.33, 134.33)
  - Red: (135.67, 134.33)

- **Incenter:** Center of inscribed circle (touches all three sides)
  - Blue: (5.95, 131.82)
  - Red: (138.05, 131.82)

**Why incenter is better:**
- More centered relative to all basket edges
- Sits deeper in the basket opening
- Better clearance from all sides
- Less likely to clip basket rim

### Projectile Arc Adjustment

If your projectile has an arc, you may need to aim slightly different than the incenter:

**High arc (ball goes up then down):**
- Aim further back: `BasketTargets.deltaY = +2` to `+5`

**Low/flat arc:**
- Aim more forward: `BasketTargets.deltaY = -2` to `-5`

**To find the right deltaY:**
1. Use fixed-angle method to find angle that scores
2. Calculate what Y coordinate produces that angle from launch line
3. Set `deltaY` to match that coordinate

---

## 📊 COMPARISON MATRIX

| Feature | Geometry | Vision-Centered | Fixed-Angle | Capture-Aim |
|---------|----------|-----------------|-------------|-------------|
| Works without AprilTag | ✅ | ❌ | ✅ | Partial* |
| Works from anywhere | ✅ | ✅ | ❌ | ✅ |
| Driver can translate | ✅ | ✅ | ✅ | ❌ |
| Self-correcting | ❌ | ✅ | ❌ | Partial* |
| Simple (no tuning) | ❌ | ✅ | ✅ | ❌ |
| Real-time tracking | ✅ | ✅ | ✅ | ❌ |
| Best for autonomous | ❌ | ❌ | ❌ | ✅ |
| Best for teleop | ⚠️ | ✅ | ⚠️ | ❌ |

*Partial = Falls back to geometry-based if vision unavailable

---

## 🎮 TESTING CHECKLIST

Use this checklist during testing sessions:

### Setup
- [ ] Robot fully charged
- [ ] FTC Dashboard connected (`http://192.168.49.1:8080/dash`)
- [ ] Alliance color set correctly
- [ ] Vision relocalization completed (A button)
- [ ] Launch line position marked on field

### Per Method Testing
- [ ] Button bound correctly
- [ ] Telemetry showing aim error
- [ ] Test from 5 different field positions
- [ ] Record success rate (shots made / shots taken)
- [ ] Note any consistent bias (left/right/short/long)
- [ ] Tune parameters if needed
- [ ] Re-test after tuning

### Driver Feedback
- [ ] Which method feels most natural?
- [ ] Which is easiest to use under pressure?
- [ ] Any confusing behaviors?
- [ ] Preference for teleop matches?

### Final Decision
- [ ] Primary aiming method selected
- [ ] Backup method selected
- [ ] Button bindings finalized
- [ ] Parameters tuned and saved
- [ ] Drivers trained on final setup

---

## 🚀 QUICK START

**Don't want to read everything? Start here:**

1. **Bind all three continuous methods:**
   - B: Geometry
   - X: Vision-centered
   - Y: Fixed-angle

2. **Start with Y (fixed-angle) on launch line:**
   - Find angle that scores
   - Record it

3. **Test X (vision-centered) from same spot:**
   - Should match fixed-angle performance
   - Test from other positions too

4. **Test B (geometry) with vision relocalization:**
   - Press A to relocalize
   - Then press B to aim
   - Compare to vision-centered

5. **Pick your favorite for matches!**

---

## 📞 TROUBLESHOOTING

### "Geometry-based aims wrong even after relocalization"

- Check `deltaY` offset in FTC Dashboard
- Verify incenter calculation matches VisionRedo
- Compare aimed angle to vision-centered angle
- If consistently off by X degrees, adjust deltaY

### "Vision-centered oscillates/chatters"

- Decrease `kP` (try 1.0-1.5)
- Increase `deadbandDeg` (try 2.0-3.0)
- Check camera mount - must be stable

### "Fixed-angle works but others don't"

- This tells you launcher mechanics are good
- Problem is in aiming calculation or odometry
- Focus on vision-centered (eliminates odometry)
- Use vision relocalization before testing geometry

### "All methods aim left/right consistently"

- Check camera alignment (vision methods)
- Check field coordinate system (geometry method)
- For fixed-angle: Just adjust the fixed angle

---

**Good luck with testing! Start simple (fixed-angle), then add complexity.**
