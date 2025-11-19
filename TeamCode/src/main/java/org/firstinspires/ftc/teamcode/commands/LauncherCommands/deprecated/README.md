# Deprecated Launcher Commands

This folder contains commands that are no longer actively used but preserved for reference.

## Commands in this folder

### LaunchModeAwareCommand.java
**Replaced by:** `UniversalSmartShotCommand`

**What it did:** Mode-aware firing that adapted between THROUGHPUT (all lanes rapid fire) and DECODE (obelisk sequence) based on `RobotState.getLauncherMode()`.

**Why deprecated:** `UniversalSmartShotCommand` does the same thing PLUS distance-based RPM calculation, making this command redundant.

**Last used:** D-Pad Down binding (removed in operator controls simplification)

---

### SpinUpUntilReadyCommand.java
**Replaced by:** Integrated into `UniversalSmartShotCommand`

**What it did:** Spun up flywheels to target RPM and waited until ready, then completed.

**Why deprecated:** Only used by `LaunchModeAwareCommand`. When that was deprecated, this became unused.

**Last used:** Internal component of `LaunchModeAwareCommand`

---

### LaunchAllAtAutoRangeCommand.java
**Replaced by:** `ContinuousDistanceBasedSpinCommand` and `UniversalSmartShotCommand`

**What it did:** Automatically selected SHORT/MID/LONG range based on distance to goal using discrete range buckets.

**Why deprecated:** Discrete range selection (SHORT/MID/LONG) is less accurate than continuous RPM interpolation provided by newer distance-based commands.

**Last used:** Only in factory method, never bound to buttons

---

### LaunchAllAtDistanceBasedRPMCommand.java
**Replaced by:** `ContinuousDistanceBasedSpinCommand` and `UniversalSmartShotCommand`

**What it did:** One-shot command that calculated distance, set RPM, spun up, and fired all at once.

**Why deprecated:** Hold-to-spin approach (`ContinuousDistanceBasedSpinCommand`) provides better operator control and continuous distance updates. For one-press firing, `UniversalSmartShotCommand` adds mode-awareness.

**Last used:** Only in factory method, never bound to buttons

---

## When to use these

**Do not use these commands in new code.** They are kept only for:
1. Historical reference
2. Understanding command evolution
3. Potential bug investigation (if old behavior needs to be compared)

## Current recommended commands

For distance-based firing:
- **Hold-to-spin:** `ContinuousDistanceBasedSpinCommand` (X button)
- **One-press ultimate:** `UniversalSmartShotCommand` (D-Pad Left)

For mode-aware firing:
- **Use:** `UniversalSmartShotCommand` (includes mode-awareness + distance-based RPM)

For manual range selection:
- **Use:** `LaunchAllAtPresetRangeCommand` with SHORT/MID/LONG (A/B buttons)

---

**Date deprecated:** November 19, 2024
**Deprecated by:** Operator controls simplification PR
