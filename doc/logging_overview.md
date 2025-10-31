# Logging & AdvantageScope Lite Overview

## Live Streaming
- `RobotLogger` now owns the single pipeline for AdvantageScope Lite and optional PsiKit CSV capture.
- Each subsystem exposes an `Inputs` data class (`DriveSubsystem.Inputs`, `ShooterSubsystem.Inputs`, etc.).
- During each loop, `Robot` calls `populateInputs(...)` for every subsystem and forwards the result to `RobotLogger.logInputs(...)`.
- `RobotLogger` reflects over the fields and publishes numbers/booleans/strings under `Subsystem/field` topics. AdvantageScope Lite and PsiKit see identical data.
- Pedro tuning OpModes still use FTControl Panels directly; logging updates do not touch `pedroPathing/*`.
- Utility subsystems that sit outside of `Robot` (e.g. `LauncherCoordinator`, `VisionSubsystemLimelight`) expose `attachLogger(...)` helpers so diagnostics can opt-in to the same pipeline.

## Extending A Subsystem
1. Add a new public field to that subsystem’s `Inputs` class (keep it primitive/enum/string where possible).
2. Populate it inside `populateInputs(...)`.
3. AdvantageScope Lite automatically discovers the new topic the next time you run the OpMode.

## Configuration & Toggles
- Keep runtime knobs in `@Configurable` classes beside the subsystem. AdvantageScope Lite reads them via NT4 so you can tune live.
- Global logging toggles (e.g. `TelemetrySettings.enablePsiKitLogging`) stay in `TelemetrySettings`.

## PsiKit CSV Replay
- When `TelemetrySettings.enablePsiKitLogging` is `true`, RobotLogger mirrors every published input into the on-robot CSV (`/sdcard/FIRST/PsiKitLogs/log_*.csv`).
- Drag the CSV into AdvantageScope to replay the same topics you saw live.
