# Logging & Telemetry Overview

## Live Streaming Architecture
- `RobotLogger` collects subsystem Inputs data via reflection and stores topics in a Map
- Each subsystem exposes an `Inputs` data class (`DriveSubsystem.Inputs`, `LauncherSubsystem.Inputs`, etc.)
- During each loop, `Robot` calls `populateInputs(...)` for every subsystem and forwards the result to `RobotLogger.logInputs(...)`
- `RobotLogger` reflects over the fields and stores numbers/booleans/strings under `Subsystem/field` topics
- `TelemetryService` retrieves all topics from `RobotLogger` and publishes them to FTC Dashboard packets
- AdvantageScope desktop app connects to FTC Dashboard for real-time visualization of all topics
- Pedro tuning OpModes still use FTControl Panels directly; logging updates do not touch `pedroPathing/*`
- Utility subsystems outside `Robot` (e.g. `LauncherCoordinator`, `VisionSubsystemLimelight`) can opt into the pipeline

## Offline Logging with KoalaLog
- KoalaLog produces `.wpilog` files compatible with AdvantageScope for post-match analysis
- Initialized via `KoalaLog.setup(hardwareMap)` in `Robot` constructor
- `AutoLogManager.periodic()` called in all OpModes to sample logged data
- Robot pose automatically logged for 2D/3D field visualization
- Retrieve logs using LogPuller tools from [KoalaLog GitHub](https://github.com/Koala-Log/Koala-Log)

## Extending A Subsystem
1. Add a new public field to that subsystem's `Inputs` class (keep it primitive/enum/string where possible)
2. Populate it inside `populateInputs(...)`
3. Topic automatically appears in FTC Dashboard (and AdvantageScope desktop) as `Subsystem/fieldName`
4. Field is also automatically logged to WPILOG files via KoalaLog

## Configuration & Toggles
- Keep runtime knobs in `@Configurable` classes beside the subsystem for live tuning via FTC Dashboard
- Global telemetry toggles (e.g. `TelemetrySettings.enableDashboardTelemetry`) stay in `TelemetrySettings`
