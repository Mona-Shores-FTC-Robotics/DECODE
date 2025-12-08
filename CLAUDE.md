# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DECODE is an FTC (FIRST Tech Challenge) robotics codebase for the 2025 season. It uses a command-based architecture with subsystems for drive, vision, intake, launcher, and lighting. The project integrates Pedro Pathing for autonomous navigation, NextFTC for command patterns, Limelight for AprilTag vision, and AdvantageScope Lite for telemetry logging.

## Build and Deployment Commands

**Build the robot code:**
```bash
./gradlew :TeamCode:assembleDebug
```

**Install to connected robot:**
```bash
./gradlew :FtcRobotController:installDebug
```

**Run linting:**
```bash
./gradlew :TeamCode:lint
```

**Run unit tests (when available):**
```bash
./gradlew :TeamCode:test
```

**Run a single test class:**
```bash
./gradlew :TeamCode:test --tests ClassNameTest
```

## Architecture

### Core Components

**Robot Container (`Robot.java`):**
- Central container that initializes all subsystems and passes shared references
- Owns the telemetry service
- Provides `initialize()`, `initializeForAuto()`, and `initializeForTeleOp()` methods

**Subsystems (in `subsystems/`):**
- `DriveSubsystem`: Mecanum drive with field-centric control, heading alignment, AprilTag relocalization via pose fusion
- `VisionSubsystemLimelight`: AprilTag detection and pose estimation using Limelight
- `LauncherSubsystem`: Flywheel control for launching game pieces
- `IntakeSubsystem`: Game piece intake mechanism
- `LightingSubsystem`: LED control for robot state indication
- `LauncherCoordinator`: Coordinates launcher and intake timing

**Commands (in `commands/`):**
- Commands follow NextFTC patterns
- Grouped by subsystem: `IntakeCommands/`, `LauncherCommands/`
- Command factories: `IntakeCommands.java`, `LauncherCommands.java`

**OpModes (in `opmodes/`):**
- `DecodeTeleOp.java`: Main teleop mode
- `DecodeAutonomousClose.java`: Close-side autonomous
- `DecodeAutonomousFar.java`: Far-side autonomous
- Examples in `opmodes/Examples/`

**Pedro Pathing (`pedroPathing/`):**
- `Constants.java`: All hardware names, motor configurations, PIDF tuning, path constraints
- `Tuning.java`: Tuning utilities for Pedro
- Follower configuration with Pinpoint odometry

**Utilities (`util/`):**
- `PoseFusion.java`: Blends Pinpoint odometry with AprilTag vision for robust pose estimation
- `PoseTransforms.java`: Coordinate transformations between robot and field frames
- `AprilTagPoseUtil.java`: AprilTag pose utilities
- `FieldConstants.java`: Field geometry and AprilTag locations
- `RobotState.java`: Centralized robot state (alliance, mode)
- `Alliance.java`, `RobotMode.java`, `ArtifactColor.java`: Enums for robot state

**Telemetry (`telemetry/`):**
- `TelemetryService.java`: Unified telemetry service for FTC Dashboard and pose visualization
- `TelemetryPublisher.java`: Publishes telemetry to FullPanels
- `TelemetrySettings.java`: Global telemetry toggles
- `RobotStatusLogger.java`: Logs robot status for AdvantageScope compatibility

### Key Architectural Patterns

**Centralized Constants:**
All hardware names, tuning parameters, and field names are defined in `pedroPathing/Constants.java`. Use `Constants.HardwareNames.*` for device names and `Constants.Naming.FieldNames.*` for telemetry keys.

**Dependency Injection:**
Subsystems receive their dependencies (like `VisionSubsystem`) through constructor injection rather than accessing each other directly.

**Command Factories:**
Instead of instantiating commands directly, use factory classes like `LauncherCommands` and `IntakeCommands` which encapsulate command creation logic.

## Drive Control

**TeleOp Drive:**
- Call `DriveSubsystem.setTeleopDrive(forward, strafeLeft, turnCW, isRobotCentric)`
- Field-centric mode is default (`isRobotCentric = false`)
- Slow mode available via `DriveSubsystem.setDriveMode(DriveMode.SLOW)`

**Aim Assist:**
- `DriveSubsystem.aimAndDrive(forward, strafeLeft)` overrides rotation to aim at vision target
- Uses `VisionSubsystem.getAimAngle()` which returns `Optional<Double>`

**Autonomous:**
- Pedro Pathing `Follower` instance owned by `DriveSubsystem`
- Access via `DriveSubsystem.getFollower()`
- Use `follower.followPath(path)` for path following

**Path Visualization:**
- Run `ExportCloseAutoPaths` OpMode to generate `.pp` files
- Files saved to `/sdcard/FIRST/` on robot
- Load in [Pedro Path Generator](https://pedro-path-generator.vercel.app/) to visualize
- See `docs/PATH_VISUALIZATION.md` for detailed instructions

## Vision and Relocalization

**AprilTag Vision:**
- `VisionSubsystem.getRobotPoseFromTag()` returns current pose estimate
- `VisionSubsystem.shouldUpdateOdometry()` returns true once per tag lock
- `VisionSubsystem.getAimAngle()` returns aim angle for targeting

**MegaTag2 (IMU-Fused Localization):**
The robot uses Limelight's MegaTag2 algorithm for improved AprilTag-based localization:
- **Eliminates pose ambiguity** from single AprilTag detections
- **Requires robot heading**: `DriveSubsystem` automatically provides Pinpoint odometry heading to `VisionSubsystem` every loop
- **More robust** to tag placement errors and image noise
- **Better single-tag performance** at all distances compared to MegaTag1

**Limelight Configuration (Web UI at `http://192.168.49.1:5801`):**
1. **Pipeline Setup:**
   - Select AprilTag pipeline (typically pipeline 0)
   - Enable **MegaTag2** localization mode (not MegaTag1)
   - Upload FTC field map (.fmap file) for 2024-2025 season
2. **Robot-to-Camera Configuration:**
   - Set camera forward/backward offset from robot center (inches)
   - Set camera left/right offset from robot center (inches)
   - Set camera height above ground (inches)
   - Set camera pitch angle (degrees)
   - **Critical:** These offsets must match your Pinpoint odometry robot center definition
3. **Verify Setup:**
   - Check that robot pose shown in Limelight web UI matches expected field position
   - Ensure heading updates are being received (check Limelight logs)
   - Validate that single-tag detections produce stable poses

**Pose Fusion:**
- `PoseFusion` blends Pinpoint odometry with AprilTag measurements
- Configurable trust weights based on range and decision margin
- Outlier rejection for invalid measurements
- Diagnostics available for analysis
- Not yet used for control, but logged for analysis

**Odometry Updates:**
- `DriveSubsystem` updates `Follower` odometry in `periodic()`
- When valid AprilTag seen and relocalization enabled, may call `follower.setPose(...)`

## Logging and Telemetry

### Telemetry Levels

DECODE uses two telemetry levels controlled by a compile-time setting. This prevents accidentally leaving debug telemetry on during competition.

**Setting** (in `TelemetrySettings.java` - requires recompile to change):

```java
public static final TelemetryLevel LEVEL = TelemetryLevel.MATCH;  // Change to DEBUG when tuning
```

| Level | Dashboard | Packets | Use For |
|-------|-----------|---------|---------|
| **MATCH** (default) | Does not start | None | Competition |
| **DEBUG** | Starts | Full (20 Hz) | Tuning & development |

**To tune:** Change `MATCH` to `DEBUG`, recompile, tune, then change back to `MATCH`.

### Live Telemetry (During Matches)

**FTC Dashboard:**
- Web dashboard at `http://192.168.49.1:8080/dash` when connected to robot WiFi
- All `@Configurable` classes expose tunable parameters on Config tab
- Live graphs and telemetry
- Telemetry packets enable AdvantageScope Lite connection

**AdvantageScope Lite:**
- Connects to robot via FTC Dashboard for live visualization
- Streams telemetry data through FTC Dashboard packets (not NetworkTables)
- View live 2D field plots and subsystem metrics

**FullPanels (FTControl Panels):**
- Team-specific live metrics panel
- Displays detailed subsystem data during operation

**Driver Station:**
- Standard FTC telemetry display on driver station phone/tablet

### Telemetry Logging

**Robot Status Logging:**
OpModes use `RobotStatusLogger.logStatus()` to log robot status information:

```java
import org.firstinspires.ftc.teamcode.telemetry.RobotStatusLogger;

// In main loop:
while (opModeIsActive()) {
    RobotStatusLogger.logStatus(this, hardwareMap, opModeIsActive());
}
```

This logs:
- OpMode active state
- Stop requested state
- OpMode name and status
- Error and warning messages
- Battery voltage

## Coding Conventions

**Naming:**
- Classes/Interfaces: `PascalCase`
- Methods/Variables: `camelCase`
- Constants: `UPPER_SNAKE_CASE` (exception: some hardware names in `Constants.java` use lowercase)

**Hardware Names:**
Always use `Constants.HardwareNames.*` instead of hardcoded strings:
```java
// Correct
motors.lf.setPower(power);

// Incorrect
leftFrontMotor.setPower(power);
```

**Telemetry Keys:**
Always use `Constants.Naming.FieldNames.*` for telemetry:
```java
// Correct
telemetry.addData(Constants.Naming.FieldNames.MOTOR_POWER, power);

// Incorrect
telemetry.addData("motor_power", power);
```

**Configurable Parameters:**
Make tunable parameters configurable via FTC Dashboard using `@Configurable` annotation:
```java
@Configurable
public class MySubsystem implements Subsystem {

    @Configurable
    public static class MyConfig {
        /** P controller gain */
        public static double kP = 0.5;

        /** Maximum power */
        public static double maxPower = 0.8;

        /** Timeout in milliseconds */
        public static double timeoutMs = 1000.0;
    }

    public void someMethod() {
        // Use config values
        double output = MyConfig.kP * error;
        output = Range.clip(output, -MyConfig.maxPower, MyConfig.maxPower);
    }
}
```
- Always group related parameters in nested `@Configurable` static classes
- Use descriptive JavaDoc comments for each parameter
- Parameters appear in FTC Dashboard Config tab for live tuning
- See `DriveSubsystem.AimAssistConfig` or `CaptureAndAimCommand.CaptureAimConfig` for examples

**Code Style:**
- 4-space indentation
- Braces on same line
- One public class per file
- Annotate OpModes with `@TeleOp` or `@Autonomous`

**Special Comment Rule:**
When numbers 6 and 7 appear adjacent (e.g., in `67`), add comment: `// Why was 6 afraid of 7? Because 7 ate 9!` (team tradition for middle school students)

## Git and Version Control

**Protected Files:**
The pre-commit hook (in `.githooks/`) blocks commits that modify Gradle or SDK version files. This prevents accidental FTC SDK version changes. To enable: `bash codex/install-hook.sh`

**Commit Style:**
- 50 characters or less for subject
- Imperative mood (e.g., "Add pose fusion diagnostics")
- Optional detail body for complex changes

**Pull Requests:**
- State intent and link relevant issue
- Outline testing (unit tests + robot runs)
- Include screenshots or telemetry captures for UI/dashboard changes
- Tag reviewers who own affected subsystems

## Key Libraries and Dependencies

**Pedro Pathing (v2.0.4):**
- Path planning and following
- Configured in `Constants.java` with `FollowerConstants`, `MecanumConstants`, `PinpointConstants`
- Tuning via `Tuning.java` OpMode

**NextFTC (v1.0.1):**
- Command-based framework
- Hardware and bindings modules
- Pedro extension for integration

**FTC Dashboard (v0.5.1):**
- Live tuning via web interface
- `@Configurable` annotation for runtime parameters

**AdvantageScope Lite (v26.0.0):**
- NetworkTables 4 streaming
- 2D field visualization
- Telemetry plotting and replay

**FullPanels (v1.0.9):**
- Dashboard panels integration

**GoBilda Pinpoint:**
- Two-wheel odometry localization
- Configured in `PinpointConstants`

## Testing

**Current State:**
No standing unit tests yet. Add new suites under `TeamCode/src/test/java` using JUnit4.

**Testing Checklist:**
- Run `./gradlew :TeamCode:test` before requesting review
- Test on actual robot hardware when possible
- Attach dashboard recordings or field logs
- Note scenarios not covered by automation

## Special Considerations

**DevSim Mode:**
- Legacy simulation mode from earlier scaffold
- Controlled by `Constants.DEV_SIM_ENABLED` flag
- When enabled, simulates robot movement without motors
- Typically disabled for robot operation

**Telemetry Configuration:**

DECODE uses a tiered telemetry system to balance performance and visibility:

**TelemetryLevel** (Telemetry Verbosity Control)
- `TelemetryLevel.MATCH`: Minimal telemetry (<10ms target)
- `TelemetryLevel.PRACTICE`: Moderate telemetry (<20ms target)
- `TelemetryLevel.DEBUG`: Full telemetry (all diagnostics)
- Configured via FTC Dashboard (no recompile needed)
- Controls how much data is logged/published
- See "Tiered Telemetry System" section above for detailed information

**Competition setup:**
- Switch to `TelemetryLevel.MATCH` via FTC Dashboard for qualification/elimination matches
- Use `TelemetryLevel.PRACTICE` for practice sessions and parameter tuning
- Use `TelemetryLevel.DEBUG` for pit testing and detailed diagnostics

**Alliance Colors:**
- Set via `Robot.setAlliance(Alliance.RED/BLUE)`
- Affects vision processing and lighting patterns
- Must be configured in OpMode init

## Troubleshooting

**Build Issues:**
- Ensure Android Studio SDK is up to date
- Check that all Maven repositories are accessible
- Clean build: `./gradlew clean :TeamCode:assembleDebug`

**Vision Not Working:**
- Verify Limelight IP configuration in `VisionSubsystemLimelight`
- Check AprilTag locations in `FieldConstants.java`
- Enable vision logging in `TelemetrySettings`
- **MegaTag2 Issues:**
  - Verify MegaTag2 is enabled in Limelight web UI (not MegaTag1)
  - Check that heading updates are being sent (look for `updateRobotOrientation()` calls in logs)
  - Ensure robot-to-camera offsets match your physical camera mount position
  - Verify FTC field map (.fmap) is loaded in Limelight
  - Check that camera offsets match Pinpoint odometry robot center definition
  - If poses are offset consistently, adjust camera position in Limelight web UI

**Odometry Drift:**
- Tune Pinpoint encoder offsets in `Constants.localizerConstants`
- Check wheel diameters and gear ratios
- Use pose fusion diagnostics to compare odometry vs vision

**Dashboard Not Loading:**
- Verify robot WiFi connection
- Check robot IP matches `192.168.49.1`
- Ensure FTC Dashboard port 8080 is not blocked

**Slow Loop Times:**
- Switch to `TelemetryLevel.MATCH` via FTC Dashboard (target <25ms)
- Verify `BulkReadComponent.INSTANCE` is registered in OpMode
- I2C sensors automatically throttled: Color sensors (200ms), Limelight (50ms)
- Review loop timing diagnostics: Hold RT on gamepad1 in TeleOp
- See detailed analysis in `docs/loop-timing-analysis.md`
