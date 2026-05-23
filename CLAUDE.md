# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DECODE is an FTC (FIRST Tech Challenge) robotics codebase for the 2025 season. It uses a command-based architecture with subsystems for drive, vision, intake, launcher, and lighting. The project integrates Pedro Pathing for autonomous navigation, Pedro Pathing Ivy for command patterns and scheduling, Limelight for AprilTag vision, and AdvantageScope Lite for telemetry logging.

The same code runs on two physical robots (19429 and 20245). The active robot is identified at boot from the Control Hub's WiFi SSID, and per-robot configuration is consolidated in `util/RobotProfile.java` — that is the single file to read or edit if you want to know what differs between the two robots.

## Build and Deployment Commands

**Build the robot code (compile only, no install):**
```bash
./gradlew :TeamCode:assembleDebug
```

**Deploy to robot — two paths:**

The project uses the Sloth runtime (Dairy Foundation) for hot code reload. There are two deploy commands; pick based on what changed.

| Command | When to use | Speed |
|---|---|---|
| `./gradlew :TeamCode:installDebug` | First deploy after a fresh APK, library/dependency change, change outside `org.firstinspires.ftc.teamcode`, or robot-day baseline | ~40s |
| `./gradlew :TeamCode:deploySloth` | Iterative TeamCode-only edits (subsystems, commands, opmodes, etc.) | sub-1s |

`deploySloth` only swaps classes in `org.firstinspires.ftc.teamcode` and subpackages. Changes to anything else (build.gradle, libraries, FtcRobotController) require a full `installDebug`. Sloth applies the swap when the current OpMode ends — it does not interrupt a running OpMode.

**Bootstrap requirement:** Sloth must be resident on the RC before `deploySloth` works. After any fresh image or `installDebug` from a non-Sloth branch, run `installDebug` once first. Subsequent iterations can use `deploySloth`.

**Reset Sloth state on the robot** (if hot reload gets into a weird state):
```bash
./gradlew :TeamCode:removeSlothRemote
```
Follow with a fresh `installDebug` to bootstrap again.

**Opting classes out of hot reload:**
Annotate a class `@Pinned` (from `dev.frozenmilk.sinister.Pinned`) to prevent Sloth from swapping it. Useful for classes with long-lived static state that would misbehave under live reload.

**Caveats:**
- "Code that compiles ≠ code that works" under hot reload — changes to `@Pinned` classes, library code, or pinned-class instances may need a full install to take effect.
- Do NOT rely on hot reload on competition day. Bootstrap a fresh `installDebug` from the `ivy-migration-pre-sloth` tag (or a known-good commit) before matches.
- The Sloth-forked FTC Dashboard and Bylazar Panels keep the same package paths as the originals (`com.acmerobotics.dashboard.*`, `com.bylazar.*`), so existing `FtcDashboard`, `PanelsTelemetry`, and `@Configurable` imports work unchanged. (Tuning surfaces all go through `@Configurable` / Panels; the FTC Dashboard side is just the packet conduit AdvantageScope Lite reads from.)

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
- `LauncherSubsystem`: Flywheel control for launching game pieces (manages spin-up, hood positions, feeder timing)
- `IntakeSubsystem`: Game piece intake mechanism with lane-color sensing
- `LightingSubsystem`: LED control for robot state indication

**Commands (in `commands/LauncherCommands/`):**
- One class per command, built with Pedro Ivy's `CommandBuilder` (no central factory class)
- `DistanceBasedSpinCommand`, `PresetRangeSpinCommand`, `LaunchInSequenceCommand`, `LaunchAllCommand`, `ModeAwareLaunchCommand`
- Intake-related commands live as factory methods directly on `IntakeSubsystem` (`setIntakeModeCmd`, `smartIntakeCmd`, etc.) — there is no separate `IntakeCommands/` package

**OpModes (in `opmodes/`):**
- `TeleOp/DecodeTeleOp.java`: Main teleop mode
- `Autos/` autonomous routines:
  - `DecodeAutonomousCloseThreeAtOnce` / `DecodeAutonomousCloseTogether`: close-side strategies
  - `DecodeAutonomousFarThreeAtOnce` / `DecodeAutonomousFarTogether`: far-side strategies
  - `DecodeAutonomousMichianaShort`: regional event variant
  - `BaseAutonomousOpMode`: shared init/alliance/start-pose logic
- `Calibration/` diagnostic and tuning OpModes (`@Disabled` by default)
- `Examples/` template OpModes (`@Disabled` by default)

**Pedro Pathing (`pedroPathing/`):**
- `Constants.java`: hardware names, plus `followerConstants()`/`driveConstants()`/`localizerConstants()` that resolve to the active robot's values via `RobotProfile`
- `Tuning.java`: Tuning utilities for Pedro
- `FusionLocalizer.java`: Blends Pinpoint odometry with AprilTag vision for robust pose estimation
- Follower configuration with Pinpoint odometry

**Utilities (`util/`):**
- `RobotProfile.java`: Per-robot configuration bundle — owns every value that differs between 19429 and 20245
- `ControlHubIdentifierUtil.java`: Identifies the active robot from the Control Hub's WiFi SSID at boot
- `RobotState.java`: Centralized robot state (robot identity, alliance, mode)
- `PoseFrames.java`: Coordinate transformations between Pedro and FTC field frames
- `FieldConstants.java`: Field geometry, AprilTag locations, and `getAimAngleTo(robot, target)`
- `GamepadBindings.java`: Gamepad-button-to-Command binding shim on top of Pedro Ivy
- `Alliance.java`, `RobotMode.java`, `ArtifactColor.java`, `LauncherLane.java`, `LauncherMode.java`, `LauncherRange.java`: Enums for robot state

**Telemetry (`telemetry/`):**
- `TelemetryService.java`: Unified telemetry service for FTC Dashboard, Panels, and Driver Station
- `TelemetrySettings.java`: Global telemetry toggles
- `data/` subsystem-specific telemetry payload classes
- `formatters/DashboardFormatter` and `formatters/DriverStationFormatter` translate those payloads to outputs

### Key Architectural Patterns

**Centralized Hardware Names:**
Hardware names live in `pedroPathing/Constants.HardwareNames` (e.g. `Constants.HardwareNames.LF`). Use these everywhere instead of hardcoded strings so a rename only has to happen in one place.

**Per-Robot Tuning:**
Every value that differs between robots 19429 and 20245 lives in `util/RobotProfile.java`. Subsystems consume `RobotProfile.forCurrent().<config>` (e.g. `RobotProfile.forCurrent().aimAssist`) — they never branch on robot identity themselves.

**Dependency Injection:**
Subsystems receive their dependencies (like `VisionSubsystemLimelight`) through constructor injection rather than accessing each other directly.

**Commands:**
Commands are built with `com.pedropathing.ivy.CommandBuilder` (start/execute/done lambdas plus `.requiring(subsystem)`). Each command lives in its own class under `commands/LauncherCommands/`; intake commands are factory methods on `IntakeSubsystem` itself.

## Drive Control

**TeleOp Drive:**
- Call `DriveSubsystem.driveTeleOp(fieldX, fieldY, rotationInput, slowMode, rampMode)` (or the shorter `driveScaled(...)` overloads)
- Field-centric mode is the default; toggle via `DriveSubsystem.robotCentricConfig`
- Slow mode available via `DriveSubsystem.setDriveMode(DriveMode.SLOW)`

**Aim Assist:**
- `DriveSubsystem.aimAndDrive(fieldX, fieldY, slowMode)` overrides rotation to aim at the vision target
- Aim angle math lives in `FieldConstants.getAimAngleTo(robotPose, targetPose)` (returns a `double`, no `Optional`)

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
- `VisionSubsystemLimelight.getRobotPoseFromTagPedro()` / `getRobotPoseFromTagFtc()` return the current pose estimate in Pedro/FTC coordinates
- `VisionSubsystemLimelight.shouldUpdateOdometry()` returns true once per tag lock
- Aim angles come from `FieldConstants.getAimAngleTo(robotPose, targetPose)`

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
- `pedroPathing/FusionLocalizer` blends Pinpoint odometry with AprilTag measurements
- Configurable trust weights based on range and decision margin
- Outlier rejection for invalid measurements
- Diagnostics available for analysis

**Odometry Updates:**
- `DriveSubsystem` updates `Follower` odometry in `periodic()`
- When valid AprilTag seen and relocalization enabled, may call `follower.setPose(...)`

## Logging and Telemetry

### Telemetry Levels

DECODE uses three telemetry levels (`TelemetrySettings.TelemetryLevel`). Live-tunable via Bylazar Panels:

```java
public static TelemetryLevel LEVEL = TelemetryLevel.PRACTICE;  // default
```

| Level | Use For |
|-------|---------|
| **MATCH** | Competition — minimal output, fastest loop |
| **PRACTICE** (default) | Practice sessions and parameter tuning |
| **VERBOSE** | Pit testing and detailed diagnostics |

`TelemetrySettings.isVerbose()` gates the heaviest debug output. Toggle the level in Panels, then your changes take effect immediately — no recompile needed.

### Live Telemetry (During Matches)

**Bylazar Panels — THE tuning UI:**
- Web UI at `http://192.168.49.1:8080/` when connected to robot WiFi
- All `@Configurable` classes expose tunable parameters
- Live graphs and field overlay
- This is where every per-robot config and global tunable lives

**FTC Dashboard — packet conduit only (no tuning here anymore):**
- Web UI at `http://192.168.49.1:8080/dash`
- The codebase no longer uses `@Config` for tuning surfaces; everything is `@Configurable`
- Still useful as the transport layer that AdvantageScope Lite consumes

**AdvantageScope Lite:**
- Connects to the robot via the FTC Dashboard packet stream
- View live 2D field plots and subsystem metrics

**Driver Station:**
- Standard FTC telemetry display on driver station phone/tablet

### Telemetry Logging

OpModes publish payload classes from `telemetry/data/` (`DriveTelemetryData`, `LauncherTelemetryData`, etc.) through `TelemetryService`, which routes them to the Driver Station and to FTC Dashboard packets (which Panels and AdvantageScope Lite both consume) via the formatters in `telemetry/formatters/`.

## Coding Conventions

These rules govern *where code lives* and *which pattern to pick* — the things a style guide alone doesn't cover. Apply them to new code and use them as a checklist when reviewing.

### Where things live

**Configs (tunable values that appear in Dashboard / Panels):**
- One config class per file, in `subsystems/<name>/config/<Name>Config.java`
- Name them `<Subsystem><Concern>Config` — e.g. `DriveAimAssistConfig`, `DriveFusionConfig`, `LauncherTimingConfig`, `IntakeGateConfig`
- The data SHAPE lives in the config class. Per-robot VALUES live in `util/RobotProfile.java` (one place to diff what's different between 19429 and 20245)
- Subsystems hold one `public static <Config> configName = RobotProfile.forCurrent().<config>;` field per config — no per-robot triplets

**Static vs. instance fields inside a config class — pick one pattern:**

| Pattern | When to use | Example |
|---|---|---|
| **Instance fields** (`public double kP;`) | The value might differ per robot, OR you might want multiple independent copies. The class is held by a `public static` field on an `@Configurable` holder (a subsystem or RobotProfile). The Dashboard path is `<Holder>.<holderField>.<configField>`. | `DriveAimAssistConfig` held by `DriveSubsystem.aimAssistConfig`. `LauncherFeederConfig` injected at construction time. |
| **Static fields** (`public static double kP = 0.5;`) | One shared global tunable. The class itself is `@Configurable` so Sloth scans it directly. The Dashboard path is `<ConfigClass>.<configField>`. | `DriveFusionConfig` — single set of vision-fusion gains for the whole robot, regardless of which physical robot is running. |

Mixing both styles in one class confuses readers and creates two Dashboard paths for the same data — don't do it. If you're not sure which to use, default to **instance fields**: it composes cleanly with `RobotProfile` and the rest of the codebase.

**Enums:**
- Used by ONE class only → nest inside that class (e.g. `LightingPattern` inside `LightingSubsystem`, the launch-sequence `Stage` inside `LaunchInSequenceCommand`)
- Used by TWO OR MORE classes → standalone in `util/<Name>.java` (e.g. `Alliance`, `LauncherLane`, `IntakeMode`)

**Commands:**
- Trivial single-action wrapper (one method call, no state) → factory method on the owning subsystem, suffixed `Cmd` (e.g. `intake.setIntakeModeCmd(...)`, `drive.aimAndDriveCmd(...)`)
- Multi-step, stateful, or configurable command → standalone class in `commands/<Subsystem>Commands/<Name>Command.java` with a static `create(...)` factory (e.g. `DistanceBasedSpinCommand`, `LaunchInSequenceCommand`)
- When in doubt, prefer the standalone class — easier to find and test

**Constants:**
- Implementation detail of one class → `private static final` near where used, with a one-line Javadoc
- Tunable from Dashboard → make it a field on a config class (not a `static final` constant)
- Shared identity strings (hardware names, telemetry keys) → `pedroPathing/Constants.java`
- Field geometry (tag IDs, basket positions) → `util/FieldConstants.java`

### Patterns to follow

**Robot identity:**
- Never branch on `RobotState.getRobotName()` outside of `RobotProfile.java`. If you find yourself writing `if (robotName == "DECODE_19429")` somewhere else, the value belongs in `RobotProfile` instead.

**Dependency injection:**
- Subsystems take their collaborators in the constructor. Don't reach across via static singletons mid-loop.

**Exception handling:**
- A `catch (Exception ignored) { }` is acceptable ONLY when the failure is harmless (shutdown paths, optional telemetry, missing-method reflection). Add a one-line comment explaining why it's safe. If you can't explain why, don't ignore it.

**Naming:**
- Classes/Interfaces: `PascalCase`
- Methods/Variables: `camelCase`
- Constants: `UPPER_SNAKE_CASE` (exception: some hardware names in `Constants.java` use lowercase to match the FTC config XML)

**Hardware Names:**
Always use `Constants.HardwareNames.*` instead of hardcoded strings:
```java
// Correct
motors.lf.setPower(power);

// Incorrect
leftFrontMotor.setPower(power);
```

**Configurable Parameters:**
Make tunable parameters configurable via Bylazar Panels using the `@Configurable` annotation:
```java
@Configurable
public class MySubsystem {

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
- Parameters appear in Bylazar Panels for live tuning
- See `DriveSubsystem.AimAssistConfig` or `CaptureAndAimCommand.CaptureAimConfig` for examples

**Code Style:**
- 4-space indentation (no tabs)
- Braces on same line (Java "Egyptian" style)
- One public class per file
- Annotate OpModes with `@TeleOp` or `@Autonomous`
- String comparison: always `.equals()`, never `==`
- Null checks: direct `if (x == null)` — `Optional<T>` is not used in this codebase

**Recommended formatter (optional):**

The codebase follows **Google Java Style (AOSP variant)** — Google's style with 4-space indent instead of 2-space. To match it automatically:

1. **IntelliJ IDEA:** `Settings → Editor → Code Style → Java → Scheme dropdown → Import from "Google Java Style"`, then change the indent from 2 to 4 spaces. Format a file with `Ctrl+Alt+L`.
2. **VS Code:** Install the "Google Java Format" extension and set 4-space tab width.

Gradle-driven formatting (Spotless) is intentionally not wired up yet — newer Spotless releases ship Java 25 bytecode that the project's Gradle/JDK can't load. Revisit when we upgrade the Gradle wrapper.

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

**Pedro Pathing Ivy (v1.0.0):**
- Command framework + global static Scheduler
- `com.pedropathing.ivy.Command` / `CommandBuilder` for command construction
- `Commands.instant` / `Commands.infinite` / `Commands.waitMs` / `Commands.lazy` factories
- `Groups.sequential` / `Groups.deadline` / `Groups.parallel` / `Groups.race` for composition
- `PedroCommands.follow(follower, pathChain, holdEnd, maxPower)` for path following
- Scheduler is process-global static — call `Scheduler.reset()` in OpMode init and `Scheduler.execute()` in OpMode loop
- No bindings module ships with Ivy; we use the `util/GamepadBindings.java` shim on top of SDK 11.1 gamepad edge primitives

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

**Alliance Colors:**
- Set via `Robot.setAlliance(Alliance.RED/BLUE)` (or the on-field `AllianceSelector` D-pad picker)
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
- Switch `TelemetrySettings.LEVEL` to `MATCH` via Bylazar Panels (target <25ms)
- I2C sensors automatically throttled: Color sensors (200ms), Limelight (50ms)
- Review loop timing diagnostics: Hold RT on gamepad1 in TeleOp
- See detailed analysis in `docs/loop-timing-analysis.md`
