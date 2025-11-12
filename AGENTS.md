# Repository Guidelines

## Project Structure & Module Organization
This project mirrors the standard FTC two-module layout: `TeamCode` contains runnable robot code while `FtcRobotController` holds the untouched SDK app. Within `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`, code is grouped by responsibility—`subsystems/` for hardware abstractions, `opmodes/auto` and `opmodes/teleop` for match-ready routines, `pedroPathing/` for path-planning utilities, and `util/` for shared helpers. Resources (dash layouts, tuning configs) live in `TeamCode/src/main/res`. The `codex/` folder bundles automation helpers and the `.githooks/` directory ships a protective pre-commit hook; run `bash codex/install-hook.sh` once to enable it.

## Build, Test, and Development Commands
- `./gradlew :TeamCode:assembleDebug` – builds the team module and produces the deployable APK.
- `./gradlew :FtcRobotController:installDebug` – installs the controller app on a connected device; use after assembling to push updates.
- `./gradlew :TeamCode:lint` – runs Android Lint against the team module; fix warnings before posting a PR.
- `./gradlew :TeamCode:test` – executes JVM unit tests when they exist; combine with `--tests ClassNameTest` to focus on a suite.

## Coding Style & Naming Conventions
Java sources use 4-space indentation, braces on the same line, and one public class per file. Classes stay in UpperCamelCase (`FieldCentricTeleOp`), members and locals in lowerCamelCase, and constants in SCREAMING_SNAKE_CASE (see `Constants.java`). Keep OpModes annotated with the appropriate `@TeleOp`/`@Autonomous` metadata and place new helpers beside their peers in the package tree. Prefer dashboard-friendly logging via `TelemetryPublisher` and favour dependency injection through the `Robot` container when adding subsystems.

## Testing Guidelines
There are no standing unit tests yet, so add new suites under `TeamCode/src/test/java` using JUnit4 naming (`DriveSubsystemTest`). Use mocks or the SDK-provided `HardwareMap` stubs to isolate hardware-dependent logic. Run `./gradlew :TeamCode:test` locally before requesting reviews, and attach dashboard recordings or field logs when behaviour is validated on hardware; note any scenarios not covered by automation.

## Commit & Pull Request Guidelines
Existing history shows short, descriptive commit subjects (“code from end of meeting”, “SLOWBINDING needs work”). Follow suit with a 50-character-or-less imperative summary and optional detail body. Keep version bumps out of scope—the pre-commit hook will block staged edits to protected Gradle files. Pull requests should state intent, link the relevant issue or task, outline testing (unit + robot runs), and include screenshots or telemetry captures when UI or dashboard output changes. Tag reviewers who own the touched subsystem so changes reach the robot quickly.
