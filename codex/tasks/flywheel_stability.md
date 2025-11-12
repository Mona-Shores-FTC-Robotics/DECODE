Goal: Reduce RPM oscillation in the Flywheel subsystem at 3000–4500 RPM while keeping spin-up under 1.5 s.

Context:
You are operating on an FTC SDK Android Studio project. Do not change Gradle, Android plugin, or SDK versions.
Do not modify these files:
- gradle/wrapper/gradle-wrapper.properties
- build.gradle
- settings.gradle
- gradle.properties
- FtcRobotController/build.gradle
- TeamCode/build.gradle

Scope:
- Only modify files under TeamCode/src.
- Work in classes matching *Flywheel* and *TeleOp* that reference flywheel control.
- Add a simple moving-average or first-order filter for derivative term.
- Add a dashboard boolean to toggle 'stability mode' on and off.
- Provide a telemetry block that reports: target RPM, measured RPM, error, control output, and filtered derivative.

Constraints:
- Do not introduce new external libraries.
- Keep public APIs stable; if you must add a method, document it with Javadoc.
- Keep diffs minimal and self-contained.

Deliverables:
- Updated Flywheel classes with filtered derivative and dashboard toggle.
- A short CHANGELOG at the end of this file describing the edits and suggested tuning steps.

Tests:
- If feasible, add a test that feeds a synthetic step input and verifies the filtered derivative reduces peak overshoot in control output.

CHANGELOG:
- <Codex will append a brief summary of edits here>
