Goal: <one sentence describing the outcome>

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
Only modify files under TeamCode/src unless explicitly stated. Keep public APIs stable unless noted.
Prefer small, isolated diffs. Add Javadoc for all new public methods.

Coding style:
Use clear names. Add comments that would help a high school developer understand the logic.
Keep lines under ~120 characters.

Deliverables:
- Updated source files
- A short CHANGELOG entry appended at the end of this file under 'CHANGELOG'
- If tests are feasible, add them under TeamCode/src/test/java

Tests:
Add unit tests or simulation stubs when reasonable. Avoid Android-specific dependencies in tests.

CHANGELOG:
- <Codex will append a brief summary of edits here>
