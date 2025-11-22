# .pp File Autonomous Generator

This system allows you to create command-based autonomous routines from Pedro Pathing `.pp` files with granular control over speed, heading interpolation, and game actions.

## Overview

Instead of hardcoding poses and paths in Java, you can:
1. **Design paths visually** using the Pedro Pathing GUI
2. **Export as .pp file** (JSON format)
3. **Load and customize** paths in your autonomous OpMode with fine-grained control

## Quick Start

### 0. Test Without Hardware (Recommended First Step!)

**Visualize paths on FTControl Panels before running on robot:**

```java
// Use the VisualizePathsFromPpFile OpMode
// 1. Connect to Panels: http://192.168.49.1:5800
// 2. Init the OpMode (don't press START!)
// 3. See paths drawn on field
// 4. Change config.ppFileName or config.alliance in FTC Dashboard
// 5. Stop and re-init to refresh
```

Or in your own OpMode:
```java
@Override
public void onWaitForStart() {
    AutoRoutineBuilder builder = AutoRoutineBuilder.fromPpFile(
        "trajectory.pp", robot, hardwareMap, alliance);
    builder.visualizePaths(alliance);  // Draw on Panels!
}
```

### 1. Create Your Path File

Use the Pedro Pathing path editor to design your autonomous path. Save as a `.pp` file.

**Place the .pp file here:**
```
TeamCode/src/main/assets/your_path_name.pp
```

Example .pp files already in assets:
- `trajectory.pp` - Example 4-segment path
- `short_auto_logic.pp` - Short autonomous routine with 6 segments

### 2. Create Your OpMode

```java
@Autonomous(name = "My Generated Auto", group = "Generated")
public class MyAutoOpMode extends NextFTCOpMode {

    private Robot robot;
    private Alliance activeAlliance = Alliance.BLUE;

    @Override
    public void onStartButtonPressed() {
        Command autoRoutine = buildAutonomousRoutine();
        CommandManager.INSTANCE.scheduleCommand(autoRoutine);
    }

    private Command buildAutonomousRoutine() {
        return AutoRoutineBuilder
            .fromPpFile("trajectory.pp", robot, hardwareMap, activeAlliance)

            .followNext()
                .withPower(0.8)
                .during(spinUpLauncher())
            .andThen()

            .followNext()
                .withPower(0.5)  // Slower
            .andThen()

            .build();
    }
}
```

## Granular Control Features

### Per-Segment Speed Control

```java
.followNext()
    .withPower(0.9)  // Fast (0.0 - 1.0)
.andThen()

.followNext()
    .withPower(0.5)  // SLOW DOWN THE 3RD PATH!
.andThen()
```

### Heading Interpolation Control

```java
// Linear heading (default, 50/50 mix)
.followNext()
    .withLinearHeading()
.andThen()

// Tangential heading (follows path curve)
.followNext()
    .withTangentialHeading()  // USE TANGENTIAL INSTEAD OF LINEAR!
.andThen()

// Constant heading (maintain start heading)
.followNext()
    .withConstantHeading()
.andThen()

// Custom interpolation (0.0 = constant, 0.5 = linear, 1.0 = tangential)
.followNext()
    .withHeadingInterpolation(0.7)  // 70% tangential, 30% linear
.andThen()
```

### Actions Before/During/After Segments

```java
.followNext()
    .before(spinUpLauncher())     // Execute BEFORE path starts
    .during(prepareIntake())      // Execute IN PARALLEL while following
    .after(scoreSequence())       // Execute AFTER path completes
.andThen()
```

### Follow Segments by Name

Instead of following in order, reference by name:

```java
.followSegment("Path 1")
    .withPower(0.9)
.andThen()

.followSegment("Path 3")  // Skip Path 2
    .withPower(0.8)
.andThen()
```

### Skip Segments

```java
.followNext()
    .withPower(0.9)
.andThen()

.skip(2)  // Skip the next 2 segments

.followNext()  // Now on segment 4
    .withPower(0.8)
.andThen()
```

### Insert Custom Commands

```java
.followNext()
    .withPower(0.8)
.andThen()

.delay(0.5)  // Wait 0.5 seconds

.then(new InstantCommand(() -> robot.intake.stop()))  // Custom command

.followNext()
    .withPower(0.9)
.andThen()
```

## Complete Example

Here's a real-world autonomous using all features:

```java
private Command buildAutonomousRoutine() {
    return AutoRoutineBuilder
        .fromPpFile("trajectory.pp", robot, hardwareMap, activeAlliance)

        // Phase 1: Drive to launch position with spinner
        .followNext()
            .withPower(0.9)               // Fast
            .withLinearHeading()          // Linear interpolation
            .during(spinUpLauncher())     // Spin up while driving
        .andThen()

        .then(scoreSequence())  // Score preload

        // Phase 2: Drive to pickup with tangential heading
        .followNext()
            .withPower(0.8)
            .withTangentialHeading()      // Follow path curve naturally
            .during(prepareIntake())
        .andThen()

        // Phase 3: SLOW collection pass
        .followNext()
            .withPower(0.4)               // SLOW for accurate collection
            .withHeadingInterpolation(0.7)
            .during(new InstantCommand(() ->
                robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD)))
        .andThen()

        .delay(0.2)  // Wait for intake

        // Phase 4: Fast return to score
        .followNext()
            .withPower(0.95)              // Maximum speed
            .withLinearHeading()
            .during(spinUpLauncher())
        .andThen()

        .then(scoreSequence())

        .build();
}
```

## Real-World Use Cases

### "Slow down the 3rd path"
```java
.followNext().withPower(0.8).andThen()  // Path 1: Normal
.followNext().withPower(0.8).andThen()  // Path 2: Normal
.followNext().withPower(0.4).andThen()  // Path 3: SLOW!
.followNext().withPower(0.8).andThen()  // Path 4: Normal
```

### "Use tangential interpolation instead of linear"
```java
.followNext()
    .withTangentialHeading()  // Changed from linear to tangential!
.andThen()
```

### "Different speeds for collecting vs scoring"
```java
// Fast approach to pickup
.followNext().withPower(0.9).andThen()

// Slow collection pass
.followNext().withPower(0.4).during(collectSamples()).andThen()

// Fast return to score
.followNext().withPower(0.95).during(spinUpLauncher()).andThen()
```

## .pp File Format

Pedro Pathing `.pp` files are JSON with this structure:

```json
{
  "startPoint": {
    "x": 56,
    "y": 8,
    "heading": "linear",
    "startDeg": 90
  },
  "lines": [
    {
      "name": "Path 1",
      "endPoint": {"x": 56.27, "y": 19.81, "degrees": 90},
      "controlPoints": [],
      "reverse": false,
      "heading": "linear"
    },
    {
      "name": "Path 2",
      "endPoint": {"x": 23.77, "y": 23.77, "degrees": 90},
      "controlPoints": [{"x": 40, "y": 15}],
      "reverse": false,
      "heading": "tangential"
    }
  ]
}
```

**Key fields:**
- `name`: Segment identifier for `followSegment()`
- `endPoint`: Destination pose (x, y in inches, degrees in degrees)
- `controlPoints`: Empty for straight line, one or more for curves
- `reverse`: Drive backwards if true
- `heading`: Heading mode hint (overridden by `.withHeadingInterpolation()`)

## Alliance Mirroring

Paths are automatically mirrored for red alliance:

```java
AutoRoutineBuilder.fromPpFile("trajectory.pp", robot, hardwareMap, Alliance.RED)
```

**Mirroring logic:**
- X coordinate: `mirroredX = FIELD_WIDTH - x`
- Y coordinate: unchanged
- Heading: `mirroredHeading = 180° - heading`

Design your paths for **BLUE alliance**, and red alliance gets mirrored automatically!

## Troubleshooting

### "Could not find .pp file"
- Ensure file is in `TeamCode/src/main/assets/`
- Use exact filename with `.pp` extension
- Check file is included in build (run `./gradlew :TeamCode:assembleDebug`)

### "No more segments to follow"
- Check how many segments are in your .pp file
- You called `.followNext()` too many times
- Use `.getSegmentCount()` to check available segments

### "No segment found with name"
- Check segment name in .pp file (open in text editor)
- Names are case-sensitive
- Use exact name: `followSegment("Path 1")`

### "Paths not mirrored correctly"
- Ensure you're passing correct alliance to `.fromPpFile()`
- Check alliance is set before building routine
- Verify FieldConstants.FIELD_WIDTH_INCHES is correct (144 inches)

### "Heading interpolation not working"
- Remember: 0.0 = constant, 0.5 = linear, 1.0 = tangential
- Heading is always interpolated (can't disable completely)
- For exact heading control, use multiple short segments

## Path Visualization (No Hardware Required!)

### Dedicated Visualization OpMode

The **`VisualizePathsFromPpFile`** OpMode lets you preview paths without moving the robot:

**How to use:**
1. Connect to FTControl Panels: `http://192.168.49.1:5800`
2. Select the OpMode on driver station
3. **Init the OpMode (DO NOT press START!)**
4. Paths appear on Panels field view
5. Change configuration in FTC Dashboard:
   - `ppFileName` - which .pp file to visualize
   - `alliance` - RED or BLUE (see mirroring)
   - `showSegmentDetails` - show/hide segment info
6. Stop and re-init to refresh visualization

**What you'll see:**
- All path segments drawn on field
- Start pose (robot position)
- Curved vs straight paths
- Alliance-specific mirroring

### Visualization in Your OpMode

Add visualization to any OpMode's `onWaitForStart()`:

```java
@Override
public void onWaitForStart() {
    // Load builder
    AutoRoutineBuilder builder = AutoRoutineBuilder.fromPpFile(
        "trajectory.pp", robot, hardwareMap, activeAlliance);

    // Visualize paths (draws on Panels)
    builder.visualizePaths(activeAlliance);

    // Continue with normal init...
}
```

This lets you:
- Preview paths before pressing START
- Verify path changes before running
- Check alliance mirroring is correct
- Debug path loading issues

### Quick Path Comparison

Compare different .pp files or alliances:

```java
// Blue alliance
AutoRoutineBuilder builderBlue = AutoRoutineBuilder.fromPpFile(
    "trajectory.pp", robot, hardwareMap, Alliance.BLUE);
builderBlue.visualizePaths(Alliance.BLUE);

// Red alliance (mirrored)
AutoRoutineBuilder builderRed = AutoRoutineBuilder.fromPpFile(
    "trajectory.pp", robot, hardwareMap, Alliance.RED);
builderRed.visualizePaths(Alliance.RED);
```

## Advanced: Testing and Debugging

### Get segment information
```java
AutoRoutineBuilder builder = AutoRoutineBuilder
    .fromPpFile("trajectory.pp", robot, hardwareMap, activeAlliance);

int count = builder.getSegmentCount();  // How many segments?
List<String> names = builder.getSegmentNames();  // What are they named?

telemetry.addData("Segments", count);
for (String name : names) {
    telemetry.addLine("- " + name);
}
```

### Partial routine testing
```java
// Test just the first 2 segments
return AutoRoutineBuilder
    .fromPpFile("trajectory.pp", robot, hardwareMap, activeAlliance)
    .followNext().withPower(0.5).andThen()  // Slow for testing
    .followNext().withPower(0.5).andThen()
    // Don't call more .followNext() - routine ends here
    .build();
```

### Visualize paths before running
```java
// In onInit() or onWaitForStart()
List<PathSegment> segments = PedroPathLoader.loadFromAssets(
    "trajectory.pp", hardwareMap, activeAlliance);

for (PathSegment segment : segments) {
    telemetry.addData(segment.name,
        "Start: (%.1f, %.1f) → End: (%.1f, %.1f)",
        segment.startPose.getX(), segment.startPose.getY(),
        segment.endPose.getX(), segment.endPose.getY());
}
telemetry.update();
```

## API Reference

### AutoRoutineBuilder

**Creation:**
- `fromPpFile(ppFileName, robot, hardwareMap, alliance)` - Load from assets
- `fromSegments(segments, follower)` - Create from pre-loaded segments

**Segment Control:**
- `followNext()` - Follow next segment (returns SegmentConfigurator)
- `followSegment(name)` - Follow segment by name (returns SegmentConfigurator)
- `skip(count)` - Skip next N segments

**Command Insertion:**
- `then(command)` - Add custom command
- `delay(seconds)` - Add delay

**Build:**
- `build()` - Create final Command

**Utilities:**
- `getSegmentCount()` - Get number of segments
- `getSegmentNames()` - Get list of segment names
- `visualizePaths(alliance)` - Draw paths on PanelsBridge
- `getSegments()` - Get raw segment list

### SegmentConfigurator

**Speed:**
- `withPower(power)` - Set max power (0.0 - 1.0)

**Heading:**
- `withLinearHeading()` - Linear interpolation (default)
- `withTangentialHeading()` - Tangential (follow path curve)
- `withConstantHeading()` - Maintain start heading
- `withHeadingInterpolation(value)` - Custom (0.0 - 1.0)

**Actions:**
- `before(command)` - Execute before segment
- `during(command)` - Execute in parallel during segment
- `after(command)` - Execute after segment

**Finalize:**
- `andThen()` - Finish this segment, return to builder

### PedroPathLoader

**Loading:**
- `loadFromAssets(ppFileName, hardwareMap, alliance)` - Load from assets
- `parsePathFile(jsonContent, alliance)` - Parse JSON string

**PathSegment:**
- `name` - Segment name
- `startPose`, `endPose` - Pedro Pose objects
- `controlPoints` - List of control points for curves
- `isReverse` - Drive backwards flag
- `headingMode` - Heading mode hint
- `buildPathChain(follower)` - Convert to PathChain
- `buildPathChain(follower, headingInterpolation)` - With custom interpolation

## See Also

- **Example OpMode:** `DecodeAutoGeneratedExample.java`
- **Existing Autos:** `DecodeAutonomousFarCommand.java`, `DecodeAutonomousCloseCommand.java`
- **Pedro Pathing Docs:** [Pedro Pathing v2 Documentation](https://pedropathing.com)
- **Command Framework:** NextFTC command patterns in `commands/` directory
