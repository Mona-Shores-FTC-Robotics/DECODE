# How to Use the .pp File Command Generator

## Quick Start

### Step 1: Create your path in Pedro Pathing GUI

1. Open Pedro Pathing path editor
2. Design your autonomous path
3. **Name your segments meaningfully** (e.g., "drive_to_launch", "pickup_wall", "score_position")
4. Save as a `.pp` file
5. Copy the `.pp` file to `TeamCode/src/main/assets/`

### Step 2: Edit the generator settings

Open `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/GenerateAutoCommand.java`

Edit these two lines:
```java
String ppFileName = "trajectory.pp";  // ← Your .pp file name
String commandName = "TrajectoryAuto";  // ← Your command name
```

### Step 3: Run the generator

**Option A: Using Gradle (Recommended)**
```bash
./gradlew :TeamCode:generateAuto
```

**Option B: Using Android Studio**
1. Open `GenerateAutoCommand.java`
2. Right-click in the file
3. Select "Run 'GenerateAutoCommand.main()'"

**Option C: From terminal (if Java is set up)**
```bash
cd /home/user/DECODE
java -cp "TeamCode/build/intermediates/javac/debug/classes:..." \
    org.firstinspires.ftc.teamcode.util.GenerateAutoCommand
```

### Step 4: Check the generated files

Two files will be created in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/generated/`:

1. **`YourCommandNameConfig.java`** - Dashboard-tunable waypoints and settings
2. **`YourCommandNameCommand.java`** - The command structure with TODO comments

### Step 5: Fill in robot actions

Open `YourCommandNameCommand.java` and look for `// TODO` comments:

```java
// ======================================
// SEGMENT 1: drive_to_launch
// start → driveToLaunch (linear heading)
// ======================================
new ParallelGroup(
    new FollowPath(
        robot.drive.getFollower().pathBuilder()
            .addPath(new BezierLine(
                Waypoints.start.toPose(),
                Waypoints.driveToLaunch.toPose()
            ))
            .setLinearHeadingInterpolation(
                Waypoints.start.toPose().getHeading(),
                Waypoints.driveToLaunch.toPose().getHeading()
            )
            .build(),
        false,
        pathPower.driveToLaunch
    )
    // TODO: Add robot actions during driveToLaunch
    // ↓ YOU ADD THIS ↓
    launcherCommands.spinUpForPosition(FieldPoint.LAUNCH_FAR)
),
// TODO: Add actions after reaching driveToLaunch
// ↓ YOU ADD THIS ↓
launcherCommands.launchAllAtRangePreset(LauncherRange.LONG, false),
```

### Step 6: Create an OpMode

```java
@Autonomous(name = "My Generated Auto")
public class MyAuto extends NextFTCOpMode {

    private Robot robot;
    private IntakeCommands intakeCommands;
    private LauncherCommands launcherCommands;

    // ... standard setup (same as your other autos) ...

    @Override
    public void onStartButtonPressed() {
        // Just instantiate your generated command!
        Command auto = new YourCommandNameCommand(robot, intakeCommands, launcherCommands);
        CommandManager.INSTANCE.scheduleCommand(auto);
    }
}
```

### Step 7: Test and tune

1. Deploy to robot
2. Run your OpMode
3. If waypoints need adjustment:
   - Open FTC Dashboard: `http://192.168.49.1:8080/dash`
   - Go to Config tab
   - Find `YourCommandNameConfig` → `Waypoints`
   - Adjust x, y, headingDeg values
   - Changes take effect immediately (no redeployment!)

---

## Path Changed? Regenerate Config Only

If you change your path in Pedro GUI:

### Option 1: Regenerate everything (overwrites your robot actions!)
```bash
./gradlew :TeamCode:generateAuto
```
⚠️ **Warning:** This will overwrite your TODOs! Only do this if starting fresh.

### Option 2: Manually update waypoints
1. Run generator to a temporary location
2. Copy only the waypoint values from the new Config file
3. Paste into your existing Config file
4. Your robot actions in the Command file are preserved!

---

## Examples

### Generate from "close_side.pp" named "CloseSideAuto"

Edit `GenerateAutoCommand.java`:
```java
String ppFileName = "close_side.pp";
String commandName = "CloseSideAuto";
```

Run:
```bash
./gradlew :TeamCode:generateAuto
```

Generates:
- `CloseSideAutoConfig.java`
- `CloseSideAutoCommand.java`

### Generate from "far_side.pp" named "FarSideAuto"

Edit `GenerateAutoCommand.java`:
```java
String ppFileName = "far_side.pp";
String commandName = "FarSideAuto";
```

Run:
```bash
./gradlew :TeamCode:generateAuto
```

Generates:
- `FarSideAutoConfig.java`
- `FarSideAutoCommand.java`

---

## Naming Your Segments

The segment names in your .pp file become:
- Class names in Config
- Variable names in Command
- Comments in generated code

**Good naming examples:**
- "drive_to_launch" → `Waypoints.driveToLaunch`
- "pickup_wall" → `Waypoints.pickupWall`
- "score_position" → `Waypoints.scorePosition`

**Avoid:**
- Generic names like "Path 1", "Path 2" (harder to understand)
- Special characters (will be converted to underscores)
- Very long names (keep it concise)

---

## What Gets Generated

### Config File Structure
```java
@Configurable
public class YourCommandConfig {

    @Configurable
    public static class Waypoints {
        @Configurable
        public static class Start {
            public double x = 56.0;
            public double y = 8.0;
            public double headingDeg = 90.0;
            public Pose toPose() { ... }
        }
        public static Start start = new Start();

        // ... more waypoints
    }

    @Configurable
    public static class PathPower {
        public double segment1 = 0.8;
        // ... more power settings
    }
}
```

### Command File Structure
```java
public class YourCommandCommand extends SequentialGroup {

    public YourCommandCommand(Robot robot, IntakeCommands intakeCommands,
                              LauncherCommands launcherCommands) {
        super(
            // Path segments using Pedro API
            new ParallelGroup(
                new FollowPath(
                    robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(...))
                        .setLinearHeadingInterpolation(...)
                        .build(),
                    false,
                    pathPower.segment1
                )
                // TODO: Your robot actions here
            )
        );
    }
}
```

---

## Troubleshooting

### "Could not find .pp file"
- Check the file is in `TeamCode/src/main/assets/`
- Check the filename is correct (case-sensitive!)
- Include the `.pp` extension

### "Invalid .pp file format"
- Open the .pp file in a text editor
- Verify it's valid JSON
- Re-export from Pedro GUI if corrupted

### "Class already exists"
- Check `commands/generated/` directory
- Delete old generated files if you want to regenerate
- Or use a different `commandName`

### Generated code doesn't compile
- Check you have all the imports
- Verify Robot, IntakeCommands, LauncherCommands are available
- Check Pedro Pathing library is included in dependencies

---

## Advanced Usage

### Generate with custom paths

You can call the generator directly in your own code:

```java
PpFileCommandGenerator.generate(
    "my_path.pp",           // .pp file name
    "MyAutoCommand",         // command name
    "TeamCode/src/main/assets/",  // assets path
    "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/generated/"  // output path
);
```

### Batch generate multiple autos

Create a script or Java program that calls the generator multiple times:

```java
String[] autos = {"close_side.pp", "far_side.pp", "endgame.pp"};
String[] names = {"CloseSideAuto", "FarSideAuto", "EndgameAuto"};

for (int i = 0; i < autos.length; i++) {
    PpFileCommandGenerator.generate(autos[i], names[i], assetsPath, outputPath);
}
```

---

## Tips

✅ **Name segments descriptively** in Pedro GUI before generating
✅ **Generate once, edit TODOs** - don't keep regenerating
✅ **Use FTC Dashboard** to tune waypoints without redeploying
✅ **Keep .pp files** under version control in `assets/`
✅ **Test on robot** - simulated paths may not match reality
✅ **Iterate quickly** - Change path → regenerate config → test

---

## Need Help?

See the full documentation in `docs/pp-file-auto-generator.md`

Or look at the example files:
- `TrajectoryAutoConfig.java`
- `TrajectoryAutoCommand.java`
- `TrajectoryAutoOpMode.java`
