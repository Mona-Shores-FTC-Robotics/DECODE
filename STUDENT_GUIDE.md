# DECODE FTC Robotics - Student Codebase Guide

Welcome to the DECODE robotics team! This guide will teach you about our 2025 FTC robot's codebase. Think of this robot as a system with multiple "experts" (subsystems) that work together to accomplish tasks.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Components](#hardware-components)
3. [Architecture: How Everything Connects](#architecture-how-everything-connects)
4. [Subsystems: The Robot's "Experts"](#subsystems-the-robots-experts)
5. [Operation Loops: How the Robot Runs](#operation-loops-how-the-robot-runs)
6. [Commands: Making the Robot Do Things](#commands-making-the-robot-do-things)
7. [OPModes: Different Robot Modes](#opmodes-different-robot-modes)
8. [Key Methods and Patterns](#key-methods-and-patterns)

---

## System Overview

Our robot uses a **command-based architecture**, which is like a task scheduling system:

**Key Principle:** Each component has a single responsibility:
- OpModes: Orchestrate the match flow
- Commands: Perform specific tasks
- Subsystems: Control specific hardware
- Hardware: The actual robot parts


```
┌─────────────────────────────────────────────────────────┐
│                  FTC OpMode (TeleOp/Auto)               │
│  - Reads inputs (gamepad, sensors, vision)              │
│  - Schedules commands to run                            │
│  - Monitors when they're done                           │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│             Command Manager (NextFTC)                   │
│  - Manages which commands are running                   │
│  - Calls update() on each command 50x per second        │
│  - Handles interruption when new commands start         │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│             Individual Commands                         │
│  - DefaultDriveCommand (running gamepad to motors)      │
│  - LaunchCommand (spin up flywheels, launch pieces)     │
│  - IntakeCommand (pull in game pieces)                  │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│             Subsystems ("Robot Experts")                │
│  - DriveSubsystem (handles wheel motors)                │
│  - LauncherSubsystem (handles flywheels + servos)       │
│  - IntakeSubsystem (handles intake motor + sensors)     │
│  - LightingSubsystem (controls RGB lights)              │
│  - VisionSubsystemLimelight (reads camera)              │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│             Hardware (Real Motors & Sensors)            │
│  - Mecanum drive motors                                 │
│  - Launcher flywheels                                   │
│  - Intake motor                                         │
│  - Color sensors, encoder, servos                       │
└─────────────────────────────────────────────────────────┘
```

---

## Hardware Components

### Drive System
- **4x Mecanum Motors**: LF (Left Front), RF (Right Front), LB (Left Back), RB (Right Back)
- **Pinpoint Odometry**: Two motor encoders for position tracking
- **IMU**: For heading/orientation

### Launcher System
- **3x Flywheel Motors**: LEFT, CENTER, RIGHT lanes for launching game pieces
- **Feeder Servos**: Control when pieces are fed into flywheels
- **Hood Configuration**: Adjustable angle for different shot distances

### Intake System
- **Intake Motor**: Pulls game pieces into robot
- **Roller Servo**: Controls intake gate
- **3x Color Sensors**: Detect artifact color in each lane (GREEN, PURPLE, or NONE)

### Lighting System
- **3x RGB Indicator Lights**: One per launcher lane, shows:
  - Alliance color (RED or BLUE)
  - Artifact color being held
  - "Busy" state when launching

### Vision System
- **Limelight Camera**: Detects AprilTags on field for:
  - Robot relocalization (knowing where we are)
  - Aiming at the goal
  - Pattern recognition

### Other Sensors
- **Servos**: Feeder gate, intake roller, lighting indicators
- **Battery Monitor**: Track voltage for performance compensation

---

## Architecture: How Everything Connects

### The Robot Container (`Robot.java`)

This is the "main hub" that connects everything:

```
                    Robot Container
                          │
        ┌───────┬─────────┼─────────┬───────────┐
        │       │         │         │           │
      Drive  Launcher  Intake   Lighting    Vision
     (Expert) (Expert) (Expert) (Expert)   (Expert)
```

When an OpMode starts, it creates a `Robot` object which automatically initializes all subsystems.

### Dependency Injection Pattern

Each subsystem receives what it needs through its constructor:

```java
// Example: DriveSubsystem needs the Vision subsystem to relocalize
public DriveSubsystem(HardwareMap hardware, VisionSubsystem vision) {
    this.vision = vision;  // Injected dependency
}

// In Robot.java:
vision = new VisionSubsystemLimelight(hardwareMap);
drive = new DriveSubsystem(hardwareMap, vision);  // Pass vision to drive
```

This keeps subsystems loosely coupled - each only knows about what it truly needs.

---

## Subsystems: The Robot's "Experts"

### 1. DriveSubsystem - The Movement Expert

**Responsibility:** Control the robot's movement

**Key Features:**
- **Field-Centric Drive**: "Forward" always means away from driver, regardless of robot rotation
- **Robot-Centric Drive**: "Forward" means forward relative to robot's orientation
- **Slow Mode**: Reduced speed for precision
- **Ramp Mode**: Smooth acceleration (no jerky starts)
- **Autonomous Follower**: Runs pre-planned paths using Pedro Pathing
- **Vision Relocalization**: Updates position when AprilTag is seen

**Key Methods:**
```java
// TeleOp driving
setTeleopDrive(forward, strafeLeft, turnCW, isRobotCentric)

// Autonomous path following
Follower getFollower()  // Access Pedro Pathing path follower

// Get current position
Pose getPosition()

// Update position from vision
setPose(Pose newPose)
```

**Configuration:**
```java
TeleOpDriveConfig {
    double forwardMultiplier;      // Slow down forward movement
    double strafeMultiplier;       // Slow down strafing
    double turnMultiplier;         // Slow down rotation
}
```

---

### 2. LauncherSubsystem - The Shooter Expert

**Responsibility:** Launch game pieces at the goal

**Key Features:**
- **Three Lanes**: LEFT, CENTER, RIGHT lanes operate independently
- **Smart Spin-Up**: Flywheels accelerate to optimal speed before firing
- **Shot Queue**: Queue multiple shots in sequence
- **Voltage Compensation**: Maintain consistent launch speed despite battery drain
- **Readiness Detection**: Knows when launcher is ready to fire

**States:**
- `DISABLED` - Launcher off
- `IDLE` - Idle speed (energy conservation)
- `SPINNING_UP` - Accelerating flywheels
- `READY` - At speed, ready to feed piece
- `FEEDING` - Feeder servo pushing piece into flywheel
- `RECOVERING` - Waiting before next shot

**Key Methods:**
```java
// Queue shots
queueShot(LauncherLane.LEFT)          // Queue left lane only
queueBurstAll()                       // Queue all three lanes

// Control spin mode
setSpinMode(SpinMode.FULL)            // Full speed
setSpinMode(SpinMode.IDLE)            // Slow idle
setSpinMode(SpinMode.OFF)             // Stop flywheels

// Check status
boolean isReady()                      // Can fire now?
```

**Configuration:**
```java
class LauncherConfig {
    double fullRPM;                    // Flywheel target RPM
    double idleRPM;                    // Idle RPM to save battery
    double spinUpTimeMs;               // How long to accelerate
    double feederHoldTimeMs;           // How long to push piece
    double recoveryTimeMs;             // Delay between shots
}
```

---

### 3. IntakeSubsystem - The Collection Expert

**Responsibility:** Collect game pieces and detect their color

**Key Features:**
- **Motor Control**: Forward to intake, reverse to eject
- **Roller Gate**: Servo controls whether pieces enter
- **Color Detection**: Three sensors read artifact colors
- **Lane Tracking**: Knows which lane has which color
- **Automatic Updates**: Color sensors polled every loop

**Lane States:**
For each of the 3 lanes:
- `GREEN` - Green artifact detected
- `PURPLE` - Purple artifact detected
- `NONE` - No artifact
- `UNKNOWN` - Sensor error or ambiguous

**Key Methods:**
```java
// Control motor
setIntakeMode(IntakeMode.ACTIVE_FORWARD)    // Pull in pieces
setIntakeMode(IntakeMode.PASSIVE_REVERSE)   // Eject pieces
setIntakeMode(IntakeMode.STOPPED)           // Stop

// Read color sensors
ArtifactColor getLaneColor(LauncherLane.LEFT)

// Check if lanes are full
boolean isLaneFull(LauncherLane.CENTER)
boolean getAllLaneFull()              // All three lanes full?

// Listen for color changes (for lighting updates)
addLaneColorListener(lighting)
```

**Configuration:**
```java
class LaneSensorConfig {
    double colorPollingIntervalMs;     // How often to read sensors
    ColorThreshold greenThreshold;     // RGB values for green
    ColorThreshold purpleThreshold;    // RGB values for purple
}
```

---

### 4. LightingSubsystem - The Status Display Expert

**Responsibility:** Control RGB lights to show robot status

**Key Features:**
- **Alliance Coloring**: Shows RED or BLUE based on team
- **Artifact Indication**: Shows what color piece is held
- **Busy Indicator**: Flashes when launching

**States:**
- `OFF` - Lights off
- `ALLIANCE` - Show alliance color
- `BUSY` - Busy animation

**Key Methods:**
```java
setAlliance(Alliance.BLUE)            // Configure for blue team
setLaneColor(LauncherLane.LEFT, ArtifactColor.GREEN)
setBusy(true)                         // Show busy state
```

---

### 5. VisionSubsystemLimelight - The "Eyes" Expert

**Responsibility:** Read camera and provide vision-based intelligence

**Key Features:**
- **AprilTag Detection**: Recognize and locate field markers
- **MegaTag2**: Fused localization using IMU + AprilTags
- **Pose Estimation**: Calculate robot's position and heading
- **Goal Targeting**: Calculate aim angle to goal
- **Relocalization**: Update odometry when AprilTag seen

**Key Methods:**
```java
// Get position from AprilTag
Pose getRobotPoseFromTag()

// Is the latest tag detection reliable?
boolean shouldUpdateOdometry()        // Relocalize this loop?

// What angle should we aim at?
Optional<Double> getAimAngle()        // Degrees to turn to face goal

// Update camera with robot's heading (for MegaTag2)
updateRobotHeading(double heading)
```

---

## Operation Loops: How the Robot Runs

### Main Loop Diagram

Every 20 milliseconds (50 times per second), this cycle runs:

```
START OF LOOP (20ms cycle)
│
├─→ [1] Bulk Read (all sensors at once)
│   └─ Faster than reading individually
│
├─→ [2] Command Manager Updates
│   ├─ update() on EVERY running command
│   └─ Calls subsystem methods
│
├─→ [3] Subsystem periodic() Methods
│   ├─ DriveSubsystem.periodic()
│   │  ├─ Update odometry from encoders
│   │  ├─ Apply motor powers
│   │  └─ Check for vision relocalization
│   │
│   ├─ LauncherSubsystem.periodic()
│   │  ├─ Update flywheel RPM
│   │  ├─ Check if ready to fire
│   │  └─ Process shot queue
│   │
│   ├─ IntakeSubsystem.periodic()
│   │  ├─ Poll color sensors
│   │  └─ Update lane states
│   │
│   ├─ LightingSubsystem.periodic()
│   │  └─ Update light colors
│   │
│   └─ VisionSubsystemLimelight.periodic()
│      └─ Read latest camera frame
│
├─→ [4] Telemetry/Logging
│   ├─ Prepare telemetry data
│   ├─ Send to dashboard
│   └─ Update driver station display
│
└─→ END OF LOOP (wait for next 20ms)
```

### TeleOp Operation Loop

Here's what happens during a TeleOp match:

```
┌─────────────────────────────────────────────────────┐
│  onInit() - Called ONCE at start                    │
│  ├─ Create Robot (initialize all subsystems)        │
│  ├─ Show alliance selector (RED or BLUE)            │
│  └─ Wait for driver to press START                  │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│  onWaitForStart() - After pressing START            │
│  ├─ Finalize alliance selection                     │
│  └─ Ready for match                                 │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│  main() - Main loop (repeated ~50 times/second)     │
│  Each iteration:                                    │
│  ├─ [1] Sample gamepad inputs                       │
│  ├─ [2] Command manager runs all active commands    │
│  ├─ [3] DefaultDriveCommand.update()                │
│  │   └─ drive.driveTeleOp(fx, fy, rot, slow, ramp)  │
│  ├─ [4] All subsystem.periodic() methods            │
│  │   ├─ Update odometry, apply motors               │
│  │   ├─ Update launcher state                       │
│  │   ├─ Poll color sensors                          │
│  │   └─ Read camera                                 │
│  ├─ [5] Telemetry updates                           │
│  └─ Continue to next iteration (20ms)               │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│  When match ends (time expires or stop pressed)     │
│  └─ Motors stop, thread exits                       │
└─────────────────────────────────────────────────────┘
```

### Autonomous Operation Loop

```
┌─────────────────────────────────────────────────────┐
│  onInit() - Called ONCE at start                    │
│  ├─ Create Robot (initialize all subsystems)        │
│  ├─ Attach Pedro Pathing follower                   │
│  └─ Build command sequence (all paths + actions)    │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│  main() - Main loop (repeated ~50 times/second)     │
│  Each iteration:                                    │
│  ├─ [1] Command manager executes next command       │
│  ├─ [2] Current command updates (e.g., FollowPath)  │
│  │   ├─ Check if at path endpoint                   │
│  │   ├─ Calculate motor powers                      │
│  │   └─ Check if done                               │
│  ├─ [3] All subsystem.periodic() methods            │
│  ├─ [4] When command done, move to next             │
│  └─ Continue until all commands done                │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│  When final command completes                       │
│  └─ Robot stops                                     │
└─────────────────────────────────────────────────────┘
```

---

## Commands: Making the Robot Do Things

Commands are like recipes - they describe how to accomplish a task.

### What is a Command?

A command implements three core methods:

```java
public class ExampleCommand extends Command {

    public void start() {
        // Called ONCE when command starts
        // Initialize state, reset timers, etc.
    }

    public void update() {
        // Called EVERY LOOP while command runs
        // Sample inputs, calculate outputs, update hardware
    }

    public boolean isDone() {
        // Called every loop
        // Return true when task is complete
    }
}
```

### Key Command Patterns

#### 1. Timed Commands (run for X seconds)
```java
// Run launcher for 2 seconds
new TimedLaunchCommand(launcher, 2.0)
    .schedule();  // Start running

// In update():
if (elapsedTime() > 2.0) {
    isDone = true;
}
```

#### 2. Sensor-Based Commands (run until condition met)
```java
// Run intake until all lanes full
new IntakeUntilFullCommand(intake, 5.0)  // timeout 5 sec
    .schedule();

// In update():
if (intake.getAllLaneFull()) {
    isDone = true;
}
```

#### 3. Sequential Commands (run one after another)
```java
// First intake, then launch
SequentialCommandGroup seq = new SequentialCommandGroup(
    new IntakeUntilFullCommand(intake, 5.0),
    new LaunchAllCommand(launcher)  // Runs after intake done
);
seq.schedule();
```

#### 4. Parallel Commands (run simultaneously)
```java
// Drive AND intake at same time
ParallelCommandGroup parallel = new ParallelCommandGroup(
    new FollowPathCommand(drive, path),
    new IntakeUntilFullCommand(intake, 5.0)
);
parallel.schedule();
```

### Common Commands

#### DefaultDriveCommand

**When it runs:** Always, unless another drive command interrupts

**What it does:**
1. Read driver gamepad inputs (left stick for movement, right stick for rotation)
2. Check for slow mode (right bumper) and ramp mode (left bumper)
3. Call `drive.driveTeleOp(fieldX, fieldY, rotation, slowMode, rampMode)`

**Code Flow:**
```
Gamepad Input
    ↓
DefaultDriveCommand.update()
    ├─ Sample sticks: fieldX, fieldY, rotation
    ├─ Sample bumpers: slowMode, rampMode
    ↓
drive.driveTeleOp(...)
    ├─ Apply multipliers (slow/ramp)
    ├─ Convert to motor powers (for mecanum)
    ├─ Apply to motors
    ↓
Motors move robot
```

#### LaunchCommand

**When it runs:** When operator presses a launch button

**What it does:**
1. Start launcher spin-up (flywheel accelerates)
2. Wait for ready signal
3. Activate feeder servo
4. Push piece into launcher
5. Wait for next piece or timeout

**State Machine:**
```
START
  ↓
SPINNING_UP (flywheels accelerate)
  ↓ (when at speed)
READY (waiting for feeder signal)
  ↓ (feeder activated)
FEEDING (pushing piece)
  ↓ (piece fired)
RECOVERING (waiting before next shot)
  ↓
DONE or next shot
```

#### IntakeCommand

**When it runs:** When operator presses intake button

**What it does:**
1. Turn on intake motor (forward for collecting, reverse for ejecting)
2. Continuously poll color sensors
3. Update intake lane states
4. Notify listeners (e.g., lighting subsystem) of color changes

---

## OPModes: Different Robot Modes

An OpMode is a complete "program" for a specific match situation.

### 1. DecodeTeleOp

**Purpose:** Main driver-controlled match

**Structure:**
```java
@TeleOp(name = "Decode TeleOp", group = "TeleOp")
public class DecodeTeleOp extends NextFTCOpMode {

    @Override
    public void onInit() {
        // Setup - called ONCE
        robot = new Robot(hardwareMap);
        robot.initializeForTeleOp();
        setupGamepadBindings();
    }

    @Override
    public void onWaitForStart() {
        // Waiting for driver to press START
        allianceSelector.showAlliance();
    }

    @Override
    public void main() {
        // Main loop - called ~50 times/second
        // Command manager automatically calls command updates
        // Subsystems' periodic() methods run automatically
    }
}
```

**Key Features:**
- **Gamepad Bindings**: Maps buttons to commands
  - Driver controls movement and aiming
  - Operator controls intake and launcher
- **Alliance Selector**: Choose RED or BLUE before match starts
- **Endgame Mode Switch**: Auto-switches strategy at 30 seconds remaining
- **Live Telemetry**: Shows robot state, temperatures, battery voltage

**Default Commands:**
- **Drive**: `DefaultDriveCommand` (always active unless interrupted)
- **Launcher**: None (commands run on-demand from buttons)
- **Intake**: None (commands run on-demand from buttons)

---

### 2. DecodeAutonomousClose (Command-Based)

**Purpose:** Auto-pilot routine for close-side starting position

**Strategy:**
1. Start at LAUNCH_CLOSE position
2. Drive to and collect samples from Gate Close
3. Return and score at LAUNCH_CLOSE
4. Drive to Gate Far and collect
5. Return and score again
6. Park in safe zone

**Structure:**
```java
@Autonomous(name = "Close Command", group = "Autonomous")
public class DecodeAutonomousClose extends NextFTCOpMode {

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        robot.initializeForAuto();
        robot.attachPedroFollower();
        buildCommandSequence();
    }

    void buildCommandSequence() {
        SequentialCommandGroup auto = new SequentialCommandGroup(
            new FollowPathCommand(drive, pathToGateClose),
            new IntakeUntilFullCommand(intake, 3.0),
            new FollowPathCommand(drive, pathToLaunchClose),
            new LaunchAllCommand(launcher),
            // ... more commands
        );
        auto.schedule();
    }
}
```

**Key Components:**
- **Pedro Paths**: Pre-planned routes on field
- **Command Groups**: Sequential operations
- **Sensor-Based Transitions**: Move to next step when previous completes
- **Distance-Based Launch**: Flywheel speed adjusts based on distance to goal

---

### 3. DecodeAutonomousFar

Similar to Close, but for far-side starting position.

---

### 4. Diagnostic OPModes

**Diagnostic Modes for Testing:**
- `DiagnoseMegaTag2` - Test vision calibration and AprilTag detection
- `ArtifactColorCalibration` - Calibrate color sensors under field lighting
- `AprilTagPatternRecognition` - Test detection of motif patterns

---

## Key Methods and Patterns

### The "Requires" Pattern

When a command uses a subsystem, it must declare that requirement:

```java
public class LaunchCommand extends Command {
    public LaunchCommand(LauncherSubsystem launcher) {
        this.launcher = launcher;
        requires(launcher);  // ← This is important!
    }
}
```

**Why?** The command manager ensures only ONE command uses a subsystem at a time. If drive command is running and you try to interrupt with another drive command, the old one stops first.

---

### The "@Configurable" Pattern

Make parameters tunable without recompiling:

```java
@Configurable
public class DriveSubsystem implements Subsystem {

    @Configurable
    public static class TeleOpDriveConfig {
        /** Speed multiplier when slow mode enabled */
        public static double slowModeMultiplier = 0.5;

        /** How quickly to ramp up speed */
        public static double rampFactor = 0.1;
    }

    public void driveTeleOp(...) {
        // Can change these values via FTC Dashboard without recompiling
        double speedMultiplier = TeleOpDriveConfig.slowModeMultiplier;
    }
}
```

Changes appear in **FTC Dashboard** → **Config** tab for live tuning.

---

### The "Supplier" Pattern

Pass dynamic values to commands:

```java
// Old way (bad):
Command cmd = new LaunchCommand(launcher, 2000);  // Fixed RPM

// New way (good):
Command cmd = new LaunchCommand(
    launcher,
    () -> calculateRPMFromDistance()  // Supplier - called each loop
);
```

The supplier function is called fresh every loop, allowing dynamic values.

---

### Enums for Safety

Use enums to prevent typos and invalid states:

```java
// Bad - string could be misspelled:
intake.setMode("FORWARD");  // What if you typo this?

// Good - compiler checks this:
intake.setIntakeMode(IntakeMode.ACTIVE_FORWARD);  // Can't typo an enum
```

**Key Enums:**
- `Alliance.BLUE` / `Alliance.RED` - Team color
- `LauncherLane.LEFT` / `CENTER` / `RIGHT` - Which lane
- `ArtifactColor.GREEN` / `PURPLE` / `NONE` - Detected color
- `IntakeMode.ACTIVE_FORWARD` / `PASSIVE_REVERSE` / `STOPPED`

---

## Learning Progression

### Week 1: Understand the Structure
1. Read `Robot.java` - see what subsystems exist
2. Look at `DecodeTeleOp.onInit()` - see initialization
3. Trace a button press:
   - Button pressed in gamepad
   - Binding calls a command
   - Command calls subsystem method
   - Motor runs

### Week 2: Simple Commands
1. Read `DefaultDriveCommand` - simplest command
2. Read `IntakeUntilFullCommand` - sensor-based logic
3. Write a simple test command (e.g., `SpinLauncherCommand`)

### Week 3: Autonomous
1. Read `DecodeAutonomousClose` - command sequencing
2. Understand `SequentialCommandGroup` vs `ParallelCommandGroup`
3. Study Pedro Pathing path following

### Week 4: Advanced
1. Study `CaptureAndAimCommand` - aiming logic
2. Read `PoseFusion` - vision + odometry blending
3. Understand `LauncherSubsystem` state machine

---

## Common Tasks

### "How do I make the robot spin the launcher?"

1. Create a command or use `LauncherCommands.launchCenter()`
2. Call `.schedule()` to start it
3. Command's `update()` runs every loop
4. Command calls `launcher.setSpinMode(SpinMode.FULL)`
5. LauncherSubsystem accelerates flywheel
6. When ready, command feeds piece
7. Piece fires!

### "How do I read the intake color sensors?"

```java
ArtifactColor centerColor = intake.getLaneColor(LauncherLane.CENTER);
if (centerColor == ArtifactColor.GREEN) {
    // Do something with green piece
}
```

### "How do I make the robot drive to a specific location?"

1. Create a Pedro path to that location
2. Create a `FollowPathCommand` with that path
3. Schedule the command
4. DriveSubsystem continuously checks position
5. Updates motor powers to follow path
6. When at target, command finishes

### "How do I aim at the goal?"

```java
// Use vision-based aiming
new AimAndDriveVisionCenteredCommand(
    driver inputs,
    drive subsystem,
    vision subsystem
).schedule();
```

This continuously:
1. Gets aim angle from Limelight
2. Rotates robot to face goal
3. Drives in requested direction

---

## Telemetry: Seeing What the Robot is Thinking

The telemetry system shows robot state in three places:

**1. Driver Station (phone/tablet)**
- Basic info: pose, launcher state, battery voltage
- Used during matches

**2. FTC Dashboard (web browser)**
- Detailed metrics, graphs, live tuning
- Access at `http://192.168.49.1:8080/dash`

**3. AdvantageScope (replay tool)**
- Review entire match after it completes
- See robot position over time
- Analyze what went wrong

**Telemetry Levels:**
- **MATCH**: Minimal (~10ms overhead) - for competition
- **PRACTICE**: Moderate (~20ms overhead) - for testing
- **DEBUG**: Full (~50ms overhead) - for development

Change via FTC Dashboard **Config** tab, no recompile needed!

---

## Common Debugging Tips

### Robot Not Moving When I Press Button

1. Check gamepad binding in `DriverBindings`
2. Verify command's `requires(subsystem)` is called
3. Check subsystem `periodic()` is updating motors
4. Verify motor names in `Constants.java` match actual hardware

### Launcher Inconsistent Speed

1. Check battery voltage (low = slow launcher)
2. Verify voltage compensation is enabled
3. Check `fullRPM` configuration is correct
4. Review flywheel motor power calculations

### Color Sensors Not Detecting

1. Check sensor names in `Constants.java`
2. Verify sensors calibrated under field lighting
3. Check color threshold values in `LaneSensorConfig`
4. Use `ArtifactColorCalibration` OpMode to test

### Path Following Inaccurate

1. Verify Pinpoint odometry calibration
2. Check wheel diameter and gear ratio in `Constants.java`
3. Review Pedro Pathing PIDF tuning values
4. Test with `Tuning.java` OpMode

---

## File Organization Quick Reference

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
│
├── Robot.java                          ← Main robot container
│
├── subsystems/                         ← Core "experts"
│   ├── DriveSubsystem.java
│   ├── LauncherSubsystem.java
│   ├── IntakeSubsystem.java
│   ├── LightingSubsystem.java
│   └── VisionSubsystemLimelight.java
│
├── commands/                           ← Tasks/actions
│   ├── DefaultDriveCommand.java
│   ├── AimAndDriveCommand.java
│   ├── IntakeCommands/
│   │   └── IntakeCommands.java (factory)
│   └── LauncherCommands/
│       └── LauncherCommands.java (factory)
│
├── opmodes/                            ← Main robot programs
│   ├── DecodeTeleOp.java              (driver-controlled)
│   ├── DecodeAutonomousClose.java     (auto-pilot)
│   └── DecodeAutonomousFar.java       (auto-pilot)
│
├── util/                               ← Helpers & constants
│   ├── Alliance.java                  (RED, BLUE enums)
│   ├── LauncherLane.java              (LEFT, CENTER, RIGHT)
│   ├── ArtifactColor.java             (GREEN, PURPLE, NONE)
│   ├── RobotState.java                (global state)
│   ├── FieldConstants.java            (goal positions, AprilTags)
│   └── RobotConfigs.java              (robot-specific settings)
│
├── bindings/                           ← Gamepad mappings
│   ├── DriverBindings.java
│   └── OperatorBindings.java
│
└── telemetry/                          ← Logging system
    ├── TelemetryService.java
    └── data/ + formatters/
```

---

## Key Takeaways

1. **Modular Design**: Each subsystem is independent and can be tested alone
2. **Command-Based**: Robot actions are composable, reusable commands
3. **50 Hz Loop**: Everything updates 50 times per second
4. **Dependency Injection**: Subsystems receive what they need, nothing more
5. **Configurable**: Most tuning happens via FTC Dashboard, not recompiles
6. **Multi-Robot**: One codebase supports two different robots
7. **Robust Vision**: AprilTag localization with MegaTag2 provides accurate positioning

---

## Next Steps

1. **Build a Simple Command**: Create a command that spins the launcher for 3 seconds
2. **Add a Binding**: Map a gamepad button to your command
3. **Read the Code**: Pick a subsystem and understand all its methods
4. **Run Diagnostics**: Use diagnostic OpModes to test components
5. **Trace Execution**: Follow a button press from gamepad all the way to motor

Good luck, and welcome to DECODE! 🤖

