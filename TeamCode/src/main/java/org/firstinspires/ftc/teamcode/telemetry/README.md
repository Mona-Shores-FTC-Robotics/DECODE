# DECODE Telemetry System

This document explains the reorganized telemetry system for the DECODE FTC robot codebase.

## Overview

The telemetry system provides tiered output (MATCH/PRACTICE/DEBUG) to three destinations:
- **Driver Station**: FTC telemetry display on driver station phone/tablet
- **FTC Dashboard**: Web dashboard (`http://192.168.49.1:8080/dash`) and AdvantageScope Lite
- **FullPanels**: FTControl Panels for detailed live metrics

## Architecture

### Key Design Principles

1. **Capture once, format many ways**: All telemetry data is captured into data model objects once per loop, then formatted for different outputs
2. **Clear separation of concerns**: Data models hold robot state, formatters handle presentation
3. **Easy to extend**: Adding new telemetry fields requires updates in only 1-2 places
4. **Performance tiering**: Different telemetry levels balance visibility vs loop time

### Directory Structure

```
telemetry/
├── TelemetryService.java          # Thin orchestrator (~280 lines, down from 867!)
├── TelemetrySettings.java         # Configuration (levels, intervals)
├── TelemetryPublisher.java        # Legacy (deprecated, use formatters)
├── RobotStatusLogger.java         # FTC status logging (currently disabled)
│
├── data/                           # Telemetry data models
│   ├── RobotTelemetryData.java    # Root container for all telemetry
│   ├── MatchContextData.java      # Match/OpMode context
│   ├── PoseTelemetryData.java     # Robot pose (Pedro, FTC, vision)
│   ├── DriveTelemetryData.java    # Drive subsystem (4-motor hierarchy)
│   ├── LauncherTelemetryData.java # Launcher (per-lane RPM, hood, feeder)
│   ├── VisionTelemetryData.java   # Vision (AprilTag detection, pose)
│   └── IntakeTelemetryData.java   # Intake (mode, power, roller, artifacts)
│
├── formatters/                     # Telemetry formatters
│   ├── DriverStationFormatter.java # Formats for driver station display
│   ├── DashboardFormatter.java     # Formats for FTC Dashboard packets
│   └── FullPanelsFormatter.java    # Formats for FullPanels (FTControl)
│
└── README.md                       # This file
```

## Telemetry Levels

Change telemetry level via FTC Dashboard → Config → TelemetrySettings → level (no recompile needed).

### MATCH Mode
**Use for:** Qualification and elimination matches
**Target:** <10ms telemetry overhead
**Driver Station:** Minimal (alliance, match time, pose, launcher ready, artifacts)
**FTC Dashboard:** **DISABLED** (no packets sent)
**FullPanels:** **DISABLED**

### PRACTICE Mode (Default)
**Use for:** Practice sessions, parameter tuning
**Target:** <20ms telemetry overhead
**Driver Station:** Moderate detail (adds drive mode, vision range, intake mode)
**FTC Dashboard:** Essential metrics (10 Hz / 100ms, ~20 fields)
**FullPanels:** Configurable (can be enabled/disabled)

### DEBUG Mode
**Use for:** Development, bench testing, detailed diagnostics
**Target:** No performance limit
**Driver Station:** Full detail (per-motor powers/velocities, per-lane launcher, hood/feeder positions)
**FTC Dashboard:** Comprehensive (20 Hz / 50ms, ~80 fields)
**FullPanels:** **ENABLED** (full diagnostics)

## How to Add New Telemetry

### 1. Add Field to Data Model

**Example**: Add "wrist position" to IntakeSubsystem

**Edit `telemetry/data/IntakeTelemetryData.java`:**
```java
public class IntakeTelemetryData {
    // ... existing fields ...
    public final double wristPosition;  // NEW!

    public IntakeTelemetryData(
            // ... existing parameters ...
            double wristPosition  // NEW!
    ) {
        // ... existing assignments ...
        this.wristPosition = wristPosition;  // NEW!
    }

    public static IntakeTelemetryData capture(IntakeSubsystem intake, LauncherCoordinator coordinator) {
        return new IntakeTelemetryData(
                // ... existing captures ...
                intake.getWristPosition()  // NEW!
        );
    }
}
```

### 2. Add to Formatters (as needed)

**Driver Station (DEBUG mode only):**

**Edit `telemetry/formatters/DriverStationFormatter.java`:**
```java
public void publishDebug(Telemetry telemetry, RobotTelemetryData data) {
    // ... existing telemetry ...
    telemetry.addData("  Wrist", "%.2f", data.intake.wristPosition);  // NEW!
}
```

**FTC Dashboard (DEBUG mode):**

**Edit `telemetry/formatters/DashboardFormatter.java`:**
```java
private TelemetryPacket createDebugPacket(RobotTelemetryData data) {
    // ... existing packet data ...
    packet.put("intake/wrist_position", data.intake.wristPosition);  // NEW!
    return packet;
}
```

**FullPanels:**

**Edit `telemetry/formatters/FullPanelsFormatter.java`:**
```java
public void publish(TelemetryManager panels, RobotTelemetryData data) {
    // ... existing panels data ...
    panels.debug("intake/wrist/position", data.intake.wristPosition);  // NEW!
}
```

That's it! The new field is now captured and published.

## Field Naming Conventions

### FTC Dashboard Fields

Dashboard fields are sorted **alphabetically**, so use underscores to group related fields:

**✅ Good (grouped alphabetically):**
```
launcher/left/current_rpm      ← Alphabetically sorted together
launcher/left/feeder_position
launcher/left/hood_position
launcher/left/power
launcher/left/target_rpm
```

**❌ Bad (scattered):**
```
launcher/left/currentRpm       ← Separated alphabetically
launcher/left/feederPosition
launcher/left/hoodPosition
launcher/left/power
launcher/left/targetRpm
```

### Hierarchy Format

Use `/` for hierarchy levels and `_` within names:
```
<subsystem>/<component>/<metric>

Examples:
  alliance/id
  drive/motor_lf_power
  launcher/left/hood_position
  vision/tag_id
```

### Special Case: Pose

`Pose/` uses capital P for AdvantageScope compatibility:
```
Pose/Pose x
Pose/Pose y
Pose/Pose heading
Pose/FTC Pose x
Pose/Vision Pose x
```

## OpMode Usage

### TeleOp Example

```java
robot.telemetry.publishLoopTelemetry(
        robot.drive,
        robot.launcher,
        robot.intake,
        robot.vision,
        driveRequest,                    // From DriverBindings
        robot.launcherCoordinator,
        selectedAlliance,
        getRuntime(),
        null,                            // dsTelemetry (null = separate display)
        "TeleOp",
        false,                           // isAutonomous (false for TeleOp)
        null                             // poseOverride (null for odometry)
);
```

### Autonomous Example

```java
robot.telemetry.publishLoopTelemetry(
        robot.drive,
        robot.launcher,
        robot.intake,
        robot.vision,
        null,                            // driveRequest (null for auto)
        robot.launcherCoordinator,
        activeAlliance,
        getRuntime(),
        null,
        "Autonomous",
        true,                            // isAutonomous (true hides driver controls)
        followerPose                     // poseOverride (use path follower pose)
);
```

## Troubleshooting

### Problem: New field not appearing in telemetry

1. Check if field is added to data model `capture()` method
2. Check if formatter includes the field for the current telemetry level
3. Verify telemetry level in FTC Dashboard → Config → TelemetrySettings

### Problem: Telemetry causing slow loop times

1. Switch to MATCH mode: FTC Dashboard → Config → TelemetrySettings → level = MATCH
2. Check loop timings (hold RT on gamepad1 in TeleOp to show diagnostics)
3. Verify `BulkReadComponent.INSTANCE` is registered in OpMode

### Problem: FTC Dashboard not showing packets

1. Verify connected to robot WiFi
2. Check `http://192.168.49.1:8080/dash` is accessible
3. Ensure telemetry level is PRACTICE or DEBUG (MATCH disables packets)
4. Check FTC Dashboard port 8080 is not blocked

### Problem: FullPanels not updating

1. Verify telemetry level is DEBUG (FullPanels disabled in MATCH/PRACTICE by default)
2. Enable manually: FTC Dashboard → Config → TelemetrySettings → enableFullPanels = true
3. Check PanelsBridge.preparePanels() is called in OpMode init

## Performance Guidelines

**MATCH Mode Targets:**
- Total loop time: 15-25ms
- Telemetry overhead: <10ms
- Driver station only, no dashboard/panels

**PRACTICE Mode Targets:**
- Total loop time: 25-35ms
- Telemetry overhead: <20ms
- Dashboard at 10 Hz (100ms interval)

**DEBUG Mode:**
- No performance target
- Dashboard at 20 Hz (50ms interval)
- Full diagnostics enabled

## Key Improvements Over Old System

1. **Reduced code size**: TelemetryService is ~280 lines (down from 867)
2. **Single data capture**: Telemetry extracted once per loop, not 3x for each formatter
3. **Easy to extend**: New fields added in 1-2 places, not scattered across 800 lines
4. **Clear naming**: `isAutonomous` instead of confusing `suppressDriveTelemetry`
5. **Complete data**: Hood positions, feeder positions, match time, full intake state
6. **Better organization**: Four-motor drive hierarchy, per-lane launcher data
7. **Hierarchical fields**: Alphabetically sensible dashboard field naming
8. **Type safety**: Data models prevent typos and ensure consistency

## Migration Notes

### Deprecated APIs

- `TelemetryService.publisher()` - Use formatters directly
- `TelemetryService.panelsTelemetry()` - Use FullPanelsFormatter
- `TelemetryService.updateDriverStation()` - No-op, handled in publishLoopTelemetry
- `TelemetryService.setRoutineStepTelemetry()` - TODO: Re-implement if needed

### Breaking Changes

- `publishLoopTelemetry()` now requires `IntakeSubsystem` parameter
- All OpModes updated to pass `robot.intake`

## Future Enhancements

- Re-implement autonomous routine step tracking
- Add match time estimation based on FTC field data
- Integration with AdvantageScope Lite streaming
- Custom telemetry profiles per competition/practice venue
