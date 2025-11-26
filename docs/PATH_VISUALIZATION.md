# Path Visualization with Pedro Pathing

This guide explains how to export your autonomous paths and visualize them in the Pedro Pathing web-based visualizer.

## Overview

The autonomous routines can export their paths as `.pp` files that are compatible with the Pedro Pathing GUI. This allows you to:
- Visualize your complete autonomous routine
- Verify path geometry before testing on the robot
- Identify potential collisions or issues
- Quickly iterate on path adjustments
- Share paths with team members
- Document your autonomous strategy

## Quick Start

### Method 1: Command Line (Recommended)

**Export paths directly from your development machine without needing the robot:**

```bash
# From project root directory
./gradlew :TeamCode:exportPaths
```

This generates:
- `autonomous_close_blue.pp`
- `autonomous_close_red.pp`

**Files are saved to the project root directory** (where you can immediately load them in the visualizer).

**Load in Pedro Visualizer:**
1. Open [Pedro Path Generator](https://pedro-path-generator.vercel.app/)
2. Click **File → Load**
3. Select your `.pp` file
4. View your complete autonomous routine!

**Custom Output Directory:**
```bash
./gradlew :TeamCode:exportPaths -PoutputDir=/path/to/output
```

### Method 2: Export via Robot OpMode

If you need to export from the robot (e.g., to capture Dashboard-adjusted waypoints):

1. **Run the Export OpMode**
   - Deploy code to robot
   - Select `Export Close Auto Paths` from TeleOp menu
   - Press START

2. **Retrieve Files**
   - Connect robot via USB
   - Navigate to `/sdcard/FIRST/` on robot
   - Copy generated files

3. **Load in visualizer** (same as Method 1)

## File Format

The `.pp` files use Pedro Pathing's JSON format:

```json
{
  "startPoint": {
    "x": 26.445,
    "y": 131.374,
    "heading": "linear",
    "startDeg": 144,
    "endDeg": 136
  },
  "lines": [
    {
      "name": "To Launch",
      "endPoint": {
        "x": 30.199,
        "y": 112.948,
        "heading": "linear",
        "startDeg": 144,
        "endDeg": 136
      },
      "controlPoints": [],
      "color": "#A1B2C3"
    }
    // ... more paths
  ],
  "shapes": []
}
```

## Path Segments

The exported file includes these path segments in order:

1. **To Launch** - Start position → Launch position
2. **To Pre-Artifacts** - Launch → Pre-artifacts transition
3. **Collect Set 1** - Pre-artifacts → Artifacts Set 1
4. **Score Set 1** - Artifacts Set 1 → Launch
5. **To Transition 2** - Launch → Transition point 2
6. **Collect Set 2** - Transition 2 → Artifacts Set 2
7. **Score Set 2** - Artifacts Set 2 → Launch
8. **To Transition 3** - Launch → Transition point 3
9. **Collect Set 3** - Transition 3 → Artifacts Set 3
10. **To Final** - Artifacts Set 3 → Final position

## Alliance Mirroring

The exporter automatically mirrors paths for red alliance:
- Blue alliance uses coordinates as-is
- Red alliance mirrors across the field centerline (x-axis reflection)
- Headings are adjusted appropriately

## Updating Paths

### Quick Iteration Workflow (Development)

1. **Edit waypoints** in `CloseAutoWaypoints` class (`DecodeAutonomousCloseCommand.java`)
2. **Re-export**: `./gradlew :TeamCode:exportPaths`
3. **Reload** .pp file in visualizer (File → Load)
4. **Repeat** until paths look good

**No robot needed!** This workflow lets you rapidly iterate on path design from your laptop.

### Competition Field Adjustments

If you adjusted waypoints via FTC Dashboard during competition:

1. **Note the values** from Dashboard Config tab
2. **Update** `CloseAutoWaypoints` in code with competition-tuned values
3. **Re-export**: `./gradlew :TeamCode:exportPaths`
4. **Verify** in visualizer before next match

Alternatively, export directly from robot using `Export Close Auto Paths` OpMode to capture Dashboard values.

## Troubleshooting

### Files Not Generated

- Check that `/sdcard/FIRST/` directory exists
- Verify robot has write permissions
- Check telemetry for error messages

### Paths Look Wrong

- Verify alliance color is correct (BLUE vs RED)
- Check waypoint coordinates in `CloseAutoWaypoints`
- Ensure field dimensions match Pedro Pathing settings (144" × 144")

### Can't Open .pp File

- Verify file is valid JSON (check with JSON validator)
- Try opening in text editor to inspect contents
- Ensure Pedro Path Generator is up to date

## Pedro Path Generator Links

- **Web App**: https://pedro-path-generator.vercel.app/
- **Documentation**: https://pedropathing.com/
- **GitHub**: https://github.com/Pedro-Pathing/Pedro-Pathing

## Tips

- **Export both alliances** to verify mirroring is correct
- **Use descriptive path names** when adding custom segments
- **Keep .pp files in version control** to track path evolution
- **Share .pp files** with alliance partners for coordination
