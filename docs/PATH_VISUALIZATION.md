# Path Visualization with Pedro Pathing

This guide explains how to export your autonomous paths and visualize them in the Pedro Pathing web-based visualizer.

## Overview

The autonomous routines can export their paths as `.pp` files that are compatible with the Pedro Pathing GUI. This allows you to:
- Visualize your complete autonomous routine
- Verify path geometry before testing on the robot
- Identify potential collisions or issues
- Share paths with team members
- Document your autonomous strategy

## Quick Start

### Method 1: Export via OpMode (Recommended)

1. **Run the Export OpMode**
   - Deploy code to robot
   - Select `Export Close Auto Paths` from TeleOp menu
   - Press START
   - Wait for success message

2. **Retrieve Files**
   - Connect robot to computer via USB
   - Navigate to `/sdcard/FIRST/` on robot
   - Copy the generated files:
     - `autonomous_close_blue.pp`
     - `autonomous_close_red.pp`

3. **Load in Pedro Visualizer**
   - Open [Pedro Path Generator](https://pedro-path-generator.vercel.app/)
   - Click **File → Load**
   - Select your `.pp` file
   - View your complete autonomous routine!

### Method 2: Generate Locally

If you need to generate the files without deploying to the robot:

```java
// In any test or utility class
String blueJson = DecodeAutonomousCloseCommand.exportToPedroPathFile(Alliance.BLUE);
System.out.println(blueJson);

// Save to file
Files.write(Paths.get("autonomous_close_blue.pp"), blueJson.getBytes());
```

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

After adjusting waypoints in `CloseAutoWaypoints`:

1. **Via FTC Dashboard** (preferred during competition):
   - Adjust waypoints in Config tab
   - Re-run Export OpMode
   - Reload .pp file in visualizer

2. **Via Code Edit** (for permanent changes):
   - Edit `CloseAutoWaypoints` in `DecodeAutonomousCloseCommand.java`
   - Rebuild and deploy
   - Run Export OpMode
   - Reload .pp file in visualizer

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
