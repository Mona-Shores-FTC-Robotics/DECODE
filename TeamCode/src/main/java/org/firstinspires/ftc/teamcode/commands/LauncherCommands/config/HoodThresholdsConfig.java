package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Distance breakpoints (inches) that pick the hood position band for
 * distance-based launching. Independent from {@link DistanceCalibrationConfig}
 * because hood angle and flywheel speed may want different transition points.
 */
public class HoodThresholdsConfig {
    /** At/under this distance, use the SHORT hood position. */
    public double shortRangeDistanceIn = 0;
    /** Above this distance, use the LONG hood position; in between, interpolate. */
    public double midRangeDistanceIn = 90;
}
