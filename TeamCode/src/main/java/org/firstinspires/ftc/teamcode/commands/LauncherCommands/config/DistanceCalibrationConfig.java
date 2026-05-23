package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Distance breakpoints (inches) that pick the RPM table band for
 * distance-based launching. Values are tuned by reading the goal-distance
 * telemetry at known robot positions and editing here.
 */
public class DistanceCalibrationConfig {
    /** At/under this distance, use the SHORT range RPMs. */
    public double shortRangeDistanceIn = 18.4;
    /** Boundary between SHORT-MID interpolation and MID-LONG interpolation. */
    public double midRangeDistanceIn = 98.0;
    /** Lower bound of the LONG range interpolation window. */
    public double longRangeMinDistanceIn = 125.4;
    /** Upper bound — past this, use the LONG_MAX RPMs and stop interpolating. */
    public double longRangeMaxDistanceIn = 153;
}
