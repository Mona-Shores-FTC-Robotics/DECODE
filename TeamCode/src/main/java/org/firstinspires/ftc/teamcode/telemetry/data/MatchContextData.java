package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotConfigs;

/**
 * Match and OpMode context data for telemetry.
 * Provides essential metadata about the current match state.
 */
public class MatchContextData {
    public final Alliance alliance;
    public final double runtimeSec;
    public final double matchTimeSec;
    public final String opMode;
    public final boolean isAutonomous;
    public final String robotConfig;
    public final String launcherMode;
    public final String motifPattern;
    public final int motifTail;

    public MatchContextData(Alliance alliance,
                            double runtimeSec,
                            double matchTimeSec,
                            String opMode,
                            boolean isAutonomous,
                            String robotConfig,
                            String launcherMode,
                            String motifPattern,
                            int motifTail) {
        this.alliance = alliance != null ? alliance : Alliance.UNKNOWN;
        this.runtimeSec = runtimeSec;
        this.matchTimeSec = matchTimeSec;
        this.opMode = opMode != null ? opMode : "Unknown";
        this.isAutonomous = isAutonomous;
        this.robotConfig = robotConfig != null ? robotConfig : "Unknown";
        this.launcherMode = launcherMode != null ? launcherMode : "UNKNOWN";
        this.motifPattern = motifPattern != null ? motifPattern : "UNKNOWN";
        this.motifTail = motifTail;
    }

    /**
     * Convenience constructor that automatically fetches active robot config.
     */
    public MatchContextData(Alliance alliance, double runtimeSec, double matchTimeSec, String opMode, boolean isAutonomous) {
        this(alliance,
                runtimeSec,
                matchTimeSec,
                opMode,
                isAutonomous,
                RobotConfigs.getActiveConfigName(),
                null,  // launcherMode
                null,  // motifPattern
                0);    // motifTail
    }

    /**
     * Calculate match time remaining for TeleOp (120 seconds total).
     */
    public static double calculateTeleOpMatchTime(double runtimeSec) {
        return Math.max(0.0, 120.0 - runtimeSec);
    }

    /**
     * Calculate match time remaining for Autonomous (30 seconds total).
     */
    public static double calculateAutoMatchTime(double runtimeSec) {
        return Math.max(0.0, 30.0 - runtimeSec);
    }
}
