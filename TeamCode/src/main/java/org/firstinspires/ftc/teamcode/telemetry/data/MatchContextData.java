package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.util.Alliance;

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

    public MatchContextData(Alliance alliance, double runtimeSec, double matchTimeSec, String opMode, boolean isAutonomous) {
        this.alliance = alliance != null ? alliance : Alliance.UNKNOWN;
        this.runtimeSec = runtimeSec;
        this.matchTimeSec = matchTimeSec;
        this.opMode = opMode != null ? opMode : "Unknown";
        this.isAutonomous = isAutonomous;
    }

    /**
     * Calculate match time remaining for TeleOp (150 seconds total).
     */
    public static double calculateTeleOpMatchTime(double runtimeSec) {
        return Math.max(0.0, 150.0 - runtimeSec);
    }

    /**
     * Calculate match time remaining for Autonomous (30 seconds total).
     */
    public static double calculateAutoMatchTime(double runtimeSec) {
        return Math.max(0.0, 30.0 - runtimeSec);
    }
}
