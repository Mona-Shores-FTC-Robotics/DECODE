package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Phase 2: Data collection for building distance-based RPM formula
 *
 * Logs every shot attempt with:
 * - Distance to goal
 * - RPM commanded
 * - Robot pose
 * - Timestamp
 *
 * Use AdvantageScope to analyze offline and fit formula.
 *
 * Note: This class is currently a stub as RobotLogger has been removed.
 * Logging is now handled by KoalaLog's @AutoLog annotation system.
 */
@Configurable
public class ShotDataLogger {

    @Configurable
    public static class ShotLoggingConfig {
        /** Enable detailed shot data logging (may impact performance) */
        public static boolean enableShotLogging = true;

        /** Log every Nth shot (1 = all shots, 2 = every other shot) */
        public static int logThrottle = 1;
    }

    private int shotCounter = 0;

    public ShotDataLogger() {
        // No-op constructor - logging now handled by @AutoLog
    }

    /**
     * Logs a shot attempt for later analysis
     *
     * @param distanceToGoalInches Distance from robot to goal (from vision or known position)
     * @param commandedRpmLeft RPM commanded for left launcher
     * @param commandedRpmRight RPM commanded for right launcher
     * @param actualRpmLeft Actual measured RPM at launch time
     * @param actualRpmRight Actual measured RPM at launch time
     * @param robotPose Robot pose at launch time
     * @param launchPosition Known field position (LAUNCH_FAR, etc.) or null
     */
    public void logShotAttempt(
            double distanceToGoalInches,
            double commandedRpmLeft,
            double commandedRpmRight,
            double actualRpmLeft,
            double actualRpmRight,
            Pose robotPose,
            String launchPosition) {

        if (!ShotLoggingConfig.enableShotLogging) {
            return;
        }

        shotCounter++;
        if (shotCounter % ShotLoggingConfig.logThrottle != 0) {
            return;
        }

        // TODO: Implement using @AutoLog annotations when needed
        // For now, this is a no-op stub
    }

    /**
     * Simplified logging when you only know position
     */
    public void logShotAtPosition(
            String launchPosition,
            double commandedRpm,
            double actualRpmLeft,
            double actualRpmRight,
            Pose robotPose) {

        // Estimate distance based on position (can refine later)
        double estimatedDistance = estimateDistanceFromPosition(launchPosition);

        logShotAttempt(
            estimatedDistance,
            commandedRpm,
            commandedRpm,
            actualRpmLeft,
            actualRpmRight,
            robotPose,
            launchPosition
        );
    }

    /**
     * Phase 1 helper: Rough distance estimates from known positions
     * Phase 3: Replace with actual AprilTag-based distance
     */
    private double estimateDistanceFromPosition(String position) {
        // These are placeholder values - measure your actual field
        if (position == null) {
            return 72.0; // Default ~6 feet
        }

        switch (position) {
            case "LAUNCH_FAR":
                return 96.0; // ~8 feet (adjust based on your field)
            case "LAUNCH_CLOSE":
                return 48.0; // ~4 feet (adjust based on your field)
            default:
                return 72.0;
        }
    }

    /**
     * Call this after a shot to manually record success/failure
     * (Could be automated with vision or sensors in future)
     */
    public void recordShotOutcome(boolean success) {
        if (!ShotLoggingConfig.enableShotLogging) {
            return;
        }

        // TODO: Implement using @AutoLog annotations when needed
        // For now, this is a no-op stub
    }

    public int getShotCount() {
        return shotCounter;
    }

    public void reset() {
        shotCounter = 0;
    }
}
