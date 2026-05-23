package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Timing constants for launcher operations. Currently identical for both
 * robots — see {@code util/RobotProfile.java} if you ever need to diverge.
 */
public class LauncherTimingConfig {
    /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
    public double minimalSpinUpMs = 100;

    /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
    public double fallbackReadyMs = 100;

    /** Time to keep flywheel at launch speed after firing to ensure artifact clears (ms). */
    public double launchHoldAfterFireMs = 500;

    /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
    public double recoveryMs = 150;
}
