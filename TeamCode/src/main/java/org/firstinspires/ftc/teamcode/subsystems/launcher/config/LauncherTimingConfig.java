package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Timing configuration for launcher operations.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
public class LauncherTimingConfig {
    /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
    public double minimalSpinUpMs = 500;
    /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
    public double fallbackReadyMs = 500;
    /** Time to keep flywheel at launch speed after firing to ensure artifact clears (ms). */
    public double launchHoldAfterFireMs = 500;
    /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
    public double recoveryMs = 150;
    /** Delay between sequential shots when bursting all three lanes (ms). */
    public double burstSpacingMs = 120.0;

    // Robot-specific instances
    public static LauncherTimingConfig timing19429 = createTiming19429();
    public static LauncherTimingConfig timing20245 = createTiming20245();

    /**
     * Creates timing configuration for robot 19429.
     */
    private static LauncherTimingConfig createTiming19429() {
        LauncherTimingConfig timing = new LauncherTimingConfig();
        return timing;
    }

    /**
     * Creates timing configuration for robot 20245.
     */
    private static LauncherTimingConfig createTiming20245() {
        LauncherTimingConfig timing = new LauncherTimingConfig();
        return timing;
    }
}
