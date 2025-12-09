package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Timing configuration for launcher operations.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods below
 *
 * This config CORRECTLY follows best practices:
 * - All timing values are shared (same for both robots)
 * - Robot-specific create methods don't override anything
 */
public class LauncherTimingConfig {
    // ===== SHARED VALUES (same for both robots) =====
    /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
    public double minimalSpinUpMs = 500;

    /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
    public double fallbackReadyMs = 500;

    /** Time to keep flywheel at launch speed after firing to ensure artifact clears (ms). */
    public double launchHoldAfterFireMs = 500;

    /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
    public double recoveryMs = 150;

    // ===== ROBOT-SPECIFIC VALUES (no defaults - set per robot) =====
    // (none currently - all values are shared)

    // Robot-specific instances
    public static LauncherTimingConfig timing19429 = createTiming19429();
    public static LauncherTimingConfig timing20245 = createTiming20245();

    /**
     * Creates timing configuration for robot 19429.
     * Currently uses all default values - both robots have identical timing.
     */
    private static LauncherTimingConfig createTiming19429() {
        LauncherTimingConfig timing = new LauncherTimingConfig();
        // No overrides needed - using shared defaults
        return timing;
    }

    /**
     * Creates timing configuration for robot 20245.
     * Currently uses all default values - both robots have identical timing.
     */
    private static LauncherTimingConfig createTiming20245() {
        LauncherTimingConfig timing = new LauncherTimingConfig();
        // No overrides needed - using shared defaults
        return timing;
    }
}
