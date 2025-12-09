package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Hood configuration for launcher angle adjustment.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods below
 */
@Configurable
public class LauncherHoodConfig {
    // ===== SHARED VALUES (same for both robots) =====
    /** Hood retracted (fully in) servo position */
    public double retractedPosition = 1;

    /** Hood extended (fully out) servo position */
    public double extendedPosition = 0;

    public LeftHoodConfig hoodLeft = new LeftHoodConfig();
    public CenterHoodConfig hoodCenter = new CenterHoodConfig();
    public RightHoodConfig hoodRight = new RightHoodConfig();

    public static class LeftHoodConfig {
        // SHARED - same for both robots
        /** Hardware name for left hood servo */
        public String servoName = "hood_left";
        public double shortPosition;
        public double midPosition;
        public double longPosition;
    }

    public static class CenterHoodConfig {
        // SHARED - same for both robots
        /** Hardware name for center hood servo */
        public String servoName = "hood_center";
        public double shortPosition;
        public double midPosition;
        public double longPosition;
    }

    public static class RightHoodConfig {
        // SHARED - same for both robots
        /** Hardware name for right hood servo */
        public String servoName = "hood_right";
        public double shortPosition;
        public double midPosition;
        public double longPosition;
    }

    // ===== ROBOT-SPECIFIC VALUES (no defaults - set per robot) =====
    // (none currently - all values are shared)

    // Robot-specific instances
    public static LauncherHoodConfig hoodConfig19429 = createHoodConfig19429();
    public static LauncherHoodConfig hoodConfig20245 = createHoodConfig20245();

    /**
     * Creates hood configuration for robot 19429.
     * Currently uses all default values - both robots have identical hood tuning.
     */
    private static LauncherHoodConfig createHoodConfig19429() {
        LauncherHoodConfig config = new LauncherHoodConfig();
        /** Short range hood angle servo position */
        config.hoodLeft.shortPosition = .3;
        config.hoodCenter.shortPosition = .3;
        config.hoodRight.shortPosition = .3;

        /** Mid range hood angle servo position */
        config.hoodLeft.midPosition = .05;
        config.hoodCenter.midPosition = .05;
        config.hoodRight.midPosition = .05;

        /** Long range hood angle servo position */
        config.hoodLeft.longPosition = 0;
        config.hoodCenter.longPosition = 0;
        config.hoodRight.longPosition = 0;

        return config;
    }

    /**
     * Creates hood configuration for robot 20245.
     * Currently uses all default values - both robots have identical hood tuning.
     */
    private static LauncherHoodConfig createHoodConfig20245() {
        LauncherHoodConfig config = new LauncherHoodConfig();
        config.hoodLeft.shortPosition = .3;
        config.hoodCenter.shortPosition = .3;
        config.hoodRight.shortPosition = .3;

        /** Mid range hood angle servo position */
        config.hoodLeft.midPosition = .05;
        config.hoodCenter.midPosition = .05;
        config.hoodRight.midPosition = .05;

        /** Long range hood angle servo position */
        config.hoodLeft.longPosition = 0;
        config.hoodCenter.longPosition = 0;
        config.hoodRight.longPosition = 0;

        return config;
    }
}
