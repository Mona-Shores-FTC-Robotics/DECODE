package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Hood configuration for launcher angle adjustment.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
public class LauncherHoodConfig {
    public double retractedPosition = 1;
    public double extendedPosition = 0;

    public LeftHoodConfig hoodLeft = new LeftHoodConfig();
    public CenterHoodConfig hoodCenter = new CenterHoodConfig();
    public RightHoodConfig hoodRight = new RightHoodConfig();

    @Configurable
    public static class LeftHoodConfig {
        public String servoName = "hood_left";

        /**
         * Hood position for short range shots
         */
        public double shortPosition = .3;
        /**
         * Hood position for mid range shots
         */
        public double midPosition = .05;
        /**
         * Hood position for long range shots
         */
        public double longPosition = 0;
    }

    @Configurable
    public static class CenterHoodConfig {
        public String servoName = "hood_center";
        /**
         * Hood position for short range shots
         */
        public double shortPosition = .3;
        /**
         * Hood position for mid range shots
         */
        public double midPosition = .05;
        /**
         * Hood position for long range shots
         */
        public double longPosition = 0;
    }

    @Configurable
    public static class RightHoodConfig {
        public String servoName = "hood_right";
        /**
         * Hood position for short range shots
         */
        public double shortPosition =.3;
        //TODO consider idle position and weirdness if idle is above the short shot.
        /**
         * Hood position for mid range shots
         */
        public double midPosition = .05;
        /**
         * Hood position for long range shots
         */
        public double longPosition = 0;
    }

    // Robot-specific instances
    public static LauncherHoodConfig hoodConfig19429 = createHoodConfig19429();
    public static LauncherHoodConfig hoodConfig20245 = createHoodConfig20245();

    /**
     * Creates hood configuration for robot 19429.
     */
    private static LauncherHoodConfig createHoodConfig19429() {
        LauncherHoodConfig config = new LauncherHoodConfig();
        config.hoodRight.midPosition = 0.05;
        config.hoodRight.longPosition = 0;
        config.hoodRight.shortPosition = .3;

        config.hoodCenter.midPosition = 0.05;
        config.hoodCenter.longPosition = 0;
        config.hoodCenter.shortPosition = .3;

        config.hoodLeft.midPosition = 0.05;
        config.hoodLeft.longPosition = 0;
        config.hoodLeft.shortPosition = .3;
        // Apply 20245-specific values if needed
        // (currently using default values - customize as needed)
        return config;
    }

    /**
     * Creates hood configuration for robot 20245.
     */
    private static LauncherHoodConfig createHoodConfig20245() {
        LauncherHoodConfig config = new LauncherHoodConfig();
        config.hoodRight.midPosition = 0.05;
        config.hoodRight.longPosition = 0;
        config.hoodRight.shortPosition = .3;

        config.hoodCenter.midPosition = 0.05;
        config.hoodCenter.longPosition = 0;
        config.hoodCenter.shortPosition = .3;

        config.hoodLeft.midPosition = 0.05;
        config.hoodLeft.longPosition = 0;
        config.hoodLeft.shortPosition = .3;
        // Apply 20245-specific values if needed
        // (currently using default values - customize as needed)
        return config;
    }
}
