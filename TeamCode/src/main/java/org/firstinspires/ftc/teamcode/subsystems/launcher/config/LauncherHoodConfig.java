package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Hood configuration for launcher angle adjustment.
 * Contains base configuration and robot-specific instances.
 */
public class LauncherHoodConfig {
    public double retractedPosition = 1;
    public double extendedPosition = 0;

    public LeftHoodConfig hoodLeft = new LeftHoodConfig();
    public CenterHoodConfig hoodCenter = new CenterHoodConfig();
    public RightHoodConfig hoodRight = new RightHoodConfig();

    public static class LeftHoodConfig {
        public String servoName = "hood_left";
        public double shortPosition = .3;
        public double midPosition = .05;
        public double longPosition = 0;
    }

    public static class CenterHoodConfig {
        public String servoName = "hood_center";
        public double shortPosition = .3;
        public double midPosition = .05;
        public double longPosition = 0;
    }

    public static class RightHoodConfig {
        public String servoName = "hood_right";
        public double shortPosition =.3;
        public double midPosition = .05;
        public double longPosition = 0;
    }

    // Robot-specific instances
    public static LauncherHoodConfig hoodConfig19429 = createHoodConfig19429();
    public static LauncherHoodConfig hoodConfig20245 = createHoodConfig20245();

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
        return config;
    }

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
        return config;
    }
}
