package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Hood servo configuration shape. Values are filled in by
 * {@code RobotProfile} — see {@code util/RobotProfile.java} for the
 * per-robot tuning. Discovered by Dashboard via
 * {@code LauncherSubsystem.hoodConfig} (the public-static holder).
 */
public class LauncherHoodConfig {
    /** Hood retracted (fully in) servo position */
    public double retractedPosition = 1;

    /** Hood extended (fully out) servo position */
    public double extendedPosition = 0;

    public LeftHoodConfig hoodLeft = new LeftHoodConfig();
    public CenterHoodConfig hoodCenter = new CenterHoodConfig();
    public RightHoodConfig hoodRight = new RightHoodConfig();

    public static class LeftHoodConfig {
        public String servoName = "hood_left";
        public double shortPosition;
        public double midPosition;
        public double longPosition;
    }

    public static class CenterHoodConfig {
        public String servoName = "hood_center";
        public double shortPosition;
        public double midPosition;
        public double longPosition;
    }

    public static class RightHoodConfig {
        public String servoName = "hood_right";
        public double shortPosition;
        public double midPosition;
        public double longPosition;
    }
}
