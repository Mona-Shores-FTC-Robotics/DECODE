package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Feeder (kicker) servo configuration shape. Values are filled in by
 * {@code RobotProfile} — see {@code util/RobotProfile.java} for the
 * per-robot tuning.
 */
public class LauncherFeederConfig {
    public LeftFeederConfig left = new LeftFeederConfig();
    public CenterFeederConfig center = new CenterFeederConfig();
    public RightFeederConfig right = new RightFeederConfig();

    public static class CenterFeederConfig {
        public String servoName = "feeder_center";
        public boolean reversed = false;
        public double holdMs = 1000;

        public double loadPosition;
        public double pinchPosition;
        public double firePosition;
    }

    public static class LeftFeederConfig {
        public String servoName = "feeder_left";
        public boolean reversed = false;
        public double holdMs = 1000;

        public double loadPosition;
        public double pinchPosition;
        public double firePosition;
    }

    public static class RightFeederConfig {
        public String servoName = "feeder_right";
        public boolean reversed = false;
        public double holdMs = 1000;

        public double loadPosition;
        public double pinchPosition;
        public double firePosition;
    }
}
