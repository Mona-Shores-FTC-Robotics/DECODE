package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Flywheel motor configuration shape. Values are filled in by
 * {@code RobotProfile} — see {@code util/RobotProfile.java} for the
 * per-robot tuning.
 */
public class LauncherFlywheelConfig {
    public FlywheelParameters parameters = new FlywheelParameters();

    public static class FlywheelParameters {
        /** Encoder ticks per motor revolution (adjust for the selected motor). */
        public double ticksPerRev = 28;

        /** Output wheel revolutions per motor revolution. */
        public double gearRatio = 1.0;

        /** Acceptable RPM error when considering a lane ready to fire. */
        public double rpmTolerance = 50;
    }

    public LeftFlywheelConfig flywheelLeft = new LeftFlywheelConfig();
    public CenterFlywheelConfig flywheelCenter = new CenterFlywheelConfig();
    public RightFlywheelConfig flywheelRight = new RightFlywheelConfig();

    public static class LeftFlywheelConfig {
        public String motorName = "launcher_left";
        public double idleRpm = 1500;
        public boolean reversed;

        /** Minimum motor power to overcome friction and start spinning (0.0–1.0).
         *  Increase if the flywheel won't spin up from rest; decrease if it creeps when it shouldn't. */
        public double kS;

        /** Extra power added per RPM of target speed (feedforward velocity gain).
         *  This is the main tuning knob — higher values spin up faster but may overshoot. */
        public double kV;

        /** How aggressively to correct RPM errors (proportional feedback gain).
         *  Set to 0 for pure feedforward. Increase slowly if RPM drifts under load. */
        public double kP;
    }

    public static class CenterFlywheelConfig {
        public String motorName = "launcher_center";
        public double idleRpm = 1500;
        public boolean reversed;

        /** Minimum motor power to overcome friction and start spinning (0.0–1.0). */
        public double kS;

        /** Extra power added per RPM of target speed (feedforward velocity gain). */
        public double kV;

        /** How aggressively to correct RPM errors (proportional feedback gain). */
        public double kP;
    }

    public static class RightFlywheelConfig {
        public String motorName = "launcher_right";
        public double idleRpm = 1500;
        public boolean reversed;

        /** Minimum motor power to overcome friction and start spinning (0.0–1.0). */
        public double kS;

        /** Extra power added per RPM of target speed (feedforward velocity gain). */
        public double kV;

        /** How aggressively to correct RPM errors (proportional feedback gain). */
        public double kP;
    }
}
