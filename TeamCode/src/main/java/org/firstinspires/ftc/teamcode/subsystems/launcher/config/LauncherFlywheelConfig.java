package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Flywheel configuration for launcher motors.
 * Contains base configuration and robot-specific instances.
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
        public double idleRpm;
        public boolean reversed;
        public double kS;
        public double kV;
        public double kP;
    }

    public static class CenterFlywheelConfig {
        public String motorName = "launcher_center";
        public boolean reversed;
        public double idleRpm;
        public double kS;
        public double kV;
        public double kP;
    }

    public static class RightFlywheelConfig { //actually left
        public String motorName = "launcher_right";
        public boolean reversed;
        public double idleRpm;
        public double kS;
        public double kV;
        public double kP;
    }

    // Robot-specific instances
    public static LauncherFlywheelConfig flywheelConfig19429 = createFlywheelConfig19429();
    public static LauncherFlywheelConfig flywheelConfig20245 = createFlywheelConfig20245();

    private static LauncherFlywheelConfig createFlywheelConfig19429() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.flywheelLeft.reversed = false;
        config.flywheelLeft.idleRpm = 1500;
        config.flywheelLeft.kS = 0.10;
        config.flywheelLeft.kV = 0.0002;
        config.flywheelLeft.kP = 0.003;

        config.flywheelCenter.reversed = false;
        config.flywheelCenter.idleRpm = 1500;
        config.flywheelCenter.kS = 0.10;
        config.flywheelCenter.kV = 0.0002;
        config.flywheelCenter.kP = .001;

        config.flywheelRight.reversed = true;
        config.flywheelRight.idleRpm = 1500;
        config.flywheelRight.kS = 0.10;
        config.flywheelRight.kV = 0.0002;
        config.flywheelRight.kP = .001;

        return config;
    }

    /**
     * Creates flywheel configuration for robot 20245.
     */
    private static LauncherFlywheelConfig createFlywheelConfig20245() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.flywheelLeft.reversed = false;
        config.flywheelLeft.idleRpm = 1500;
        config.flywheelLeft.kS = 0.10;
        config.flywheelLeft.kV = 0.0002;
        config.flywheelLeft.kP = .003;

        config.flywheelCenter.reversed = false;
        config.flywheelCenter.idleRpm = 1500;
        config.flywheelCenter.kS = 0.10;
        config.flywheelCenter.kV = 0.0002;
        config.flywheelCenter.kP = .001;

        config.flywheelRight.reversed = true;
        config.flywheelRight.idleRpm = 1500;
        config.flywheelRight.kS = 0.10;
        config.flywheelRight.kV = 0.0002;
        config.flywheelRight.kP = .001;

        return config;
    }
}
