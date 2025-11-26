package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Flywheel configuration for launcher motors.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
public class LauncherFlywheelConfig {

    public FlywheelParameters parameters = new FlywheelParameters();
    public PidConfig pidConfig = new PidConfig();

    @Configurable
    public static class FlywheelParameters {
        /** Encoder ticks per motor revolution (adjust for the selected motor). */
        public double ticksPerRev = 28;
        /** Output wheel revolutions per motor revolution. */
        public double gearRatio = 1.0;
        /** Acceptable RPM error when considering a lane ready to fire. */
        public double rpmTolerance = 50;
        /** Velocity smoothing factor (0-1). Higher = more smoothing, slower response. */
        public double velocitySmoothingAlpha = 0.3;
    }

    @Configurable
    public static class PidConfig {
        /** Proportional gain - corrects current error */
        public double kP = 0.008;

        /** Integral gain - eliminates steady-state error */
        public double kI = 0.0001;

        /** Derivative gain - dampens oscillations */
        public double kD = 0.00002;

        /** Feedforward gain per RPM - baseline power for target velocity */
        public double kF_perRpm = 0.00008;

        /** Base feedforward power - minimum power at any RPM */
        public double kF_base = 0.05;

        /** Maximum integral accumulator value - prevents windup */
        public double integralLimit = 0.1;

        /** Maximum output power */
        public double maxPower = 1.0;
    }

    public LeftFlywheelConfig flywheelLeft = new LeftFlywheelConfig();
    public CenterFlywheelConfig flywheelCenter = new CenterFlywheelConfig();
    public RightFlywheelConfig flywheelRight = new RightFlywheelConfig();

    @Configurable
    public static class LeftFlywheelConfig {
        public String motorName = "launcher_left";
        public boolean reversed; //TODO Is this really reversed depending on robot?
        public double idleRpm = 1500;
    }

    @Configurable
    public static class CenterFlywheelConfig {
        public String motorName = "launcher_center";
        public boolean reversed = false;
        public double idleRpm = 1500;
    }

    @Configurable
    public static class RightFlywheelConfig { //actually left
        public String motorName = "launcher_right";
        public boolean reversed = true;
        public double idleRpm = 1500;
    }

    // Robot-specific instances
    public static LauncherFlywheelConfig flywheelConfig19429 = createFlywheelConfig19429();
    public static LauncherFlywheelConfig flywheelConfig20245 = createFlywheelConfig20245();

    private static LauncherFlywheelConfig createFlywheelConfig19429() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.flywheelLeft.reversed = true;  // 19429 has left motor reversed
        return config;
    }

    /**
     * Creates flywheel configuration for robot 20245.
     */
    private static LauncherFlywheelConfig createFlywheelConfig20245() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.flywheelLeft.reversed = false;

        return config;
    }
}
