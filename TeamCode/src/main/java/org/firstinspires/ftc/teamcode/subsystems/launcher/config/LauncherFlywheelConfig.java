package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Flywheel configuration for launcher motors.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods below
 */
@Configurable
public class LauncherFlywheelConfig {
    public FlywheelParameters parameters = new FlywheelParameters();

    public static class FlywheelParameters {
        // SHARED - hardware constants same for both robots
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
        // SHARED - hardware name same for both robots
        public String motorName = "launcher_left";
        public double idleRpm = 1500;
        public boolean reversed;
        public double kS;
        public double kV;
        public double kP;
    }

    public static class CenterFlywheelConfig {
        // SHARED - hardware name same for both robots
        public String motorName = "launcher_center";
        public double idleRpm = 1500;
        public boolean reversed;
        public double kS;
        public double kV;
        public double kP;
    }

    public static class RightFlywheelConfig { //actually left
        // SHARED - hardware name same for both robots
        public String motorName = "launcher_right";
        public double idleRpm = 1500;
        public boolean reversed;
        public double kS;
        public double kV;
        public double kP;
    }
    // Robot-specific instances
    public static LauncherFlywheelConfig flywheelConfig19429 = createFlywheelConfig19429();
    public static LauncherFlywheelConfig flywheelConfig20245 = createFlywheelConfig20245();

    /**
     * Creates flywheel configuration for robot 19429.
     * Only overrides left.kP (0.003) - all other values use shared defaults.
     */
    private static LauncherFlywheelConfig createFlywheelConfig19429() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        // Only override robot-specific value
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

    /**
     * Creates flywheel configuration for robot 20245.
     * Only overrides left.kP (0.001) - all other values use shared defaults.
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
