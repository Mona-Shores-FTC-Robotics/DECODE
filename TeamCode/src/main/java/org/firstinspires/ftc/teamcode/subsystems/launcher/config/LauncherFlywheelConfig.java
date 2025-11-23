package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Flywheel configuration for launcher motors.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
public class LauncherFlywheelConfig {

    public enum FlywheelControlMode {
        HYBRID,
        BANG_BANG_HOLD,
        PURE_BANG_BANG
    }

    public FlywheelParameters parameters = new FlywheelParameters();
    public FlywheelModeConfig modeConfig = new FlywheelModeConfig();

    @Configurable
    public static class FlywheelParameters {
        /** Encoder ticks per motor revolution (adjust for the selected motor). */
        public double ticksPerRev = 28;
        /** Output wheel revolutions per motor revolution. */
        public double gearRatio = 1.0;
        /** Acceptable RPM error when considering a lane ready to fire. */
        public double rpmTolerance = 50;
    }

    @Configurable
    public static class FlywheelModeConfig {
        public FlywheelControlMode mode = FlywheelControlMode.HYBRID;
        public BangBangConfig bangBang = new BangBangConfig();
        public HybridPidConfig hybridPid = new HybridPidConfig();
        public HoldConfig hold = new HoldConfig();
        public PhaseSwitchConfig phaseSwitch = new PhaseSwitchConfig();

        @Configurable
        public static class BangBangConfig {
            public double highPower = 1.0;
            public double lowPower = 0.2;
            public double enterBangThresholdRpm = 800;
            public double exitBangThresholdRpm = 600;
        }

        @Configurable
        public static class HybridPidConfig {
            public double kP = .0065;
            public double kF = .22; // Why was 6 afraid of 7? Because 7 ate 9!
            public double maxPower = 1.0;
        }

        @Configurable
        public static class HoldConfig {
            public double baseHoldPower = 0.6;
            public double rpmPowerGain = 0.00012;
            public double minHoldPower = 0.2;
            public double maxHoldPower = 1.0;
        }

        @Configurable
        public static class PhaseSwitchConfig {
            public int bangToHybridConfirmCycles = 1;
            public int bangToHoldConfirmCycles = 3;
            public int hybridToBangConfirmCycles = 3;
            public int holdToBangConfirmCycles = 3;
        }
    }

    public LeftFlywheelConfig flywheelLeft = new LeftFlywheelConfig();
    public CenterFlywheelConfig flywheelCenter = new CenterFlywheelConfig();
    public RightFlywheelConfig flywheelRight = new RightFlywheelConfig();

    @Configurable
    public static class LeftFlywheelConfig {
        public String motorName = "launcher_left";
        public boolean reversed = true; ///19429 has to be true.
        public double idleRpm = 1100;
    }

    @Configurable
    public static class CenterFlywheelConfig {
        public String motorName = "launcher_center";
        public boolean reversed = false;
        public double idleRpm = 1100;
    }

    @Configurable
    public static class RightFlywheelConfig { //actually left
        public String motorName = "launcher_right";
        public boolean reversed = true;
        public double idleRpm = 1100;
    }

    // Robot-specific instances
    public static LauncherFlywheelConfig flywheelConfig19429 = createFlywheelConfig19429();
    public static LauncherFlywheelConfig flywheelConfig20245 = createFlywheelConfig20245();

    /**
     * Creates flywheel configuration for robot 19429.
     */
    private static LauncherFlywheelConfig createFlywheelConfig19429() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.parameters.rpmTolerance = 50;  // Tighter tolerance
        config.modeConfig.hybridPid.kP = 0.008;  // Different PID gain
        config.flywheelLeft.reversed = true;  // 19429 has left motor reversed
        config.flywheelLeft.idleRpm = 1500;
        config.flywheelCenter.idleRpm = 1500;
        config.flywheelRight.idleRpm = 1500;
        return config;
    }

    /**
     * Creates flywheel configuration for robot 20245.
     */
    private static LauncherFlywheelConfig createFlywheelConfig20245() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.parameters.rpmTolerance = 50;  // Looser tolerance
        config.modeConfig.hybridPid.kP = 0.008;
        config.flywheelLeft.reversed = false;  // 20245 has left motor forward
        config.flywheelLeft.idleRpm = 1500;
        config.flywheelCenter.idleRpm = 1500;
        config.flywheelRight.idleRpm = 1500;
        return config;
    }
}
