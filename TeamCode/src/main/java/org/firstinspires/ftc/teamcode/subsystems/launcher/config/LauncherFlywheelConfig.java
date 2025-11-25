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
        /** Velocity smoothing factor (0-1). Higher = more smoothing, slower response. */
        public double velocitySmoothingAlpha = 0.3;
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
            /** RPM error threshold to enter BANG mode (widened for stability) */
            public double enterBangThresholdRpm = 1000;
            /** RPM error threshold to exit BANG mode (widened for stability) */
            public double exitBangThresholdRpm = 400;
        }

        @Configurable
        public static class HybridPidConfig {
            public double kP = 0.008;
            /** Feedforward gain per RPM (replaces flat kF for RPM-proportional control) */
            public double kF_perRpm = 0.00008;
            /** Base feedforward power (minimum power at any RPM) */
            public double kF_base = 0.05;
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
            /** Cycles to confirm before switching BANG→HYBRID (increased for stability) */
            public int bangToHybridConfirmCycles = 3;
            /** Cycles to confirm before switching BANG→HOLD */
            public int bangToHoldConfirmCycles = 3;
            /** Cycles to confirm before switching HYBRID→BANG */
            public int hybridToBangConfirmCycles = 3;
            /** Cycles to confirm before switching HOLD→BANG */
            public int holdToBangConfirmCycles = 3;
        }
    }

    public LeftFlywheelConfig flywheelLeft = new LeftFlywheelConfig();
    public CenterFlywheelConfig flywheelCenter = new CenterFlywheelConfig();
    public RightFlywheelConfig flywheelRight = new RightFlywheelConfig();

    @Configurable
    public static class LeftFlywheelConfig {
        public String motorName = "launcher_left";
        public boolean reversed; //TODO Is this really reversed depending on robot?
        public double idleRpm = 1500;
        /** Default launch RPM when no command sets an override. Prevents target=0 issues. */
        public double defaultLaunchRpm = 2400;
    }

    @Configurable
    public static class CenterFlywheelConfig {
        public String motorName = "launcher_center";
        public boolean reversed = false;
        public double idleRpm = 1500;
        /** Default launch RPM when no command sets an override. Prevents target=0 issues. */
        public double defaultLaunchRpm = 2400;
    }

    @Configurable
    public static class RightFlywheelConfig { //actually left
        public String motorName = "launcher_right";
        public boolean reversed = true;
        public double idleRpm = 1500;
        /** Default launch RPM when no command sets an override. Prevents target=0 issues. */
        public double defaultLaunchRpm = 2400;
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
