package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotConfigs;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

/**
 * Launcher subsystem that manages three flywheels and their paired bootkicker servos.
 * Callers queue individual shots or bursts while this class coordinates spin-up,
 * feed timing, and recovery delays.
 */
@Configurable
public class LauncherSubsystem implements Subsystem {

    public enum SpinMode {
        OFF,
        IDLE,
        FULL
    }

    public enum FlywheelControlMode {
        HYBRID,
        BANG_BANG_HOLD,
        PURE_BANG_BANG
    }

    private enum ControlPhase {
        BANG,
        HYBRID,
        HOLD
    }

    public enum LauncherState {
        DISABLED,
        IDLE,
        SPINNING_UP,
        READY,
        FEEDING,
        RECOVERING
    }

    @Configurable
    public static class Timing {
        /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
        public double minimalSpinUpMs = 500;
        /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
        public double fallbackReadyMs = 500;
        /** Time to keep flywheel at launch speed after firing to ensure artifact clears (ms). */
        public double launchHoldAfterFireMs = 500;
        /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
        public double recoveryMs = 150;
        /** Delay between sequential shots when bursting all three lanes (ms). */
        public double burstSpacingMs = 120.0;
    }

    @Configurable
    public static class VoltageCompensationConfig {
        /** Enable battery voltage compensation for consistent motor performance */
        public boolean enabled = true;
        /** Nominal battery voltage for calibration (typically 12.5V for full charge) */
        public double nominalVoltage = 12.5;
        /** Minimum voltage threshold - below this, compensation is clamped for safety */
        public double minVoltage = 9.0;
    }

    @Configurable
    public static class FlywheelConfig {
        public FlywheelParameters parameters = new FlywheelParameters();
        public FlywheelModeConfig modeConfig = new FlywheelModeConfig();

        @Configurable
        public static class FlywheelParameters {
            /** Encoder ticks per motor revolution (adjust for the selected motor). */
            public double ticksPerRev = 28;
            /** Output wheel revolutions per motor revolution. */
            public double gearRatio = 1.0;
            /** Acceptable RPM error when considering a lane ready to fire. */
            public double rpmTolerance = 200;
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
                public double exitBangThresholdRpm = 1200;
            }

            @Configurable
            public static class HybridPidConfig {
                public double kP = .0025;
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
            public boolean reversed = true;
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
    }

    @Configurable
    public static class FeederConfig {
        public LeftFeederConfig left = new LeftFeederConfig();
        public CenterFeederConfig center = new CenterFeederConfig();
        public RightFeederConfig right = new RightFeederConfig();

        @Configurable
        public static class LeftFeederConfig {
            public String servoName = "feeder_left";
            public boolean reversed = false;
            public double loadPosition = .8;
            public double firePosition = .61; //toward 0 moves toward fire position
            public double holdMs = 1000;
        }

        @Configurable
        public static class CenterFeederConfig {
            public String servoName = "feeder_center";
            public boolean reversed = false;
            public double loadPosition = .93;
            public double firePosition = .75; //toward 0 moves toward fire position
            public double holdMs = 1000;
        }

        @Configurable
        public static class RightFeederConfig {
            public String servoName = "feeder_right";
            public boolean reversed = false;
            public double loadPosition = .75;
            public double firePosition = .56; //toward 0 moves toward fire position
            public double holdMs = 1000;
        }
    }

    /**
     * Robot-specific FeederConfig for DECODE_19429.
     * These values are tuned specifically for the 19429 robot.
     */
    @Configurable
    public static class FeederConfig19429 {
        public LeftFeederConfig left = new LeftFeederConfig();
        public CenterFeederConfig center = new CenterFeederConfig();
        public RightFeederConfig right = new RightFeederConfig();

        @Configurable
        public static class LeftFeederConfig {
            public String servoName = "feeder_left";
            public boolean reversed = false;
            public double loadPosition = .8;
            public double firePosition = .61;
            public double holdMs = 1000;
        }

        @Configurable
        public static class CenterFeederConfig {
            public String servoName = "feeder_center";
            public boolean reversed = false;
            public double loadPosition = .93;
            public double firePosition = .75;
            public double holdMs = 1000;
        }

        @Configurable
        public static class RightFeederConfig {
            public String servoName = "feeder_right";
            public boolean reversed = false;
            public double loadPosition = .75;
            public double firePosition = .56;
            public double holdMs = 1000;
        }
    }

    /**
     * Robot-specific FeederConfig for DECODE_20245.
     * These values are tuned specifically for the 20245 robot.
     */
    @Configurable
    public static class FeederConfig20245 {
        public LeftFeederConfig left = new LeftFeederConfig();
        public CenterFeederConfig center = new CenterFeederConfig();
        public RightFeederConfig right = new RightFeederConfig();

        @Configurable
        public static class LeftFeederConfig {
            public String servoName = "feeder_left";
            public boolean reversed = false;
            public double loadPosition = .8;
            public double firePosition = .61;
            public double holdMs = 1000;
        }

        @Configurable
        public static class CenterFeederConfig {
            public String servoName = "feeder_center";
            public boolean reversed = false;
            public double loadPosition = .18;  // Tuned for 20245
            public double firePosition = .07;  // Tuned for 20245
            public double holdMs = 1000;
        }

        @Configurable
        public static class RightFeederConfig {
            public String servoName = "feeder_right";
            public boolean reversed = false;
            public double loadPosition = .75;
            public double firePosition = .56;
            public double holdMs = 1000;
        }
    }

    @Configurable
    public static class HoodConfig {
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
            public double shortPosition = .45;
            /**
             * Hood position for mid range shots
             */
            public double midPosition = 0;
            /**
             * Hood position for long range shots
             */
            public double longPosition = 0;
            public double launcherGoalPosition = .9; //.9 2100 for shoot with launchers adj
        }

        @Configurable
        public static class CenterHoodConfig {
            public String servoName = "hood_center";
            /**
             * Hood position for short range shots
             */
            public double shortPosition = .45;
            /**
             * Hood position for mid range shots
             */
            public double midPosition = 0;
            /**
             * Hood position for long range shots
             */
            public double longPosition = 0;
            public double launcherGoalPosition = .9; //.9 2100 for shoot with launchers adj
        }

        @Configurable
        public static class RightHoodConfig {
            public String servoName = "hood_right";
            /**
             * Hood position for short range shots
             */
            public double shortPosition = .45;
            //TODO consider idle position and weirdness if idle is above the short shot.
            /**
             * Hood position for mid range shots
             */
            public double midPosition = 0;
            /**
             * Hood position for long range shots
             */
            public double longPosition = 0;
            public double launcherGoalPosition = .9; //.9 2100 for shoot with launchers adj
        }
    }

    /**
     * Robot-specific HoodConfig for DECODE_19429.
     * These values are tuned specifically for the 19429 robot.
     */
    @Configurable
    public static class HoodConfig19429 {
        public double retractedPosition = 1;
        public double extendedPosition = 0;

        public LeftHoodConfig hoodLeft = new LeftHoodConfig();
        public CenterHoodConfig hoodCenter = new CenterHoodConfig();
        public RightHoodConfig hoodRight = new RightHoodConfig();

        @Configurable
        public static class LeftHoodConfig {
            public String servoName = "hood_left";
            public double shortPosition = .45;
            public double midPosition = 0;
            public double longPosition = 0;
            public double launcherGoalPosition = .9;
        }

        @Configurable
        public static class CenterHoodConfig {
            public String servoName = "hood_center";
            public double shortPosition = .45;
            public double midPosition = 0;
            public double longPosition = 0;
            public double launcherGoalPosition = .9;
        }

        @Configurable
        public static class RightHoodConfig {
            public String servoName = "hood_right";
            public double shortPosition = .45;
            public double midPosition = 0;
            public double longPosition = 0;
            public double launcherGoalPosition = .9;
        }
    }

    /**
     * Robot-specific HoodConfig for DECODE_20245.
     * These values are tuned specifically for the 20245 robot.
     */
    @Configurable
    public static class HoodConfig20245 {
        public double retractedPosition = 1;
        public double extendedPosition = 0;

        public LeftHoodConfig hoodLeft = new LeftHoodConfig();
        public CenterHoodConfig hoodCenter = new CenterHoodConfig();
        public RightHoodConfig hoodRight = new RightHoodConfig();

        @Configurable
        public static class LeftHoodConfig {
            public String servoName = "hood_left";
            public double shortPosition = .45;  // Tune for 20245
            public double midPosition = 0;  // Tune for 20245
            public double longPosition = 0;  // Tune for 20245
            public double launcherGoalPosition = .9;  // Tune for 20245
        }

        @Configurable
        public static class CenterHoodConfig {
            public String servoName = "hood_center";
            public double shortPosition = .45;  // Tune for 20245
            public double midPosition = 0;  // Tune for 20245
            public double longPosition = 0;  // Tune for 20245
            public double launcherGoalPosition = .9;  // Tune for 20245
        }

        @Configurable
        public static class RightHoodConfig {
            public String servoName = "hood_right";
            public double shortPosition = .45;  // Tune for 20245
            public double midPosition = 0;  // Tune for 20245
            public double longPosition = 0;  // Tune for 20245
            public double launcherGoalPosition = .9;  // Tune for 20245
        }
    }

    @Configurable
    public static class ReverseFlywheelForHumanLoadingConfig {
        /** Power level for reverse intake (negative runs motors backward) */
        public double reversePower = -0.7;
    }

    public static Timing timing = new Timing();
    public static VoltageCompensationConfig voltageCompensationConfig = new VoltageCompensationConfig();
    public static FlywheelConfig flywheelConfig = new FlywheelConfig();

    // Robot-specific FeederConfig instances - both visible in FTC Dashboard for tuning
    public static FeederConfig19429 feederConfig19429 = new FeederConfig19429();
    public static FeederConfig20245 feederConfig20245 = new FeederConfig20245();

    // Robot-specific HoodConfig instances - both visible in FTC Dashboard for tuning
    public static HoodConfig19429 hoodConfig19429 = new HoodConfig19429();
    public static HoodConfig20245 hoodConfig20245 = new HoodConfig20245();

    /**
     * Gets the robot-specific FeederConfig based on RobotState.getRobotName().
     * @return FeederConfig19429 or FeederConfig20245
     */
    public static FeederConfig feederConfig() {
        return RobotConfigs.getFeederConfig();
    }

    /**
     * Gets the robot-specific HoodConfig based on RobotState.getRobotName().
     * @return HoodConfig19429 or HoodConfig20245
     */
    public static HoodConfig hoodConfig() {
        return RobotConfigs.getHoodConfig();
    }

    public static ReverseFlywheelForHumanLoadingConfig reverseFlywheelForHumanLoadingConfig = new ReverseFlywheelForHumanLoadingConfig();

    private final HardwareMap hardwareMap;
    private final EnumMap<LauncherLane, Flywheel> flywheels = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Feeder> feeders = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Hood> hoods = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneRecoveryDeadlineMs = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneLaunchHoldDeadlineMs = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> launchRpmOverrides = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> idleRpmOverrides = new EnumMap<>(LauncherLane.class);
    private final Deque<ShotRequest> shotQueue = new ArrayDeque<>();
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private SpinMode requestedSpinMode = SpinMode.OFF;
    private LauncherState state = LauncherState.DISABLED;
    private double lastPeriodicMs = 0.0;
    private boolean reverseFlywheelActive = false;

    /**
     * Safely retrieves a motor from the hardware map. Returns null if the motor
     * is not found or the name is empty, allowing graceful degradation when
     * hardware is not connected.
     */
    private static DcMotorEx tryGetMotor(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    /**
     * Safely retrieves a servo from the hardware map. Returns null if the servo
     * is not found or the name is empty, allowing graceful degradation when
     * hardware is not connected.
     */
    private static Servo tryGetServo(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    public static FlywheelControlMode getFlywheelControlMode() {
        return flywheelConfig.modeConfig.mode;
    }

     public String getPhaseName(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? "UNKNOWN" : flywheel.getPhaseName();
    }

    private boolean debugOverrideEnabled = false;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        for (LauncherLane lane : LauncherLane.values()) {
            flywheels.put(lane, new Flywheel(lane, hardwareMap));
            feeders.put(lane, new Feeder(lane, hardwareMap));
            hoods.put(lane, new Hood(lane, hardwareMap));
            laneRecoveryDeadlineMs.put(lane, 0.0);
            laneLaunchHoldDeadlineMs.put(lane, 0.0);
        }
    }

    @Override
    public void initialize() {
        clock.reset();
        stateTimer.reset();
        shotQueue.clear();
        requestedSpinMode = SpinMode.OFF;
        reverseFlywheelActive = false;

        for (Map.Entry<LauncherLane, Flywheel> entry : flywheels.entrySet()) {
            entry.getValue().initialize();
            laneRecoveryDeadlineMs.put(entry.getKey(), 0.0);
            laneLaunchHoldDeadlineMs.put(entry.getKey(), 0.0);
        }
        for (Feeder feeder : feeders.values()) {
            feeder.initialize();
        }
        for (Hood hood : hoods.values()) {
            hood.initialize();
        }
        setState(LauncherState.DISABLED);
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        double now = clock.milliseconds();

        for (Feeder feeder : feeders.values()) {
            feeder.update();
        }

        SpinMode effectiveSpinMode = computeEffectiveSpinMode();
        applySpinMode(effectiveSpinMode);
        for (Flywheel flywheel : flywheels.values()) {
            flywheel.updateControl();
        }
        updateStateMachine(now, effectiveSpinMode);
        updateLaneRecovery(now);
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public void setSpinMode(SpinMode mode) {
        requestedSpinMode = mode == null ? SpinMode.OFF : mode;
        applySpinMode(computeEffectiveSpinMode());
    }

    public void requestSpinUp() {
        setSpinMode(SpinMode.FULL);
    }

    public void requestStandbySpin() {
        setSpinMode(SpinMode.IDLE);
    }

    public boolean atTarget() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (!flywheels.get(lane).isAtLaunch()) {
                return false;
            }
        }
        return true;
    }

    public boolean isBusy() {
        if (!shotQueue.isEmpty()) {
            return true;
        }
        double now = clock.milliseconds();
        for (LauncherLane lane : LauncherLane.values()) {
            if (now < laneRecoveryDeadlineMs.getOrDefault(lane, 0.0)) {
                return true;
            }
            Feeder feeder = feeders.get(lane);
            if (feeder != null && feeder.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public LauncherState getState() {
        return state;
    }

    public double getStateElapsedSeconds() {
        return stateTimer.seconds();
    }

    public int getQueuedShots() {
        return shotQueue.size();
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void queueShot(LauncherLane lane) {
        if (lane == null) {
            return;
        }
        scheduleShot(lane, 0.0);
    }

    public void queueShot(LauncherLane lane, double delayMs) {
        if (lane == null) {
            return;
        }
        scheduleShot(lane, delayMs);
    }

    public void queueBurstAll() {
        double delayMs = 0.0;
        for (LauncherLane lane : LauncherLane.DEFAULT_BURST_ORDER) {
            scheduleShot(lane, delayMs);
            delayMs += timing.burstSpacingMs;
        }
    }

    public void clearQueue() {
        shotQueue.clear();
    }

    public void abort() {
        clearQueue();
        reverseFlywheelActive = false;
        double now = clock.milliseconds();
        for (LauncherLane lane : LauncherLane.values()) {
            laneLaunchHoldDeadlineMs.put(lane, 0.0);
            laneRecoveryDeadlineMs.put(lane, now + timing.recoveryMs);
            feeders.get(lane).stop();
            flywheels.get(lane).stop();
        }
        setSpinMode(SpinMode.OFF);
        setState(LauncherState.DISABLED);
    }

    public void homeFeeder(LauncherLane lane) {
        Feeder feeder = feeders.get(lane);
        if (feeder != null) {
            feeder.toLoadPosition();
        }
    }

    public void moveFeederToFire(LauncherLane lane) {
        Feeder feeder = feeders.get(lane);
        if (feeder != null) {
            feeder.fire();
        }
    }

    public void moveFeederToLoad(LauncherLane lane) {
        Feeder feeder = feeders.get(lane);
        if (feeder != null) {
            feeder.toLoadPosition();
        }
    }

    public void homeAllFeeders() {
        for (Feeder feeder : feeders.values()) {
            feeder.toLoadPosition();
        }
    }

    public double getFeederPosition(LauncherLane lane) {
        Feeder feeder = feeders.get(lane);
        return feeder == null ? Double.NaN : feeder.getPosition();
    }

    public void setHoodPosition(LauncherLane lane, double position) {
        Hood hood = hoods.get(lane);
        if (hood != null) {
            hood.setPosition(position);
        }
    }

    public void setAllHoodPositions(double position) {
        for (Hood hood : hoods.values()) {
            hood.setPosition(position);
        }
    }

    public void setAllHoodsRetracted() {
        for (Hood hood : hoods.values()) {
            hood.setPosition(hoodConfig().retractedPosition);
        }
    }

    public void setAllHoodsExtended() {
        for (Hood hood : hoods.values()) {
            hood.setPosition(hoodConfig().extendedPosition);
        }
    }

    public void setHoodForRange(LauncherLane lane, org.firstinspires.ftc.teamcode.util.LauncherRange range) {
        Hood hood = hoods.get(lane);
        if (hood != null) {
            hood.setForRange(range);
        }
    }

    public void setAllHoodsForRange(org.firstinspires.ftc.teamcode.util.LauncherRange range) {
        for (Hood hood : hoods.values()) {
            hood.setForRange(range);
        }
    }

    public double getHoodPosition(LauncherLane lane) {
        Hood hood = hoods.get(lane);
        return hood == null ? Double.NaN : hood.getPosition();
    }

    public boolean isLaneReady(LauncherLane lane) {
        if (lane == null) {
            return false;
        }
        return isLaneReadyForShot(lane, clock.milliseconds());
    }

    public double getCurrentRpm() {
        double sum = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            sum += getCurrentRpm(lane);
        }
        return sum / LauncherLane.values().length;
    }

    public double getCurrentRpm(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? 0.0 : flywheel.getCurrentRpm();
    }

    public double getTargetRpm() {
        double sum = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            sum += getTargetRpm(lane);
        }
        return sum / LauncherLane.values().length;
    }

    public double getTargetRpm(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? 0.0 : flywheel.getTargetRpm();
    }

    public double getLastPower() {
        double sum = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            sum += getLastPower(lane);
        }
        return sum / LauncherLane.values().length;
    }

    public double getLastPower(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? 0.0 : flywheel.getAppliedPower();
    }

    public int getBangToHoldCount(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? 0 : flywheel.getBangToHoldCounter();
    }

    public SpinMode getRequestedSpinMode() {
        return requestedSpinMode;
    }

    public SpinMode getEffectiveSpinMode() {
        return computeEffectiveSpinMode();
    }

    public void setLaunchRpm(LauncherLane lane, double rpm) {
        if (lane == null) {
            return;
        }
        if (rpm <= 0.0) {
            launchRpmOverrides.remove(lane);
        } else {
            launchRpmOverrides.put(lane, rpm);
        }
        applySpinMode(computeEffectiveSpinMode());
    }

    public void setIdleRpm(LauncherLane lane, double rpm) {
        if (lane == null) {
            return;
        }
        if (rpm <= 0.0) {
            idleRpmOverrides.remove(lane);
        } else {
            idleRpmOverrides.put(lane, rpm);
        }
        applySpinMode(computeEffectiveSpinMode());
    }

    public double getLaunchRpm(LauncherLane lane) {
        return launchRpmFor(lane);
    }

    public double getIdleRpm(LauncherLane lane) {
        return idleRpmFor(lane);
    }

    public void clearOverrides() {
        launchRpmOverrides.clear();
        idleRpmOverrides.clear();
        applySpinMode(computeEffectiveSpinMode());
    }

    public void debugSetLaneTargetRpm(LauncherLane lane, double rpm) {
        if (lane == null) {
            return;
        }
        Flywheel flywheel = flywheels.get(lane);
        if (flywheel == null) {
            return;
        }
        shotQueue.clear();
        requestedSpinMode = SpinMode.OFF;
        flywheel.commandCustom(rpm);
    }

    public double debugGetLaneTargetRpm(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? 0.0 : flywheel.getTargetRpm();
    }

    public void setDebugOverrideEnabled(boolean enabled) {
        debugOverrideEnabled = enabled;
        if (!enabled) {
            for (LauncherLane lane : LauncherLane.values()) {
                Flywheel flywheel = flywheels.get(lane);
                if (flywheel != null) {
                    flywheel.commandCustom(0.0);
                }
            }
        }
    }

    /**
     * Runs all launcher motors in reverse at low speed for human player intake.
     * This allows game pieces to be fed into the launcher from above.
     */
    public void runReverseFlywheelForHumanLoading() {
        reverseFlywheelActive = true;
        for (LauncherLane lane : LauncherLane.values()) {
            Flywheel flywheel = flywheels.get(lane);
            if (flywheel != null) {
                flywheel.setReverseIntake();
            }
        }
    }

    /**
     * Stops the reverse intake mode and returns motors to normal control.
     */
    public void stopReverseFlywheelForHumanLoading() {
        reverseFlywheelActive = false;
        for (LauncherLane lane : LauncherLane.values()) {
            Flywheel flywheel = flywheels.get(lane);
            if (flywheel != null) {
                flywheel.stop();
            }
        }
    }

    /**
     * Returns true if reverse intake mode is currently active.
     */
    public boolean isReverseFlywheelForHumanLoadingActive() {
        return reverseFlywheelActive;
    }

    /**
     * Calculates voltage compensation multiplier to maintain consistent motor performance
     * across different battery voltage levels.
     *
     * @return Multiplier to apply to motor power (1.0 = no compensation needed)
     */
    private double getVoltageCompensationMultiplier() {
        if (!voltageCompensationConfig.enabled) {
            return 1.0;
        }

        try {
            // Get current battery voltage from the control hub
            double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            // Clamp to minimum safe voltage to prevent excessive compensation
            currentVoltage = Math.max(currentVoltage, voltageCompensationConfig.minVoltage);

            // Calculate compensation: (nominalVoltage / currentVoltage)
            // When battery is lower than nominal, multiplier > 1.0 to compensate
            // When battery is at nominal, multiplier = 1.0
            double multiplier = voltageCompensationConfig.nominalVoltage / currentVoltage;

            // Clamp multiplier to reasonable range (0.5 to 2.0) for safety
            return Range.clip(multiplier, 0.5, 2.0);
        } catch (Exception e) {
            // If voltage sensor unavailable, disable compensation
            return 1.0;
        }
    }

    /**
     * Gets the current battery voltage for telemetry/logging.
     *
     * @return Current battery voltage in volts, or 0.0 if unavailable
     */
    public double getBatteryVoltage() {
        try {
            return hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception e) {
            return 0.0;
        }
    }

    private void scheduleShot(LauncherLane lane, double delayMs) {
        double when = clock.milliseconds() + Math.max(0.0, delayMs);
        shotQueue.addLast(new ShotRequest(lane, when));
    }

    private void updateStateMachine(double now, SpinMode effectiveSpinMode) {
        // Allow per-lane independent firing
        if (!shotQueue.isEmpty()) {
            Iterator<ShotRequest> iterator = shotQueue.iterator();
            while (iterator.hasNext()) {
                ShotRequest next = iterator.next();

                if (now < next.scheduledTimeMs) {
                    continue;
                }
                if (!isLaneReadyForShot(next.lane, now)) {
                    continue;
                }

                Feeder feeder = feeders.get(next.lane);
                if (feeder != null && !feeder.isBusy()) {
                    feeder.fire();
                    laneLaunchHoldDeadlineMs.put(next.lane, now + timing.launchHoldAfterFireMs);
                    laneRecoveryDeadlineMs.put(next.lane, now + timing.recoveryMs);
                    iterator.remove();
                }
            }
        }

        reflectSpinState(effectiveSpinMode);
    }

    private void updateLaneRecovery(double now) {
        for (LauncherLane lane : LauncherLane.values()) {
            // Clear launch hold deadline when it expires
            double launchHoldDeadline = laneLaunchHoldDeadlineMs.getOrDefault(lane, 0.0);
            if (launchHoldDeadline > 0.0 && now >= launchHoldDeadline) {
                laneLaunchHoldDeadlineMs.put(lane, 0.0);
            }

            // Command to idle when recovery period ends
            double recoveryDeadline = laneRecoveryDeadlineMs.getOrDefault(lane, 0.0);
            if (recoveryDeadline > 0.0 && now >= recoveryDeadline) {
                Flywheel flywheel = flywheels.get(lane);
                if (flywheel != null) {
                    flywheel.commandIdle();
                }
                laneRecoveryDeadlineMs.put(lane, 0.0);
            }
        }
    }


    private void reflectSpinState(SpinMode effectiveSpinMode) {
        switch (effectiveSpinMode) {
            case FULL:
                setState(atTarget() ? LauncherState.READY : LauncherState.SPINNING_UP);
                break;
            case IDLE:
                setState(LauncherState.IDLE);
                break;
            case OFF:
            default:
                setState(LauncherState.DISABLED);
                break;
        }
    }

    protected boolean isLaneReadyForShot(LauncherLane lane, double now) {
        double recoveryDeadline = laneRecoveryDeadlineMs.getOrDefault(lane, 0.0);
        if (now < recoveryDeadline) {
            return false;
        }
        Feeder feeder = feeders.get(lane);
        if (feeder != null && feeder.isBusy()) {
            return false;
        }
        Flywheel flywheel = flywheels.get(lane);
        return flywheel != null && flywheel.isAtLaunch();
    }

    private void applySpinMode(SpinMode mode) {
        if (debugOverrideEnabled || reverseFlywheelActive) {
            return;
        }

        double now = clock.milliseconds();

        for (LauncherLane lane : LauncherLane.values()) {
            Flywheel flywheel = flywheels.get(lane);
            if (flywheel == null) {
                continue;
            }

            // Actively hold at launch speed after firing to ensure artifact clears
            if (now < laneLaunchHoldDeadlineMs.getOrDefault(lane, 0.0)) {
                flywheel.commandLaunch();
                continue;
            }

            // Skip lanes that are currently in recovery (already fired, cleared hold period)
            if (now < laneRecoveryDeadlineMs.getOrDefault(lane, 0.0)) {
                continue;
            }

            // Per-lane control: only apply when actively processing shots
            // When queue is empty (command starting OR all done), use requested mode directly
            SpinMode laneMode = mode;
            if (mode == SpinMode.FULL && !shotQueue.isEmpty()) {
                // Get lanes with queued shots for per-lane spin control
                Set<LauncherLane> queuedLanes = lanesWithQueuedShots();
                if (!queuedLanes.contains(lane)) {
                    laneMode = SpinMode.IDLE;
                }
            }

            switch (laneMode) {
                case FULL:
                    flywheel.commandLaunch();
                    break;
                case IDLE:
                    flywheel.commandIdle();
                    break;
                case OFF:
                default:
                    flywheel.stop();
                    break;
            }
        }
    }

    private SpinMode computeEffectiveSpinMode() {
        if (!shotQueue.isEmpty()) {
            return SpinMode.FULL;
        }
        return requestedSpinMode;
    }

    private Set<LauncherLane> lanesWithQueuedShots() {
        if (shotQueue.isEmpty()) {
            return EnumSet.noneOf(LauncherLane.class);
        }
        EnumSet<LauncherLane> lanes = EnumSet.noneOf(LauncherLane.class);
        for (ShotRequest request : shotQueue) {
            if (request != null && request.lane != null) {
                lanes.add(request.lane);
            }
        }
        return lanes;
    }

    private void setState(LauncherState newState) {
        if (state != newState) {
            state = newState;
            stateTimer.reset();
        }
    }

    /**
     * Returns the launch RPM for a lane. Launch RPM must be set explicitly by commands
     * via setLaunchRpm() - there is no default because optimal RPM depends on shot context
     * (range, distance to target, etc.).
     *
     * @param lane The launcher lane
     * @return The launch RPM if set via setLaunchRpm(), otherwise 0.0 (lane disabled)
     */
    protected double launchRpmFor(LauncherLane lane) {
        Double override = launchRpmOverrides.get(lane);
        if (override != null && override > 0.0) {
            return override;
        }
        // No default launch RPM - commands must set explicitly via setLaunchRpm()
        return 0.0;
    }

    protected double idleRpmFor(LauncherLane lane) {
        Double override = idleRpmOverrides.get(lane);
        if (override != null && override >= 0.0) {
            return override;
        }
        switch (lane) {
            case LEFT:
                return Math.max(0.0, flywheelConfig.flywheelLeft.idleRpm);
            case CENTER:
                return Math.max(0.0, flywheelConfig.flywheelCenter.idleRpm);
            case RIGHT:
            default:
                return Math.max(0.0, flywheelConfig.flywheelRight.idleRpm);
        }
    }

    private static double feederHoldMsFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return Math.max(0.0, feederConfig().left.holdMs);
            case CENTER:
                return Math.max(0.0, feederConfig().center.holdMs);
            case RIGHT:
            default:
                return Math.max(0.0, feederConfig().right.holdMs);
        }
    }

    private static double feederLoadPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(feederConfig().left.loadPosition);
            case CENTER:
                return clampServo(feederConfig().center.loadPosition);
            case RIGHT:
            default:
                return clampServo(feederConfig().right.loadPosition);
        }
    }

    private static double feederFirePositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(feederConfig().left.firePosition);
            case CENTER:
                return clampServo(feederConfig().center.firePosition);
            case RIGHT:
            default:
                return clampServo(feederConfig().right.firePosition);
        }
    }

    private static boolean feederReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return feederConfig().left.reversed;
            case CENTER:
                return feederConfig().center.reversed;
            case RIGHT:
            default:
                return feederConfig().right.reversed;
        }
    }

    private static String motorNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return flywheelConfig.flywheelLeft.motorName;
            case CENTER:
                return flywheelConfig.flywheelCenter.motorName;
            case RIGHT:
            default:
                return flywheelConfig.flywheelRight.motorName;
        }
    }

    private static boolean motorReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return flywheelConfig.flywheelLeft.reversed;
            case CENTER:
                return flywheelConfig.flywheelCenter.reversed;
            case RIGHT:
            default:
                return flywheelConfig.flywheelRight.reversed;
        }
    }

    private static String feederNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return feederConfig().left.servoName;
            case CENTER:
                return feederConfig().center.servoName;
            case RIGHT:
            default:
                return feederConfig().right.servoName;
        }
    }

    private static double rpmToTicksPerSecond(double rpm) {
        if (rpm <= 0.0) {
            return 0.0;
        }
        return rpm * flywheelConfig.parameters.ticksPerRev * flywheelConfig.parameters.gearRatio / 60.0;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / (flywheelConfig.parameters.ticksPerRev * flywheelConfig.parameters.gearRatio);
    }

    private static double clampServo(double position) {
        if (position < 0.0) {
            return 0.0;
        }
        if (position > 1.0) {
            return 1.0;
        }
        return position;
    }

    private static class ShotRequest {
        final LauncherLane lane;
        final double scheduledTimeMs;

        ShotRequest(LauncherLane lane, double scheduledTimeMs) {
            this.lane = lane;
            this.scheduledTimeMs = scheduledTimeMs;
        }
    }

    private class Flywheel {
        private final LauncherLane lane;
        private final DcMotorEx motor;
        private final ElapsedTime launchTimer = new ElapsedTime();

        private double commandedRpm = 0.0;
        private boolean launchCommandActive = false;
        private ControlPhase phase = ControlPhase.BANG;
        private int bangToHybridCounter = 0;
        private int hybridToBangCounter = 0;
        private int bangToHoldCounter = 0;
        private int holdToBangCounter = 0;
        private int lastPositionTicks = 0;
        private double lastSampleTimeMs = Double.NaN;
        private double estimatedTicksPerSec = 0.0;

        Flywheel(LauncherLane lane, HardwareMap hardwareMap) {
            this.lane = lane;
            this.motor = tryGetMotor(hardwareMap, motorNameFor(lane));
            if (this.motor != null) {
                configureMotor();
            }
        }

        void initialize() {
            commandedRpm = 0.0;
            launchCommandActive = false;
            phase = ControlPhase.BANG;
            bangToHybridCounter = 0;
            hybridToBangCounter = 0;
            bangToHoldCounter = 0;
            holdToBangCounter = 0;
            launchTimer.reset();
            if (motor != null) {
                motor.setPower(0.0);
                lastPositionTicks = motor.getCurrentPosition();
            }
            lastSampleTimeMs = clock.milliseconds();
            estimatedTicksPerSec = 0.0;
        }

        void commandLaunch() {
            setTargetRpm(launchRpmFor(lane));
        }

        void commandIdle() {
            setTargetRpm(idleRpmFor(lane));
        }

        void stop() {
            setTargetRpm(0.0);
        }

        void commandCustom(double rpm) {
            setTargetRpm(rpm);
        }

        void setReverseIntake() {
            commandedRpm = 0.0;
            launchCommandActive = false;
            if (motor != null) {
                motor.setPower(reverseFlywheelForHumanLoadingConfig.reversePower);
            }
        }

        double getCurrentRpm() {
            return Math.abs(ticksPerSecondToRpm(estimatedTicksPerSec));
        }

        double getTargetRpm() {
            return commandedRpm;
        }

        double getAppliedPower() {
            return motor == null ? 0.0 : motor.getPower();
        }

        String getPhaseName() {
            return phase.name();
        }

        int getBangToHoldCounter() {
            return bangToHoldCounter;
        }

        boolean isAtLaunch() {
            double launchRpm = launchRpmFor(lane);
            if (launchRpm <= 0.0) {
                return false;
            }
            double currentRpm = getCurrentRpm();
            double error = Math.abs(currentRpm - launchRpm);

            // Primary check: within tolerance of target RPM
            if (error <= flywheelConfig.parameters.rpmTolerance) {
                return true;
            }

            if (!launchCommandActive) {
                return false;
            }

            double elapsed = launchTimer.milliseconds();

            // Fallback for spinning UP: allow if minimal time elapsed and we've commanded the right RPM
            // This handles encoder issues but still requires the command to be sent
            if (elapsed >= timing.fallbackReadyMs) {
                return true;  // Timeout - assume ready
            }

            // For early ready (before fallback timeout), check both:
            // 1. Minimal spin-up time elapsed
            // 2. Current RPM is appropriate for target (works for both spin-up and spin-down)
            if (elapsed >= timing.minimalSpinUpMs) {
                // Check if we're close enough OR if spinning down, at least below a reasonable threshold
                // Allow some headroom when spinning down (within 10% above target is acceptable)
                double upperThreshold = launchRpm * 1.1;  // 10% overspeed tolerance
                return currentRpm <= upperThreshold;
            }

            return false;
        }

        void updateControl() {
            updateVelocityEstimate();

            if (motor == null) {
                return;
            }

            // Skip normal control if reverse intake is active
            if (reverseFlywheelActive) {
                return;
            }

            if (commandedRpm <= 0.0) {
                motor.setPower(0.0);
                phase = ControlPhase.BANG;
                return;
            }

            double error = commandedRpm - getCurrentRpm();
            double absError = Math.abs(error);

            // Pure bang-bang mode - simple and fast
            if (getFlywheelControlMode() == FlywheelControlMode.PURE_BANG_BANG) {
                phase = ControlPhase.BANG;
                applyBangBangControl(error);
                return;
            }

            // Legacy multi-phase control for HYBRID and BANG_BANG_HOLD modes
            switch (phase) {
                case BANG:
                    if (getFlywheelControlMode() == FlywheelControlMode.BANG_BANG_HOLD) {
                        if (absError <= flywheelConfig.modeConfig.bangBang.exitBangThresholdRpm) {
                            bangToHoldCounter++;
                            if (bangToHoldCounter >= Math.max(1, flywheelConfig.modeConfig.phaseSwitch.bangToHoldConfirmCycles)) {
                                phase = ControlPhase.HOLD;
                                bangToHoldCounter = 0;
                            }
                        } else {
                            bangToHoldCounter = 0;
                        }
                        bangToHybridCounter = 0;
                    } else {
                        if (absError <= flywheelConfig.modeConfig.bangBang.exitBangThresholdRpm) {
                            bangToHybridCounter++;
                            if (bangToHybridCounter >= Math.max(1, flywheelConfig.modeConfig.phaseSwitch.bangToHybridConfirmCycles)) {
                                phase = ControlPhase.HYBRID;
                                bangToHybridCounter = 0;
                            }
                        } else {
                            bangToHybridCounter = 0;
                        }
                        bangToHoldCounter = 0;
                    }
                    hybridToBangCounter = 0;
                    holdToBangCounter = 0;
                    break;
                case HYBRID:
                    if (absError >= flywheelConfig.modeConfig.bangBang.enterBangThresholdRpm) {
                        hybridToBangCounter++;
                        if (hybridToBangCounter >= Math.max(1, flywheelConfig.modeConfig.phaseSwitch.hybridToBangConfirmCycles)) {
                            phase = ControlPhase.BANG;
                            hybridToBangCounter = 0;
                            bangToHybridCounter = 0;
                        }
                    } else {
                        hybridToBangCounter = 0;
                    }
                    bangToHoldCounter = 0;
                    holdToBangCounter = 0;
                    break;
                case HOLD:
                    if (absError >= flywheelConfig.modeConfig.bangBang.enterBangThresholdRpm) {
                        holdToBangCounter++;
                        if (holdToBangCounter >= Math.max(1, flywheelConfig.modeConfig.phaseSwitch.holdToBangConfirmCycles)) {
                            phase = ControlPhase.BANG;
                            holdToBangCounter = 0;
                            bangToHoldCounter = 0;
                        }
                    } else {
                        holdToBangCounter = 0;
                    }
                    bangToHybridCounter = 0;
                    hybridToBangCounter = 0;
                    break;
            }

            if (phase == ControlPhase.BANG) {
                applyBangBangControl(error);
            } else if (phase == ControlPhase.HYBRID) {
                applyHybridControl(error);
            } else {
                applyHoldControl(error);
            }
        }

        private void setTargetRpm(double rpm) {
            double sanitized = Math.max(0.0, rpm);
            commandedRpm = sanitized;
            launchCommandActive = sanitized > 0.0;
            if (launchCommandActive) {
                launchTimer.reset();
            }
            phase = ControlPhase.BANG;
            bangToHybridCounter = 0;
            hybridToBangCounter = 0;
            bangToHoldCounter = 0;
            holdToBangCounter = 0;
        }

        private void applyBangBangControl(double error) {
            double high = Range.clip(flywheelConfig.modeConfig.bangBang.highPower, -1.0, 1.0);
            double low = Range.clip(flywheelConfig.modeConfig.bangBang.lowPower, -1.0, 1.0);

            // Apply voltage compensation
            double voltageMultiplier = getVoltageCompensationMultiplier();
            high = Range.clip(high * voltageMultiplier, -1.0, 1.0);
            low = Range.clip(low * voltageMultiplier, -1.0, 1.0);

            // Simple bang-bang: high power when below target, low power when at/above target
            if (error > 0.0) {
                motor.setPower(high);
            } else {
                motor.setPower(low);
            }
        }

        private void applyHybridControl(double error) {
            double power = flywheelConfig.modeConfig.hybridPid.kF + flywheelConfig.modeConfig.hybridPid.kP * error;
            power = Range.clip(power, 0.0, Math.max(0.0, flywheelConfig.modeConfig.hybridPid.maxPower));

            // Apply voltage compensation
            double voltageMultiplier = getVoltageCompensationMultiplier();
            power = Range.clip(power * voltageMultiplier, 0.0, 1.0);

            motor.setPower(power);
        }

        private void applyHoldControl(double error) {
            double holdPower = flywheelConfig.modeConfig.hold.baseHoldPower + flywheelConfig.modeConfig.hold.rpmPowerGain * commandedRpm;
            holdPower = Range.clip(holdPower, flywheelConfig.modeConfig.hold.minHoldPower, flywheelConfig.modeConfig.hold.maxHoldPower);

            // Apply voltage compensation
            double voltageMultiplier = getVoltageCompensationMultiplier();
            holdPower = Range.clip(holdPower * voltageMultiplier, 0.0, 1.0);
            double lowPower = Range.clip(flywheelConfig.modeConfig.bangBang.lowPower * voltageMultiplier, 0.0, 1.0);

            if (error < 0.0) {
                motor.setPower(lowPower);
            } else {
                motor.setPower(holdPower);
            }
        }

        private void configureMotor() {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (motorReversedFor(lane)) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        private void updateVelocityEstimate() {
            if (motor == null) {
                return;
            }
            double nowMs = clock.milliseconds();
            int position = motor.getCurrentPosition();
            if (!Double.isNaN(lastSampleTimeMs)) {
                double deltaMs = nowMs - lastSampleTimeMs;
                if (deltaMs > 1.0) {
                    int deltaTicks = position - lastPositionTicks;
                    // Validate encoder delta to prevent glitch-induced RPM spikes
                    // Max physical RPM ~6000, which is ~2800 ticks/sec
                    // Allow 2x safety margin = 5600 ticks/sec max
                    double maxTicksDelta = 5600.0 * deltaMs / 1000.0;
                    if (Math.abs(deltaTicks) < maxTicksDelta) {
                        double ticksPerSec = deltaTicks * 1000.0 / deltaMs;
                        estimatedTicksPerSec = Math.abs(ticksPerSec);
                    }
                    // If delta is too large, keep previous estimate (encoder glitch detected)
                }
            }
            lastPositionTicks = position;
            lastSampleTimeMs = nowMs;
        }
    }

    private class Feeder {
        private final LauncherLane lane;
        private final Servo servo;
        private final ElapsedTime timer = new ElapsedTime();

        private boolean busy = false;

        Feeder(LauncherLane lane, HardwareMap hardwareMap) {
            this.lane = lane;
            this.servo = tryGetServo(hardwareMap, feederNameFor(lane));
            if (this.servo != null) {
                if (feederReversedFor(lane)) {
                    servo.setDirection(Servo.Direction.REVERSE);
                } else {
                    servo.setDirection(Servo.Direction.FORWARD);
                }
            }
        }

        void initialize() {
            busy = false;
            if (servo != null) {
                servo.setPosition(feederLoadPositionFor(lane));
            }
        }

        void fire() {
            if (servo != null) {
                servo.setPosition(feederFirePositionFor(lane));
                busy = true;
                timer.reset();
            }
        }

        void stop() {
            if (servo != null) {
                servo.setPosition(feederLoadPositionFor(lane));
            }
            busy = false;
        }

        void toLoadPosition() {
            if (servo != null) {
                servo.setPosition(feederLoadPositionFor(lane));
            }
            busy = false;
        }

        void update() {
            if (!busy) {
                return;
            }
            if (timer.milliseconds() >= feederHoldMsFor(lane)) {
                if (servo != null) {
                    servo.setPosition(feederLoadPositionFor(lane));
                }
                busy = false;
            }
        }

        boolean isBusy() {
            return busy;
        }

        double getPosition() {
            return servo == null ? Double.NaN : servo.getPosition();
        }
    }

    private static class Hood {
        private final LauncherLane lane;
        private final Servo servo;

        Hood(LauncherLane lane, HardwareMap hardwareMap) {
            this.lane = lane;
            this.servo = tryGetServo(hardwareMap, hoodNameFor(lane));
        }

        void initialize() {
            if (servo != null) {
                // Initialize to mid-range position by default
                servo.setPosition(hoodMidPositionFor(lane));
            }
        }

        void setPosition(double position) {
            if (servo != null) {
                servo.setPosition(clampServo(position));
            }
        }

        void setForRange(org.firstinspires.ftc.teamcode.util.LauncherRange range) {
            if (servo == null) {
                return;
            }
            double position;
            switch (range) {
                case SHORT:
                    position = hoodShortPositionFor(lane);
                    break;
                case MID:
                    position = hoodMidPositionFor(lane);
                    break;
                case LONG:
                    position = hoodLongPositionFor(lane);
                    break;
                default:
                    position = hoodMidPositionFor(lane);
                    break;
            }
            servo.setPosition(clampServo(position));
        }

        double getPosition() {
            return servo == null ? Double.NaN : servo.getPosition();
        }
    }

    private static String hoodNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return hoodConfig().hoodLeft.servoName;
            case CENTER:
                return hoodConfig().hoodCenter.servoName;
            case RIGHT:
            default:
                return hoodConfig().hoodRight.servoName;
        }
    }

    public static double hoodShortPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(hoodConfig().hoodLeft.shortPosition);
            case CENTER:
                return clampServo(hoodConfig().hoodCenter.shortPosition);
            case RIGHT:
            default:
                return clampServo(hoodConfig().hoodRight.shortPosition);
        }
    }

    public static double hoodMidPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(hoodConfig().hoodLeft.midPosition);
            case CENTER:
                return clampServo(hoodConfig().hoodCenter.midPosition);
            case RIGHT:
            default:
                return clampServo(hoodConfig().hoodRight.midPosition);
        }
    }

    public static double hoodLongPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(hoodConfig().hoodLeft.longPosition);
            case CENTER:
                return clampServo(hoodConfig().hoodCenter.longPosition);
            case RIGHT:
            default:
                return clampServo(hoodConfig().hoodRight.longPosition);
        }
    }



}
