package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.EnumMap;
import java.util.Map;

/**
 * Shooter subsystem that manages three flywheels and their paired bootkicker servos.
 * Callers queue individual shots or bursts while this class coordinates spin-up,
 * feed timing, and recovery delays.
 */
@Configurable
public class ShooterSubsystem implements Subsystem {

    public enum SpinMode {
        OFF,
        HOLD,
        FULL
    }

    public enum FlywheelControlMode {
        SDK_PID,
        BANG_BANG,
        HYBRID
    }

    public enum ShooterState {
        DISABLED,
        HOLDING,
        SPINNING_UP,
        READY,
        FEEDING,
        RECOVERING
    }

    @Configurable
    public static class FlywheelParameters {
        /** Encoder ticks per motor revolution (adjust for the selected motor). */
        public static double ticksPerRev = 537.7;
        /** Output wheel revolutions per motor revolution. */
        public static double gearRatio = 1.0;
        /** Acceptable RPM error when considering a lane ready to fire. */
        public static double rpmTolerance = 60.0;
    }

    @Configurable
    public static class Timing {
        /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
        public static double minimalSpinUpMs = 250.0;
        /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
        public static double fallbackReadyMs = 600.0;
        /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
        public static double recoveryMs = 150.0;
        /** Delay between sequential shots when bursting all three lanes (ms). */
        public static double burstSpacingMs = 120.0;
    }

    @Configurable
    public static class VelocityPID {
        /**
         * PIDF parameters applied to each flywheel motor while RUN_USING_ENCODER.
         * Adjust on the dashboard to tune velocity control.
         */
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    @Configurable
    public static class BangBangConfig {
        public static double highPower = 1.0;
        public static double lowPower = 0.0;
        public static double thresholdRpm = 40.0;
    }

    @Configurable
    public static class HybridConfig {
        public static double bangEntryThresholdRpm = 150.0;
        public static double bangExitThresholdRpm = 60.0;
        public static double kP = 0.0005;
        public static double kF = 0.12;
        public static double maxPower = 1.0;
    }

    @Configurable
    public static class ControlConfig {
        public static int flywheelModeIndex = FlywheelControlMode.SDK_PID.ordinal();
    }

    @Configurable
    public static class LeftFlywheelConfig {
        public static String motorName = "shooter_left";
        public static boolean reversed = false;
        public static double launchRpm = 4200.0;
        public static double idleRpm = 2600.0;
    }

    @Configurable
    public static class CenterFlywheelConfig {
        public static String motorName = "shooter_center";
        public static boolean reversed = false;
        public static double launchRpm = 4200.0;
        public static double idleRpm = 2600.0;
    }

    @Configurable
    public static class RightFlywheelConfig {
        public static String motorName = "shooter_right";
        public static boolean reversed = false;
        public static double launchRpm = 4200.0;
        public static double idleRpm = 2600.0;
    }

    @Configurable
    public static class LeftFeederConfig {
        public static String servoName = "feeder_left";
        public static boolean reversed = false;
        public static double loadPosition = 0.12;
        public static double firePosition = 0.48;
        public static double holdMs = 140.0;
    }

    @Configurable
    public static class CenterFeederConfig {
        public static String servoName = "feeder_center";
        public static boolean reversed = false;
        public static double loadPosition = 0.12;
        public static double firePosition = 0.48;
        public static double holdMs = 140.0;
    }

    @Configurable
    public static class RightFeederConfig {
        public static String servoName = "feeder_right";
        public static boolean reversed = false;
        public static double loadPosition = 0.12;
        public static double firePosition = 0.48;
        public static double holdMs = 140.0;
    }

    private static final LauncherLane[] DEFAULT_BURST_ORDER = {LauncherLane.LEFT, LauncherLane.CENTER, LauncherLane.RIGHT};
    private static final LauncherLane DEFAULT_AUTON_LANE = LauncherLane.CENTER;

    private final EnumMap<LauncherLane, Flywheel> flywheels = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Feeder> feeders = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneRecoveryDeadlineMs = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> launchRpmOverrides = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> idleRpmOverrides = new EnumMap<>(LauncherLane.class);
    private final Deque<ShotRequest> shotQueue = new ArrayDeque<>();
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private SpinMode requestedSpinMode = SpinMode.OFF;
    private ShooterState state = ShooterState.DISABLED;
    private ShotRequest activeShot;
    private double lastShotCompletionMs = 0.0;

    public static final class Inputs {
        public ShooterState state = ShooterState.DISABLED;
        public SpinMode requestedSpinMode = SpinMode.OFF;
        public SpinMode effectiveSpinMode = SpinMode.OFF;
        public FlywheelControlMode controlMode = FlywheelControlMode.SDK_PID;
        public boolean busy;
        public int queuedShots;
        public double stateElapsedSec;
        public String activeShotLane = "NONE";
        public double activeShotAgeMs;
        public double lastShotCompletionMs;
        public double averageTargetRpm;
        public double averageCurrentRpm;
        public double averagePower;
        public double leftTargetRpm;
        public double leftCurrentRpm;
        public double leftPower;
        public boolean leftReady;
        public double leftFeederPosition;
        public double leftLaunchRpm;
        public double leftIdleRpm;
        public double leftRecoveryRemainingMs;
        public double centerTargetRpm;
        public double centerCurrentRpm;
        public double centerPower;
        public boolean centerReady;
        public double centerFeederPosition;
        public double centerLaunchRpm;
        public double centerIdleRpm;
        public double centerRecoveryRemainingMs;
        public double rightTargetRpm;
        public double rightCurrentRpm;
        public double rightPower;
        public boolean rightReady;
        public double rightFeederPosition;
        public double rightLaunchRpm;
        public double rightIdleRpm;
        public double rightRecoveryRemainingMs;
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        for (LauncherLane lane : LauncherLane.values()) {
            flywheels.put(lane, new Flywheel(lane, hardwareMap));
            feeders.put(lane, new Feeder(lane, hardwareMap));
            laneRecoveryDeadlineMs.put(lane, 0.0);
        }
    }

    public static FlywheelControlMode getFlywheelControlMode() {
        int index = ControlConfig.flywheelModeIndex;
        if (index < 0) {
            index = 0;
        }
        FlywheelControlMode[] modes = FlywheelControlMode.values();
        if (index >= modes.length) {
            index = modes.length - 1;
        }
        return modes[index];
    }

    @Override
    public void initialize() {
        clock.reset();
        stateTimer.reset();
        shotQueue.clear();
        activeShot = null;
        lastShotCompletionMs = 0.0;
        requestedSpinMode = SpinMode.OFF;

        for (Map.Entry<LauncherLane, Flywheel> entry : flywheels.entrySet()) {
            entry.getValue().initialize();
            laneRecoveryDeadlineMs.put(entry.getKey(), 0.0);
        }
        for (Feeder feeder : feeders.values()) {
            feeder.initialize();
        }
        setState(ShooterState.DISABLED);
    }

    @Override
    public void periodic() {
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
    }

    public void toggleSpinUp() {
        if (requestedSpinMode == SpinMode.FULL) {
            setSpinMode(SpinMode.OFF);
        } else {
            setSpinMode(SpinMode.FULL);
        }
    }

    public void setSpinMode(SpinMode mode) {
        requestedSpinMode = mode == null ? SpinMode.OFF : mode;
        applySpinMode(computeEffectiveSpinMode());
    }

    public void requestSpinUp() {
        setSpinMode(SpinMode.FULL);
    }

    public void requestStandbySpin() {
        setSpinMode(SpinMode.HOLD);
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
        return activeShot != null
                || !shotQueue.isEmpty()
                || state == ShooterState.SPINNING_UP
                || state == ShooterState.FEEDING
                || state == ShooterState.RECOVERING;
    }

    public ShooterState getState() {
        return state;
    }

    public double getStateElapsedSeconds() {
        return stateTimer.seconds();
    }

    public int getQueuedShots() {
        return shotQueue.size() + (activeShot != null ? 1 : 0);
    }

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        inputs.state = state;
        inputs.requestedSpinMode = requestedSpinMode;
        inputs.effectiveSpinMode = computeEffectiveSpinMode();
        inputs.controlMode = getFlywheelControlMode();
        inputs.busy = isBusy();
        inputs.queuedShots = getQueuedShots();
        inputs.stateElapsedSec = getStateElapsedSeconds();
        inputs.activeShotLane = activeShot == null ? "NONE" : activeShot.lane.name();
        inputs.activeShotAgeMs = activeShot == null ? 0.0 : Math.max(0.0, clock.milliseconds() - activeShot.scheduledTimeMs);
        inputs.lastShotCompletionMs = lastShotCompletionMs;
        inputs.averageTargetRpm = getTargetRpm();
        inputs.averageCurrentRpm = getCurrentRpm();
        inputs.averagePower = getLastPower();
        populateLane(inputs, LauncherLane.LEFT);
        populateLane(inputs, LauncherLane.CENTER);
        populateLane(inputs, LauncherLane.RIGHT);
    }

    public void shootLeft() {
        queueShot(LauncherLane.LEFT);
    }

    public void shootMiddle() {
        queueShot(LauncherLane.CENTER);
    }

    public void shootRight() {
        queueShot(LauncherLane.RIGHT);
    }

    public void shootAll() {
        queueBurstAll();
    }

    public void shoot(int count) {
        requestBurst(count);
    }

    public void burst(int count) {
        requestBurst(count);
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
        for (LauncherLane lane : DEFAULT_BURST_ORDER) {
            scheduleShot(lane, delayMs);
            delayMs += Timing.burstSpacingMs;
        }
    }

    public void requestBurst(int count) {
        if (count <= 0) {
            return;
        }
        double delayMs = 0.0;
        for (int i = 0; i < count; i++) {
            scheduleShot(DEFAULT_AUTON_LANE, delayMs);
            delayMs += Timing.burstSpacingMs;
        }
    }

    public void clearQueue() {
        shotQueue.clear();
        activeShot = null;
    }

    public void abort() {
        clearQueue();
        lastShotCompletionMs = clock.milliseconds();
        for (LauncherLane lane : LauncherLane.values()) {
            laneRecoveryDeadlineMs.put(lane, lastShotCompletionMs);
            feeders.get(lane).stop();
            flywheels.get(lane).stop();
        }
        setSpinMode(SpinMode.OFF);
        setState(ShooterState.DISABLED);
    }

    public void homeFeeder(LauncherLane lane) {
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

    private void scheduleShot(LauncherLane lane, double delayMs) {
        double when = clock.milliseconds() + Math.max(0.0, delayMs);
        shotQueue.addLast(new ShotRequest(lane, when));
    }

    private void updateStateMachine(double now, SpinMode effectiveSpinMode) {
        if (activeShot != null) {
            Feeder feeder = feeders.get(activeShot.lane);
            if (feeder != null && feeder.isBusy()) {
                setState(ShooterState.FEEDING);
                return;
            }
            laneRecoveryDeadlineMs.put(activeShot.lane, now + Timing.recoveryMs);
            lastShotCompletionMs = now;
            activeShot = null;
        }

        if (!shotQueue.isEmpty()) {
            ShotRequest next = shotQueue.peekFirst();
            if (now < next.scheduledTimeMs) {
                reflectSpinState(effectiveSpinMode);
                return;
            }
            if (!isLaneReadyForShot(next.lane, now)) {
                boolean recovering = now - lastShotCompletionMs < Timing.recoveryMs;
                setState(recovering ? ShooterState.RECOVERING : ShooterState.SPINNING_UP);
                return;
            }
            Feeder feeder = feeders.get(next.lane);
            if (feeder != null) {
                feeder.fire();
            }
            activeShot = shotQueue.removeFirst();
            setState(ShooterState.FEEDING);
            return;
        }

        boolean recovering = now - lastShotCompletionMs < Timing.recoveryMs;
        if (recovering) {
            setState(ShooterState.RECOVERING);
            return;
        }

        reflectSpinState(effectiveSpinMode);
    }

    private void reflectSpinState(SpinMode effectiveSpinMode) {
        switch (effectiveSpinMode) {
            case FULL:
                setState(atTarget() ? ShooterState.READY : ShooterState.SPINNING_UP);
                break;
            case HOLD:
                setState(ShooterState.HOLDING);
                break;
            case OFF:
            default:
                setState(ShooterState.DISABLED);
                break;
        }
    }

    private boolean isLaneReadyForShot(LauncherLane lane, double now) {
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
        for (LauncherLane lane : LauncherLane.values()) {
            Flywheel flywheel = flywheels.get(lane);
            if (flywheel == null) {
                continue;
            }
            switch (mode) {
                case FULL:
                    flywheel.commandLaunch();
                    break;
                case HOLD:
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
        if (activeShot != null || !shotQueue.isEmpty()) {
            return SpinMode.FULL;
        }
        return requestedSpinMode;
    }

    private void setState(ShooterState newState) {
        if (state != newState) {
            state = newState;
            stateTimer.reset();
        }
    }

    private double launchRpmFor(LauncherLane lane) {
        Double override = launchRpmOverrides.get(lane);
        if (override != null && override > 0.0) {
            return override;
        }
        switch (lane) {
            case LEFT:
                return Math.max(0.0, LeftFlywheelConfig.launchRpm);
            case CENTER:
                return Math.max(0.0, CenterFlywheelConfig.launchRpm);
            case RIGHT:
            default:
                return Math.max(0.0, RightFlywheelConfig.launchRpm);
        }
    }

    private double idleRpmFor(LauncherLane lane) {
        Double override = idleRpmOverrides.get(lane);
        if (override != null && override >= 0.0) {
            return override;
        }
        switch (lane) {
            case LEFT:
                return Math.max(0.0, LeftFlywheelConfig.idleRpm);
            case CENTER:
                return Math.max(0.0, CenterFlywheelConfig.idleRpm);
            case RIGHT:
            default:
                return Math.max(0.0, RightFlywheelConfig.idleRpm);
        }
    }

    private static double feederHoldMsFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return Math.max(0.0, LeftFeederConfig.holdMs);
            case CENTER:
                return Math.max(0.0, CenterFeederConfig.holdMs);
            case RIGHT:
            default:
                return Math.max(0.0, RightFeederConfig.holdMs);
        }
    }

    private static double feederLoadPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(LeftFeederConfig.loadPosition);
            case CENTER:
                return clampServo(CenterFeederConfig.loadPosition);
            case RIGHT:
            default:
                return clampServo(RightFeederConfig.loadPosition);
        }
    }

    private static double feederFirePositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(LeftFeederConfig.firePosition);
            case CENTER:
                return clampServo(CenterFeederConfig.firePosition);
            case RIGHT:
            default:
                return clampServo(RightFeederConfig.firePosition);
        }
    }

    private static boolean feederReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return LeftFeederConfig.reversed;
            case CENTER:
                return CenterFeederConfig.reversed;
            case RIGHT:
            default:
                return RightFeederConfig.reversed;
        }
    }

    private static String motorNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return LeftFlywheelConfig.motorName;
            case CENTER:
                return CenterFlywheelConfig.motorName;
            case RIGHT:
            default:
                return RightFlywheelConfig.motorName;
        }
    }

    private static boolean motorReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return LeftFlywheelConfig.reversed;
            case CENTER:
                return CenterFlywheelConfig.reversed;
            case RIGHT:
            default:
                return RightFlywheelConfig.reversed;
        }
    }

    private static String feederNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return LeftFeederConfig.servoName;
            case CENTER:
                return CenterFeederConfig.servoName;
            case RIGHT:
            default:
                return RightFeederConfig.servoName;
        }
    }

    private static double rpmToTicksPerSecond(double rpm) {
        if (rpm <= 0.0) {
            return 0.0;
        }
        return rpm * FlywheelParameters.ticksPerRev * FlywheelParameters.gearRatio / 60.0;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / (FlywheelParameters.ticksPerRev * FlywheelParameters.gearRatio);
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

    private void populateLane(Inputs inputs, LauncherLane lane) {
        double target = getTargetRpm(lane);
        double current = getCurrentRpm(lane);
        double power = getLastPower(lane);
        boolean ready = isLaneReady(lane);
        double feederPos = getFeederPosition(lane);
        double launchRpm = getLaunchRpm(lane);
        double idleRpm = getIdleRpm(lane);
        double recoveryDeadline = laneRecoveryDeadlineMs.getOrDefault(lane, 0.0);
        double remainingMs = Math.max(0.0, recoveryDeadline - clock.milliseconds());

        switch (lane) {
            case LEFT:
                inputs.leftTargetRpm = target;
                inputs.leftCurrentRpm = current;
                inputs.leftPower = power;
                inputs.leftReady = ready;
                inputs.leftFeederPosition = feederPos;
                inputs.leftLaunchRpm = launchRpm;
                inputs.leftIdleRpm = idleRpm;
                inputs.leftRecoveryRemainingMs = remainingMs;
                break;
            case CENTER:
                inputs.centerTargetRpm = target;
                inputs.centerCurrentRpm = current;
                inputs.centerPower = power;
                inputs.centerReady = ready;
                inputs.centerFeederPosition = feederPos;
                inputs.centerLaunchRpm = launchRpm;
                inputs.centerIdleRpm = idleRpm;
                inputs.centerRecoveryRemainingMs = remainingMs;
                break;
            case RIGHT:
            default:
                inputs.rightTargetRpm = target;
                inputs.rightCurrentRpm = current;
                inputs.rightPower = power;
                inputs.rightReady = ready;
                inputs.rightFeederPosition = feederPos;
                inputs.rightLaunchRpm = launchRpm;
                inputs.rightIdleRpm = idleRpm;
                inputs.rightRecoveryRemainingMs = remainingMs;
                break;
        }
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
        private FlywheelControlMode appliedMode = null;

        Flywheel(LauncherLane lane, HardwareMap hardwareMap) {
            this.lane = lane;
            this.motor = hardwareMap.get(DcMotorEx.class, motorNameFor(lane));
            configureMotor();
        }

        void initialize() {
            commandedRpm = 0.0;
            launchCommandActive = false;
            appliedMode = null;
            launchTimer.reset();
            ensureControlMode();
            motor.setPower(0.0);
            motor.setVelocity(0.0);
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

        double getCurrentRpm() {
            return ticksPerSecondToRpm(motor.getVelocity());
        }

        double getTargetRpm() {
            return commandedRpm;
        }

        double getAppliedPower() {
            return motor.getPower();
        }

        boolean isAtLaunch() {
            double launchRpm = launchRpmFor(lane);
            if (launchRpm <= 0.0) {
                return false;
            }
            double error = Math.abs(getCurrentRpm() - launchRpm);
            if (error <= FlywheelParameters.rpmTolerance) {
                return true;
            }
            if (!launchCommandActive) {
                return false;
            }
            double elapsed = launchTimer.milliseconds();
            return elapsed >= Timing.fallbackReadyMs
                    || (elapsed >= Timing.minimalSpinUpMs && commandedRpm >= launchRpm);
        }

        void updateControl() {
            ensureControlMode();
            FlywheelControlMode mode = ShooterSubsystem.getFlywheelControlMode();
            if (mode == FlywheelControlMode.SDK_PID) {
                return; // Built-in velocity loop already active.
            }
            if (commandedRpm <= 0.0) {
                motor.setPower(0.0);
                return;
            }
            double currentRpm = getCurrentRpm();
            double error = commandedRpm - currentRpm;
            switch (mode) {
                case BANG_BANG:
                    applyBangBangControl(error);
                    break;
                case HYBRID:
                    applyHybridControl(error);
                    break;
                case SDK_PID:
                default:
                    break;
            }
        }

        private void setTargetRpm(double rpm) {
            ensureControlMode();
            double sanitized = Math.max(0.0, rpm);
            commandedRpm = sanitized;
            FlywheelControlMode mode = ShooterSubsystem.getFlywheelControlMode();
            if (mode == FlywheelControlMode.SDK_PID) {
                double ticksPerSecond = rpmToTicksPerSecond(sanitized);
                motor.setVelocity(ticksPerSecond);
                if (sanitized <= 0.0) {
                    motor.setPower(0.0);
                }
            } else {
                if (sanitized <= 0.0) {
                    motor.setPower(0.0);
                }
            }
            boolean launching = sanitized >= launchRpmFor(lane) && sanitized > 0.0;
            if (launching && !launchCommandActive) {
                launchTimer.reset();
            }
            launchCommandActive = launching;
            if (!launching && sanitized <= 0.0) {
                launchCommandActive = false;
            }
        }

        private void ensureControlMode() {
            FlywheelControlMode desired = ShooterSubsystem.getFlywheelControlMode();
            if (desired == appliedMode) {
                return;
            }
            if (desired == FlywheelControlMode.SDK_PID) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                applyVelocityPidCoefficients();
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(0.0);
            }
            appliedMode = desired;
        }

        private void applyBangBangControl(double error) {
            double threshold = Math.max(0.0, BangBangConfig.thresholdRpm);
            double high = Range.clip(BangBangConfig.highPower, -1.0, 1.0);
            double low = Range.clip(BangBangConfig.lowPower, -1.0, 1.0);
            if (error > threshold) {
                motor.setPower(high);
            } else if (error < -threshold) {
                motor.setPower(low);
            } else {
                motor.setPower(low);
            }
        }

        private void applyHybridControl(double error) {
            double absError = Math.abs(error);
            double entry = Math.max(0.0, HybridConfig.bangEntryThresholdRpm);
            double exit = Math.max(0.0, HybridConfig.bangExitThresholdRpm);
            if (absError > entry) {
                applyBangBangControl(error);
                return;
            }
            if (absError > exit && error > 0) {
                applyBangBangControl(error);
                return;
            }
            double kP = HybridConfig.kP;
            double kF = HybridConfig.kF;
            double maxPower = Range.clip(HybridConfig.maxPower, 0.0, 1.0);
            double power = kF + kP * error;
            power = Range.clip(power, 0.0, maxPower);
            motor.setPower(power);
        }

        private void configureMotor() {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (motorReversedFor(lane)) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            applyVelocityPidCoefficients();
        }

        private void applyVelocityPidCoefficients() {
            PIDFCoefficients pidf = new PIDFCoefficients(
                    VelocityPID.kP,
                    VelocityPID.kI,
                    VelocityPID.kD,
                    VelocityPID.kF
            );
            try {
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            } catch (RuntimeException ignored) {
                // Some controllers do not support per-motor PIDF configuration.
            }
        }
    }

    private class Feeder {
        private final LauncherLane lane;
        private final Servo servo;
        private final ElapsedTime timer = new ElapsedTime();

        private boolean busy = false;

        Feeder(LauncherLane lane, HardwareMap hardwareMap) {
            this.lane = lane;
            this.servo = hardwareMap.get(Servo.class, feederNameFor(lane));
            if (feederReversedFor(lane)) {
                servo.setDirection(Servo.Direction.REVERSE);
            } else {
                servo.setDirection(Servo.Direction.FORWARD);
            }
        }

        void initialize() {
            busy = false;
            servo.setPosition(feederLoadPositionFor(lane));
        }

        void fire() {
            servo.setPosition(feederFirePositionFor(lane));
            busy = true;
            timer.reset();
        }

        void stop() {
            servo.setPosition(feederLoadPositionFor(lane));
            busy = false;
        }

        void toLoadPosition() {
            servo.setPosition(feederLoadPositionFor(lane));
            busy = false;
        }

        void update() {
            if (!busy) {
                return;
            }
            if (timer.milliseconds() >= feederHoldMsFor(lane)) {
                servo.setPosition(feederLoadPositionFor(lane));
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
}
