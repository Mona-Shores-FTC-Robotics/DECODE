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
import org.firstinspires.ftc.teamcode.util.RobotMode;

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
        BANG_BANG_HOLD
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
    public static class FlywheelParameters {
        /** Encoder ticks per motor revolution (adjust for the selected motor). */
        public static double ticksPerRev = 28;
        /** Output wheel revolutions per motor revolution. */
        public static double gearRatio = 1.0;
        /** Acceptable RPM error when considering a lane ready to fire. */
        public static double rpmTolerance = 60.0;
    }

    @Configurable
    public static class Timing {
        /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
        public static double minimalSpinUpMs = 500;
        /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
        public static double fallbackReadyMs = 3000;
        /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
        public static double recoveryMs = 1000;
        /** Delay between sequential shots when bursting all three lanes (ms). */
        public static double burstSpacingMs = 120.0;
    }

    @Configurable
    public static class BangBangConfig {
        public static double highPower = 1.0;
        public static double lowPower = 0.3;
        public static double enterBangThresholdRpm = 1200;
        public static double exitBangThresholdRpm = 500;
        public static double bangDeadbandRpm = 50;
    }

    @Configurable
    public static class HybridPidConfig {
        public static double kP = 0.000075;
        public static double kF = 0.32;
        public static double maxPower = 1.0;
    }

    @Configurable
    public static class HoldConfig {
        public static double baseHoldPower = 0.32;
        public static double rpmPowerGain = 0.000075;
        public static double minHoldPower = 0.2;
        public static double maxHoldPower = 1.0;
    }

    @Configurable
    public static class FlywheelModeConfig {
        public static FlywheelControlMode mode = FlywheelControlMode.BANG_BANG_HOLD;
    }

    @Configurable
    public static class PhaseSwitchConfig {
        public static int bangToHybridConfirmCycles = 3;
        public static int bangToHoldConfirmCycles = 3;
        public static int hybridToBangConfirmCycles = 2;
        public static int holdToBangConfirmCycles = 2;
    }

    @Configurable
    public static class LeftFlywheelConfig {
        public static String motorName = "launcher_left";
        public static boolean reversed = true;
        public static double launchRpm = 0;
        public static double idleRpm = 0;
    }

    @Configurable
    public static class CenterFlywheelConfig {
        public static String motorName = "launcher_center";
        public static boolean reversed = false;
        public static double launchRpm = 0;
        public static double idleRpm = 0;
    }

    @Configurable
    public static class RightFlywheelConfig { //actually left
        public static String motorName = "launcher_right";
        public static boolean reversed = true;
        public static double launchRpm = 0;
        public static double idleRpm = 0;
    }

    @Configurable
    public static class LeftFeederConfig {
        public static String servoName = "feeder_left";
        public static boolean reversed = false;
        public static double loadPosition = 1.0;
        public static double firePosition = .7;
        public static double holdMs = 3000;
    }

    @Configurable
    public static class CenterFeederConfig {
        public static String servoName = "feeder_center";
        public static boolean reversed = false;
        public static double loadPosition = .7;
        public static double firePosition = 1.0;
        public static double holdMs = 2000;
    }

    @Configurable
    public static class RightFeederConfig {
        public static String servoName = "feeder_right";
        public static boolean reversed = false;
        public static double loadPosition = .7;
        public static double firePosition = 1;
        public static double holdMs = 2000;
    }

    private final EnumMap<LauncherLane, Flywheel> flywheels = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Feeder> feeders = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneRecoveryDeadlineMs = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> launchRpmOverrides = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> idleRpmOverrides = new EnumMap<>(LauncherLane.class);
    private final Deque<ShotRequest> shotQueue = new ArrayDeque<>();
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private SpinMode requestedSpinMode = SpinMode.OFF;
    private LauncherState state = LauncherState.DISABLED;
    private double lastPeriodicMs = 0.0;

    public static FlywheelControlMode getFlywheelControlMode() {
        return FlywheelModeConfig.mode;
    }

     public String getPhaseName(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? "UNKNOWN" : flywheel.getPhaseName();
    }

    private boolean debugOverrideEnabled = false;

    public static final class Inputs {
        public LauncherState state = LauncherState.DISABLED;
        public SpinMode requestedSpinMode = SpinMode.OFF;
        public SpinMode effectiveSpinMode = SpinMode.OFF;
        public String controlMode = FlywheelControlMode.HYBRID.name();
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

    public LauncherSubsystem(HardwareMap hardwareMap) {
        for (LauncherLane lane : LauncherLane.values()) {
            flywheels.put(lane, new Flywheel(lane, hardwareMap));
            feeders.put(lane, new Feeder(lane, hardwareMap));
            laneRecoveryDeadlineMs.put(lane, 0.0);
        }
    }

    @Override
    public void initialize() {
        clock.reset();
        stateTimer.reset();
        shotQueue.clear();
        requestedSpinMode = SpinMode.OFF;

        for (Map.Entry<LauncherLane, Flywheel> entry : flywheels.entrySet()) {
            entry.getValue().initialize();
            laneRecoveryDeadlineMs.put(entry.getKey(), 0.0);
        }
        for (Feeder feeder : feeders.values()) {
            feeder.initialize();
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

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        inputs.state = state;
        inputs.requestedSpinMode = requestedSpinMode;
        inputs.effectiveSpinMode = computeEffectiveSpinMode();
        inputs.controlMode = getFlywheelControlMode().name();
        inputs.busy = isBusy();
        double now = clock.milliseconds();
        ShotRequest pendingShot = shotQueue.peekFirst();
        if (pendingShot == null) {
            inputs.activeShotLane = "NONE";
            inputs.activeShotAgeMs = 0.0;
        } else {
            inputs.activeShotLane = pendingShot.lane.name();
            inputs.activeShotAgeMs = Math.max(0.0, now - pendingShot.scheduledTimeMs);
        }
        double lastCompletionMs = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            double deadline = laneRecoveryDeadlineMs.getOrDefault(lane, 0.0);
            if (deadline > 0.0) {
                lastCompletionMs = Math.max(lastCompletionMs, deadline - Timing.recoveryMs);
            }
        }
        inputs.lastShotCompletionMs = lastCompletionMs;
        inputs.queuedShots = getQueuedShots();
        inputs.stateElapsedSec = getStateElapsedSeconds();
        inputs.averageTargetRpm = getTargetRpm();
        inputs.averageCurrentRpm = getCurrentRpm();
        inputs.averagePower = getLastPower();
        populateLane(inputs, LauncherLane.LEFT);
        populateLane(inputs, LauncherLane.CENTER);
        populateLane(inputs, LauncherLane.RIGHT);
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
            delayMs += Timing.burstSpacingMs;
        }
    }

    public void clearQueue() {
        shotQueue.clear();
    }

    public void abort() {
        clearQueue();
        double now = clock.milliseconds();
        for (LauncherLane lane : LauncherLane.values()) {
            laneRecoveryDeadlineMs.put(lane, now + Timing.recoveryMs);
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
                    laneRecoveryDeadlineMs.put(next.lane, now + Timing.recoveryMs);
                    iterator.remove();
                }
            }
        }

        reflectSpinState(effectiveSpinMode);
    }

    private void updateLaneRecovery(double now) {
        for (LauncherLane lane : LauncherLane.values()) {
            double deadline = laneRecoveryDeadlineMs.getOrDefault(lane, 0.0);
            if (deadline > 0.0 && now >= deadline) {
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
        if (debugOverrideEnabled) {
            return;
        }

        double now = clock.milliseconds();
        boolean shotDrivenFull = !shotQueue.isEmpty() && requestedSpinMode != SpinMode.FULL;
        Set<LauncherLane> queuedLanes = shotDrivenFull ? lanesWithQueuedShots() : EnumSet.noneOf(LauncherLane.class);
        for (LauncherLane lane : LauncherLane.values()) {
            Flywheel flywheel = flywheels.get(lane);
            if (flywheel == null) {
                continue;
            }

            // Skip lanes that are currently in recovery
            if (now < laneRecoveryDeadlineMs.getOrDefault(lane, 0.0)) {
                continue;
            }

            SpinMode laneMode = mode;
            if (shotDrivenFull && !queuedLanes.contains(lane)) {
                laneMode = SpinMode.IDLE;
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
            this.motor = hardwareMap.get(DcMotorEx.class, motorNameFor(lane));
            configureMotor();
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
            motor.setPower(0.0);
            lastPositionTicks = motor.getCurrentPosition();
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

        double getCurrentRpm() {
            return Math.abs(ticksPerSecondToRpm(estimatedTicksPerSec));
        }

        double getTargetRpm() {
            return commandedRpm;
        }

        double getAppliedPower() {
            return motor.getPower();
        }

        String getPhaseName() {
            return phase.name();
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
            updateVelocityEstimate();

            if (commandedRpm <= 0.0) {
                motor.setPower(0.0);
                phase = ControlPhase.BANG;
                return;
            }

            double error = commandedRpm - getCurrentRpm();
            double absError = Math.abs(error);

            switch (phase) {
                case BANG:
                    if (getFlywheelControlMode() == FlywheelControlMode.BANG_BANG_HOLD) {
                        if (absError <= BangBangConfig.exitBangThresholdRpm) {
                            bangToHoldCounter++;
                            if (bangToHoldCounter >= Math.max(1, PhaseSwitchConfig.bangToHoldConfirmCycles)) {
                                phase = ControlPhase.HOLD;
                                bangToHoldCounter = 0;
                            }
                        } else {
                            bangToHoldCounter = 0;
                        }
                        bangToHybridCounter = 0;
                    } else {
                        if (absError <= BangBangConfig.exitBangThresholdRpm) {
                            bangToHybridCounter++;
                            if (bangToHybridCounter >= Math.max(1, PhaseSwitchConfig.bangToHybridConfirmCycles)) {
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
                    if (absError >= BangBangConfig.enterBangThresholdRpm) {
                        hybridToBangCounter++;
                        if (hybridToBangCounter >= Math.max(1, PhaseSwitchConfig.hybridToBangConfirmCycles)) {
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
                    if (absError >= BangBangConfig.enterBangThresholdRpm) {
                        holdToBangCounter++;
                        if (holdToBangCounter >= Math.max(1, PhaseSwitchConfig.holdToBangConfirmCycles)) {
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
            double threshold = Math.max(0.0, BangBangConfig.bangDeadbandRpm);
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
            double power = HybridPidConfig.kF + HybridPidConfig.kP * error;
            power = Range.clip(power, 0.0, Math.max(0.0, HybridPidConfig.maxPower));
            motor.setPower(power);
        }

        private void applyHoldControl(double error) {
            double holdPower = HoldConfig.baseHoldPower + HoldConfig.rpmPowerGain * commandedRpm;
            holdPower = Range.clip(holdPower, HoldConfig.minHoldPower, HoldConfig.maxHoldPower);
            if (error < 0.0) {
                motor.setPower(BangBangConfig.lowPower);
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
            double nowMs = clock.milliseconds();
            int position = motor.getCurrentPosition();
            if (!Double.isNaN(lastSampleTimeMs)) {
                double deltaMs = nowMs - lastSampleTimeMs;
                if (deltaMs > 1.0) {
                    double ticksPerSec = (position - lastPositionTicks) * 1000.0 / deltaMs;
                    estimatedTicksPerSec = Math.abs(ticksPerSec);
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
