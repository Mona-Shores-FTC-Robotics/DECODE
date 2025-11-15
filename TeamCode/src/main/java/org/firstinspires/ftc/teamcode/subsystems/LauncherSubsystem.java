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
        public double ticksPerRev = 28;
        /** Output wheel revolutions per motor revolution. */
        public double gearRatio = 1.0;
        /** Acceptable RPM error when considering a lane ready to fire. */
        public double rpmTolerance = 60.0;
    }

    @Configurable
    public static class Timing {
        /** Minimum time the wheel should be commanded at launch speed before trusting fallback readiness. */
        public double minimalSpinUpMs = 1000;
        /** If encoders are unavailable, treat the wheel as ready after this many milliseconds at full power. */
        public double fallbackReadyMs = 1000;
        /** Servo dwell time to allow the artifact to clear before re-closing (ms). */
        public double recoveryMs = 1000;
        /** Delay between sequential shots when bursting all three lanes (ms). */
        public double burstSpacingMs = 120.0;
    }

    @Configurable
    public static class BangBangConfig {
        public double highPower = 1.0;
        public double lowPower = 0.3;
        public double enterBangThresholdRpm = 1200;
        public double exitBangThresholdRpm = 800;
        public double bangDeadbandRpm = 50;
    }

    @Configurable
    public static class HybridPidConfig {
        public double kP = 0.000075;
        public double kF = 0.32;
        public double maxPower = 1.0;
    }

    @Configurable
    public static class HoldConfig {
        public double baseHoldPower = 0.5;
        public double rpmPowerGain = 0.000075;
        public double minHoldPower = 0.2;
        public double maxHoldPower = 1.0;
    }

    @Configurable
    public static class FlywheelModeConfig {
        public FlywheelControlMode mode = FlywheelControlMode.BANG_BANG_HOLD;
    }

    @Configurable
    public static class PhaseSwitchConfig {
        public int bangToHybridConfirmCycles = 3;
        public int bangToHoldConfirmCycles = 5;
        public int hybridToBangConfirmCycles = 2;
        public int holdToBangConfirmCycles = 5;
    }

    public static FlywheelParameters flywheelParameters = new FlywheelParameters();
    public static Timing timing = new Timing();
    public static BangBangConfig bangBangConfig = new BangBangConfig();
    public static HybridPidConfig hybridPidConfig = new HybridPidConfig();
    public static HoldConfig holdConfig = new HoldConfig();
    public static FlywheelModeConfig flywheelModeConfig = new FlywheelModeConfig();
    public static PhaseSwitchConfig phaseSwitchConfig = new PhaseSwitchConfig();

    @Configurable
    public static class LeftFlywheelConfig {
        public String motorName = "launcher_left";
        public boolean reversed = true;
        public double launchRpm = 3600;
        public double idleRpm = 2600;
    }

    @Configurable
    public static class CenterFlywheelConfig {
        public String motorName = "launcher_center";
        public boolean reversed = false;
        public double launchRpm = 0;
        public double idleRpm = 0;
    }

    @Configurable
    public static class RightFlywheelConfig { //actually left
        public String motorName = "launcher_right";
        public boolean reversed = true;
        public double launchRpm = 3600;
        public double idleRpm = 2800;
    }

    @Configurable
    public static class LeftFeederConfig {
        public String servoName = "feeder_left";
        public boolean reversed = false;
        public double loadPosition = .93;
        public double firePosition = .65;
        public double holdMs = 4000;
    }

    @Configurable
    public static class CenterFeederConfig {
        public String servoName = "feeder_center";
        public boolean reversed = false;
        public double loadPosition = 1.0;
        public double firePosition = .7;
        public double holdMs = 4000;
    }

    @Configurable
    public static class RightFeederConfig {
        public String servoName = "feeder_right";
        public boolean reversed = false;
        public double loadPosition = .7;
        public double firePosition = .3;
        public double holdMs = 2000;
    }

    @Configurable
    public static class LeftHoodConfig {
        public String servoName = "hood_left";
        public boolean reversed = false;
        /** Hood position for short range shots */
        public double shortPosition = 0.5;
        /** Hood position for mid range shots */
        public double midPosition = 0.5;
        /** Hood position for long range shots */
        public double longPosition = 0.5;
    }

    @Configurable
    public static class CenterHoodConfig {
        public String servoName = "hood_center";
        public boolean reversed = false;
        /** Hood position for short range shots */
        public double shortPosition = 0.5;
        /** Hood position for mid range shots */
        public double midPosition = 0.5;
        /** Hood position for long range shots */
        public double longPosition = 0.5;
    }

    @Configurable
    public static class RightHoodConfig {
        public String servoName = "hood_right";
        public boolean reversed = false;
        /** Hood position for short range shots */
        public double shortPosition = 0.5;
        /** Hood position for mid range shots */
        public double midPosition = 0.5;
        /** Hood position for long range shots */
        public double longPosition = 0.5;
    }

    @Configurable
    public static class ReverseFlywheelForHumanLoadingConfig {
        /** Power level for reverse intake (negative runs motors backward) */
        public double reversePower = -0.25;
    }

    public static LeftFlywheelConfig leftFlywheelConfig = new LeftFlywheelConfig();
    public static CenterFlywheelConfig centerFlywheelConfig = new CenterFlywheelConfig();
    public static RightFlywheelConfig rightFlywheelConfig = new RightFlywheelConfig();
    public static LeftFeederConfig leftFeederConfig = new LeftFeederConfig();
    public static CenterFeederConfig centerFeederConfig = new CenterFeederConfig();
    public static RightFeederConfig rightFeederConfig = new RightFeederConfig();
    public static LeftHoodConfig leftHoodConfig = new LeftHoodConfig();
    public static CenterHoodConfig centerHoodConfig = new CenterHoodConfig();
    public static RightHoodConfig rightHoodConfig = new RightHoodConfig();
    public static ReverseFlywheelForHumanLoadingConfig reverseFlywheelForHumanLoadingConfig = new ReverseFlywheelForHumanLoadingConfig();

    private final EnumMap<LauncherLane, Flywheel> flywheels = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Feeder> feeders = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Hood> hoods = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneRecoveryDeadlineMs = new EnumMap<>(LauncherLane.class);
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
        return flywheelModeConfig.mode;
    }

     public String getPhaseName(LauncherLane lane) {
        Flywheel flywheel = flywheels.get(lane);
        return flywheel == null ? "UNKNOWN" : flywheel.getPhaseName();
    }

    private boolean debugOverrideEnabled = false;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        for (LauncherLane lane : LauncherLane.values()) {
            flywheels.put(lane, new Flywheel(lane, hardwareMap));
            feeders.put(lane, new Feeder(lane, hardwareMap));
            hoods.put(lane, new Hood(lane, hardwareMap));
            laneRecoveryDeadlineMs.put(lane, 0.0);
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
                    laneRecoveryDeadlineMs.put(next.lane, now + timing.recoveryMs);
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

    protected double launchRpmFor(LauncherLane lane) {
        Double override = launchRpmOverrides.get(lane);
        if (override != null && override > 0.0) {
            return override;
        }
        switch (lane) {
            case LEFT:
                return Math.max(0.0, leftFlywheelConfig.launchRpm);
            case CENTER:
                return Math.max(0.0, centerFlywheelConfig.launchRpm);
            case RIGHT:
            default:
                return Math.max(0.0, rightFlywheelConfig.launchRpm);
        }
    }

    protected double idleRpmFor(LauncherLane lane) {
        Double override = idleRpmOverrides.get(lane);
        if (override != null && override >= 0.0) {
            return override;
        }
        switch (lane) {
            case LEFT:
                return Math.max(0.0, leftFlywheelConfig.idleRpm);
            case CENTER:
                return Math.max(0.0, centerFlywheelConfig.idleRpm);
            case RIGHT:
            default:
                return Math.max(0.0, rightFlywheelConfig.idleRpm);
        }
    }

    private static double feederHoldMsFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return Math.max(0.0, leftFeederConfig.holdMs);
            case CENTER:
                return Math.max(0.0, centerFeederConfig.holdMs);
            case RIGHT:
            default:
                return Math.max(0.0, rightFeederConfig.holdMs);
        }
    }

    private static double feederLoadPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(leftFeederConfig.loadPosition);
            case CENTER:
                return clampServo(centerFeederConfig.loadPosition);
            case RIGHT:
            default:
                return clampServo(rightFeederConfig.loadPosition);
        }
    }

    private static double feederFirePositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(leftFeederConfig.firePosition);
            case CENTER:
                return clampServo(centerFeederConfig.firePosition);
            case RIGHT:
            default:
                return clampServo(rightFeederConfig.firePosition);
        }
    }

    private static boolean feederReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return leftFeederConfig.reversed;
            case CENTER:
                return centerFeederConfig.reversed;
            case RIGHT:
            default:
                return rightFeederConfig.reversed;
        }
    }

    private static String motorNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return leftFlywheelConfig.motorName;
            case CENTER:
                return centerFlywheelConfig.motorName;
            case RIGHT:
            default:
                return rightFlywheelConfig.motorName;
        }
    }

    private static boolean motorReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return leftFlywheelConfig.reversed;
            case CENTER:
                return centerFlywheelConfig.reversed;
            case RIGHT:
            default:
                return rightFlywheelConfig.reversed;
        }
    }

    private static String feederNameFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return leftFeederConfig.servoName;
            case CENTER:
                return centerFeederConfig.servoName;
            case RIGHT:
            default:
                return rightFeederConfig.servoName;
        }
    }

    private static double rpmToTicksPerSecond(double rpm) {
        if (rpm <= 0.0) {
            return 0.0;
        }
        return rpm * flywheelParameters.ticksPerRev * flywheelParameters.gearRatio / 60.0;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / (flywheelParameters.ticksPerRev * flywheelParameters.gearRatio);
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

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    public LauncherState getLogState() {
        return state;
    }

    public SpinMode getLogRequestedSpinMode() {
        return requestedSpinMode;
    }

    public SpinMode getLogEffectiveSpinMode() {
        return computeEffectiveSpinMode();
    }

    public String getLogControlMode() {
        return getFlywheelControlMode().name();
    }

    public boolean getLogBusy() {
        return isBusy();
    }

    public int getLogQueuedShots() {
        return getQueuedShots();
    }

    public double getLogStateElapsedSec() {
        return getStateElapsedSeconds();
    }

    public String getLogActiveShotLane() {
        ShotRequest pendingShot = shotQueue.peekFirst();
        return pendingShot == null ? "NONE" : pendingShot.lane.name();
    }

    public double getLogActiveShotAgeMs() {
        ShotRequest pendingShot = shotQueue.peekFirst();
        if (pendingShot == null) {
            return 0.0;
        }
        double now = clock.milliseconds();
        return Math.max(0.0, now - pendingShot.scheduledTimeMs);
    }

    public double getLogLastShotCompletionMs() {
        double lastCompletionMs = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            double deadline = laneRecoveryDeadlineMs.getOrDefault(lane, 0.0);
            if (deadline > 0.0) {
                lastCompletionMs = Math.max(lastCompletionMs, deadline - timing.recoveryMs);
            }
        }
        return lastCompletionMs;
    }

    public double getLogAverageTargetRpm() {
        return getTargetRpm();
    }

    public double getLogAverageCurrentRpm() {
        return getCurrentRpm();
    }

    public double getLogAveragePower() {
        return getLastPower();
    }

    public double getLogLeftTargetRpm() {
        return getTargetRpm(LauncherLane.LEFT);
    }

    public double getLogLeftCurrentRpm() {
        return getCurrentRpm(LauncherLane.LEFT);
    }

    public double getLogLeftPower() {
        return getLastPower(LauncherLane.LEFT);
    }

    public boolean getLogLeftReady() {
        return isLaneReady(LauncherLane.LEFT);
    }

    public double getLogLeftFeederPosition() {
        return getFeederPosition(LauncherLane.LEFT);
    }

    public double getLogLeftRecoveryRemainingMs() {
        double recoveryDeadline = laneRecoveryDeadlineMs.getOrDefault(LauncherLane.LEFT, 0.0);
        return Math.max(0.0, recoveryDeadline - clock.milliseconds());
    }

    public double getLogCenterTargetRpm() {
        return getTargetRpm(LauncherLane.CENTER);
    }

    public double getLogCenterCurrentRpm() {
        return getCurrentRpm(LauncherLane.CENTER);
    }

    public double getLogCenterPower() {
        return getLastPower(LauncherLane.CENTER);
    }

    public boolean getLogCenterReady() {
        return isLaneReady(LauncherLane.CENTER);
    }

    public double getLogCenterFeederPosition() {
        return getFeederPosition(LauncherLane.CENTER);
    }

    public double getLogCenterRecoveryRemainingMs() {
        double recoveryDeadline = laneRecoveryDeadlineMs.getOrDefault(LauncherLane.CENTER, 0.0);
        return Math.max(0.0, recoveryDeadline - clock.milliseconds());
    }

    public double getLogRightTargetRpm() {
        return getTargetRpm(LauncherLane.RIGHT);
    }

    public double getLogRightCurrentRpm() {
        return getCurrentRpm(LauncherLane.RIGHT);
    }

    public double getLogRightPower() {
        return getLastPower(LauncherLane.RIGHT);
    }

    public boolean getLogRightReady() {
        return isLaneReady(LauncherLane.RIGHT);
    }

    public double getLogRightFeederPosition() {
        return getFeederPosition(LauncherLane.RIGHT);
    }

    public double getLogRightRecoveryRemainingMs() {
        double recoveryDeadline = laneRecoveryDeadlineMs.getOrDefault(LauncherLane.RIGHT, 0.0);
        return Math.max(0.0, recoveryDeadline - clock.milliseconds());
    }

    public double getLogLeftHoodPosition() {
        return getHoodPosition(LauncherLane.LEFT);
    }

    public double getLogCenterHoodPosition() {
        return getHoodPosition(LauncherLane.CENTER);
    }

    public double getLogRightHoodPosition() {
        return getHoodPosition(LauncherLane.RIGHT);
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
            double error = Math.abs(getCurrentRpm() - launchRpm);
            if (error <= flywheelParameters.rpmTolerance) {
                return true;
            }
            if (!launchCommandActive) {
                return false;
            }
            double elapsed = launchTimer.milliseconds();
            return elapsed >= timing.fallbackReadyMs
                    || (elapsed >= timing.minimalSpinUpMs && commandedRpm >= launchRpm);
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

            switch (phase) {
                case BANG:
                    if (getFlywheelControlMode() == FlywheelControlMode.BANG_BANG_HOLD) {
                        if (absError <= bangBangConfig.exitBangThresholdRpm) {
                            bangToHoldCounter++;
                            if (bangToHoldCounter >= Math.max(1, phaseSwitchConfig.bangToHoldConfirmCycles)) {
                                phase = ControlPhase.HOLD;
                                bangToHoldCounter = 0;
                            }
                        } else {
                            bangToHoldCounter = 0;
                        }
                        bangToHybridCounter = 0;
                    } else {
                        if (absError <= bangBangConfig.exitBangThresholdRpm) {
                            bangToHybridCounter++;
                            if (bangToHybridCounter >= Math.max(1, phaseSwitchConfig.bangToHybridConfirmCycles)) {
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
                    if (absError >= bangBangConfig.enterBangThresholdRpm) {
                        hybridToBangCounter++;
                        if (hybridToBangCounter >= Math.max(1, phaseSwitchConfig.hybridToBangConfirmCycles)) {
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
                    if (absError >= bangBangConfig.enterBangThresholdRpm) {
                        holdToBangCounter++;
                        if (holdToBangCounter >= Math.max(1, phaseSwitchConfig.holdToBangConfirmCycles)) {
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
            double threshold = Math.max(0.0, bangBangConfig.bangDeadbandRpm);
            double high = Range.clip(bangBangConfig.highPower, -1.0, 1.0);
            double low = Range.clip(bangBangConfig.lowPower, -1.0, 1.0);
            if (error > threshold) {
                motor.setPower(high);
            } else if (error < -threshold) {
                motor.setPower(low);
            } else {
                motor.setPower(low);
            }
        }

        private void applyHybridControl(double error) {
            double power = hybridPidConfig.kF + hybridPidConfig.kP * error;
            power = Range.clip(power, 0.0, Math.max(0.0, hybridPidConfig.maxPower));
            motor.setPower(power);
        }

        private void applyHoldControl(double error) {
            double holdPower = holdConfig.baseHoldPower + holdConfig.rpmPowerGain * commandedRpm;
            holdPower = Range.clip(holdPower, holdConfig.minHoldPower, holdConfig.maxHoldPower);
            if (error < 0.0) {
                motor.setPower(bangBangConfig.lowPower);
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
            if (this.servo != null) {
                if (hoodReversedFor(lane)) {
                    servo.setDirection(Servo.Direction.REVERSE);
                } else {
                    servo.setDirection(Servo.Direction.FORWARD);
                }
            }
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
                return leftHoodConfig.servoName;
            case CENTER:
                return centerHoodConfig.servoName;
            case RIGHT:
            default:
                return rightHoodConfig.servoName;
        }
    }

    private static boolean hoodReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return leftHoodConfig.reversed;
            case CENTER:
                return centerHoodConfig.reversed;
            case RIGHT:
            default:
                return rightHoodConfig.reversed;
        }
    }

    private static double hoodShortPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(leftHoodConfig.shortPosition);
            case CENTER:
                return clampServo(centerHoodConfig.shortPosition);
            case RIGHT:
            default:
                return clampServo(rightHoodConfig.shortPosition);
        }
    }

    private static double hoodMidPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(leftHoodConfig.midPosition);
            case CENTER:
                return clampServo(centerHoodConfig.midPosition);
            case RIGHT:
            default:
                return clampServo(rightHoodConfig.midPosition);
        }
    }

    private static double hoodLongPositionFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return clampServo(leftHoodConfig.longPosition);
            case CENTER:
                return clampServo(centerHoodConfig.longPosition);
            case RIGHT:
            default:
                return clampServo(rightHoodConfig.longPosition);
        }
    }

}
