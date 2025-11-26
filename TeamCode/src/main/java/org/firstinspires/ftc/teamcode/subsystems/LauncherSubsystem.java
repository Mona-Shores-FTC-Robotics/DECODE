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

import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFeederConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFlywheelConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFlywheelConfig.FlywheelControlMode;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherHoodConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherReverseIntakeConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherTimingConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherVoltageCompensationConfig;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotConfigs;
import org.firstinspires.ftc.teamcode.util.RobotState;

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

    // Global configuration instances
    public static LauncherVoltageCompensationConfig voltageCompensationConfig = new LauncherVoltageCompensationConfig();
    public static LauncherReverseIntakeConfig reverseFlywheelForHumanLoadingConfig = new LauncherReverseIntakeConfig();

    /**
     * Gets the robot-specific Timing based on RobotState.getRobotName().
     * @return timing19429 or timing20245
     */
    public static LauncherTimingConfig timing() {
        return RobotConfigs.getTiming();
    }

    /**
     * Gets the robot-specific FlywheelConfig based on RobotState.getRobotName().
     * @return flywheelConfig19429 or flywheelConfig20245
     */
    public static LauncherFlywheelConfig flywheelConfig() {
        return RobotConfigs.getFlywheelConfig();
    }

    /**
     * Gets the robot-specific FeederConfig based on RobotState.getRobotName().
     * @return feederConfig19429 or feederConfig20245
     */
    public static LauncherFeederConfig feederConfig() {
        return RobotConfigs.getFeederConfig();
    }

    /**
     * Gets the robot-specific HoodConfig based on RobotState.getRobotName().
     * @return hoodConfig19429 or hoodConfig20245
     */
    public static LauncherHoodConfig hoodConfig() {
        return RobotConfigs.getHoodConfig();
    }

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
        return flywheelConfig().modeConfig.mode;
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
            if (flywheel.motor != null) {
                // Log velocity measurements
                double velocityTps = flywheel.motor.getVelocity();  // SDK velocity in ticks/sec
                double velocityRpm = ticksPerSecondToRpm(Math.abs(velocityTps));  // Converted to RPM
                RobotState.packet.put("velocity_tps_" + flywheel.lane.name(), velocityTps);
                RobotState.packet.put("velocity_rpm_" + flywheel.lane.name(), velocityRpm);

                // Log control diagnostics for feedforward tuning
                if (getFlywheelControlMode() == FlywheelControlMode.FEEDFORWARD) {
                    double targetRpm = flywheel.getTargetRpm();
                    double error = targetRpm - velocityRpm;
                    double kS = kSFor(flywheel.lane);
                    double kV = kVFor(flywheel.lane);
                    double kP = kPFor(flywheel.lane);
                    double feedforward = kS + kV * targetRpm;
                    double feedback = kP * error;
                    double totalPower = flywheel.getAppliedPower();

                    RobotState.packet.put("ff_error_" + flywheel.lane.name(), error);
                    RobotState.packet.put("ff_feedforward_" + flywheel.lane.name(), feedforward);
                    RobotState.packet.put("ff_feedback_" + flywheel.lane.name(), feedback);
                    RobotState.packet.put("ff_power_" + flywheel.lane.name(), totalPower);
                }
            }
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

    /**
     * Checks if ALL lanes are at their launch target.
     *
     * NOTE: This is for telemetry/status display only. Shot execution does NOT
     * require all lanes to be ready - each lane fires independently when ready.
     * Use isLaneReady(lane) for actual shot decisions.
     *
     * @return true if all three lanes are at launch speed, false otherwise
     */
    public boolean allLanesReady() {
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
            delayMs += timing().burstSpacingMs;
        }
    }

    public void clearQueue() {
        shotQueue.clear();
    }

    /**
     * Clears recovery deadlines for all lanes, allowing them to fire immediately.
     * Use this when starting a new spin-up sequence to ensure lanes can respond to new RPM targets.
     */
    public void clearRecoveryDeadlines() {
        laneRecoveryDeadlineMs.clear();
        laneLaunchHoldDeadlineMs.clear();
    }

    public void abort() {
        clearQueue();
        reverseFlywheelActive = false;
        double now = clock.milliseconds();
        for (LauncherLane lane : LauncherLane.values()) {
            laneLaunchHoldDeadlineMs.put(lane, 0.0);
            laneRecoveryDeadlineMs.put(lane, now + timing().recoveryMs);
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

    /**
     * Checks if a specific lane is ready to fire.
     *
     * IMPORTANT: Lanes are INDEPENDENT. Each lane can fire as soon as it's ready,
     * without waiting for other lanes. Use this method for shot decisions, not
     * allLanesReady() which is only for telemetry/status display.
     *
     * @param lane The lane to check
     * @return true if this lane is at launch speed and can fire
     */
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
        // Per-lane independent firing: Each lane fires as soon as it's ready,
        // without waiting for other lanes. If LEFT is ready but CENTER isn't,
        // LEFT fires immediately. This maximizes throughput with per-lane tuning.
        if (!shotQueue.isEmpty()) {
            Iterator<ShotRequest> iterator = shotQueue.iterator();
            while (iterator.hasNext()) {
                ShotRequest next = iterator.next();

                if (now < next.scheduledTimeMs) {
                    continue;  // Scheduled for later
                }
                if (!isLaneReadyForShot(next.lane, now)) {
                    continue;  // This lane not ready - check next shot in queue
                }

                Feeder feeder = feeders.get(next.lane);
                if (feeder != null && !feeder.isBusy()) {
                    feeder.fire();
                    laneLaunchHoldDeadlineMs.put(next.lane, now + timing().launchHoldAfterFireMs);
                    laneRecoveryDeadlineMs.put(next.lane, now + timing().recoveryMs);
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


    /**
     * Updates the global launcher state to reflect the current spin mode request.
     *
     * NOTE: State reflects the overall intent, not per-lane achievement. Individual
     * lanes fire independently when ready - they don't wait for global state.
     * Use isLaneReady(lane) to check if a specific lane can fire.
     */
    private void reflectSpinState(SpinMode effectiveSpinMode) {
        switch (effectiveSpinMode) {
            case FULL:
                // When full speed requested, state is SPINNING_UP
                // Lanes will independently reach readiness and fire when ready
                setState(LauncherState.SPINNING_UP);
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
                return Math.max(0.0, flywheelConfig().flywheelLeft.idleRpm);
            case CENTER:
                return Math.max(0.0, flywheelConfig().flywheelCenter.idleRpm);
            case RIGHT:
            default:
                return Math.max(0.0, flywheelConfig().flywheelRight.idleRpm);
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
                return flywheelConfig().flywheelLeft.motorName;
            case CENTER:
                return flywheelConfig().flywheelCenter.motorName;
            case RIGHT:
            default:
                return flywheelConfig().flywheelRight.motorName;
        }
    }

    private static boolean motorReversedFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return flywheelConfig().flywheelLeft.reversed;
            case CENTER:
                return flywheelConfig().flywheelCenter.reversed;
            case RIGHT:
            default:
                return flywheelConfig().flywheelRight.reversed;
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
        return rpm * flywheelConfig().parameters.ticksPerRev * flywheelConfig().parameters.gearRatio / 60.0;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / (flywheelConfig().parameters.ticksPerRev * flywheelConfig().parameters.gearRatio);
    }

    /**
     * Gets the per-lane static friction coefficient (kS).
     * This is the minimum power needed to overcome friction and start the motor spinning.
     */
    private static double kSFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return flywheelConfig().flywheelLeft.kS;
            case CENTER:
                return flywheelConfig().flywheelCenter.kS;
            case RIGHT:
            default:
                return flywheelConfig().flywheelRight.kS;
        }
    }

    /**
     * Gets the per-lane velocity gain (kV).
     * This maps target RPM to required power: power = kS + kV * rpm
     */
    private static double kVFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return flywheelConfig().flywheelLeft.kV;
            case CENTER:
                return flywheelConfig().flywheelCenter.kV;
            case RIGHT:
            default:
                return flywheelConfig().flywheelRight.kV;
        }
    }

    /**
     * Gets the per-lane proportional gain (kP).
     * This adds feedback correction: power += kP * (target - actual)
     * Set to 0 for pure feedforward, >0 to add error correction.
     */
    private static double kPFor(LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return flywheelConfig().flywheelLeft.kP;
            case CENTER:
                return flywheelConfig().flywheelCenter.kP;
            case RIGHT:
            default:
                return flywheelConfig().flywheelRight.kP;
        }
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
            }
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
            if (motor == null) {
                return 0.0;
            }
            // Use SDK's built-in velocity (already in ticks/sec)
            double ticksPerSec = Math.abs(motor.getVelocity());
            return ticksPerSecondToRpm(ticksPerSec);
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
            if (error <= flywheelConfig().parameters.rpmTolerance) {
                return true;
            }

            if (!launchCommandActive) {
                return false;
            }

            double elapsed = launchTimer.milliseconds();

            // Fallback for spinning UP: allow if minimal time elapsed and we've commanded the right RPM
            // This handles encoder issues but still requires the command to be sent
            if (elapsed >= timing().fallbackReadyMs) {
                return true;  // Timeout - assume ready
            }

            // For early ready (before fallback timeout), check both:
            // 1. Minimal spin-up time elapsed
            // 2. Current RPM is appropriate for target (works for both spin-up and spin-down)
            if (elapsed >= timing().minimalSpinUpMs) {
                // Check if we're close enough OR if spinning down, at least below a reasonable threshold
                // Allow some headroom when spinning down (within 10% above target is acceptable)
                double upperThreshold = launchRpm * 1.1;  // 10% overspeed tolerance
                return currentRpm <= upperThreshold;
            }

            return false;
        }

        void updateControl() {
            if (motor == null) {
                return;
            }

            // Skip normal control if reverse intake is active
            if (reverseFlywheelActive) {
                return;
            }

            // Stop motor if no velocity commanded
            if (commandedRpm <= 0.0) {
                motor.setPower(0.0);
                phase = ControlPhase.BANG;
                return;
            }

            double error = commandedRpm - getCurrentRpm();
            double absError = Math.abs(error);

            // ==================== RECOMMENDED: FEEDFORWARD MODE ====================
            // Direct velocity-to-power mapping with optional proportional feedback.
            // - Simple, predictable, easy to tune per-lane
            // - Feedforward (kS + kV*rpm) predicts required power
            // - Proportional feedback (kP*error) corrects for disturbances
            // - Set kP=0 for pure feedforward, kP>0 to add error correction
            if (getFlywheelControlMode() == FlywheelControlMode.FEEDFORWARD) {
                applyFeedforwardControl(commandedRpm);
                return;
            }

            // ==================== ALTERNATIVE: PURE BANG-BANG ====================
            // Simple on/off control - fast spin-up but oscillates at target.
            // Good for quick testing or as a fallback mode.
            if (getFlywheelControlMode() == FlywheelControlMode.PURE_BANG_BANG) {
                phase = ControlPhase.BANG;
                applyBangBangControl(error);
                return;
            }

            // Legacy multi-phase control for HYBRID and BANG_BANG_HOLD modes
            switch (phase) {
                case BANG:
                    if (getFlywheelControlMode() == FlywheelControlMode.BANG_BANG_HOLD) {
                        if (absError <= flywheelConfig().modeConfig.bangBang.exitBangThresholdRpm) {
                            bangToHoldCounter++;
                            if (bangToHoldCounter >= Math.max(1, flywheelConfig().modeConfig.phaseSwitch.bangToHoldConfirmCycles)) {
                                phase = ControlPhase.HOLD;
                                bangToHoldCounter = 0;
                            }
                        } else {
                            bangToHoldCounter = 0;
                        }
                        bangToHybridCounter = 0;
                    } else {
                        if (absError <= flywheelConfig().modeConfig.bangBang.exitBangThresholdRpm) {
                            bangToHybridCounter++;
                            if (bangToHybridCounter >= Math.max(1, flywheelConfig().modeConfig.phaseSwitch.bangToHybridConfirmCycles)) {
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
                    if (absError >= flywheelConfig().modeConfig.bangBang.enterBangThresholdRpm) {
                        hybridToBangCounter++;
                        if (hybridToBangCounter >= Math.max(1, flywheelConfig().modeConfig.phaseSwitch.hybridToBangConfirmCycles)) {
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
                    if (absError >= flywheelConfig().modeConfig.bangBang.enterBangThresholdRpm) {
                        holdToBangCounter++;
                        if (holdToBangCounter >= Math.max(1, flywheelConfig().modeConfig.phaseSwitch.holdToBangConfirmCycles)) {
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
            double high = Range.clip(flywheelConfig().modeConfig.bangBang.highPower, -1.0, 1.0);
            double low = Range.clip(flywheelConfig().modeConfig.bangBang.lowPower, -1.0, 1.0);

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
            double power = flywheelConfig().modeConfig.hybridPid.kF + flywheelConfig().modeConfig.hybridPid.kP * error;
            power = Range.clip(power, 0.0, Math.max(0.0, flywheelConfig().modeConfig.hybridPid.maxPower));

            // Apply voltage compensation
            double voltageMultiplier = getVoltageCompensationMultiplier();
            power = Range.clip(power * voltageMultiplier, 0.0, 1.0);

            motor.setPower(power);
        }

        private void applyHoldControl(double error) {
            double holdPower = flywheelConfig().modeConfig.hold.baseHoldPower + flywheelConfig().modeConfig.hold.rpmPowerGain * commandedRpm;
            holdPower = Range.clip(holdPower, flywheelConfig().modeConfig.hold.minHoldPower, flywheelConfig().modeConfig.hold.maxHoldPower);

            // Apply voltage compensation
            double voltageMultiplier = getVoltageCompensationMultiplier();
            holdPower = Range.clip(holdPower * voltageMultiplier, 0.0, 1.0);
            double lowPower = Range.clip(flywheelConfig().modeConfig.bangBang.lowPower * voltageMultiplier, 0.0, 1.0);

            if (error < 0.0) {
                motor.setPower(lowPower);
            } else {
                motor.setPower(holdPower);
            }
        }

        private void applyFeedforwardControl(double targetRpm) {
            // Feedforward + Proportional control model:
            //   Feedforward: Predicts power needed based on target velocity
            //   Feedback: Corrects for errors based on actual velocity
            //
            // Formula: power = (kS + kV * targetRpm) + kP * error
            //   kS: Static friction - minimum power to start spinning
            //   kV: Velocity gain - additional power per RPM
            //   kP: Proportional gain - error correction (0 = pure feedforward)
            //
            // Tuning per lane allows each flywheel to be independently optimized.

            // Get per-lane gains
            double kS = kSFor(lane);
            double kV = kVFor(lane);
            double kP = kPFor(lane);

            // Feedforward term: predicts power based on target velocity
            double feedforward = kS + kV * targetRpm;

            // Feedback term: corrects for velocity error (optional, disabled if kP=0)
            double currentRpm = getCurrentRpm();
            double error = targetRpm - currentRpm;
            double feedback = kP * error;

            // Combined control output
            double power = feedforward + feedback;

            // Apply power limits (use global limits from config)
            power = Range.clip(power,
                flywheelConfig().modeConfig.feedforward.minPower,
                flywheelConfig().modeConfig.feedforward.maxPower);

            // Apply voltage compensation to maintain consistent performance across battery voltages
            double voltageMultiplier = getVoltageCompensationMultiplier();
            power = Range.clip(power * voltageMultiplier, 0.0, 1.0);

            motor.setPower(power);
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
