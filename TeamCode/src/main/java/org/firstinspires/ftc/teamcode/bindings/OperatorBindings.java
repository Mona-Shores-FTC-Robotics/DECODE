package org.firstinspires.ftc.teamcode.bindings;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import java.util.EnumMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Operator bindings for secondary gamepad. Handles intake toggles and shooter requests.
 */
public class OperatorBindings {

    private static final double TRIGGER_DEADBAND = 0.15;

    @Configurable
    public static class OperatorConfig {
        public static double burstSpacingMs = 200.0;
        public static boolean enableShooterTuningMode = true;
        public static double rpmIncrement = 50.0;
    }

    public static final class LaneDebugState {
        public boolean enabled;
        public double targetRpm;

        LaneDebugState(double targetRpm) {
            this.targetRpm = Math.max(0.0, targetRpm);
            this.enabled = false;
        }
    }

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final Button spinUpToggle;
    private final Button shootLeft;
    private final Button shootMiddle;
    private final Button shootRight;
    private final Button shootAll;
    private final Button increaseRpmButton;
    private final boolean shooterDebugMode;
    private final EnumMap<LauncherLane, LaneDebugState> laneDebugStates;
    private LauncherLane lastAdjustedLane = LauncherLane.CENTER;
    private boolean manualIntakeActive = false;

    public OperatorBindings(GamepadEx operator, Robot robot) {
        this(operator, robot, null);
    }

    public OperatorBindings(GamepadEx operator, Robot robot, LauncherCoordinator launcherCoordinator) {
        this.robot = robot;
        this.launcherCoordinator = launcherCoordinator;

        spinUpToggle = operator.leftBumper();
        shootLeft = operator.x();
        shootMiddle = operator.y();
        shootRight = operator.b();
        shootAll = operator.a();
        increaseRpmButton = operator.rightBumper();
        laneDebugStates = new EnumMap<>(LauncherLane.class);
        shooterDebugMode = OperatorConfig.enableShooterTuningMode;

        if (shooterDebugMode) {
            configureShooterDebugMode();
        } else {
            configureStandardMode();
        }
    }

    private void configureStandardMode() {
        robot.shooter.setDebugOverrideEnabled(false);
        spinUpToggle.whenBecomesTrue(robot.shooter::toggleSpinUp);
        shootLeft.whenBecomesTrue(() -> fireLane(LauncherLane.LEFT));
        shootMiddle.whenBecomesTrue(() -> fireLane(LauncherLane.CENTER));
        shootRight.whenBecomesTrue(() -> fireLane(LauncherLane.RIGHT));
        shootAll.whenBecomesTrue(this::fireBurst);
    }

    private void configureShooterDebugMode() {
        if (launcherCoordinator != null) {
            launcherCoordinator.enableAutoSpin(false);
        }
        robot.shooter.setDebugOverrideEnabled(true);
        robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.OFF);
        laneDebugStates.clear();
        for (LauncherLane lane : LauncherLane.values()) {
            LaneDebugState state = new LaneDebugState(robot.shooter.getLaunchRpm(lane));
            laneDebugStates.put(lane, state);
            robot.shooter.debugSetLaneTargetRpm(lane, 0.0);
        }
        lastAdjustedLane = LauncherLane.CENTER;

        shootLeft.whenBecomesTrue(() -> toggleDebugLane(LauncherLane.LEFT));
        shootAll.whenBecomesTrue(() -> toggleDebugLane(LauncherLane.CENTER));
        shootRight.whenBecomesTrue(() -> toggleDebugLane(LauncherLane.RIGHT));
        increaseRpmButton.whenBecomesTrue(() -> adjustDebugRpm(OperatorConfig.rpmIncrement));
        spinUpToggle.whenBecomesTrue(() -> adjustDebugRpm(-OperatorConfig.rpmIncrement));
    }

    private void fireLane(LauncherLane lane) {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestKick(lane);
            return;
        }
        switch (lane) {
            case LEFT:
                robot.shooter.shootLeft();
                break;
            case CENTER:
                robot.shooter.shootMiddle();
                break;
            case RIGHT:
            default:
                robot.shooter.shootRight();
                break;
        }
    }

    private void fireBurst() {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(OperatorConfig.burstSpacingMs);
        } else {
            robot.shooter.shootAll();
        }
    }

    public void update(double reverseTrigger, double forwardTrigger) {
        double power = 0.0;
        boolean reverseActive = reverseTrigger > TRIGGER_DEADBAND;
        boolean forwardActive = forwardTrigger > TRIGGER_DEADBAND;

        if (reverseActive && (!forwardActive || reverseTrigger >= forwardTrigger)) {
            power = IntakeSubsystem.MotorConfig.defaultReversePower;
        } else if (forwardActive) {
            power = IntakeSubsystem.MotorConfig.defaultForwardPower;
        }

        robot.intake.setManualPower(power);

        boolean activeNow = Math.abs(power) > TRIGGER_DEADBAND;
        if (activeNow != manualIntakeActive) {
            manualIntakeActive = activeNow;
            if (manualIntakeActive) {
                robot.lighting.indicateBusy();
            } else {
                robot.lighting.indicateIdle();
            }
        }
    }

    private void toggleDebugLane(LauncherLane lane) {
        LaneDebugState state = laneDebugStates.get(lane);
        if (state == null) {
            state = new LaneDebugState(robot.shooter.getLaunchRpm(lane));
            laneDebugStates.put(lane, state);
        }
        state.enabled = !state.enabled;
        lastAdjustedLane = lane;
        double rpm = state.enabled ? state.targetRpm : 0.0;
        robot.shooter.debugSetLaneTargetRpm(lane, rpm);
    }

    private void adjustDebugRpm(double delta) {
        LaneDebugState state = laneDebugStates.get(lastAdjustedLane);
        if (state == null) {
            state = new LaneDebugState(robot.shooter.getLaunchRpm(lastAdjustedLane));
            laneDebugStates.put(lastAdjustedLane, state);
        }
        state.targetRpm = Math.max(0.0, state.targetRpm + delta);
        if (state.enabled) {
            robot.shooter.debugSetLaneTargetRpm(lastAdjustedLane, state.targetRpm);
        }
    }

    public boolean isShooterDebugMode() {
        return shooterDebugMode;
    }

    public LaneDebugState getLaneDebugState(LauncherLane lane) {
        return laneDebugStates.get(lane);
    }

    public void reset() {
        robot.intake.stop();
        manualIntakeActive = false;
        robot.lighting.indicateIdle();
        if (shooterDebugMode) {
            for (LauncherLane lane : LauncherLane.values()) {
                LaneDebugState state = laneDebugStates.get(lane);
                if (state != null) {
                    state.enabled = false;
                    robot.shooter.debugSetLaneTargetRpm(lane, 0.0);
                }
            }
            robot.shooter.setDebugOverrideEnabled(false);
        }
    }
}
