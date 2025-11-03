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
 * Developer-focused binding set that exposes direct control over launcher RPMs
 * and manual intake power. Mirrors the previous OperatorBindings implementation
 * used for subsystem bring-up.
 */
public class DebugOperatorControls implements OperatorControls {

    private static final double TRIGGER_DEADBAND = 0.15;

    @Configurable
    public static class OperatorConfig {
        public static double burstSpacingMs = 200.0;
        public static double rpmIncrement = 50.0;
    }

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final Button shootLeft;
    private final Button shootMiddle;
    private final Button shootRight;
    private final Button shootAll;
    private final Button increaseRpmButton;
    private final Button decreaseRpmButton;
    private final EnumMap<LauncherLane, LaneDebugState> laneDebugStates = new EnumMap<>(LauncherLane.class);
    private LauncherLane lastAdjustedLane = LauncherLane.CENTER;
    private boolean manualIntakeActive = false;

    public DebugOperatorControls(GamepadEx operator, Robot robot, LauncherCoordinator launcherCoordinator) {
        this.robot = robot;
        this.launcherCoordinator = launcherCoordinator;

        LauncherCoordinator coordinator = this.launcherCoordinator;
        if (coordinator != null) {
            coordinator.enableAutoSpin(false);
        }

        robot.shooter.setDebugOverrideEnabled(true);
        robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.OFF);
        for (LauncherLane lane : LauncherLane.values()) {
            LaneDebugState state = new LaneDebugState(robot.shooter.getLaunchRpm(lane));
            laneDebugStates.put(lane, state);
            robot.shooter.debugSetLaneTargetRpm(lane, 0.0);
        }

        shootLeft = operator.x();
        shootMiddle = operator.y();
        shootRight = operator.b();
        shootAll = operator.a();
        increaseRpmButton = operator.rightBumper();
        decreaseRpmButton = operator.leftBumper();

        shootLeft.whenBecomesTrue(() -> toggleDebugLane(LauncherLane.LEFT));
        shootAll.whenBecomesTrue(() -> toggleDebugLane(LauncherLane.CENTER));
        shootRight.whenBecomesTrue(() -> toggleDebugLane(LauncherLane.RIGHT));
        increaseRpmButton.whenBecomesTrue(() -> adjustDebugRpm(OperatorConfig.rpmIncrement));
        decreaseRpmButton.whenBecomesTrue(() -> adjustDebugRpm(-OperatorConfig.rpmIncrement));
    }

    @Override
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

    @Override
    public void reset() {
        robot.intake.stop();
        manualIntakeActive = false;
        robot.lighting.indicateIdle();
        for (LauncherLane lane : LauncherLane.values()) {
            LaneDebugState state = laneDebugStates.get(lane);
            if (state != null) {
                state.enabled = false;
                robot.shooter.debugSetLaneTargetRpm(lane, 0.0);
            }
        }
        robot.shooter.setDebugOverrideEnabled(false);
    }

    @Override
    public boolean isShooterDebugMode() {
        return true;
    }

    @Override
    public LaneDebugState getLaneDebugState(LauncherLane lane) {
        return laneDebugStates.get(lane);
    }
}
