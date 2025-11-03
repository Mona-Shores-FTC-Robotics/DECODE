package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Competition-oriented control scheme that keeps the intake running in reverse
 * by default and delegates launcher sequencing to the coordinator.
 */
public class MatchOperatorControls implements OperatorControls {

    private static final double TRIGGER_DEADBAND = 0.15;

    private static final double MATCH_BURST_SPACING_MS = 150.0;

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final Button spinUpToggle;
    private final Button shootLeft;
    private final Button shootMiddle;
    private final Button shootRight;
    private final Button shootAll;
    private boolean manualIntakeActive = false;
    private boolean fullSpinRequested = false;

    public MatchOperatorControls(GamepadEx operator, Robot robot, LauncherCoordinator launcherCoordinator) {
        this.robot = robot;
        this.launcherCoordinator = launcherCoordinator;

        robot.shooter.setDebugOverrideEnabled(false);
        robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.HOLD);

        spinUpToggle = operator.leftBumper();
        shootLeft = operator.x();
        shootMiddle = operator.y();
        shootRight = operator.b();
        shootAll = operator.a();

        spinUpToggle.whenBecomesTrue(this::toggleSpinMode);
        shootLeft.whenBecomesTrue(() -> fireLane(LauncherLane.LEFT));
        shootMiddle.whenBecomesTrue(() -> fireLane(LauncherLane.CENTER));
        shootRight.whenBecomesTrue(() -> fireLane(LauncherLane.RIGHT));
        shootAll.whenBecomesTrue(this::fireBurst);
    }

    @Override
    public void update(double reverseTrigger, double forwardTrigger) {
        double power = IntakeSubsystem.MotorConfig.defaultReversePower;
        boolean forwardActive = forwardTrigger > TRIGGER_DEADBAND;
        if (forwardActive) {
            power = IntakeSubsystem.MotorConfig.defaultForwardPower;
        }

        robot.intake.setManualPower(power);
        if (manualIntakeActive != forwardActive) {
            manualIntakeActive = forwardActive;
            if (manualIntakeActive) {
                robot.lighting.indicateBusy();
            } else {
                robot.lighting.indicateIdle();
            }
        }
    }

    private void toggleSpinMode() {
        fullSpinRequested = !fullSpinRequested;
        if (fullSpinRequested) {
            robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.FULL);
        } else {
            robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.HOLD);
        }
    }

    private void fireLane(LauncherLane lane) {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestKick(lane);
        } else {
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
    }

    private void fireBurst() {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(MATCH_BURST_SPACING_MS);
        } else {
            robot.shooter.shootAll();
        }
    }

    @Override
    public void reset() {
        robot.intake.stop();
        manualIntakeActive = false;
        robot.lighting.indicateIdle();
        fullSpinRequested = false;
        robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.HOLD);
    }

    @Override
    public boolean isShooterDebugMode() {
        return false;
    }

    @Override
    public LaneDebugState getLaneDebugState(LauncherLane lane) {
        return null;
    }
}
