package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Match-oriented operator bindings. Keeps the intake running in reverse by default,
 * forwards when the operator pulls the trigger, and maps launcher commands to the
 * standard match buttons.
 */
public class OperatorBindings {

    private static final double TRIGGER_DEADBAND = 0.15;
    private static final double MATCH_BURST_SPACING_MS = 150.0;

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final Button spinModeToggle;
    private final Button shootLeft;
    private final Button shootMiddle;
    private final Button shootRight;
    private final Button shootAll;
    private boolean manualIntakeActive = false;
    private boolean fullSpinRequested = false;

    public OperatorBindings(GamepadEx operator,
                            Robot robot,
                            LauncherCoordinator launcherCoordinator) {
        this.robot = robot;
        this.launcherCoordinator = launcherCoordinator;

        spinModeToggle = operator.leftBumper();
        shootLeft = operator.x();
        shootMiddle = operator.y();
        shootRight = operator.b();
        shootAll = operator.a();

        configureMatchBindings();
    }

    private void configureMatchBindings() {
        if (launcherCoordinator != null) {
            launcherCoordinator.enableAutoSpin(false);
        }
        robot.shooter.setDebugOverrideEnabled(false);
        robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.HOLD);

        spinModeToggle.whenBecomesTrue(this::toggleSpinMode);
        shootLeft.whenBecomesTrue(() -> fireLane(LauncherLane.LEFT));
        shootMiddle.whenBecomesTrue(() -> fireLane(LauncherLane.CENTER));
        shootRight.whenBecomesTrue(() -> fireLane(LauncherLane.RIGHT));
        shootAll.whenBecomesTrue(this::fireMatchBurst);
    }

    public void update(double reverseTrigger, double forwardTrigger) {
        double power = IntakeSubsystem.MotorConfig.defaultReversePower;
        boolean forwardActive = forwardTrigger > TRIGGER_DEADBAND;
        if (forwardActive) {
            power = IntakeSubsystem.MotorConfig.defaultForwardPower;
        }

        robot.intake.setManualPower(power);
        updateLighting(forwardActive);
    }

    private void updateLighting(boolean active) {
        if (active != manualIntakeActive) {
            manualIntakeActive = active;
            if (manualIntakeActive) {
                robot.lighting.indicateBusy();
            } else {
                robot.lighting.indicateIdle();
            }
        }
    }

    private void toggleSpinMode() {
        fullSpinRequested = !fullSpinRequested;
        robot.shooter.setSpinMode(fullSpinRequested
                ? ShooterSubsystem.SpinMode.FULL
                : ShooterSubsystem.SpinMode.HOLD);
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

    private void fireMatchBurst() {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(MATCH_BURST_SPACING_MS);
        } else {
            robot.shooter.shootAll();
        }
    }

    public void reset() {
        robot.intake.stop();
        manualIntakeActive = false;
        robot.lighting.indicateIdle();
        fullSpinRequested = false;
        robot.shooter.setSpinMode(ShooterSubsystem.SpinMode.HOLD);
        if (launcherCoordinator != null) {
            launcherCoordinator.enableAutoSpin(false);
        }
    }
}
