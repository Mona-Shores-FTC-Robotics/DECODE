package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Match-oriented operator bindings. Keeps the intake running in reverse by default,
 * forwards when the operator pulls the trigger, and maps launcher commands to the
 * standard match buttons.
 */
public class OperatorBindings {

    private static final double MATCH_BURST_SPACING_MS = 150.0;

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final Button spinModeToggle;
    private final Button launchLeft;
    private final Button launchMiddle;
    private final Button launchRight;
    private final Button launchAll;
    private final Button intakeForwardHold;
    private boolean manualIntakeActive = false;
    private boolean fullSpinRequested = false;

    public OperatorBindings(GamepadEx operator,
                            Robot robot) {
        this.robot = robot;
        this.launcherCoordinator = robot.launcherCoordinator;

        spinModeToggle = operator.leftBumper();
        launchLeft = operator.x();
        launchMiddle = operator.y();
        launchRight = operator.b();
        launchAll = operator.a();
        intakeForwardHold = operator.rightBumper();

        configureMatchBindings();
    }

    private void configureMatchBindings() {
        if (launcherCoordinator != null) {
            launcherCoordinator.enableAutoSpin(false);
        }
        robot.launcher.setDebugOverrideEnabled(false);
        robot.launcher.setSpinMode(LauncherSubsystem.SpinMode.HOLD);
        if (launcherCoordinator != null) {
            launcherCoordinator.clearIntakeOverride();
        }
        syncLightingWithIntake();

        spinModeToggle.whenBecomesTrue(this::toggleSpinMode);
        launchLeft.whenBecomesTrue(() -> fireLane(LauncherLane.LEFT));
        launchMiddle.whenBecomesTrue(() -> fireLane(LauncherLane.CENTER));
        launchRight.whenBecomesTrue(() -> fireLane(LauncherLane.RIGHT));
        launchAll.whenBecomesTrue(this::fireMatchBurst);
        intakeForwardHold.whenBecomesTrue(this::requestForwardIntake);
        intakeForwardHold.whenBecomesFalse(this::releaseForwardIntake);
    }

    private void requestForwardIntake() {
        if (launcherCoordinator != null) {
            launcherCoordinator.overrideIntakeMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        } else {
            robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        }
        syncLightingWithIntake();
    }

    private void releaseForwardIntake() {
        if (launcherCoordinator != null) {
            launcherCoordinator.clearIntakeOverride();
        } else {
            robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        }
        syncLightingWithIntake();
    }

    private void syncLightingWithIntake() {
        IntakeSubsystem.IntakeMode appliedMode;
        if (launcherCoordinator != null) {
            appliedMode = launcherCoordinator.getAppliedIntakeMode();
        } else {
            appliedMode = robot.intake.getResolvedMode();
        }
        boolean intakeForward = appliedMode == IntakeSubsystem.IntakeMode.ACTIVE_FORWARD;
        updateLighting(intakeForward);
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
        robot.launcher.setSpinMode(fullSpinRequested
                ? LauncherSubsystem.SpinMode.FULL
                : LauncherSubsystem.SpinMode.HOLD);
    }

    private void fireLane(LauncherLane lane) {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestKick(lane);
            return;
        }

        switch (lane) {
            case LEFT:
                robot.launcher.launchLeft();
                break;
            case CENTER:
                robot.launcher.launchMiddle();
                break;
            case RIGHT:
            default:
                robot.launcher.launchRight();
                break;
        }
    }

    private void fireMatchBurst() {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(MATCH_BURST_SPACING_MS);
        } else {
            robot.launcher.launchAll();
        }
    }

    public void reset() {
        robot.intake.stop();
        if (launcherCoordinator != null) {
            launcherCoordinator.clearIntakeOverride();
        }
        manualIntakeActive = false;
        robot.lighting.indicateIdle();
        fullSpinRequested = false;
        robot.launcher.setSpinMode(LauncherSubsystem.SpinMode.HOLD);
        if (launcherCoordinator != null) {
            launcherCoordinator.enableAutoSpin(false);
        }
    }
}
