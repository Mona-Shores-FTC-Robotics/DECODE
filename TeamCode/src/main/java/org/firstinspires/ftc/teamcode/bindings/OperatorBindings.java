package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

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
    private final Button launchCenter;
    private final Button launchRight;
    private final Button launchAll;
    private final Button intakeForwardHold;
    private boolean manualIntakeActive = false;

    private final LauncherCommands launcherCommands;
    private final IntakeCommands intakeCommands;


    public OperatorBindings(GamepadEx operator,
                            Robot robot) {
        this.robot = robot;
        this.launcherCoordinator = robot.launcherCoordinator;
        this.launcherCommands = robot.launcherCommands;
        this.intakeCommands = robot.intakeCommands;

        spinModeToggle = operator.leftBumper();
        launchLeft = operator.x();
        launchCenter = operator.y();
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
        robot.launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
        if (launcherCoordinator != null) {
            launcherCoordinator.clearIntakeOverride();
        }
        syncLightingWithIntake();

        spinModeToggle.whenBecomesTrue(launcherCommands::toggleSpinMode);
        launchLeft.whenBecomesTrue(launcherCommands::launchLeft);
        launchCenter.whenBecomesTrue(launcherCommands::launchCenter);
        launchRight.whenBecomesTrue(launcherCommands::launchRight);
        launchAll.whenBecomesTrue(launcherCommands::launchAll);

        intakeForwardHold.whenBecomesTrue(robot.intake::startForward);
        intakeForwardHold.whenBecomesFalse(robot.intake::startReverse);
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

}
