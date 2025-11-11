package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireLaneCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ManualSpinController;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinHoldCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinLaneCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Match-oriented operator bindings. Keeps the intake running in reverse by default,
 * forwards when the operator pulls the trigger, and maps launcher commands to the
 * standard match buttons.
 */
public class OperatorBindings {

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final ManualSpinController manualSpinController;
    private final SpinLaneCommand spinLeft;
    private final SpinLaneCommand spinCenter;
    private final SpinLaneCommand spinRight;
    private final FireLaneCommand fireLeft;
    private final FireLaneCommand fireCenter;
    private final FireLaneCommand fireRight;
    private final SpinHoldCommand launchAllSpinner;
    private final FireAllCommand fireAll;
    private final SpinHoldCommand manualSpinHold;
    private final SetIntakeModeCommand intakeForwardCommand;
    private final SetIntakeModeCommand intakeReverseCommand;
    private final Button spinModeFull;
    private final Button launchLeft;
    private final Button launchCenter;
    private final Button launchRight;
    private final Button launchAll;
    private final Button intakeForwardHold;
    private boolean manualIntakeActive = false;

    private final LauncherCommands launcherCommands;


    public OperatorBindings(GamepadEx operator,
                            Robot robot) {
        this.robot = robot;

        this.launcherCoordinator = robot.launcherCoordinator;
        this.launcherCommands = robot.launcherCommands;
        this.manualSpinController = createManualSpinController();

        intakeForwardHold = operator.rightBumper();

        spinModeFull = operator.leftBumper();
        launchLeft = operator.x();
        launchCenter = operator.y();
        launchRight = operator.b();
        launchAll = operator.a();

        spinLeft = new SpinLaneCommand(robot.launcher, manualSpinController);
        spinCenter = new SpinLaneCommand(robot.launcher, manualSpinController);
        spinRight = new SpinLaneCommand(robot.launcher, manualSpinController);

        fireLeft = new FireLaneCommand(robot.launcher, LauncherLane.LEFT, true, manualSpinController);
        fireCenter = new FireLaneCommand(robot.launcher, LauncherLane.CENTER, true, manualSpinController);
        fireRight = new FireLaneCommand(robot.launcher, LauncherLane.RIGHT, true, manualSpinController);

        launchAllSpinner = new SpinHoldCommand(robot.launcher, manualSpinController);
        fireAll = new FireAllCommand(robot.launcher, true, manualSpinController);
        manualSpinHold = new SpinHoldCommand(robot.launcher, manualSpinController);
        intakeForwardCommand = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        intakeReverseCommand = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        configureMatchBindings();
    }

    private void configureMatchBindings() {
        robot.launcher.setDebugOverrideEnabled(false);
        if (launcherCoordinator != null) {
            launcherCoordinator.clearIntakeOverride();
        }
        syncLightingWithIntake();

        spinModeFull.whenTrue(manualSpinHold);
        spinModeFull.whenBecomesFalse(launcherCommands::setSpinModeToOff);

        launchLeft.whenTrue(spinLeft);
        launchLeft.whenBecomesFalse(fireLeft);

        launchCenter.whenTrue(spinCenter);
        launchCenter.whenBecomesFalse(fireCenter);

        launchRight.whenTrue(spinRight);
        launchRight.whenBecomesFalse(fireRight);

        launchAll.whenTrue(launchAllSpinner);
        launchAll.whenBecomesFalse(fireAll);

        intakeForwardHold.whenBecomesTrue(intakeForwardCommand);
        intakeForwardHold.whenBecomesFalse(intakeReverseCommand);
    }

    private ManualSpinController createManualSpinController() {
        if (launcherCoordinator == null) {
            return ManualSpinController.NO_OP;
        }
        return new ManualSpinController() {
            private int activeSources = 0;

            @Override
            public void enterManualSpin() {
                if (activeSources++ == 0) {
                    launcherCoordinator.setManualSpinOverride(true);
                }
            }

            @Override
            public void exitManualSpin() {
                if (activeSources <= 0) {
                    return;
                }
                if (--activeSources == 0) {
                    launcherCoordinator.setManualSpinOverride(false);
                }
            }
        };
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
