package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllAtRangeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinHoldCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;

/**
 * Match-oriented operator bindings. Keeps the intake running in reverse by default,
 * forwards when the operator pulls the trigger, and maps launcher commands to the
 * standard match buttons.
 *
 * Button assignments:
 * - X: Fire all lanes at SHORT range (~2700 RPM)
 * - Y: Fire all lanes at MID range (~3600 RPM)
 * - B: Fire all lanes at LONG range (~4200 RPM)
 * - A: Manual spin hold (for pre-spinning before shots)
 * - Left Bumper: (previously manual spin, now available for other use)
 * - Right Bumper: Intake forward
 */
public class OperatorBindings {

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final FireAllAtRangeCommand fireShortRange;
    private final FireAllAtRangeCommand fireMidRange;
    private final FireAllAtRangeCommand fireLongRange;
    private final SpinHoldCommand manualSpinHold;
    private final SetIntakeModeCommand intakeForwardCommand;
    private final SetIntakeModeCommand intakeReverseCommand;
    private final Button fireShortButton;
//    private final Button fireMidButton;
    private final Button fireLongButton;
    private final Button manualSpinButton;
    private final Button intakeForwardHold;

    private final Button flywheelHumanLoadingButton;

    private boolean manualIntakeActive = false;


    private final LauncherCommands launcherCommands;


    public OperatorBindings(GamepadEx operator,
                            Robot robot) {
        this.robot = robot;

        this.launcherCoordinator = robot.launcherCoordinator;
        this.launcherCommands = robot.launcherCommands;

        // Button assignments
        intakeForwardHold = operator.rightBumper();
        fireShortButton = operator.x();
//        fireMidButton = operator.y();
        fireLongButton = operator.b();
        manualSpinButton = operator.a();


        // Range-based shooting commands
        fireShortRange = launcherCommands.fireAllShortRange();
        fireMidRange = launcherCommands.fireAllMidRange();
        fireLongRange = launcherCommands.fireAllLongRange();

        // Manual spin hold for pre-spinning
        manualSpinHold = new SpinHoldCommand(robot.launcher, robot.manualSpinController);

        // Intake control commands
        intakeForwardCommand = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        intakeReverseCommand = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        flywheelHumanLoadingButton = operator.y();


        configureMatchBindings();
    }

    private void configureMatchBindings() {
        robot.launcher.setDebugOverrideEnabled(false);
        if (launcherCoordinator != null) {
            launcherCoordinator.clearIntakeOverride();
        }
        syncLightingWithIntake();

        // Range-based shooting: press button to fire all lanes at that range
        fireShortButton.whenBecomesTrue(fireShortRange);
//        fireMidButton.whenBecomesTrue(fireMidRange);
        fireLongButton.whenBecomesTrue(fireLongRange);

        // Manual spin hold: hold A to pre-spin flywheels, release to spin down
        manualSpinButton.whenTrue(manualSpinHold);
        manualSpinButton.whenBecomesFalse(launcherCommands::setSpinModeToOff);

        // Intake control
        intakeForwardHold.whenBecomesTrue(intakeForwardCommand);
        intakeForwardHold.whenBecomesFalse(intakeReverseCommand);

        flywheelHumanLoadingButton
                .whenBecomesTrue(robot.launcher::runReverseFlywheelForHumanLoading)
                .whenBecomesFalse(robot.launcher::stopReverseFlywheelForHumanLoading);
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
