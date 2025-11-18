package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllAtAutoRangeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllAtRangeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchSequentialCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinUpUntilReadyCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Match-oriented operator bindings. Keeps the intake running in reverse by default,
 * forwards when the operator pulls the trigger, and maps launcher commands to the
 * standard match buttons.
 *
 * Button assignments:
 * - X: Fire all lanes at SHORT range (~2700 RPM)
 * - A: Fire all lanes at MID range (~3600 RPM)
 * - B: Fire all lanes at LONG range (~4200 RPM)
 * - D-Pad Up: Auto-range fire (selects short/mid/long based on distance)
 * - Left Bumper: Manual spin hold (for pre-spinning before shots)
 * - Right Bumper: Intake forward
 */
public class OperatorBindings {
    private final Button fireShort;
    private final Button fireMid;
    private final Button fireLong;
    private final Button fireRange;
    private final Button fireSequence;
//    private final Button fireRangeSequence;

    private final Button runIntake;
    private final Button humanLoading;
    private final Button spinLetGoToShoot;

    public OperatorBindings(GamepadEx operator) {

        // Planned Final Button Assignments
        runIntake = operator.rightBumper();
        fireShort = operator.x();
        fireMid = operator.a();
        fireLong = operator.b();
        humanLoading = operator.y();
        spinLetGoToShoot = operator.leftBumper();
        fireSequence = operator.dpadDown();
        fireRange = operator.dpadUp();



        //TODO analyze the launcher commands to make a command that sets RPM based on distance to goal
                // use AprilTag range for RPM calculation (maybe pose geometry as fallback)
                // make a formula to calculate RPM based on distance using FireAllAtRangeCommand constants as guideposts
                // consider setting hood position based on range too

        //TODO analyze the launcher commands to make a command that sequences shots based on current Obelisk pattern (PPG, PGP, GPP)
                // this command should leverage color data about artifacts currently in the robot

        //TODO make a command that does both 1) sets RPM (and hood) based on distance and 2) sequences shots based on current Obelisk pattern (PPG, PGP, GPP)

    }

    public void configureTeleopBindings(Robot robot) {
        // Range-based shooting commands
        //Commands
        FireAllAtRangeCommand fireShortRangeCommand = robot.launcherCommands.fireAllShortRange();
        FireAllAtRangeCommand fireMidRangeCommand = robot.launcherCommands.fireAllMidRange();
        FireAllAtRangeCommand fireLongRangeCommand = robot.launcherCommands.fireAllLongRange();

        //Commands
        SpinUpUntilReadyCommand spinUpCommand = robot.launcherCommands.spinUpUntilReady();
        FireAllCommand fireAllCommand = robot.launcherCommands.fireAll(true);
        LaunchSequentialCommand fireAllInSequenceCommand = robot.launcherCommands.launchAllInSequence();
        FireAllAtAutoRangeCommand fireAllAutoRangeCommand = robot.launcherCommands.fireAllAutoRange(robot.vision, robot.drive);

        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        // Range-based shooting: press button to fire all lanes at that range
        fireShort.whenBecomesTrue(fireShortRangeCommand);
        fireMid.whenBecomesTrue(fireMidRangeCommand);
        fireLong.whenBecomesTrue(fireLongRangeCommand);

        // Intake control
//        runIntake.whenBecomesTrue(intakeForwardCommand);
//        runIntake.whenBecomesFalse(intakeReverseCommand);

        // Reverse Flywheel and Prefeed for Human Loading
        humanLoading
                .whenBecomesTrue(robot.launcher::runReverseFlywheelForHumanLoading)
                .whenBecomesTrue(robot.intake::setPrefeedReverse)
                .whenBecomesTrue(robot.launcher::setAllHoodsRetracted)
                .whenBecomesFalse(robot.launcher::stopReverseFlywheelForHumanLoading)
                .whenBecomesFalse(robot.intake::setPrefeedForward)
                .whenBecomesFalse(robot.launcher::setAllHoodsExtended);

        spinLetGoToShoot
                .whenBecomesTrue(spinUpCommand) //this command just makes us spin up until we're ready to shoot (does not go to idle after)
                .whenBecomesFalse(fireAllCommand); //prefeeder started and stopped in the fireAll command

        fireSequence
                .whenBecomesTrue(spinUpCommand) //this command just makes us spin up until we're ready to shoot (does not go to idle after)
                .whenBecomesFalse(fireAllInSequenceCommand);


        fireRange
                .whenBecomesTrue(fireAllAutoRangeCommand);
    }

}
