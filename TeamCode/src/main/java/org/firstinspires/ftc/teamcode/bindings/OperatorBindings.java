package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllAtRangeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

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

    private final Button fireShort;
    private final Button fireMid;
    private final Button fireLong;
//    private final Button fireRange;
//    private final Button fireSequence;
//    private final Button fireRangeSequence;

    private final Button runIntake;
    private final Button humanLoading;

    //Commands
    private final FireAllAtRangeCommand fireShortRange;
    private final FireAllAtRangeCommand fireMidRange;
    private final FireAllAtRangeCommand fireLongRange;

    public OperatorBindings(GamepadEx operator,
                            Robot robot) {
        this.robot = robot;

        // Planned Final Button Assignments
        runIntake = operator.rightBumper();
        fireShort = operator.x();
        fireMid = operator.a();
        fireLong = operator.b();
        humanLoading = operator.y();

        // Range-based shooting commands
        fireShortRange = robot.launcherCommands.fireAllShortRange();
        fireMidRange = robot.launcherCommands.fireAllMidRange();
        fireLongRange = robot.launcherCommands.fireAllLongRange();

        //TODO analyze the launcher commands to make a command that sets RPM based on distance to goal
                // use AprilTag range for RPM calculation (maybe pose geometry as fallback)
                // make a formula to calculate RPM based on distance using FireAllAtRangeCommand constants as guideposts
                // consider setting hood position based on range too

        //TODO analyze the launcher commands to make a command that sequences shots based on current Obelisk pattern (PPG, PGP, GPP)
                // this command should leverage color data about artifacts currently in the robot

        //TODO make a command that does both 1) sets RPM (and hood) based on distance and 2) sequences shots based on current Obelisk pattern (PPG, PGP, GPP)

        configureMatchBindings();
    }

    private void configureMatchBindings() {

        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        // Range-based shooting: press button to fire all lanes at that range
        fireShort.whenBecomesTrue(fireShortRange);
        fireMid.whenBecomesTrue(fireMidRange);
        fireLong.whenBecomesTrue(fireLongRange);

        // Intake control
        runIntake.whenBecomesTrue(intakeForwardCommand);
        runIntake.whenBecomesFalse(intakeReverseCommand);

        //try to make the feed roller go opposite so we can human player feed.
        humanLoading
                .whenBecomesTrue(robot.launcher::runReverseFlywheelForHumanLoading)
//                .whenBecomesTrue(robot.)
                .whenBecomesTrue(robot.launcher::setAllHoodsRetracted)
                .whenBecomesFalse(robot.launcher::stopReverseFlywheelForHumanLoading)
                .whenBecomesTrue(robot.launcher::setAllHoodsExtended); // move all hoods to LONG

    }






}
