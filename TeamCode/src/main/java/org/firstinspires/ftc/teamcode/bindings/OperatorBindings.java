package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ContinuousDistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllAtPresetRangeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireModeAwareCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinUpUntilReadyCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Match-oriented operator bindings. Keeps the intake running in reverse by default,
 * forwards when the operator pulls the trigger, and maps launcher commands to the
 * standard match buttons.
 *
 * Button assignments:
 * - X: Hold to continuously calculate distance and spin up at calculated RPM, release to fire all lanes
 * - A: Fire all lanes at MID range (~3600 RPM)
 * - B: Fire all lanes at LONG range (~4200 RPM)
 * - Y: Human loading (reverse flywheel and prefeed)
 * - D-Pad Down: Mode-aware fire (THROUGHPUT or DECODE sequence based on current mode)
 * - D-Pad Left: Set motif tail to 0 (all 3 lanes blink orange)
 * - D-Pad Up: Set motif tail to 1 (left lane blinks orange)
 * - D-Pad Right: Set motif tail to 2 (left+center blink orange)
 * - Back: Toggle launcher mode (THROUGHPUT ↔ DECODE)
 * - Left Bumper: Manual spin hold (for pre-spinning before shots)
 * - Right Bumper: Intake forward
 */
public class OperatorBindings {
    private final Button fireDistanceBased;
    private final Button fireMid;
    private final Button fireLong;
    private final Button fireModeAware;

    private final Button runIntake;
    private final Button humanLoading;
    private final Button spinLetGoToShoot;

    private final Button motifTailSet0;
    private final Button motifTailSet1;
    private final Button motifTailSet2;
    private final Button toggleLauncherMode;

    public OperatorBindings(GamepadEx operator) {

        // Planned Final Button Assignments
        runIntake = operator.rightBumper();
        fireDistanceBased = operator.x();
        fireMid = operator.a();
        fireLong = operator.b();
        humanLoading = operator.y();
        spinLetGoToShoot = operator.leftBumper();
        fireModeAware = operator.dpadDown();
        motifTailSet0 = operator.dpadLeft();
        motifTailSet1 = operator.dpadUp();
        motifTailSet2 = operator.dpadRight();
        toggleLauncherMode = operator.back();

        //TODO analyze the launcher commands to make a command that sets RPM based on distance to goal
                // use AprilTag range for RPM calculation (maybe pose geometry as fallback)
                // make a formula to calculate RPM based on distance using LaunchAllAtPresetRangeCommand constants as guideposts
                // consider setting hood position based on range too

        //TODO analyze the launcher commands to make a command that sequences shots based on current Obelisk pattern (PPG, PGP, GPP)
                // this command should leverage color data about artifacts currently in the robot

        //TODO make a command that does both 1) sets RPM (and hood) based on distance and 2) sequences shots based on current Obelisk pattern (PPG, PGP, GPP)

    }

    public void configureTeleopBindings(Robot robot) {
        // Range-based shooting commands
        LaunchAllAtPresetRangeCommand fireMidRangeCommand = robot.launcherCommands.fireAllMidRange();
        LaunchAllAtPresetRangeCommand fireLongRangeCommand = robot.launcherCommands.fireAllLongRange();

        // Distance-based shooting commands
        ContinuousDistanceBasedSpinCommand spinUpAtDistanceCommand = robot.launcherCommands.spinUpAtDistance(robot.vision, robot.drive);
        FireAllCommand fireAllCommand = robot.launcherCommands.fireAll(true);

        // Mode-aware commands
        SpinUpUntilReadyCommand spinUpCommand = robot.launcherCommands.spinUpUntilReady();
        FireModeAwareCommand fireModeAwareCommand = robot.launcherCommands.fireModeAware();

        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        // X button: Hold to spin up at distance-calculated RPM, release to fire all lanes
        fireDistanceBased.whenBecomesTrue(spinUpAtDistanceCommand);
        fireDistanceBased.whenBecomesFalse(fireAllCommand);

        // Range-based shooting: press button to fire all lanes at that range
        fireMid.whenBecomesTrue(fireMidRangeCommand);
        fireLong.whenBecomesTrue(fireLongRangeCommand);

        // Intake control
        runIntake.whenBecomesTrue(intakeForwardCommand);
        runIntake.whenBecomesFalse(intakeReverseCommand);

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

        // Mode-aware firing: adapts between THROUGHPUT and DECODE modes
        fireModeAware
                .whenBecomesTrue(spinUpCommand) //spin up until ready
                .whenBecomesFalse(fireModeAwareCommand); //fire based on current mode

        // Motif tail controls: direct set with visual feedback
        motifTailSet0.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 0));
        motifTailSet1.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 1));
        motifTailSet2.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 2));

        // Mode toggle: manually switch between THROUGHPUT and DECODE
        toggleLauncherMode.whenBecomesTrue(this::toggleLauncherMode);
    }

    /**
     * Sets motif tail value and shows visual feedback on lights.
     *
     * @param robot The robot instance (for lighting subsystem access)
     * @param value The motif tail value (0, 1, or 2)
     */
    private void setMotifTailWithFeedback(Robot robot, int value) {
        RobotState.setMotifTail(value);
        if (robot != null && robot.lighting != null) {
            robot.lighting.showMotifTailFeedback(value);
        }
    }

    /**
     * Toggles launcher mode between THROUGHPUT and DECODE.
     */
    private void toggleLauncherMode() {
        LauncherMode current = RobotState.getLauncherMode();
        LauncherMode newMode = (current == LauncherMode.THROUGHPUT)
                ? LauncherMode.DECODE
                : LauncherMode.THROUGHPUT;
        RobotState.setLauncherMode(newMode);
    }

}
