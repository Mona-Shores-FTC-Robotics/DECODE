package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ContinuousDistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.HumanLoadingCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllAtPresetRangeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchModeAwareCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinUpUntilReadyCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.UniversalSmartShotCommand;
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
 * - A: Fire all lanes at SHORT range (~2700 RPM) - close shots safety net
 * - B: Fire all lanes at LONG range (~4200 RPM)
 * - Y: Toggle human loading (press once to start, press again to stop)
 * - D-Pad Down: Mode-aware fire (THROUGHPUT or DECODE sequence based on current mode)
 * - D-Pad Left: Universal smart shot (distance-based RPM + mode-aware firing) - THE ULTIMATE BUTTON!
 * - D-Pad Right: Cycle motif tail (0 → 1 → 2 → 0, visual feedback blinks corresponding lanes orange)
 * - Back: Toggle launcher mode (THROUGHPUT ↔ DECODE)
 * - Right Bumper: Intake forward
 *
 * Removed bindings (simplified controls):
 * - Left Bumper: Removed (was redundant with X button hold-to-spin)
 * - D-Pad Up: Available for future use
 *
 * Testing: D-Pad Left has UniversalSmartShotCommand for evaluation. If it proves reliable,
 * it could replace X and D-Pad Down entirely, becoming the single primary fire button.
 */
public class OperatorBindings {
    private final Button fireDistanceBased;
    private final Button fireShort;
    private final Button fireLong;
    private final Button fireModeAware;
    private final Button fireUniversalSmart;

    private final Button runIntake;
    private final Button humanLoading;

    private final Button motifTailCycle;
    private final Button toggleLauncherMode;

    private Gamepad rawGamepad;  // Raw gamepad for haptic feedback

    public OperatorBindings(GamepadEx operator) {

        // Consolidated Button Assignments
        runIntake = operator.rightBumper();
        fireDistanceBased = operator.x();
        fireShort = operator.a();  // SHORT range for close shots
        fireLong = operator.b();
        humanLoading = operator.y();
        fireModeAware = operator.dpadDown();
        fireUniversalSmart = operator.dpadLeft();  // Ultimate smart shot for testing
        motifTailCycle = operator.dpadRight();  // Cycle through 0 → 1 → 2 → 0
        toggleLauncherMode = operator.back();

        //TODO analyze the launcher commands to make a command that sets RPM based on distance to goal
                // use AprilTag range for RPM calculation (maybe pose geometry as fallback)
                // make a formula to calculate RPM based on distance using LaunchAllAtPresetRangeCommand constants as guideposts
                // consider setting hood position based on range too

        //TODO analyze the launcher commands to make a command that sequences shots based on current Obelisk pattern (PPG, PGP, GPP)
                // this command should leverage color data about artifacts currently in the robot

        //TODO make a command that does both 1) sets RPM (and hood) based on distance and 2) sequences shots based on current Obelisk pattern (PPG, PGP, GPP)

    }

    public void configureTeleopBindings(Robot robot, Gamepad operatorGamepad) {
        this.rawGamepad = operatorGamepad;

        // Range-based shooting commands
        LaunchAllAtPresetRangeCommand fireShortRangeCommand = robot.launcherCommands.fireAllShortRange();
        LaunchAllAtPresetRangeCommand fireLongRangeCommand = robot.launcherCommands.fireAllLongRange();

        // Distance-based shooting commands
        ContinuousDistanceBasedSpinCommand spinUpAtDistanceCommand = robot.launcherCommands.spinUpAtDistance(
            robot.vision, robot.drive, robot.lighting, rawGamepad);
        LaunchAllCommand fireAllCommand = robot.launcherCommands.fireAll(true);

        // Mode-aware commands
        SpinUpUntilReadyCommand spinUpCommand = robot.launcherCommands.spinUpUntilReady();
        LaunchModeAwareCommand fireModeAwareCommand = robot.launcherCommands.fireModeAware();

        // Universal smart shot command (distance-based + mode-aware)
        UniversalSmartShotCommand universalSmartShotCommand = robot.launcherCommands.fireUniversalSmart(
            robot.vision, robot.drive, robot.lighting, rawGamepad);

        // Human loading toggle command
        HumanLoadingCommand humanLoadingToggle = robot.launcherCommands.toggleHumanLoading();

        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        // X button: Hold to spin up at distance-calculated RPM, release to fire all lanes
        fireDistanceBased.whenBecomesTrue(spinUpAtDistanceCommand);
        fireDistanceBased.whenBecomesFalse(fireAllCommand);

        // Range-based shooting: press button to fire all lanes at that range
        fireShort.whenBecomesTrue(fireShortRangeCommand);  // A: Close shots
        fireLong.whenBecomesTrue(fireLongRangeCommand);    // B: Long shots

        // Intake control
        runIntake.whenBecomesTrue(intakeForwardCommand);
        runIntake.whenBecomesFalse(intakeReverseCommand);

        // Human loading toggle: press once to start, press again to stop
        humanLoading.whenBecomesTrue(humanLoadingToggle);

        // Mode-aware firing: adapts between THROUGHPUT and DECODE modes
        fireModeAware
                .whenBecomesTrue(spinUpCommand) //spin up until ready
                .whenBecomesFalse(fireModeAwareCommand); //fire based on current mode

        // Universal smart shot: distance-based RPM + mode-aware firing (TESTING)
        fireUniversalSmart.whenBecomesTrue(universalSmartShotCommand);

        // Motif tail cycle: single button cycles through 0 → 1 → 2 → 0 with visual feedback
        motifTailCycle.whenBecomesTrue(() -> cycleMotifTailWithFeedback(robot));

        // Mode toggle: manually switch between THROUGHPUT and DECODE
        toggleLauncherMode.whenBecomesTrue(this::toggleLauncherMode);
    }

    /**
     * Cycles motif tail value through 0 → 1 → 2 → 0 and shows visual feedback on lights.
     *
     * Motif tail values:
     * - 0: All 3 lanes blink orange (0, 3, 6, ... artifacts in field ramp)
     * - 1: Left lane blinks orange (1, 4, 7, ... artifacts in field ramp)
     * - 2: Left+center blink orange (2, 5, 8, ... artifacts in field ramp)
     *
     * @param robot The robot instance (for lighting subsystem access)
     */
    private void cycleMotifTailWithFeedback(Robot robot) {
        int current = RobotState.getMotifTail();
        int next = (current + 1) % 3;  // Cycle: 0 → 1 → 2 → 0
        RobotState.setMotifTail(next);
        if (robot != null && robot.lighting != null) {
            robot.lighting.showMotifTailFeedback(next);
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
