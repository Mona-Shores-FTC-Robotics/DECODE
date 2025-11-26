package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.DistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllAtPresetRangeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllCommand;
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
 * - D-Pad Left: Universal smart shot (distance-based RPM + mode-aware firing) - THE ULTIMATE BUTTON!
 * - D-Pad Right: Cycle motif tail (0 → 1 → 2 → 0, visual feedback blinks corresponding lanes orange)
 * - Back: Toggle launcher mode (THROUGHPUT ↔ DECODE)
 * - Right Bumper: Intake forward
 *
 * Removed bindings (simplified controls):
 * - Left Bumper: Removed (was redundant with X button hold-to-spin)
 * - D-Pad Down: Removed (redundant with D-Pad Left UniversalSmartShot)
 * - D-Pad Up: Available for future use
 *
 * Testing: D-Pad Left has UniversalSmartShotCommand for evaluation. If it proves reliable,
 * it could replace X entirely, becoming the single primary fire button.
 */
public class OperatorBindings {
    private final Button fireDistanceBased;
//    private final Button fireUniversalSmart;
    private final Button runIntake;
    private final Button humanLoading;

    private final Button fireShort;
    private final Button fireMid;


    private final Button motifTailCycle;
    private final Button toggleLauncherMode;

    private Gamepad rawGamepad;  // Raw gamepad for haptic feedback

    public OperatorBindings(GamepadEx operator) {

        //Robot Intake
        runIntake = operator.rightBumper();

        //Human Player Loading
        humanLoading = operator.triangle();

        //launch based on range to april tag.
        fireDistanceBased = operator.cross();

        //launch based on range to april tag - simultaneous or sequential (Obelisk Pattern) depending on launcher mode
//        fireUniversalSmart = operator.dpadLeft();  // Ultimate smart shot for testing

        toggleLauncherMode = operator.back();
        motifTailCycle = operator.dpadRight();  // Cycle through 0 → 1 → 2 → 0

        //launch based on preset RPM and hood values for specific field locations
        fireShort = operator.dpadDown();  // SHORT range for close shots (if vision is broken)
        fireMid = operator.dpadLeft(); // LONG range for far shots (if vision is broken)


    }

    public void configureTeleopBindings(Robot robot, Gamepad operatorGamepad) {
        this.rawGamepad = operatorGamepad; //Need the raw gamepad for rumble features

        // Distance-based launching commands
        DistanceBasedSpinCommand distanceBasedSpinCommand = robot.launcherCommands.distanceBasedSpin(robot.vision, robot.drive, robot.lighting, rawGamepad);
        LaunchAllCommand launchAllCommand = robot.launcherCommands.launchAll(true);

        // X button: Hold to spin up at distance-calculatedRPM, release to fire all lanes
        fireDistanceBased.whenBecomesTrue(distanceBasedSpinCommand);
        fireDistanceBased.whenBecomesFalse(launchAllCommand);

        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        // Intake control
        runIntake.whenBecomesTrue(intakeForwardCommand);
        runIntake.whenBecomesFalse(intakeReverseCommand);

        // Human Loading
        humanLoading
                .whenBecomesTrue(robot.launcher::runReverseFlywheelForHumanLoading)
                .whenBecomesTrue(robot.intake::setGateReverseConfig)
                .whenBecomesTrue(robot.intake::startForward)
                .whenBecomesTrue(robot.launcher::setAllHoodsRetracted)
                .whenBecomesTrue(robot.intake::deactivateRoller)

                .whenBecomesFalse(robot.intake::startReverseIntakeMotor)
                .whenBecomesFalse(robot.launcher::stopReverseFlywheelForHumanLoading)
                .whenBecomesFalse(robot.intake::setGatePreventArtifact)
                .whenBecomesFalse(robot.launcher::setAllHoodsExtended)
                .whenBecomesFalse(robot.intake::forwardRoller);

        // Motif tail cycle: single button cycles through 0 → 1 → 2 → 0 with visual feedback
        motifTailCycle.whenBecomesTrue(() -> cycleMotifTailWithFeedback(robot));

        // Mode toggle: manually switch between THROUGHPUT and DECODE
        toggleLauncherMode.whenBecomesTrue(this::toggleLauncherMode);

        LaunchAllAtPresetRangeCommand launchAllShortCommand = robot.launcherCommands.fireAllShortRange();
        fireShort.whenBecomesTrue(launchAllShortCommand);

        LaunchAllAtPresetRangeCommand launchAllMidCommand = robot.launcherCommands.fireAllMidRange();
        fireMid.whenBecomesTrue(launchAllMidCommand);

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
