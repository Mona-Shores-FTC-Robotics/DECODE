package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.DistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class OperatorBindings {
    private final Button distanceBasedLaunch;
    private final Button runIntake;
    private final Button humanLoading;

    private final Button launchShort;
    private final Button launchMid;

    private final Button motifTailCycle;
    private final Button toggleLauncherMode;

    private Gamepad rawGamepad;  // Raw gamepad for haptic feedback

    public OperatorBindings(GamepadEx operator) {

        //Ground Intake
        runIntake = operator.rightBumper();

        //Human Player Intake
        humanLoading = operator.triangle();

        //Distance Based Launching
        distanceBasedLaunch = operator.cross();

        //Preset Range Launching
        launchShort = operator.dpadDown();  // SHORT range for close shots (if vision is broken)
        launchMid = operator.dpadLeft(); // LONG range for far shots (if vision is broken)

        //launch based on range to april tag - simultaneous or sequential (Obelisk Pattern) depending on launcher mode
//        fireUniversalSmart = operator.dpadLeft();  // Ultimate smart shot for testing

        toggleLauncherMode = operator.back();
        motifTailCycle = operator.dpadRight();  // Cycle through 0 → 1 → 2 → 0

    }

    public void configureTeleopBindings(Robot robot, Gamepad operatorGamepad) {
        this.rawGamepad = operatorGamepad; //Need the raw gamepad for rumble features

        configureGroundIntakeBindings(robot);
        configureHumanIntakeBindings(robot);
        configureDistanceBasedLaunchBindings(robot);
        configurePresetRangeLaunchBindings(robot);

        // Motif tail cycle: single button cycles through 0 → 1 → 2 → 0 with visual feedback
        motifTailCycle.whenBecomesTrue(() -> cycleMotifTailWithFeedback(robot));

        // Mode toggle: manually switch between THROUGHPUT and DECODE
        toggleLauncherMode.whenBecomesTrue(this::toggleLauncherMode);

    }

    private void configurePresetRangeLaunchBindings(Robot robot) {
        LaunchAllCommand launchAllCommand = robot.launcherCommands.launchAll(true);

        PresetRangeSpinCommand spinShortCommand = robot.launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, false);
        launchShort
                .whenBecomesTrue(spinShortCommand)
                .whenBecomesFalse(launchAllCommand);

        PresetRangeSpinCommand spinMidCommand = robot.launcherCommands.presetRangeSpinUp(LauncherRange.MID, false);
        launchMid
                .whenBecomesTrue(spinMidCommand)
                .whenBecomesFalse(launchAllCommand);
    }

    private void configureDistanceBasedLaunchBindings(Robot robot) {

        // Distance-based launching commands
        DistanceBasedSpinCommand distanceBasedSpinCommand = robot.launcherCommands.distanceBasedSpinUp(robot.vision, robot.drive, robot.lighting, rawGamepad);
        LaunchAllCommand launchAllCommand = robot.launcherCommands.launchAll(true);

        // Cross button: Hold to spin up at distance-calculatedRPM, release to fire all lanes
        distanceBasedLaunch
                .whenBecomesTrue(distanceBasedSpinCommand)
                .whenBecomesFalse(launchAllCommand);

    }

    private void configureHumanIntakeBindings(Robot robot) {
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
    }

    private void configureGroundIntakeBindings(Robot robot) {
        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        // Intake control
        runIntake.whenBecomesTrue(intakeForwardCommand);
        runIntake.whenBecomesFalse(intakeReverseCommand);
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
