package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.DistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LaunchAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class OperatorBindings {
    private static final double MOTIF_TAIL_STICK_THRESHOLD = 0.6;
    private final Button distanceBasedLaunch;
    private final Button runIntakeButton;
    private final Button runIntakeTrigger;
    private final Range runIntakeTriggerRange;
    private final Button humanLoading;

    private final Button launchShort;
    private final Button launchMid;
    private final Button launchFar;

    private final Range motifTailStickX;
    private final Range motifTailStickY;
    private final Button motifTailLeft;
    private final Button motifTailUp;
    private final Button motifTailRight;
    private final Button toggleLauncherMode;

    private Gamepad rawGamepad;  // Raw gamepad for haptic feedback

    public OperatorBindings(GamepadEx operator) {

        //Ground Intake
        runIntakeButton = operator.rightBumper();
        runIntakeTriggerRange = operator.rightTrigger();
        runIntakeTrigger = runIntakeTriggerRange.greaterThan(0.2);

        //Human Player Intake
        humanLoading = operator.triangle();

        //Distance Based Launching
        distanceBasedLaunch = operator.cross();

        //Preset Range Launching
        launchFar = operator.dpadUp();  // FAR range for close shots (if vision is broken)
        launchMid = operator.dpadLeft(); // MID range for far shots (if vision is broken)
        launchShort = operator.dpadDown();  // SHORT range for close shots (if vision is broken)

        // Motif tail quick-set on left stick (left=0, up=1, right=2)
        motifTailStickX = operator.leftStickX().deadZone(0.15);
        motifTailStickY = operator.leftStickY().deadZone(0.15);
        motifTailLeft = motifTailStickX.negate().greaterThan(MOTIF_TAIL_STICK_THRESHOLD);
        motifTailRight = motifTailStickX.greaterThan(MOTIF_TAIL_STICK_THRESHOLD);
        motifTailUp = motifTailStickY.negate().greaterThan(MOTIF_TAIL_STICK_THRESHOLD);

        toggleLauncherMode = operator.back();

    }

    public void configureTeleopBindings(Robot robot, Gamepad operatorGamepad) {
        this.rawGamepad = operatorGamepad; //Need the raw gamepad for rumble features
        robot.launcherCommands.setOperatorGamepad(operatorGamepad);

        configureGroundIntakeBindings(robot, operatorGamepad);
        configureHumanIntakeBindings(robot);
        configureDistanceBasedLaunchBindings(robot);
        configurePresetRangeLaunchBindings(robot);

        // Motif tail quick-set: left stick directions map to exact motif tail values
        motifTailLeft.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 0));
        motifTailUp.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 1));
        motifTailRight.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 2));

        // Mode toggle: manually switch between THROUGHPUT and DECODE
        toggleLauncherMode.whenBecomesTrue(this::toggleLauncherMode);

    }

    private void configurePresetRangeLaunchBindings(Robot robot) {
        Command launchCommand = robot.launcherCommands.launchAccordingToMode(true);

        PresetRangeSpinCommand spinShortCommand = robot.launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, false);
        launchShort
                .whenBecomesTrue(spinShortCommand)
                .whenBecomesFalse(launchCommand)
                .whenBecomesFalse(robot.drive::tryRelocalizeForShot);

        PresetRangeSpinCommand spinMidCommand = robot.launcherCommands.presetRangeSpinUp(LauncherRange.MID, false);
        launchMid
                .whenBecomesTrue(spinMidCommand)
                .whenBecomesFalse(launchCommand)
                .whenBecomesFalse(robot.drive::tryRelocalizeForShot);

        PresetRangeSpinCommand spinFarCommand = robot.launcherCommands.presetRangeSpinUp(LauncherRange.LONG, false);
        launchMid
                .whenBecomesTrue(spinFarCommand)
                .whenBecomesFalse(launchCommand)
                .whenBecomesFalse(robot.drive::tryRelocalizeForShot);
    }

    private void configureDistanceBasedLaunchBindings(Robot robot) {
        // Distance-based launching commands
        Command distanceBasedSpinCommand = robot.launcherCommands.distanceBasedSpinUp(robot.vision, robot.drive, robot.lighting, rawGamepad);
        Command launchCommand = robot.launcherCommands.launchAccordingToMode(true);

        // Cross button: Hold to spin up at distance-calculatedRPM, release to fire all lanes
        distanceBasedLaunch
                .whenBecomesTrue(distanceBasedSpinCommand)
                .whenBecomesFalse(launchCommand)
                .whenBecomesFalse(robot.drive::tryRelocalizeForShot);
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

    private void configureGroundIntakeBindings(Robot robot, Gamepad rawOperatorGamepad) {
        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake , IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        SmartIntakeCommand smartIntakeCommand = new SmartIntakeCommand(robot.intake, rawOperatorGamepad );

        // Intake control
        runIntakeButton.whenBecomesTrue(intakeForwardCommand);
        runIntakeButton.whenBecomesFalse(intakeReverseCommand);

        // Allow right trigger to start smart intake as well
        runIntakeTrigger.whenBecomesTrue(smartIntakeCommand);
        runIntakeTrigger.whenBecomesFalse(intakeReverseCommand);
    }

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

    /**
     * Human-readable control summary for driver station telemetry.
     */
    public static String[] controlsSummary() {
        return new String[]{
                "Right bumper: Ground intake (hold)",
                "Right trigger: Smart ground intake (hold)",
                "Triangle: Human loading (hold)",
                "Cross: Distance-based spin (hold), release to launch all",
                "D-pad Down: Preset SHORT spin (hold), release to launch all",
                "D-pad Left: Preset MID spin (hold), release to launch all",
                "D-pad Up: Preset FAR spin (hold), release to launch all",
                "Left stick: Motif tail quick set (left=0, up=1, right=2)",
                "Back: Toggle launcher mode"
        };
    }
}
