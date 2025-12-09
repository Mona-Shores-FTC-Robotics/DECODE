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
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Optional;

public class OperatorBindings {
    private static final double MOTIF_TAIL_STICK_THRESHOLD = 0.6;
    private final Button distanceBasedLaunch;
    private final Button smartIntakeButton;
    private final Button smartIntakeTrigger;
    private final Range smartIntakeTriggerRange;
    private final Button regularIntakeTrigger;
    private final Range regularIntakeTriggerRange;
    private final Button humanLoading;
    private final Button ejectButton;

    private final Button launchShort;
    private final Button launchMid;
    private final Button launchFar;

    private final Range motifTailStickX;
    private final Range motifTailStickY;
    private final Button motifTailLeft;
    private final Button motifTailUp;
    private final Button motifTailRight;
    private final Button toggleLauncherMode;
    private final Button detectMotifButton;

    private Gamepad rawGamepad;  // Raw gamepad for haptic feedback
    private Robot robot;  // Store robot reference for motif detection

    public OperatorBindings(GamepadEx operator) {

        //Smart Ground Intake (right bumper and right trigger)
        smartIntakeButton = operator.rightBumper();
        smartIntakeTriggerRange = operator.rightTrigger();
        smartIntakeTrigger = smartIntakeTriggerRange.greaterThan(0.2);

        //Regular Ground Intake (left trigger)
        regularIntakeTriggerRange = operator.leftTrigger();
        regularIntakeTrigger = regularIntakeTriggerRange.greaterThan(0.2);

        //Human Player Intake
        humanLoading = operator.triangle();

        //Eject Button
        ejectButton = operator.square();

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

        // Detect motif from vision (dpad right - use when motif wasn't detected in auto)
        detectMotifButton = operator.dpadRight();

    }

    public void configureTeleopBindings(Robot robot, Gamepad operatorGamepad) {
        this.rawGamepad = operatorGamepad; //Need the raw gamepad for rumble features
        this.robot = robot;  // Store for motif detection
        robot.launcherCommands.setOperatorGamepad(operatorGamepad);

        configureGroundIntakeBindings(robot, operatorGamepad);
        configureHumanIntakeBindings(robot);
        configureEjectBindings(robot);
        configureDistanceBasedLaunchBindings(robot);
        configurePresetRangeLaunchBindings(robot);

        // Motif tail quick-set: left stick directions map to exact motif tail values
        motifTailLeft.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 0));
        motifTailUp.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 1));
        motifTailRight.whenBecomesTrue(() -> setMotifTailWithFeedback(robot, 2));

        // Mode toggle: manually switch between THROUGHPUT and DECODE
        toggleLauncherMode.whenBecomesTrue(this::toggleLauncherMode);

        // Detect motif from vision (for when auto missed it or crashed)
        detectMotifButton.whenBecomesTrue(this::tryDetectMotif);

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
        launchFar
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
        // NOTE: Hood retraction is now speed-gated per-lane in LauncherSubsystem.periodic()
        // Each lane's hood retracts independently when its flywheel reaches threshold RPM
        // This allows graceful degradation - if one lane is jammed, other lanes still work
        humanLoading
                .whenBecomesTrue(robot.launcher::runReverseFlywheelForHumanLoading)
                .whenBecomesTrue(robot.intake::setGateReverseConfig)
                .whenBecomesTrue(robot.intake::startForward)
                // Hood retraction removed - now happens automatically when flywheel reaches speed
                .whenBecomesTrue(robot.intake::deactivateRoller)

                .whenBecomesFalse(robot.intake::startReverseIntakeMotor)
                .whenBecomesFalse(robot.launcher::stopReverseFlywheelForHumanLoading)
                .whenBecomesFalse(robot.intake::setGatePreventArtifact)
                .whenBecomesFalse(robot.launcher::setAllHoodsExtended)
                .whenBecomesFalse(robot.intake::forwardRoller);
    }

    private void configureEjectBindings(Robot robot) {
        // Eject button: Run intake motor in reverse and reverse the roller
        ejectButton
                .whenBecomesTrue(robot.intake::startEject)
                .whenBecomesTrue(robot.intake::reverseRoller)
                .whenBecomesFalse(robot.intake::startReverseIntakeMotor)
                .whenBecomesFalse(robot.intake::forwardRoller);
    }

    private void configureGroundIntakeBindings(Robot robot, Gamepad rawOperatorGamepad) {
        // Intake control commands
        SetIntakeModeCommand intakeForwardCommand = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        SetIntakeModeCommand intakeReverseCommand = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        SetIntakeModeCommand intakeReverseCommand2 = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        SetIntakeModeCommand intakeReverseCommand3 = new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

        SmartIntakeCommand smartIntakeCommandButton = new SmartIntakeCommand(robot.intake, rawOperatorGamepad);
        SmartIntakeCommand smartIntakeCommandTrigger = new SmartIntakeCommand(robot.intake, rawOperatorGamepad);

        // Smart intake on right bumper
        smartIntakeButton.whenBecomesTrue(smartIntakeCommandButton);
        smartIntakeButton.whenBecomesFalse(intakeReverseCommand);

        // Smart intake on right trigger
        smartIntakeTrigger.whenBecomesTrue(smartIntakeCommandTrigger);
        smartIntakeTrigger.whenBecomesFalse(intakeReverseCommand2);

        // Regular (non-smart) intake on left trigger
        regularIntakeTrigger.whenBecomesTrue(intakeForwardCommand);
        regularIntakeTrigger.whenBecomesFalse(intakeReverseCommand3);
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
     * Tries to detect the motif pattern from vision.
     * Use this when auto missed detection or you crashed during auto.
     * Point the camera at the obelisk apriltag (21, 22, or 23) and press dpad right.
     */
    private void tryDetectMotif() {
        if (robot == null || robot.vision == null) {
            return;
        }

        Optional<Integer> motifTag = robot.vision.findMotifTagId();
        if (motifTag.isPresent()) {
            MotifPattern pattern = MotifPattern.fromTagId(motifTag.get());
            RobotState.setMotif(pattern);

            // Visual feedback
            if (robot.lighting != null) {
                robot.lighting.showMotifPattern(pattern.getLaneColors());
            }

            // Haptic feedback - success rumble
            if (rawGamepad != null) {
                rawGamepad.rumble(200);
            }
        }
    }

    /**
     * Human-readable control summary for driver station telemetry.
     */
    public static String[] controlsSummary() {
        return new String[]{
                "Right bumper: Smart ground intake (hold)",
                "Right trigger: Smart ground intake (hold)",
                "Left trigger: Regular ground intake (hold)",
                "Triangle: Human loading (hold)",
                "Square: Eject (hold)",
                "Cross: Distance-based spin (hold), release to launch all",
                "D-pad Down: Preset SHORT spin (hold), release to launch all",
                "D-pad Left: Preset MID spin (hold), release to launch all",
                "D-pad Up: Preset FAR spin (hold), release to launch all",
                "D-pad Right: Detect motif from vision (press)",
                "Left stick: Motif tail quick set (left=0, up=1, right=2)",
                "Back: Toggle launcher mode"
        };
    }
}
