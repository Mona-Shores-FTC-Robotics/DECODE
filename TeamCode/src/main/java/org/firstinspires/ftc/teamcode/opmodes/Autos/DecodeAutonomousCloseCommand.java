package org.firstinspires.ftc.teamcode.opmodes.Autos;

import static org.firstinspires.ftc.teamcode.util.AutoField.poseForAlliance;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotState;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Fixed Auto Close (Command)", group = "Command")
@Configurable
public class DecodeAutonomousCloseCommand extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    private Robot robot;
    private AllianceSelector allianceSelector;
    private Alliance activeAlliance = Alliance.BLUE;
    private FieldLayout currentLayout;

    // AprilTag-based start pose detection
    private Pose lastAppliedStartPosePedro;
    private Pose lastDetectedStartPosePedro;

    {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE
        );
    }
    @Override
    public void onInit() {

        BindingManager.reset();
        robot = new Robot(hardwareMap);

        robot.attachPedroFollower();

        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();

        robot.initializeForAuto();

        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, Alliance.UNKNOWN);
        activeAlliance = allianceSelector.getSelectedAlliance();
        applyAlliance(activeAlliance, null);
        allianceSelector.applySelection(robot, robot.lighting);

        addComponents(
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision)
        );
    }

    @Override
    @Override
    public void onWaitForStart() {

        // One call that:
        // - Polls vision for a basket tag
        // - Uses Pedro pose X to infer alliance (X < 72 blue, X > 72 red)
        // - Merges with manual overrides and default
        // - Applies alliance to robot and lighting
        Alliance selected = allianceSelector.updateDuringInit(
                robot.vision,
                robot,
                robot.lighting
        );

        // If alliance changed since last loop, update layout and starting pose
        if (selected != activeAlliance) {
            activeAlliance = selected;
            applyAlliance(activeAlliance, null);
        }

        // Capture the detected start pose from vision, if available
        allianceSelector.getLastSnapshot()
                .flatMap(VisionSubsystemLimelight.TagSnapshot::getRobotPosePedroMT1)
                .ifPresent(pose -> {
                    lastDetectedStartPosePedro = copyPose(pose);

                    // Optionally apply the pose as a starting override,
                    // with your sanity checks.
                    if (shouldUpdateStartPose(lastDetectedStartPosePedro)) {
                        applyAlliance(activeAlliance, lastDetectedStartPosePedro);
                    }
                });

        telemetry.clear();
        telemetry.addData("Alliance", activeAlliance.displayName());
        telemetry.addData("Artifacts", "%d detected", robot.intake.getArtifactCount());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();

        // Build and schedule the complete autonomous routine
        Command autoRoutine = LocalizeCommand.create(robot, activeAlliance);

        CommandManager.INSTANCE.scheduleCommand(autoRoutine);

        robot.intake.forwardRoller();
        robot.intake.setGateAllowArtifacts();
    }

    @Override
    public void onUpdate() {
        publishTelemetry();
    }

    @Override
    public void onStop() {
        allianceSelector.unlockSelection();
        BindingManager.reset();
        robot.launcher.abort();
        robot.drive.stop();
        robot.vision.stop();

        // Save final pose for TeleOp transition
        RobotState.setHandoffPose(robot.drive.getFollower().getPose());
    }




    private void applyAlliance(Alliance alliance, Pose startOverride) {
        Alliance safeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);

        currentLayout = AutoField.layoutForAlliance(activeAlliance);

        // Apply AprilTag-detected start pose override if provided
        if (startOverride != null) {
            currentLayout.overrideStart(startOverride);
            lastAppliedStartPosePedro = copyPose(startOverride);
        }

        Pose startPose = currentLayout.pose(FieldPoint.START_CLOSE);
        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
    }

    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        // Preview paths would be drawn here using PanelsBridge
        PanelsBridge.drawPreview(new PathChain[0], layout.pose(FieldPoint.START_CLOSE), alliance == Alliance.RED);
    }

    /**
     * Applies the last AprilTag-detected start pose (bound to dpad A during init)
     */
    private void applyLastDetectedStartPose() {
        if (!shouldUpdateStartPose(lastDetectedStartPosePedro)) {
            return;
        }
        applyAlliance(activeAlliance, lastDetectedStartPosePedro);
    }

    /**
     * Validates that a detected pose is reasonable before applying it
     */
    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) {
            return false;
        }

        // Sanity check: pose should be on the field
        double fieldWidthIn = FieldConstants.FIELD_WIDTH_INCHES ;
        if (candidate.getX() < 0 || candidate.getX() > fieldWidthIn ||
            candidate.getY() < 0 || candidate.getY() > fieldWidthIn) {
            return false;
        }

        // Additional check: if we have an applied pose, the detected pose shouldn't be wildly different
        if (lastAppliedStartPosePedro != null) {
            double deltaX = Math.abs(candidate.getX() - lastAppliedStartPosePedro.getX());
            double deltaY = Math.abs(candidate.getY() - lastAppliedStartPosePedro.getY());
            double maxDelta = 12.0; // 12 inches tolerance

            if (deltaX > maxDelta || deltaY > maxDelta) {
                return false; // Detected pose too different from expected
            }
        }

        return true;
    }

    /**
     * Creates a defensive copy of a Pose
     */
    private Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }


    private void publishTelemetry() {
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher,
                robot.intake,
                robot.vision,
                robot.lighting,
                null,
                gamepad1,
                gamepad2,
                RobotState.getAlliance(),
                getRuntime(),
                Math.max(0.0, 150.0 - getRuntime()),
                telemetry,
                "Auto",
                true,
                null,
                0,
                0
        );
    }

    /**
     * Builds a PathChain from start to end with optional control points.
     * This is public static so test OpModes can visualize the same paths.
     */
    public static PathChain buildPath(Follower follower, Pose startPose, Pose endPose, Pose... controlPoints) {
        if (controlPoints.length > 0) {
            // Curved path with control points
            return follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, controlPoints[0], endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), .7)
                    .build();
        } else {
            // Straight line
            return follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), .7)
                    .build();
        }
    }
}
