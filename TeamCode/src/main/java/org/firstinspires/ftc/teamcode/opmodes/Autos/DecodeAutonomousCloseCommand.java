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
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.AutoPrestartHelper;

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
    private AutoPrestartHelper prestartHelper;

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
        prestartHelper = new AutoPrestartHelper(robot, allianceSelector);

        addComponents(
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision)
        );
    }

    @Override
    public void onWaitForStart() {

        AutoPrestartHelper.InitStatus initStatus = prestartHelper.update(activeAlliance);
        applyInitSelections(initStatus);

        updateProximityFeedback();
        updateInitTelemetry(initStatus);
        updateDriverStationTelemetry(initStatus);
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
                "AutoInit",
                true,
                null,
                0,
                0
        );
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();

        // Build auto with vision-detected start pose (or null to use LocalizeCommand defaults)
        // Use follower's current pose as the source of truth (it was set from vision if available)
        Pose startPoseOverride = lastAppliedStartPosePedro != null
            ? lastAppliedStartPosePedro
            : null;

        Command autoRoutine = LocalizeCommand.create(robot, activeAlliance, startPoseOverride);

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

    private void updateProximityFeedback() {
        Pose currentPose = robot.drive.getFollower().getPose();
        if (currentPose == null || currentLayout == null) {
            robot.lighting.stopProximityFeedback();
            return;
        }

        Pose target = currentLayout.pose(FieldPoint.START_CLOSE);
        if (target == null) {
            robot.lighting.stopProximityFeedback();
            return;
        }

        double dx = currentPose.getX() - target.getX();
        double dy = currentPose.getY() - target.getY();
        double distance = Math.hypot(dx, dy);

        // Blink faster as distance shrinks; target radius ~18in matches other autos
        robot.lighting.showProximityFeedback(distance, 18.0);
    }

    private void applyInitSelections(AutoPrestartHelper.InitStatus status) {
        if (status == null) {
            return;
        }

        if (status.alliance != activeAlliance) {
            activeAlliance = status.alliance;
            applyAlliance(activeAlliance, null);
        }

        lastDetectedStartPosePedro = status.startPoseFromVision;
        if (shouldUpdateStartPose(lastDetectedStartPosePedro)) {
            applyAlliance(activeAlliance, lastDetectedStartPosePedro);
        }
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

    /**
     * Validates that a detected pose is reasonable before applying it.
     * Only checks that pose is on the field - no distance restrictions.
     * This allows students to adjust robot placement without getting locked to a bad pose.
     */
    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) {
            return false;
        }

        // Sanity check: pose should be on the field
        double fieldWidthIn = FieldConstants.FIELD_WIDTH_INCHES;
        if (candidate.getX() < 0 || candidate.getX() > fieldWidthIn ||
            candidate.getY() < 0 || candidate.getY() > fieldWidthIn) {
            return false;
        }

        // Log delta for diagnostics (if we have a previous pose)
        if (lastAppliedStartPosePedro != null) {
            double deltaX = Math.abs(candidate.getX() - lastAppliedStartPosePedro.getX());
            double deltaY = Math.abs(candidate.getY() - lastAppliedStartPosePedro.getY());
            RobotState.packet.put("init/deltaX", deltaX);
            RobotState.packet.put("init/deltaY", deltaY);
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

    private String formatMotif(AutoPrestartHelper.InitStatus status) {
        if (status == null || !status.hasMotif()) {
            return "No tag yet";
        }
        String tagText = status.motifTagId == null ? "n/a" : status.motifTagId.toString();
        return String.format("%s (tag %s)", status.motifPattern.name(), tagText);
    }

    private String formatRelocalize(AutoPrestartHelper.InitStatus status) {
        if (status == null || !status.hasRelocalized()) {
            return "Waiting for goal tag";
        }
        Pose pose = status.relocalizedPose;
        if (pose == null) {
            return "Tag locked but pose unavailable";
        }
        String tagText = status.relocalizeTagId > 0
                ? String.format("%d%s", status.relocalizeTagId, isOppositeGoalTag(status.relocalizeTagId) ? " (opp)" : "")
                : "unknown tag";
        long ageMs = status.relocalizePoseTimestampMs > 0L
                ? System.currentTimeMillis() - status.relocalizePoseTimestampMs
                : 0L;
        return String.format(
                "%s -> (%.1f, %.1f, %.0f°) age:%dms",
                tagText,
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                ageMs
        );
    }

    private void updateInitTelemetry(AutoPrestartHelper.InitStatus status) {
        RobotState.packet.put("init/alliance", activeAlliance.name());
        RobotState.packet.put("init/motif/name", status == null || status.motifPattern == null ? "UNKNOWN" : status.motifPattern.name());
        RobotState.packet.put("init/motif/tag", status == null || status.motifTagId == null ? -1 : status.motifTagId);
        RobotState.packet.put("init/relocalize/tag", status == null ? -1 : status.relocalizeTagId);
        RobotState.packet.put("init/relocalize/has_pose", status != null && status.relocalizedPose != null);
        if (status != null && status.relocalizedPose != null) {
            RobotState.packet.put("init/relocalize/x", status.relocalizedPose.getX());
            RobotState.packet.put("init/relocalize/y", status.relocalizedPose.getY());
            RobotState.packet.put("init/relocalize/heading_deg", Math.toDegrees(status.relocalizedPose.getHeading()));
            RobotState.packet.put("init/relocalize/age_ms", System.currentTimeMillis() - status.relocalizePoseTimestampMs);
        }
        RobotState.packet.put("init/start_pose/has_vision", status != null && status.startPoseFromVision != null);
        if (status != null && status.startPoseFromVision != null) {
            RobotState.packet.put("init/start_pose/x", status.startPoseFromVision.getX());
            RobotState.packet.put("init/start_pose/y", status.startPoseFromVision.getY());
            RobotState.packet.put("init/start_pose/heading_deg", Math.toDegrees(status.startPoseFromVision.getHeading()));
        }
        RobotState.packet.put("init/artifacts_detected", robot.intake.getArtifactCount());
        RobotState.packet.put("init/relocalize/readable", status == null ? "Waiting" : formatRelocalize(status));
        RobotState.packet.put("init/motif/readable", status == null ? "UNKNOWN" : formatMotif(status));
    }

    private void updateDriverStationTelemetry(AutoPrestartHelper.InitStatus status) {
        telemetry.clear();
        telemetry.addData("Alliance", activeAlliance.displayName());
        telemetry.addData("Motif", status == null ? "UNKNOWN" : formatMotif(status));
        telemetry.addData("Relocalize", status == null ? "Waiting" : formatRelocalize(status));
        telemetry.addData("Artifacts", "%d detected", robot.intake.getArtifactCount());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    private boolean isOppositeGoalTag(int tagId) {
        if (activeAlliance == Alliance.UNKNOWN) {
            return false;
        }
        return (activeAlliance == Alliance.BLUE && tagId == FieldConstants.RED_GOAL_TAG_ID)
                || (activeAlliance == Alliance.RED && tagId == FieldConstants.BLUE_GOAL_TAG_ID);
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
}
