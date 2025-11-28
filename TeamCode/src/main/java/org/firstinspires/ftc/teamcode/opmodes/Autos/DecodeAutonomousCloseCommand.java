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
        applyAlliance(activeAlliance, LocalizeCommand.getDefaultStartPose());
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
            // When alliance changes without vision, use LocalizeCommand default
            Pose defaultPose = lastDetectedStartPosePedro != null
                ? lastDetectedStartPosePedro
                : LocalizeCommand.getDefaultStartPose();
            applyAlliance(activeAlliance, defaultPose);
        }

        lastDetectedStartPosePedro = status.startPoseFromVision;
        if (shouldUpdateStartPose(lastDetectedStartPosePedro)) {
            applyAlliance(activeAlliance, lastDetectedStartPosePedro);
        }
    }



    /**
     * Applies alliance color and start pose to robot systems.
     * @param alliance Alliance color (BLUE or RED)
     * @param startPose Start pose (from vision or LocalizeCommand default)
     */
    private void applyAlliance(Alliance alliance, Pose startPose) {
        Alliance safeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);

        // Note: currentLayout is kept for proximity feedback target
        // Eventually this can be replaced with LocalizeCommand.getDefaultStartPose()
        currentLayout = AutoField.layoutForAlliance(activeAlliance);
        currentLayout.overrideStart(startPose);
        lastAppliedStartPosePedro = copyPose(startPose);

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

        // Vision relocalization status
        String visionStatus = computeVisionStatus(status);
        telemetry.addData(">> RELOCALIZE", visionStatus);

        // Motif detection status
        String motifStatus = computeMotifStatus(status);
        telemetry.addData(">> MOTIF", motifStatus);

        telemetry.addLine();

        // Current poses
        Pose followerPose = robot.drive.getFollower().getPose();
        Pose visionPose = status != null ? status.startPoseFromVision : null;

        telemetry.addData("Follower", "X=%.1f Y=%.1f θ=%.0f°",
            followerPose.getX(), followerPose.getY(), Math.toDegrees(followerPose.getHeading()));

        if (visionPose != null) {
            telemetry.addData("Vision", "X=%.1f Y=%.1f θ=%.0f°",
                visionPose.getX(), visionPose.getY(), Math.toDegrees(visionPose.getHeading()));

            // Show distance between vision and follower
            double dx = visionPose.getX() - followerPose.getX();
            double dy = visionPose.getY() - followerPose.getY();
            double distance = Math.hypot(dx, dy);
            double headingDelta = Math.toDegrees(Math.abs(visionPose.getHeading() - followerPose.getHeading()));
            while (headingDelta > 180) headingDelta -= 360;
            headingDelta = Math.abs(headingDelta);

            String deltaStatus = (distance < 3.0 && headingDelta < 10) ? "✓" : "⚠";
            telemetry.addData("Delta", "%s %.1f in, %.0f°", deltaStatus, distance, headingDelta);
        } else {
            telemetry.addData("Vision", "No tag detected");
        }

        telemetry.addLine();
        telemetry.addData("Alliance", activeAlliance.displayName());
        telemetry.addData("Relocalize Tag", status != null && status.relocalizeTagId > 0
            ? String.format("%d%s", status.relocalizeTagId, isOppositeGoalTag(status.relocalizeTagId) ? " (opp)" : "")
            : "none");

        if (status != null && status.hasMotif()) {
            telemetry.addData("Motif Tag", status.motifTagId != null
                ? String.format("%d", status.motifTagId)
                : "none");
            telemetry.addData("Pattern", status.motifPattern != null
                ? status.motifPattern.name()
                : "UNKNOWN");
        } else {
            telemetry.addData("Motif Tag", "none");
        }

        telemetry.addData("Artifacts", "%d detected", robot.intake.getArtifactCount());
        telemetry.addLine();
        telemetry.addLine("D-pad Left/Right override alliance");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    private String computeVisionStatus(AutoPrestartHelper.InitStatus status) {
        if (status == null || status.startPoseFromVision == null) {
            return "⚠ NO VISION - Using manual pose";
        }

        Pose followerPose = robot.drive.getFollower().getPose();
        Pose visionPose = status.startPoseFromVision;

        double dx = visionPose.getX() - followerPose.getX();
        double dy = visionPose.getY() - followerPose.getY();
        double distance = Math.hypot(dx, dy);
        double headingDelta = Math.toDegrees(Math.abs(visionPose.getHeading() - followerPose.getHeading()));
        while (headingDelta > 180) headingDelta -= 360;
        headingDelta = Math.abs(headingDelta);

        // Check vision freshness
        long ageMs = status.relocalizePoseTimestampMs > 0L
                ? System.currentTimeMillis() - status.relocalizePoseTimestampMs
                : Long.MAX_VALUE;

        if (ageMs > 2000) {
            return "⚠ VISION STALE - Move robot to see tag";
        }

        // Check vision quality
        if (distance < 3.0 && headingDelta < 10) {
            return "✓ VISION LOCKED - Ready to start";
        } else if (distance < 12.0 && headingDelta < 30) {
            return "⚠ VISION OK - Adjust placement for best results";
        } else {
            return "✗ VISION MISMATCH - Check robot placement";
        }
    }

    private String computeMotifStatus(AutoPrestartHelper.InitStatus status) {
        if (status == null || !status.hasMotif()) {
            return "⚠ NO MOTIF - Point at any AprilTag";
        }

        // Check motif detection freshness
        long ageMs = status.motifTimestampMs > 0L
                ? System.currentTimeMillis() - status.motifTimestampMs
                : Long.MAX_VALUE;

        if (ageMs > 2000) {
            return "⚠ MOTIF STALE - Point at an AprilTag";
        }

        // Motif detected and fresh
        if (status.motifPattern != null) {
            return String.format("✓ MOTIF LOCKED - %s", status.motifPattern.name());
        } else {
            return "✓ MOTIF DETECTED - Pattern unknown";
        }
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
