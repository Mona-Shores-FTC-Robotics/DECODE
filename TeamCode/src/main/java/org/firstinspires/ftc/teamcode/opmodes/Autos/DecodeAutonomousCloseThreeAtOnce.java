package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.CloseThreeAtOnceCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherModeSelector;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.AutoPrestartHelper;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Close Three At Once", group = "Command")
@Configurable
public class DecodeAutonomousCloseThreeAtOnce extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    private Robot robot;
    private AllianceSelector allianceSelector;
    private LauncherModeSelector modeSelector;
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
        GamepadEx operatorPad = new GamepadEx(() -> gamepad2);

        allianceSelector = new AllianceSelector(driverPad, Alliance.UNKNOWN);
        modeSelector = new LauncherModeSelector(operatorPad, LauncherMode.THROUGHPUT);

        activeAlliance = allianceSelector.getSelectedAlliance();

        applyAlliance(activeAlliance, CloseThreeAtOnceCommand.getDefaultStartPose());

        allianceSelector.applySelection(robot, robot.lighting);
        modeSelector.applySelection(robot.lighting);
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

        // Update launcher mode selection (operator dpad left/right)
        modeSelector.updateDuringInit(robot.lighting);

//        updateProximityFeedback();
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
        modeSelector.lockSelection();

        // Build auto with vision-detected start pose (or null to use LocalizeCommand defaults)
        // Use follower's current pose as the source of truth (it was set from vision if available)
        Pose startPoseOverride = lastAppliedStartPosePedro != null
            ? lastAppliedStartPosePedro
            : null;

        Command autoRoutine = CloseThreeAtOnceCommand.create(robot, activeAlliance, startPoseOverride);

        CommandManager.INSTANCE.scheduleCommand(autoRoutine);

        robot.lighting.resumeLaneTracking();
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
        modeSelector.unlockSelection();
        BindingManager.reset();
        robot.launcher.abort();
        robot.drive.stop();
        robot.vision.stop();

        // Save final pose for TeleOp transition
        RobotState.setHandoffPose(robot.drive.getFollower().getPose());
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
                : CloseThreeAtOnceCommand.getDefaultStartPose();
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

        // Current poses - compare to target
        Pose followerPose = robot.drive.getFollower().getPose();
        Pose visionPose = status != null ? status.startPoseFromVision : null;

        // Get default/target start pose (mirrored for current alliance)
        Pose targetPose = AutoField.poseForAlliance(
                CloseThreeAtOnceCommand.waypoints.startX,
                CloseThreeAtOnceCommand.waypoints.startY,
                CloseThreeAtOnceCommand.waypoints.startHeading,
            activeAlliance
        );

        telemetry.addData("Target", "X=%.1f Y=%.1f θ=%.0f°",
            targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));

        telemetry.addData("Follower", "X=%.1f Y=%.1f θ=%.0f°",
            followerPose.getX(), followerPose.getY(), Math.toDegrees(followerPose.getHeading()));

        // Show delta from follower to target
        double followerDx = followerPose.getX() - targetPose.getX();
        double followerDy = followerPose.getY() - targetPose.getY();
        double followerDistance = Math.hypot(followerDx, followerDy);
        double followerHeadingDelta = Math.toDegrees(Math.abs(followerPose.getHeading() - targetPose.getHeading()));
        while (followerHeadingDelta > 180) followerHeadingDelta -= 360;
        followerHeadingDelta = Math.abs(followerHeadingDelta);

        String followerDeltaStatus = (followerDistance < 3.0 && followerHeadingDelta < 10) ? "✓" : "⚠";
        telemetry.addData("  Delta", "%s %.1f in, %.0f°", followerDeltaStatus, followerDistance, followerHeadingDelta);

        if (visionPose != null) {
            telemetry.addData("Vision", "X=%.1f Y=%.1f θ=%.0f°",
                visionPose.getX(), visionPose.getY(), Math.toDegrees(visionPose.getHeading()));

            // Show delta from vision to target
            double visionDx = visionPose.getX() - targetPose.getX();
            double visionDy = visionPose.getY() - targetPose.getY();
            double visionDistance = Math.hypot(visionDx, visionDy);
            double visionHeadingDelta = Math.toDegrees(Math.abs(visionPose.getHeading() - targetPose.getHeading()));
            while (visionHeadingDelta > 180) visionHeadingDelta -= 360;
            visionHeadingDelta = Math.abs(visionHeadingDelta);

//            String visionDeltaStatus = (visionDistance < 3.0 && visionHeadingDelta < 10) ? "✓" : "⚠";
//            telemetry.addData("  Delta", "%s %.1f in, %.0f°", visionDeltaStatus, visionDistance, visionHeadingDelta);
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
        telemetry.addData("Launcher Mode", modeSelector.getDisplayText());
        telemetry.addLine();
        telemetry.addLine("Driver D-pad: Alliance (L=BLUE R=RED)");
        telemetry.addLine("Operator D-pad: Mode (L=THRU R=SEQ)");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    private String computeVisionStatus(AutoPrestartHelper.InitStatus status) {
        if (status == null || status.startPoseFromVision == null) {
            return "⚠ NO VISION - Using manual pose";
        }

        // Check vision freshness
        long ageMs = status.relocalizePoseTimestampMs > 0L
                ? System.currentTimeMillis() - status.relocalizePoseTimestampMs
                : Long.MAX_VALUE;

        if (ageMs > 2000) {
            return "⚠ VISION STALE - Move robot to see tag";
        }

        // Compare vision pose to target (not to follower)
        Pose visionPose = status.startPoseFromVision;
        Pose targetPose = AutoField.poseForAlliance(
                CloseThreeAtOnceCommand.waypoints.startX,
                CloseThreeAtOnceCommand.waypoints.startY,
                CloseThreeAtOnceCommand.waypoints.startHeading,
            activeAlliance
        );

        double dx = visionPose.getX() - targetPose.getX();
        double dy = visionPose.getY() - targetPose.getY();
        double distance = Math.hypot(dx, dy);
        double headingDelta = Math.toDegrees(Math.abs(visionPose.getHeading() - targetPose.getHeading()));
        while (headingDelta > 180) headingDelta -= 360;
        headingDelta = Math.abs(headingDelta);

        // Check vision quality against target
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
            return "⚠ NO MOTIF - Point at Motif AprilTag";
        }

        // Motif detected and fresh
        if (status.motifPattern != null) {
            return String.format("✓ MOTIF SEEN - %s", status.motifPattern.name());
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
