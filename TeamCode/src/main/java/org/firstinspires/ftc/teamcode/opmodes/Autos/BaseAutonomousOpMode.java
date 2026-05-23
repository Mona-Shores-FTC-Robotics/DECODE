package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoPrestartHelper;
import org.firstinspires.ftc.teamcode.util.ControlHubIdentifierUtil;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherModeSelector;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.List;

/**
 * Shared boilerplate for all DECODE autonomous OpModes.
 *
 * Subclasses supply three things:
 *   1. Where the robot starts (getStartX/Y/HeadingDeg).
 *   2. What command to run (buildAutoRoutine).
 *   3. Optional hooks onInit() / onStart() for per-routine setup.
 *
 * Everything else — hub caching, Scheduler lifecycle, vision relocalization,
 * init-time telemetry, stop-time cleanup — lives here exactly once.
 */
public abstract class BaseAutonomousOpMode extends OpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    // Accessible by subclasses for buildAutoRoutine() and getDefaultStartPose()
    protected Robot robot;
    protected Alliance activeAlliance = Alliance.BLUE;

    private AllianceSelector allianceSelector;
    private LauncherModeSelector modeSelector;
    private AutoPrestartHelper prestartHelper;

    private Pose lastAppliedStartPosePedro;
    private Pose lastDetectedStartPosePedro;

    private List<LynxModule> hubs;

    // -----------------------------------------------------------------------
    // Abstract contract — subclasses implement these
    // -----------------------------------------------------------------------

    /** Blue-side start X coordinate (Pedro field inches). Used for target-pose telemetry. */
    protected abstract double getStartX();

    /** Blue-side start Y coordinate (Pedro field inches). Used for target-pose telemetry. */
    protected abstract double getStartY();

    /** Blue-side start heading in degrees. Used for target-pose telemetry and prestart guard. */
    protected abstract double getStartHeadingDeg();

    /** Builds the auto command to schedule at match start. */
    protected abstract Command buildAutoRoutine(Pose startPoseOverride);

    // -----------------------------------------------------------------------
    // Optional hooks — override in subclasses that need extra init/start logic
    // -----------------------------------------------------------------------

    /**
     * Called in init() after activeAlliance is set, before applyAlliance().
     * Use for any per-routine setup that must happen before the start pose is applied
     * (e.g. loading a .pp path file).
     */
    protected void onInit() {}

    /**
     * Called at the start of start() before the auto routine is scheduled.
     * Use for any per-routine telemetry or last-moment configuration.
     */
    protected void onStart() {}

    /**
     * Returns the fallback start pose used when vision has not locked on.
     * Default: un-mirrored (Blue-side) coordinates from getStartX/Y/HeadingDeg.
     * Override in Michiana (or any .pp-based routine) to return an alliance-mirrored pose.
     */
    protected Pose getDefaultStartPose() {
        return new Pose(getStartX(), getStartY(), Math.toRadians(getStartHeadingDeg()));
    }

    // -----------------------------------------------------------------------
    // OpMode lifecycle
    // -----------------------------------------------------------------------

    @Override
    public final void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        robot = new Robot(hardwareMap);
        ControlHubIdentifierUtil.setRobotName(hardwareMap, telemetry);
        robot.attachPedroFollower();

        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();
        robot.initializeForAuto();
        robot.lighting.setDriveSubsystem(robot.drive);

        allianceSelector = new AllianceSelector(gamepad1, Alliance.UNKNOWN);
        modeSelector = new LauncherModeSelector(gamepad2, LauncherMode.DECODE);
        activeAlliance = allianceSelector.getSelectedAlliance();

        onInit(); // e.g. load .pp file for Michiana

        applyAlliance(activeAlliance, getDefaultStartPose());

        allianceSelector.applySelection(robot, robot.lighting);
        modeSelector.applySelection(robot.lighting);
        prestartHelper = new AutoPrestartHelper(robot, allianceSelector);
        updateExpectedStartPose();

        com.pedropathing.ivy.Scheduler.reset();
        com.pedropathing.ivy.Scheduler.schedule(
                robot.drive.periodic(),
                robot.launcher.periodic(),
                robot.intake.periodic(),
                robot.lighting.periodic(),
                robot.vision.periodic()
        );
    }

    @Override
    public final void init_loop() {
        AutoPrestartHelper.InitStatus initStatus = prestartHelper.update(activeAlliance);
        applyInitSelections(initStatus);
        modeSelector.updateDuringInit(robot.lighting);

        updateInitTelemetry(initStatus);
        updateDriverStationTelemetry(initStatus);
        robot.telemetry.publishLoopTelemetry(
                robot.drive, robot.launcher, robot.intake, robot.vision, robot.lighting,
                null, gamepad1, gamepad2,
                RobotState.getAlliance(), getRuntime(), Math.max(0.0, 150.0 - getRuntime()),
                telemetry, "AutoInit", true, null, 0, 0
        );
    }

    @Override
    public final void start() {
        RobotState.autoShotCount = 0;
        allianceSelector.lockSelection();
        modeSelector.lockSelection();

        onStart();

        Pose startPoseOverride = lastAppliedStartPosePedro != null ? lastAppliedStartPosePedro : null;
        buildAutoRoutine(startPoseOverride).schedule();

        robot.lighting.resumeLaneTracking();
        robot.intake.forwardRoller();
        robot.intake.setGateAllowArtifacts();
    }

    @Override
    public final void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        com.pedropathing.ivy.Scheduler.execute();
        RobotState.setHandoffPose(robot.drive.getFollower().getPose());
        publishTelemetry();
    }

    @Override
    public final void stop() {
        try {
            Pose finalPose = robot.drive.getFollower().getPose();
            if (finalPose != null && (finalPose.getX() != 0.0 || finalPose.getY() != 0.0)) {
                RobotState.setHandoffPose(finalPose);
            }
        } catch (Exception ignored) {
            // Follower may already be torn down — skip the handoff and keep stop() running.
        }

        com.pedropathing.ivy.Scheduler.reset();

        allianceSelector.unlockSelection();
        modeSelector.unlockSelection();

        // Wrap all subsystem stops — hub communication may already be terminating
        try { robot.launcher.abort(); } catch (Exception ignored) {}
        try { robot.drive.stop(); } catch (Exception ignored) {}
        try { robot.intake.stop(); } catch (Exception ignored) {}
        try { robot.vision.stop(); } catch (Exception ignored) {}
        try { robot.lighting.stop(); } catch (Exception ignored) {}
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /** Returns the alliance-mirrored target pose, used for telemetry display and prestart guard. */
    private Pose getTargetPose() {
        return AutoField.poseForAlliance(getStartX(), getStartY(), getStartHeadingDeg(), activeAlliance);
    }

    private void applyInitSelections(AutoPrestartHelper.InitStatus status) {
        if (status == null) return;

        if (status.alliance != activeAlliance) {
            activeAlliance = status.alliance;
            Pose defaultPose = lastDetectedStartPosePedro != null
                    ? lastDetectedStartPosePedro
                    : getDefaultStartPose();
            applyAlliance(activeAlliance, defaultPose);
        }

        lastDetectedStartPosePedro = status.startPoseFromVision;
        if (shouldUpdateStartPose(lastDetectedStartPosePedro)) {
            applyAlliance(activeAlliance, lastDetectedStartPosePedro);
        }
    }

    private void applyAlliance(Alliance alliance, Pose startPose) {
        activeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        robot.setAlliance(activeAlliance);
        lastAppliedStartPosePedro = copyPose(startPose);
        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
        updateExpectedStartPose();
    }

    private void updateExpectedStartPose() {
        if (prestartHelper == null) return;
        prestartHelper.setExpectedStartPose(getTargetPose());
    }

    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) return false;

        double fieldWidthIn = FieldConstants.FIELD_WIDTH_INCHES;
        if (candidate.getX() < 0 || candidate.getX() > fieldWidthIn ||
                candidate.getY() < 0 || candidate.getY() > fieldWidthIn) {
            return false;
        }

        if (lastAppliedStartPosePedro != null) {
            RobotState.packet.put("init/deltaX", Math.abs(candidate.getX() - lastAppliedStartPosePedro.getX()));
            RobotState.packet.put("init/deltaY", Math.abs(candidate.getY() - lastAppliedStartPosePedro.getY()));
        }
        return true;
    }

    private Pose copyPose(Pose pose) {
        return pose == null ? null : new Pose(pose.getX(), pose.getY(), pose.getHeading());
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
        telemetryHeader(status);
        telemetry.addLine();
        telemetryPoseBlock(status);
        telemetry.addLine();
        telemetryStatusBlock(status);
        telemetryControlHints();
        telemetry.update();
    }

    /** Top-of-screen status line: vision relocalize state + motif state. */
    private void telemetryHeader(AutoPrestartHelper.InitStatus status) {
        telemetry.addData(">> RELOCALIZE", computeVisionStatus(status));
        telemetry.addData(">> MOTIF", computeMotifStatus(status));
    }

    /** Target / follower / vision pose comparison with delta indicator. */
    private void telemetryPoseBlock(AutoPrestartHelper.InitStatus status) {
        Pose followerPose = robot.drive.getFollower().getPose();
        Pose visionPose = status != null ? status.startPoseFromVision : null;
        Pose targetPose = getTargetPose();

        telemetry.addData("Target", "X=%.1f Y=%.1f θ=%.0f°",
                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
        telemetry.addData("Follower", "X=%.1f Y=%.1f θ=%.0f°",
                followerPose.getX(), followerPose.getY(), Math.toDegrees(followerPose.getHeading()));

        double followerDist = Math.hypot(
                followerPose.getX() - targetPose.getX(),
                followerPose.getY() - targetPose.getY());
        double followerHdelta = normalizeHeadingDelta(
                Math.toDegrees(Math.abs(followerPose.getHeading() - targetPose.getHeading())));
        telemetry.addData("  Delta", "%s %.1f in, %.0f°",
                (followerDist < 3.0 && followerHdelta < 10) ? "✓" : "⚠",
                followerDist, followerHdelta);

        if (visionPose != null) {
            telemetry.addData("Vision", "X=%.1f Y=%.1f θ=%.0f°",
                    visionPose.getX(), visionPose.getY(), Math.toDegrees(visionPose.getHeading()));
        } else {
            telemetry.addData("Vision", "No tag detected");
        }
    }

    /** Alliance, relocalize tag, motif info, artifact count, launcher mode. */
    private void telemetryStatusBlock(AutoPrestartHelper.InitStatus status) {
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
    }

    /** Driver/operator control hints shown at the bottom of init telemetry. */
    private void telemetryControlHints() {
        telemetry.addLine();
        telemetry.addLine("Driver D-pad: Alliance (L=BLUE R=RED)");
        telemetry.addLine("Operator D-pad: Mode (L=THRU R=SEQ)");
        telemetry.addLine("Press START when ready");
    }

    private String computeVisionStatus(AutoPrestartHelper.InitStatus status) {
        if (status == null || status.startPoseFromVision == null) {
            return "⚠ NO VISION - Using manual pose";
        }

        long ageMs = status.relocalizePoseTimestampMs > 0L
                ? System.currentTimeMillis() - status.relocalizePoseTimestampMs
                : Long.MAX_VALUE;
        if (ageMs > 2000) {
            return "⚠ VISION STALE - Move robot to see tag";
        }

        Pose visionPose = status.startPoseFromVision;
        Pose targetPose = getTargetPose();
        double distance = Math.hypot(
                visionPose.getX() - targetPose.getX(),
                visionPose.getY() - targetPose.getY());
        double headingDelta = normalizeHeadingDelta(
                Math.toDegrees(Math.abs(visionPose.getHeading() - targetPose.getHeading())));

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
        if (status.motifPattern != null) {
            return String.format("✓ MOTIF SEEN - %s", status.motifPattern.name());
        }
        return "✓ MOTIF DETECTED - Pattern unknown";
    }

    private boolean isOppositeGoalTag(int tagId) {
        if (activeAlliance == Alliance.UNKNOWN) return false;
        return (activeAlliance == Alliance.BLUE && tagId == FieldConstants.RED_GOAL_TAG_ID)
                || (activeAlliance == Alliance.RED && tagId == FieldConstants.BLUE_GOAL_TAG_ID);
    }

    private String formatMotif(AutoPrestartHelper.InitStatus status) {
        if (status == null || !status.hasMotif()) return "No tag yet";
        String tagText = status.motifTagId == null ? "n/a" : status.motifTagId.toString();
        return String.format("%s (tag %s)", status.motifPattern.name(), tagText);
    }

    private String formatRelocalize(AutoPrestartHelper.InitStatus status) {
        if (status == null || !status.hasRelocalized()) return "Waiting for goal tag";
        Pose pose = status.relocalizedPose;
        if (pose == null) return "Tag locked but pose unavailable";
        String tagText = status.relocalizeTagId > 0
                ? String.format("%d%s", status.relocalizeTagId, isOppositeGoalTag(status.relocalizeTagId) ? " (opp)" : "")
                : "unknown tag";
        long ageMs = status.relocalizePoseTimestampMs > 0L
                ? System.currentTimeMillis() - status.relocalizePoseTimestampMs
                : 0L;
        return String.format("%s -> (%.1f, %.1f, %.0f°) age:%dms",
                tagText, pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()), ageMs);
    }

    private void publishTelemetry() {
        Pose currentPose = robot.drive.getFollower().getPose();
        if (currentPose != null) {
            RobotState.packet.put("Auto/Pose/X", currentPose.getX());
            RobotState.packet.put("Auto/Pose/Y", currentPose.getY());
            RobotState.packet.put("Auto/Pose/Heading (deg)", Math.toDegrees(currentPose.getHeading()));
            RobotState.packet.put("Auto/Pose/Heading (rad)", currentPose.getHeading());
            RobotState.packet.put("Auto/Alliance", activeAlliance.name());
            telemetry.addData("Pose", "X=%.2f Y=%.2f θ=%.1f° (%s)",
                    currentPose.getX(), currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()), activeAlliance.name());
        }
        RobotState.packet.put("Auto/shotsTotal", RobotState.autoShotCount);
        robot.telemetry.publishLoopTelemetry(
                robot.drive, robot.launcher, robot.intake, robot.vision, robot.lighting,
                null, gamepad1, gamepad2,
                RobotState.getAlliance(), getRuntime(), Math.max(0.0, 150.0 - getRuntime()),
                telemetry, "Auto", true, null, 0, 0
        );
    }

    /** Folds a raw heading delta (degrees, possibly > 180) into [0, 180]. */
    private static double normalizeHeadingDelta(double delta) {
        while (delta > 180) delta -= 360;
        return Math.abs(delta);
    }
}
