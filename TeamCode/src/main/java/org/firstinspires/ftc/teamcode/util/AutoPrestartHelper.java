package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;

import java.util.Optional;

/**
 * Shared helper for autonomous init loops.
 * Polls vision for alliance inference, re-localizes on goal tags,
 * and captures Decode motif tags while the robot is stationary.
 *
 * Drop this into any auto:
 * <pre>
 *     AutoPrestartHelper.InitStatus status = helper.update(activeAlliance);
 *     if (shouldUpdateStartPose(status.startPoseFromVision)) { ... }
 * </pre>
 */
public class AutoPrestartHelper {

    private static final long RELOCALIZE_COOLDOWN_MS = 300L;

    private final Robot robot;
    private final AllianceSelector allianceSelector;

    private Pose lastRelocalizedPose;
    private int lastRelocalizeTagId = -1;
    private long lastRelocalizePoseTimestampMs = 0L;

    private MotifPattern observedMotif = RobotState.getMotif();
    private Integer observedMotifTagId;

    public AutoPrestartHelper(Robot robot, AllianceSelector allianceSelector) {
        this.robot = robot;
        this.allianceSelector = allianceSelector;
    }

    public InitStatus update(Alliance activeAlliance) {
        // Keep Limelight heading seeded even if drive.periodic() has not run yet
        syncHeadingForVision();

        // Poll Limelight once per init loop so snapshots stay fresh
        robot.vision.periodic();

        Alliance selectedAlliance = allianceSelector.updateDuringInit(robot.vision, robot, robot.lighting);
        Pose startPoseFromVision = allianceSelector.getLastSnapshot()
                .flatMap(VisionSubsystemLimelight.TagSnapshot::getRobotPosePedroMT1)
                .map(AutoPrestartHelper::copyPose)
                .orElse(null);

        maybeRelocalizeFromGoalTag(activeAlliance);
        maybeDetectMotif();

        return new InitStatus(
                selectedAlliance,
                startPoseFromVision,
                lastRelocalizedPose,
                lastRelocalizeTagId,
                lastRelocalizePoseTimestampMs,
                observedMotif,
                observedMotifTagId
        );
    }

    private void syncHeadingForVision() {
        try {
            double headingRad = robot.drive.getFollower().getHeading();
            robot.vision.setRobotHeading(headingRad);
        } catch (Exception ignored) {
            // Follower may not be ready during very early init
        }
    }

    private void maybeRelocalizeFromGoalTag(Alliance activeAlliance) {
        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = robot.vision.getLastSnapshot();
        if (!snapshotOpt.isPresent()) {
            return;
        }

        VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
        if (!isGoalTag(snapshot.tagId)) {
            return;
        }

        long poseTimestamp = robot.vision.getLastPoseTimestampMs();
        if (poseTimestamp <= 0L || poseTimestamp <= lastRelocalizePoseTimestampMs) {
            return; // Already processed this pose
        }

        long now = System.currentTimeMillis();
        if (now - lastRelocalizePoseTimestampMs < RELOCALIZE_COOLDOWN_MS) {
            return;
        }

        boolean updated = robot.drive.forceRelocalizeFromVision();
        if (!updated) {
            return;
        }

        lastRelocalizedPose = copyPose(robot.drive.getFollower().getPose());
        lastRelocalizeTagId = snapshot.tagId;
        lastRelocalizePoseTimestampMs = poseTimestamp;

        boolean usingOppositeGoal = isOppositeGoal(snapshot.tagId, activeAlliance);
        robot.drive.visionRelocalizeStatus = usingOppositeGoal
                ? "Init re-localized on opposite goal tag"
                : "Init re-localized on goal tag";
        robot.drive.visionRelocalizeStatusMs = now;
    }

    private void maybeDetectMotif() {
        Optional<Integer> motifTag = robot.vision.findMotifTagId();
        if (!motifTag.isPresent()) {
            return;
        }

        int tagId = motifTag.get();
        if (observedMotifTagId != null && observedMotifTagId == tagId && observedMotif != MotifPattern.UNKNOWN) {
            return; // Already latched
        }

        observedMotifTagId = tagId;
        observedMotif = MotifPattern.fromTagId(tagId);
        RobotState.setMotif(observedMotif);

        if (robot.lighting != null) {
            robot.lighting.showMotifPattern(observedMotif.getLaneColors());
        }
    }

    private static boolean isGoalTag(int tagId) {
        return tagId == FieldConstants.BLUE_GOAL_TAG_ID || tagId == FieldConstants.RED_GOAL_TAG_ID;
    }

    private static boolean isOppositeGoal(int tagId, Alliance alliance) {
        if (alliance == null || alliance == Alliance.UNKNOWN) {
            return false;
        }
        return (alliance == Alliance.BLUE && tagId == FieldConstants.RED_GOAL_TAG_ID)
                || (alliance == Alliance.RED && tagId == FieldConstants.BLUE_GOAL_TAG_ID);
    }

    private static Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static final class InitStatus {
        public final Alliance alliance;
        public final Pose startPoseFromVision;
        public final Pose relocalizedPose;
        public final int relocalizeTagId;
        public final long relocalizePoseTimestampMs;
        public final MotifPattern motifPattern;
        public final Integer motifTagId;

        public InitStatus(
                Alliance alliance,
                Pose startPoseFromVision,
                Pose relocalizedPose,
                int relocalizeTagId,
                long relocalizePoseTimestampMs,
                MotifPattern motifPattern,
                Integer motifTagId
        ) {
            this.alliance = alliance;
            this.startPoseFromVision = startPoseFromVision;
            this.relocalizedPose = relocalizedPose;
            this.relocalizeTagId = relocalizeTagId;
            this.relocalizePoseTimestampMs = relocalizePoseTimestampMs;
            this.motifPattern = motifPattern == null ? MotifPattern.UNKNOWN : motifPattern;
            this.motifTagId = motifTagId;
        }

        public boolean hasRelocalized() {
            return relocalizedPose != null;
        }

        public boolean hasMotif() {
            return motifPattern != null && motifPattern != MotifPattern.UNKNOWN;
        }
    }
}
