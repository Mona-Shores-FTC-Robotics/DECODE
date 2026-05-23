package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveFusionConfig;


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

    /** Minimum time between vision-driven relocalizations during auto pre-start to avoid jitter. */
    private static final long RELOCALIZE_COOLDOWN_MS = 300L;

    private final Robot robot;
    private final AllianceSelector allianceSelector;

    private Pose lastRelocalizedPose;
    private int lastRelocalizeTagId = -1;
    private long lastRelocalizePoseTimestampMs = 0L;

    private MotifPattern observedMotif = RobotState.getMotif();
    private Integer observedMotifTagId;

    /** Expected start pose in world/Pedro coordinates (alliance-mirrored).
     *  Used as reference for jump safeguards during init relocalization. */
    private Pose expectedStartPose;

    public AutoPrestartHelper(Robot robot, AllianceSelector allianceSelector) {
        this.robot = robot;
        this.allianceSelector = allianceSelector;
    }

    /**
     * Sets the expected start pose for jump safeguards.
     * Call this whenever alliance changes to update the mirrored pose.
     *
     * @param expectedPose The expected start pose in world/Pedro coordinates (already alliance-mirrored)
     */
    public void setExpectedStartPose(Pose expectedPose) {
        this.expectedStartPose = expectedPose;
    }

    public InitStatus update(Alliance activeAlliance) {
        // Keep Limelight heading seeded even if drive.periodic() has not run yet
        syncHeadingForVision();

        // Poll Limelight once per init loop so snapshots stay fresh

        Alliance selectedAlliance = allianceSelector.updateDuringInit(robot.vision, robot, robot.lighting);
        if (robot.lighting != null) {
            robot.lighting.showSolidAlliance(selectedAlliance);
        }
        VisionSubsystemLimelight.TagSnapshot lastSnap = allianceSelector.getLastSnapshot();
        Pose mt1 = lastSnap != null ? lastSnap.getRobotPosePedroMT1() : null;
        Pose startPoseFromVision = mt1 != null ? copyPose(mt1) : null;
        // Freshness of the vision pose itself (not the relocalize event) — so the
        // status line reports STALE only when vision actually stops seeing a tag,
        // independent of whether init relocalization is enabled.
        long visionPoseTimestampNs = (lastSnap != null && startPoseFromVision != null)
                ? lastSnap.capturedAtNs : 0L;

        maybeRelocalizeFromGoalTag(activeAlliance);
        maybeDetectMotif();

        return new InitStatus(
                selectedAlliance,
                startPoseFromVision,
                visionPoseTimestampNs,
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
        // Hold the seeded start pose during init unless explicitly opted in. A vision
        // read taken while the robot is being placed should not move the start pose.
        if (!DriveFusionConfig.relocalizeDuringInit) {
            return;
        }
        VisionSubsystemLimelight.TagSnapshot snapshot = robot.vision.getLastSnapshot();
        if (snapshot == null) {
            return;
        }

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

        // Use expected start pose as reference for jump safeguards.
        // This prevents vision from wildly jumping the pose during init.
        // If no expected pose is set, falls back to no safeguard (not recommended).
        boolean updated = robot.drive.forceRelocalizeFromVisionTrusted(expectedStartPose);
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
        Integer motifTag = robot.vision.findMotifTagId();
        if (motifTag == null) {
            return;
        }

        int tagId = motifTag;
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
        /** nanoTime when the vision pose was captured (0 if no vision pose). */
        public final long visionPoseTimestampNs;
        public final Pose relocalizedPose;
        public final int relocalizeTagId;
        public final long relocalizePoseTimestampMs;
        public final MotifPattern motifPattern;
        public final Integer motifTagId;

        public InitStatus(
                Alliance alliance,
                Pose startPoseFromVision,
                long visionPoseTimestampNs,
                Pose relocalizedPose,
                int relocalizeTagId,
                long relocalizePoseTimestampMs,
                MotifPattern motifPattern,
                Integer motifTagId
        ) {
            this.alliance = alliance;
            this.startPoseFromVision = startPoseFromVision;
            this.visionPoseTimestampNs = visionPoseTimestampNs;
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
