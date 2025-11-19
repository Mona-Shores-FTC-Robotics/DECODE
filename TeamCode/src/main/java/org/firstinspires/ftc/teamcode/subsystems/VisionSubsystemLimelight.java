package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.AutoField;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.RobotState;

import dev.nextftc.core.subsystems.Subsystem;

import java.util.List;
import java.util.Optional;

/**
 * Limelight-backed vision subsystem that mirrors the public API of the legacy VisionPortal-based
 * implementation so existing OpModes can switch to Limelight without broader structural changes.
 */
public class VisionSubsystemLimelight implements Subsystem {

    private static final long ODOMETRY_RESET_TIMEOUT_MS = 3000L;


    public enum VisionState {
        OFF,
        STREAMING
    }

    public final Limelight3A limelight;
    private final Telemetry telemetry;

    private VisionState state = VisionState.OFF;
    private Alliance activeAlliance = Alliance.UNKNOWN;

    private TagSnapshot lastSnapshot;
    private Pose lastRobotPose;
    private Pose lastRobotPoseFtc;
    private boolean odometryUpdatePending;
    private long lastSnapshotTimestampMs = 0L;
    private int lastSeenTagId = -1;

    private double lastPeriodicMs = 0.0;

    // Throttle vision polling to 20Hz (50ms) instead of every loop
    private static final long VISION_POLL_INTERVAL_MS = 50L;
    private long lastVisionPollTimeMs = 0L;

    // Cache snapshot staleness check to avoid redundant System.currentTimeMillis() calls
    // in multiple @AutoLogOutput methods each loop
    private boolean snapshotStaleCache = false;
    private long lastStaleCacheUpdateMs = 0L;

    // MegaTag2: Store current robot heading for IMU-fused localization
    private double currentHeadingRad = 0.0;
    private boolean hasValidHeading = false;

    // Diagnostic mode: When true, external code controls heading updates (for testing different offsets)
    private boolean diagnosticMode = false;

    public VisionSubsystemLimelight(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public VisionSubsystemLimelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void initialize() {
        limelight.pipelineSwitch(0);
        limelight.start();
        state = VisionState.STREAMING;
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        long nowMs = System.currentTimeMillis();

        // MegaTag2: Update Limelight with current robot heading for IMU-fused localization
        // This must be called every loop before requesting pose estimates
        // Skip in diagnostic mode to allow external control for testing different offsets
        if (hasValidHeading && !diagnosticMode) {
            // Convert Pedro heading to FTC heading (Pedro - 90°)
            // Fixed 180° heading error: changed from +90° to -90°
            // Limelight with orientation=180° expects FTC coordinate frame heading
            double pedroHeadingDeg = Math.toDegrees(currentHeadingRad);
            double ftcHeadingDeg = AngleUnit.normalizeDegrees(pedroHeadingDeg - 90.0);

            limelight.updateRobotOrientation(ftcHeadingDeg);
        }

        // Throttle Limelight polling to 20Hz (50ms) to reduce loop time
        if (nowMs - lastVisionPollTimeMs >= VISION_POLL_INTERVAL_MS) {
            updateLatestSnapshot();
            lastVisionPollTimeMs = nowMs;
        }

        // Update staleness cache once per loop for use by @AutoLogOutput methods
        // This avoids 16+ redundant System.currentTimeMillis() calls in logging
        updateStalenessCache(nowMs);

        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    private void updateStalenessCache(long nowMs) {
        snapshotStaleCache = lastSnapshotTimestampMs > 0L && (nowMs - lastSnapshotTimestampMs) > ODOMETRY_RESET_TIMEOUT_MS;
        lastStaleCacheUpdateMs = nowMs;
    }

    public void stop() {
        limelight.stop();
        state = VisionState.OFF;
        clearSnapshot();
    }

    /**
     * Enable diagnostic mode to allow external code to control heading updates.
     * Used by diagnostic OpModes to test different heading offsets without interference.
     */
    public void setDiagnosticMode(boolean enabled) {
        diagnosticMode = enabled;
    }

    public boolean isDiagnosticMode() {
        return diagnosticMode;
    }

    public VisionState getState() {
        return state;
    }

    public boolean isStreaming() {
        return state == VisionState.STREAMING;
    }

    public boolean hasValidTag() {
        refreshSnapshotIfStale();
        return lastSnapshot != null;
    }

    public long getLastPoseTimestampMs() {
        refreshSnapshotIfStale();
        return lastSnapshotTimestampMs;
    }

    public int getCurrentTagId() {
        refreshSnapshotIfStale();
        return lastSnapshot == null ? -1 : lastSnapshot.getTagId();
    }

    public Optional<Pose> getRobotPoseFromTag() {
        refreshSnapshotIfStale();
        return Optional.ofNullable(lastRobotPose);
    }

    public Optional<Pose> getRobotPoseFromTagFtc() {
        refreshSnapshotIfStale();
        return Optional.ofNullable(lastRobotPoseFtc);
    }

    public boolean shouldUpdateOdometry() {
        refreshSnapshotIfStale();
        return odometryUpdatePending;
    }

    public void markOdometryUpdated() {
        odometryUpdatePending = false;
    }

    public Optional<Pose> getTargetGoalPose() {
        Alliance alliance = activeAlliance;
        if (alliance == Alliance.UNKNOWN) {
            alliance = RobotState.getAlliance();
        }
        switch (alliance) {
            case BLUE:
                return Optional.of(FieldConstants.getBlueBasketTarget());
            case RED:
                return Optional.of(FieldConstants.getRedBasketTarget());
            default:
                return Optional.empty();
        }
    }

    public Optional<Double> getAimAngle() {
        Optional<Pose> poseOpt = getRobotPoseFromTag();
        Optional<Pose> target = getTargetGoalPose();
        if (!poseOpt.isPresent() || !target.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(FieldConstants.getAimAngleTo(poseOpt.get(), target.get()));
    }

    /**
     * Gets the vision-based aiming error in radians (robot-relative).
     * This is the horizontal offset (tx) from the Limelight, converted to radians.
     * Positive tx means target is to the right, negative means left.
     *
     * @return The aiming error in radians, or empty if no valid tag
     */
    public Optional<Double> getVisionAimErrorRad() {
        refreshSnapshotIfStale();
        if (lastSnapshot == null) {
            return Optional.empty();
        }
        double tx = lastSnapshot.getTxDegrees();
        if (Double.isNaN(tx)) {
            return Optional.empty();
        }
        // Negate tx because positive tx (target right) requires negative turn (clockwise)
        return Optional.of(-Math.toRadians(tx));
    }

    public Optional<TagSnapshot> findAllianceSnapshot(Alliance preferredAlliance) {
        TagSnapshot snapshot = selectSnapshot(preferredAlliance);
        if (snapshot == null) {
            clearSnapshot();
            return Optional.empty();
        }
        onSnapshotUpdated(snapshot);
        return Optional.of(snapshot);
    }

    public Optional<TagSnapshot> getLastSnapshot() {
        refreshSnapshotIfStale();
        return Optional.ofNullable(lastSnapshot);
    }

    public void setAlliance(Alliance alliance) {
        activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        RobotState.setAlliance(activeAlliance);
    }

    public Alliance getAlliance() {
        return activeAlliance;
    }

    /**
     * Updates the robot's current heading for MegaTag2 IMU-fused localization.
     * Should be called every loop with the latest odometry heading.
     *
     * @param headingRad Robot heading in radians (Pedro coordinate system)
     */
    public void setRobotHeading(double headingRad) {
        currentHeadingRad = headingRad;
        hasValidHeading = true;
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    private TagSnapshot selectSnapshot(Alliance preferredAlliance) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        // MegaTag2: Find best valid basket tag and use MT2 fused pose
        TagSnapshot bestSnapshot = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();

            // MegaTag2: Only use basket AprilTags (ID 20 for blue, 24 for red)
            // Ignore all other tags to eliminate false positives and improve accuracy
            if (tagId != FieldConstants.BLUE_GOAL_TAG_ID && tagId != FieldConstants.RED_GOAL_TAG_ID) {
                continue;
            }

            Alliance detectionAlliance = mapTagToAlliance(tagId);
            if (preferredAlliance != null && preferredAlliance != Alliance.UNKNOWN
                    && detectionAlliance != preferredAlliance) {
                continue;
            }
            if (detectionAlliance == Alliance.UNKNOWN && preferredAlliance != Alliance.UNKNOWN) {
                continue;
            }

            // Check if this is the best tag candidate
            double score = getTagScore(fiducial);
            if (score > bestScore) {
                bestScore = score;
                // Use MegaTag2 fused pose instead of individual tag pose
                bestSnapshot = new TagSnapshot(detectionAlliance, tagId, result, result.getTx(), result.getTy(), result.getTa());
            }
        }

        if (bestSnapshot == null && preferredAlliance != null && preferredAlliance != Alliance.UNKNOWN) {
            // Retry without alliance filtering so we still capture pose data.
            return selectSnapshot(Alliance.UNKNOWN);
        }
        return bestSnapshot;
    }

    private double getTagScore(LLResultTypes.FiducialResult fiducial) {
        double area = getTargetAreaSafe(fiducial);
        if (!Double.isNaN(area) && area > 0.0) {
            return area;
        }
        double ambiguity = getPoseAmbiguitySafe(fiducial);
        return Double.isNaN(ambiguity) ? 0.0 : (1.0 - ambiguity);
    }

    private void onSnapshotUpdated(TagSnapshot snapshot) {
        long now = System.currentTimeMillis();
        boolean stale = lastSnapshotTimestampMs == 0L || (now - lastSnapshotTimestampMs) > ODOMETRY_RESET_TIMEOUT_MS;
        lastSnapshot = snapshot;
        lastSnapshotTimestampMs = now;
        Optional<Pose> poseOpt = snapshot.getRobotPose();
        Optional<Pose> ftcPoseOpt = snapshot.getFtcPose();
        if (poseOpt.isPresent()) {
            lastRobotPose = poseOpt.get();
            lastRobotPoseFtc = ftcPoseOpt.orElse(null);
            boolean newTag = snapshot.getTagId() != lastSeenTagId;
            if (newTag || stale) {
                odometryUpdatePending = true;
                lastSeenTagId = snapshot.getTagId();
            }
        }
        if (telemetry != null) {
            telemetry.addData("Vision/TagId", snapshot.getTagId());
            telemetry.addData("Vision/X", snapshot.getRobotX());
            telemetry.addData("Vision/Y", snapshot.getRobotY());
            telemetry.addData("Vision/Yaw", snapshot.getRobotYaw());
        }
    }

    private void refreshSnapshotIfStale() {
        if (lastSnapshot == null) {
            return;
        }
        // Use cached staleness check if recently updated (avoids redundant time calls)
        boolean isStale;
        if (System.currentTimeMillis() - lastStaleCacheUpdateMs < 5L) {
            isStale = snapshotStaleCache;
        } else {
            isStale = lastSnapshotTimestampMs > 0L && (System.currentTimeMillis() - lastSnapshotTimestampMs) > ODOMETRY_RESET_TIMEOUT_MS;
        }
        if (isStale) {
            odometryUpdatePending = false;
            lastSnapshot = null;
            lastRobotPose = null;
            lastRobotPoseFtc = null;
            lastSeenTagId = -1;
        }
    }

    private void updateLatestSnapshot() {
        Alliance alliance = activeAlliance;
        if (alliance == Alliance.UNKNOWN) {
            alliance = RobotState.getAlliance();
        }
        TagSnapshot snapshot = selectSnapshot(alliance);
        if (snapshot != null) {
            onSnapshotUpdated(snapshot);
        } else {
            refreshSnapshotIfStale();
        }
    }

    private void clearSnapshot() {
        lastSnapshot = null;
        lastRobotPose = null;
        lastRobotPoseFtc = null;
        odometryUpdatePending = false;
        lastSnapshotTimestampMs = 0L;
        lastSeenTagId = -1;
    }

    public void overrideRobotPose(Pose pose) {
        if (pose == null) {
            return;
        }
        lastRobotPose = pose;
        lastRobotPoseFtc = convertPedroToFtcPose(pose);
        lastSnapshotTimestampMs = System.currentTimeMillis();
    }

    private static Alliance mapTagToAlliance(int tagId) {
        if (tagId == FieldConstants.BLUE_GOAL_TAG_ID) {
            return Alliance.BLUE;
        }
        if (tagId == FieldConstants.RED_GOAL_TAG_ID) {
            return Alliance.RED;
        }
        return Alliance.UNKNOWN;
    }

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    public boolean getOdometryUpdatePending() {
        refreshSnapshotIfStale();
        return odometryUpdatePending;
    }

    public double getLastPoseXInches() {
        return lastRobotPose != null ? lastRobotPose.getX() : Double.NaN;
    }

    public double getLastPoseYInches() {
        return lastRobotPose != null ? lastRobotPose.getY() : Double.NaN;
    }

    public double getLastPoseHeadingDeg() {
        return lastRobotPose != null ? Math.toDegrees(lastRobotPose.getHeading()) : Double.NaN;
    }

    public double getPoseXInches() {
        return getLastPoseXInches();
    }

    public double getPoseYInches() {
        return getLastPoseYInches();
    }

    public double getPoseHeadingDeg() {
        return getLastPoseHeadingDeg();
    }

    public double getLastTxDegrees() {
        return lastSnapshot != null ? lastSnapshot.getTxDegrees() : Double.NaN;
    }

    public double getLastTyDegrees() {
        return lastSnapshot != null ? lastSnapshot.getTyDegrees() : Double.NaN;
    }

    public double getLastTaPercent() {
        return lastSnapshot != null ? lastSnapshot.getTargetAreaPercent() : Double.NaN;
    }

    public String getAllianceName() {
        return activeAlliance.name();
    }

    public static final class TagSnapshot {
        private final Alliance alliance;
        private final int tagId;
        private final double decisionMargin;
        private final double ftcRange;
        private final double ftcBearing;
        private final double ftcYaw;
        private final double ftcX;
        private final double ftcY;
        private final Pose pedroPose;
        private final Pose ftcPose;
        private final double txDegrees;
        private final double tyDegrees;
        private final double targetAreaPercent;

        /**
         * Constructor for MegaTag2 using fused pose from result.getBotpose_MT2()
         */
        TagSnapshot(Alliance alliance,
                    int tagId,
                    LLResult result,
                    double tx,
                    double ty,
                    double ta) {
            this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
            this.tagId = tagId;
            // For MT2, use target area as decision margin (ambiguity not applicable to fused pose)
            this.decisionMargin = Double.isNaN(ta) ? 0.0 : ta;

            // MegaTag2: Use IMU-fused pose instead of individual tag pose
            Pose3D pose = result.getBotpose_MT2();
            if (pose != null && pose.getPosition() != null) {
                double xIn = DistanceUnit.METER.toInches(pose.getPosition().x);
                double yIn = DistanceUnit.METER.toInches(pose.getPosition().y);
                double headingDeg = pose.getOrientation() == null ? Double.NaN : pose.getOrientation().getYaw();
                if (Double.isNaN(xIn) || Double.isNaN(yIn) || Double.isNaN(headingDeg)) {
                    this.pedroPose = null;
                    this.ftcPose = null;
                } else {
                    // MT2 pose is already in FTC coordinates since we send correct heading via updateRobotOrientation()
                    this.ftcPose = new Pose(xIn, yIn, Math.toRadians(headingDeg));
                    Pose pedro = convertFtcToPedroPose(xIn, yIn, headingDeg);
                    this.pedroPose = pedro;
                }
                this.ftcX = xIn;
                this.ftcY = yIn;
                this.ftcYaw = headingDeg;
                this.ftcRange = Math.hypot(xIn, yIn);
                this.ftcBearing = Math.toDegrees(Math.atan2(yIn, xIn));
            } else {
                this.pedroPose = null;
                this.ftcPose = null;
                this.ftcX = Double.NaN;
                this.ftcY = Double.NaN;
                this.ftcYaw = Double.NaN;
                this.ftcRange = Double.NaN;
                this.ftcBearing = Double.NaN;
            }
            this.txDegrees = Double.isNaN(tx) ? Double.NaN : tx;
            this.tyDegrees = Double.isNaN(ty) ? Double.NaN : ty;
            this.targetAreaPercent = Double.isNaN(ta) ? Double.NaN : ta;
        }

        private static double sanitizeDecisionMetric(LLResultTypes.FiducialResult fiducial) {
            double area = getTargetAreaSafe(fiducial);
            if (!Double.isNaN(area) && area > 0.0) {
                return area;
            }
            double ambiguity = getPoseAmbiguitySafe(fiducial);
            return Double.isNaN(ambiguity) ? 0.0 : (1.0 - ambiguity);
        }

        public Alliance getAlliance() {
            return alliance;
        }

        public int getTagId() {
            return tagId;
        }

        public double getDecisionMargin() {
            return decisionMargin;
        }

        public Optional<Pose> getRobotPose() {
            return Optional.ofNullable(pedroPose);
        }

        public double getRobotX() {
            return pedroPose == null ? Double.NaN : pedroPose.getX();
        }

        public double getRobotY() {
            return pedroPose == null ? Double.NaN : pedroPose.getY();
        }

        public double getRobotYaw() {
            return pedroPose == null ? Double.NaN : Math.toDegrees(pedroPose.getHeading());
        }

        public Optional<Pose> getFtcPose() {
            return Optional.ofNullable(ftcPose);
        }

        public double getFtcRange() {
            return ftcRange;
        }

        public double getFtcBearing() {
            return ftcBearing;
        }

        public double getFtcYaw() {
            return ftcYaw;
        }

        public double getFtcX() {
            return ftcX;
        }

        public double getFtcY() {
            return ftcY;
        }

        public double getTxDegrees() {
            return txDegrees;
        }

        public double getTyDegrees() {
            return tyDegrees;
        }

        public double getTargetAreaPercent() {
            return targetAreaPercent;
        }
    }

    public static Pose convertFtcToPedroPose(double ftcX, double ftcY, double headingDeg) {
        double halfField = AutoField.waypoints.fieldWidthIn / 2.0;
        double pedroX = ftcY + halfField;
        double pedroY = halfField - ftcX;
        // Fixed 180° heading error: changed from -90° to +90° (inverse of forward conversion)
        double pedroHeading = AngleUnit.normalizeRadians(Math.toRadians(headingDeg) + Math.PI / 2.0);
        return new Pose(pedroX, pedroY, pedroHeading);
    }

    private static Pose convertPedroToFtcPose(Pose pedroPose) {
        double halfField = AutoField.waypoints.fieldWidthIn / 2.0;
        double ftcX = halfField - pedroPose.getY();
        double ftcY = pedroPose.getX() - halfField;
        // Fixed 180° heading error: changed from +90° to -90° (matches periodic() conversion)
        double ftcHeading = AngleUnit.normalizeRadians(pedroPose.getHeading() - Math.PI / 2.0);
        return new Pose(ftcX, ftcY, ftcHeading);
    }

    private static double getTargetAreaSafe(LLResultTypes.FiducialResult fiducial) {
        try {
            return (double) fiducial.getClass().getMethod("getTargetArea").invoke(fiducial);
        } catch (Throwable ignored) {
            return Double.NaN;
        }
    }

    private static double getPoseAmbiguitySafe(LLResultTypes.FiducialResult fiducial) {
        try {
            return (double) fiducial.getClass().getMethod("getPoseAmbiguity").invoke(fiducial);
        } catch (Throwable ignored) {
            return Double.NaN;
        }
    }



}
