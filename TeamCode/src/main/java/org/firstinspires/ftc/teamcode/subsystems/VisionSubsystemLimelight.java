package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseFrames;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.List;

/**
 * Talks to the Limelight 3A camera and turns AprilTag detections into a
 * field-frame robot pose.
 *
 * <p>What it does:
 * <ul>
 *   <li><b>Pose from tag</b> — when the Limelight sees an AprilTag and its
 *       MegaTag2 pipeline produces a pose, this class exposes it via
 *       {@link #getRobotPoseFromTagPedro()} / {@link #getRobotPoseFromTagFtc()}.</li>
 *   <li><b>Heading sync</b> — every loop, {@code DriveSubsystem} feeds the
 *       robot's heading to the Limelight so MegaTag2 can disambiguate poses.</li>
 *   <li><b>Update gating</b> — {@link #shouldUpdateOdometry()} returns true at
 *       most once per fresh tag lock so callers don't repeatedly relocalize
 *       on the same measurement.</li>
 * </ul>
 *
 * <p>Field geometry (tag IDs, goal locations) lives in {@code util/FieldConstants.java}.
 */
public class VisionSubsystemLimelight {

    /** Stop trusting a tag snapshot for relocalization if it's older than this. */
    private static final long ODOMETRY_RESET_TIMEOUT_MS = 3000L;


    public enum VisionState {
        OFF,
        STREAMING
    }

    public final Limelight3A limelight;
    private final Telemetry telemetry;
    private final boolean limelightAvailable;

    private VisionState state = VisionState.OFF;
    private Alliance activeAlliance = Alliance.UNKNOWN;

    private TagSnapshot lastSnapshot;
    private Pose lastRobotPosePedro;
    private Pose lastRobotPoseFtc;
    private boolean odometryUpdatePending;
    private long lastSnapshotTimestampMs = 0L;
    private int lastSeenTagId = -1;

    private double lastPeriodicMs = 0.0;

    /** Throttle vision polling to 20Hz (50ms) so it doesn't dominate the loop. */
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

    /** Heading offset between Pedro (follower) and FTC (Limelight) frames. Pedro 0° is FTC 90°. */
    private static final double PEDRO_TO_FTC_HEADING_OFFSET_DEG = 90.0;

    public VisionSubsystemLimelight(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public VisionSubsystemLimelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        Limelight3A tempLimelight = null;
        boolean available = true;

        try {
            tempLimelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (IllegalArgumentException e) {
            // Limelight not found in hardware map - vision disabled
            available = false;
            if (telemetry != null) {
                telemetry.addData("WARNING", "Limelight not found - vision disabled");
                telemetry.addData("Error", e.getMessage());
            }
            // Log to system for debugging
            System.err.println("WARNING: Limelight hardware not found - vision subsystem disabled");
            System.err.println("Error: " + e.getMessage());
        } catch (Exception e) {
            // Catch any other hardware initialization errors
            available = false;
            if (telemetry != null) {
                telemetry.addData("ERROR", "Limelight initialization failed - vision disabled");
                telemetry.addData("Error", e.getMessage());
            }
            System.err.println("ERROR: Limelight initialization failed - vision subsystem disabled");
            System.err.println("Error: " + e.getMessage());
        }

        this.limelight = tempLimelight;
        this.limelightAvailable = available;
    }

    public void initialize() {
        if (!limelightAvailable || limelight == null) {
            state = VisionState.OFF;
            return;
        }
        limelight.pipelineSwitch(0);
        limelight.start();
        state = VisionState.STREAMING;
    }

    /**
     * Infinite Command that drives the vision update loop each scheduler tick.
     * Scheduled once in OpMode init; runs until OpMode stop.
     */
    public Command periodic() {
        return Commands.infinite(this::doPeriodic);
    }

    private void doPeriodic() {
        long start = System.nanoTime();
        long nowMs = System.currentTimeMillis();

        // Skip vision updates if limelight is not available
        if (!limelightAvailable || limelight == null) {
            lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
            return;
        }

        // MegaTag2: Update Limelight with current robot heading for IMU-fused
        // localization, before requesting pose estimates. (Also pushed from
        // setRobotHeading() so MT2 is correct during init, when this scheduled
        // loop isn't running yet.)
        pushRobotOrientationToLimelight();

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
        // Wrap hardware operations in try-catch to prevent "expansion hub stopped responding"
        // errors during OpMode shutdown when the hub communication is already terminating
        try {
            if (limelightAvailable && limelight != null) {
                limelight.stop();
            }
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
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

    /** Returns the latest Pedro-frame robot pose from the tag, or null if none. */
    public Pose getRobotPoseFromTagPedro() {
        refreshSnapshotIfStale();
        return lastRobotPosePedro;
    }

    /** Returns the latest FTC-frame robot pose from the tag, or null if none. */
    public Pose getRobotPoseFromTagFtc() {
        refreshSnapshotIfStale();
        return lastRobotPoseFtc;
    }

    public boolean shouldUpdateOdometry() {
        refreshSnapshotIfStale();
        return odometryUpdatePending;
    }

    public void markOdometryUpdated() {
        odometryUpdatePending = false;
    }

    /** Returns the target goal pose for the current alliance, or null if alliance is unknown. */
    public Pose getTargetGoalPose() {
        Alliance alliance = activeAlliance;
        if (alliance == Alliance.UNKNOWN) {
            alliance = RobotState.getAlliance();
        }
        switch (alliance) {
            case BLUE:  return FieldConstants.getBlueBasketTarget();
            case RED:   return FieldConstants.getRedBasketTarget();
            default:    return null;
        }
    }

    /**
     * Gets the vision-based aiming error in radians (robot-relative).
     * This is the horizontal offset (tx) from the Limelight, converted to radians.
     * Positive tx means target is to the right, negative means left.
     *
     * @return The aiming error in radians, or empty if no valid tag
     */
    /** Returns the horizontal aim error in radians, or null if no valid tag. */
    public Double getVisionAimErrorRad() {
        refreshSnapshotIfStale();
        if (lastSnapshot == null) return null;
        double tx = lastSnapshot.getTxDegrees();
        if (Double.isNaN(tx)) return null;
        // Negate: positive tx (target right) requires negative turn (clockwise)
        return -Math.toRadians(tx);
    }

    /** Returns a snapshot matching the preferred alliance, or null if none found. */
    public TagSnapshot findAllianceSnapshot(Alliance preferredAlliance) {
        return selectSnapshot(preferredAlliance);
    }

    /**
     * Returns the tag ID of the best visible motif tag (21–23), or null if none seen.
     * The motif tag tells the robot which color pattern to use in DECODE mode.
     */
    public Integer findMotifTagId() {
        if (!limelightAvailable || limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        double bestScore = Double.NEGATIVE_INFINITY;
        int bestTag = -1;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            if (!isMotifTag(tagId)) continue;
            double score = getTargetAreaSafe(fiducial);
            if (Double.isNaN(score)) score = 0.0;
            if (score > bestScore) {
                bestScore = score;
                bestTag = tagId;
            }
        }

        return bestTag < 0 ? null : bestTag;
    }

    /** Returns the most recent tag snapshot, or null if no tag has been seen. */
    public TagSnapshot getLastSnapshot() {
        refreshSnapshotIfStale();
        return lastSnapshot;
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
        // Push immediately so MegaTag2 has the heading even during init, where the
        // scheduled vision loop (doPeriodic) isn't ticking yet. Without this, MT2
        // solves with a stale orientation and reads wildly off during placement.
        if (!diagnosticMode) {
            pushRobotOrientationToLimelight();
        }
    }

    /**
     * Forwards the stored robot heading to the Limelight in the FTC frame
     * (Pedro 0° = FTC 90°) so MegaTag2 can solve. Safe to call anytime; no-ops
     * until a heading has been provided.
     */
    private void pushRobotOrientationToLimelight() {
        if (limelight == null || !hasValidHeading) {
            return;
        }
        double pedroHeadingDeg = Math.toDegrees(currentHeadingRad);
        double ftcHeadingDeg = AngleUnit.normalizeDegrees(
                pedroHeadingDeg + PEDRO_TO_FTC_HEADING_OFFSET_DEG);
        limelight.updateRobotOrientation(ftcHeadingDeg);
        // Diagnostic: compare this against vision/Poses/MT1 heading. If the robot is
        // correctly placed but MT1's heading disagrees with what we feed here, the
        // seed/offset is wrong — not MT2.
        RobotState.packet.put("/vision/fedPedroHeadingDeg", pedroHeadingDeg);
        RobotState.packet.put("/vision/fedFtcHeadingDeg", ftcHeadingDeg);
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }


    private TagSnapshot selectSnapshot(Alliance preferredAllianceIgnored) {
        if (!limelightAvailable || limelight == null) {
            return null;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        // MegaTag2: find best valid basket tag (either alliance)
        TagSnapshot bestSnapshot = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();

            // Only use basket AprilTags (for example BLUE_GOAL_TAG_ID and RED_GOAL_TAG_ID)
            if (tagId != FieldConstants.BLUE_GOAL_TAG_ID
                    && tagId != FieldConstants.RED_GOAL_TAG_ID) {
                continue;
            }

            // Score the tag to pick the best one
            double score = getTagScore(fiducial);
            if (score > bestScore) {
                bestScore = score;

                Alliance detectionAlliance = mapTagToAlliance(tagId); // This is just meta, not used to decide robot alliance

                bestSnapshot = new TagSnapshot(
                        detectionAlliance,
                        tagId,
                        result,
                        result.getTx(),
                        result.getTy(),
                        result.getTa()
                );
            }
        }

        return bestSnapshot;
    }

    private static boolean isMotifTag(int tagId) {
        return tagId == FieldConstants.DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID
                || tagId == FieldConstants.DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID
                || tagId == FieldConstants.DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID;
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
        // Bandaid: MT2 poses have been unstable, prefer MT1 for now
        Pose pose = snapshot.getRobotPosePedroMT1();
        if (pose == null) {
            pose = snapshot.getRobotPosePedroMT2();
        }
        if (pose != null) {
            lastRobotPosePedro = pose;
            lastRobotPoseFtc = snapshot.getFtcPose();
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
            lastRobotPosePedro = null;
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
        lastRobotPosePedro = null;
        lastRobotPoseFtc = null;
        odometryUpdatePending = false;
        lastSnapshotTimestampMs = 0L;
        lastSeenTagId = -1;
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
        return lastRobotPosePedro != null ? lastRobotPosePedro.getX() : Double.NaN;
    }

    public double getLastPoseYInches() {
        return lastRobotPosePedro != null ? lastRobotPosePedro.getY() : Double.NaN;
    }

    public double getLastPoseHeadingDeg() {
        return lastRobotPosePedro != null ? Math.toDegrees(lastRobotPosePedro.getHeading()) : Double.NaN;
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
    public static class TagSnapshot {

        public final long capturedAtNs;
        public final Alliance alliance;
        public final int tagId;

        // Raw Limelight offsets
        public final double txDegrees;
        public final double tyDegrees;
        public final double targetAreaPercent;

        // Decision metric (for MT2 we just reuse area)
        public final double decisionMargin;

        // MT1 robot pose (vision only)
        public final Pose3D mt1Pose;

        // MT2 robot pose (vision + IMU fused)
        public final Pose3D mt2Pose;

        // FTC-space poses
        public final Pose ftcPoseMT1;
        public final Pose ftcPoseMT2;

        // Pedro-space poses
        public final Pose pedroPoseMT1;
        public final Pose pedroPoseMT2;

        // Derived FTC polar data for MT1
        public final double ftcYawMT1;
        public final double ftcRangeMT1;
        public final double ftcBearingMT1;

        // Derived FTC polar data for MT2
        public final double ftcYawMT2;
        public final double ftcRangeMT2;
        public final double ftcBearingMT2;

        // Selected "best" pose (MT1 preferred, then MT2)
        public final Pose pedroPose;  // primary Pedro pose
        public final Pose ftcPose;    // primary FTC pose
        public final double ftcYaw;   // primary FTC yaw (deg)
        public final double ftcRange;
        public final double ftcBearing;

        /**
         * Create a snapshot of tag data with both MT1 and MT2 pose availability.
         */
        public TagSnapshot(
                Alliance alliance,
                int tagId,
                LLResult result,
                double tx,
                double ty,
                double ta
        ) {
            this.capturedAtNs = System.nanoTime();
            this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
            this.tagId = tagId;

            this.txDegrees = Double.isNaN(tx) ? Double.NaN : tx;
            this.tyDegrees = Double.isNaN(ty) ? Double.NaN : ty;
            this.targetAreaPercent = Double.isNaN(ta) ? Double.NaN : ta;

            // For MT2 there is no ambiguity field in the FTC wrapper, so area is the best simple score
            this.decisionMargin = Double.isNaN(ta) ? 0.0 : ta;

            // -------------------------
            // MT1 pose (botpose)
            // -------------------------
            this.mt1Pose = result.getBotpose();
            Pose candidateFtcMT1 = (mt1Pose != null && mt1Pose.getPosition() != null)
                    ? PoseFrames.mt1ToFtc(mt1Pose) : null;
            if (candidateFtcMT1 != null && !isOriginPose(candidateFtcMT1)) {
                this.ftcPoseMT1 = candidateFtcMT1;
                RobotState.putPose("vision/Poses/MT1", this.ftcPoseMT1);
                this.pedroPoseMT1 = PoseFrames.mt1ToPedro(mt1Pose);

                this.ftcYawMT1 = Math.toDegrees(ftcPoseMT1.getHeading());
                this.ftcRangeMT1 = Math.hypot(ftcPoseMT1.getX(), ftcPoseMT1.getY());
                this.ftcBearingMT1 = Math.toDegrees(Math.atan2(ftcPoseMT1.getY(), ftcPoseMT1.getX()));
            } else {
                this.ftcPoseMT1 = null;
                this.pedroPoseMT1 = null;
                this.ftcYawMT1 = Double.NaN;
                this.ftcRangeMT1 = Double.NaN;
                this.ftcBearingMT1 = Double.NaN;
            }

            // -------------------------
            // MT2 pose (botpose_orb)
            // -------------------------
            this.mt2Pose = result.getBotpose_MT2();
            Pose candidateFtcMT2 = (mt2Pose != null && mt2Pose.getPosition() != null)
                    ? PoseFrames.mt2ToFtc(mt2Pose) : null;
            if (candidateFtcMT2 != null && !isOriginPose(candidateFtcMT2)) {
                this.ftcPoseMT2 = candidateFtcMT2;
                RobotState.putPose("vision/Poses/MT2", this.ftcPoseMT2);
                this.pedroPoseMT2 = PoseFrames.ftcToPedro(ftcPoseMT2);

                this.ftcYawMT2 = Math.toDegrees(ftcPoseMT2.getHeading());
                this.ftcRangeMT2 = Math.hypot(ftcPoseMT2.getX(), ftcPoseMT2.getY());
                this.ftcBearingMT2 = Math.toDegrees(Math.atan2(ftcPoseMT2.getY(), ftcPoseMT2.getX()));
            } else {
                this.ftcPoseMT2 = null;
                this.pedroPoseMT2 = null;
                this.ftcYawMT2 = Double.NaN;
                this.ftcRangeMT2 = Double.NaN;
                this.ftcBearingMT2 = Double.NaN;
            }

            // -------------------------
            // Choose primary pose (prefer MT1, fall back to MT1)
            // -------------------------
            Pose selectedPedro;
            Pose selectedFtc;

            if (pedroPoseMT2 != null) {
                selectedPedro = pedroPoseMT2;
                selectedFtc = ftcPoseMT2;
            } else {
                selectedPedro = pedroPoseMT1;
                selectedFtc = ftcPoseMT1;
            }

            this.pedroPose = selectedPedro;
            this.ftcPose = selectedFtc;

            double yawTmp = Double.NaN;
            double rangeTmp = Double.NaN;
            double bearingTmp = Double.NaN;
            if (selectedFtc != null) {
                if (selectedFtc == ftcPoseMT2 && ftcPoseMT2 != null) {
                    yawTmp = ftcYawMT2;
                    rangeTmp = ftcRangeMT2;
                    bearingTmp = ftcBearingMT2;
                } else if (ftcPoseMT1 != null) {
                    yawTmp = ftcYawMT1;
                    rangeTmp = ftcRangeMT1;
                    bearingTmp = ftcBearingMT1;
                }
            }
            this.ftcYaw = yawTmp;
            this.ftcRange = rangeTmp;
            this.ftcBearing = bearingTmp;
        }

        // -------------------------
        // Helpers and accessors
        // -------------------------

        public boolean hasMT2() {
            return mt2Pose != null && mt2Pose.getPosition() != null;
        }

        public boolean hasMT1() {
            return mt1Pose != null && mt1Pose.getPosition() != null;
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

        /**
         * Limelight reports a botpose at the FTC field origin (0,0) when it has no
         * valid fix. A real robot is never at exact field center during init or a
         * match, so an origin pose is a "no solution" sentinel — NOT a real estimate.
         * Treat it as null; otherwise it converts to Pedro (72,72) and gets fused in
         * as a mid-field ghost that drags the pose to center. (This is the bug that
         * broke auto init: the opposite-goal-tag fix would intermittently return
         * origin, and the filter trusted it.)
         */
        private static boolean isOriginPose(Pose ftcPose) {
            return Math.abs(ftcPose.getX()) < 1.0 && Math.abs(ftcPose.getY()) < 1.0;
        }

        /** Pedro-frame pose from MegaTag1 (single-tag), or null if unavailable. */
        public Pose getRobotPosePedroMT1() {
            return pedroPoseMT1;
        }

        /** Pedro-frame pose from MegaTag2 (IMU-fused, more accurate), or null if unavailable. */
        public Pose getRobotPosePedroMT2() {
            return pedroPoseMT2;
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

        /** FTC-frame pose, or null if unavailable. */
        public Pose getFtcPose() {
            return ftcPose;
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

        public double getTxDegrees() {
            return txDegrees;
        }

        public double getTyDegrees() {
            return tyDegrees;
        }

        public double getTargetAreaPercent() {
            return targetAreaPercent;
        }

        public Alliance inferAllianceFromPose() {
            com.pedropathing.geometry.Pose pose = getRobotPosePedroMT1();
            if (pose == null) {
                return Alliance.UNKNOWN;
            }

            double x = pose.getX();

            // X < halfField means robot is physically on blue side
            // X > halfField means robot is physically on red side
            if (x < FieldConstants.FIELD_WIDTH_INCHES / 2.0) {
                return Alliance.BLUE;
            } else {
                return Alliance.RED;
            }
        }

    }

    // ========================================================================
    // Coordinate conversion helpers
    // ========================================================================

    private static double getTargetAreaSafe(LLResultTypes.FiducialResult fiducial) {
        try {
            return (double) fiducial.getClass().getMethod("getTargetArea").invoke(fiducial);
        } catch (Throwable ignored) {
            // getTargetArea() isn't available on every Limelight SDK version — reflective access on purpose.
            return Double.NaN;
        }
    }

    private static double getPoseAmbiguitySafe(LLResultTypes.FiducialResult fiducial) {
        try {
            return (double) fiducial.getClass().getMethod("getPoseAmbiguity").invoke(fiducial);
        } catch (Throwable ignored) {
            // getPoseAmbiguity() isn't available on every Limelight SDK version — reflective access on purpose.
            return Double.NaN;
        }
    }


}
