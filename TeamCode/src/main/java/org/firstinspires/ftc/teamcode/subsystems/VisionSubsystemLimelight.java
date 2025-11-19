package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
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

    @Override
    public void initialize() {
        if (!limelightAvailable || limelight == null) {
            state = VisionState.OFF;
            return;
        }
        limelight.pipelineSwitch(0);
        limelight.start();
        state = VisionState.STREAMING;
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        long nowMs = System.currentTimeMillis();

        // Skip vision updates if limelight is not available
        if (!limelightAvailable || limelight == null) {
            lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
            return;
        }

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
        if (limelightAvailable && limelight != null) {
            limelight.stop();
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

    public Optional<Pose> getRobotPoseFromTagPedro() {
        refreshSnapshotIfStale();
        return Optional.ofNullable(lastRobotPosePedro);
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
        Optional<Pose> poseOpt = getRobotPoseFromTagPedro();
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

    public Optional<Integer> findMotifTagId() {
        if (!limelightAvailable || limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Optional.empty();
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Optional.empty();
        }

        double bestScore = Double.NEGATIVE_INFINITY;
        int bestTag = -1;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            if (!isMotifTag(tagId)) {
                continue;
            }
            double score = getTargetAreaSafe(fiducial);
            if (Double.isNaN(score)) {
                score = 0.0;
            }
            if (score > bestScore) {
                bestScore = score;
                bestTag = tagId;
            }
        }

        if (bestTag < 0) {
            return Optional.empty();
        }
        return Optional.of(bestTag);
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
        Optional<Pose> poseOpt = snapshot.getRobotPosePedroMT1();
        Optional<Pose> ftcPoseOpt = snapshot.getFtcPose();
        if (poseOpt.isPresent()) {
            lastRobotPosePedro = poseOpt.get();
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

    public void overrideRobotPose(Pose pose) {
        if (pose == null) {
            return;
        }
        lastRobotPosePedro = pose;
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
            if (mt1Pose != null && mt1Pose.getPosition() != null) {
                this.ftcPoseMT1 = PoseFrames.mt1ToFtc(mt1Pose);
                RobotState.putPose("MT1 FTC Pose", this.ftcPoseMT1);
                this.pedroPoseMT1 = PoseFrames.mt1ToPedro(mt1Pose);
                RobotState.putPose("MT1 Pedro Pose", this.pedroPoseMT1);
                //if this works that means mt1ToPedro is incorrect.

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
            if (mt2Pose != null && mt2Pose.getPosition() != null) {
                this.ftcPoseMT2 = PoseFrames.mt2ToFtc(mt2Pose);
                RobotState.putPose("MT2 Pose", this.ftcPoseMT2);
                this.pedroPoseMT2 = PoseFrames.mt2ToPedro(mt2Pose);

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

            if (pedroPoseMT1 != null) {
                selectedPedro = pedroPoseMT1;
                selectedFtc = ftcPoseMT1;
            }else {
                selectedPedro = pedroPoseMT2;
                selectedFtc = ftcPoseMT2;
            }

            this.pedroPose = selectedPedro;
            this.ftcPose = selectedFtc;

            if (selectedFtc != null) {
                if (ftcPoseMT1 != null) {
                    this.ftcYaw = ftcYawMT1;
                    this.ftcRange = ftcRangeMT1;
                    this.ftcBearing = ftcBearingMT1;
                } else {
                    this.ftcYaw = ftcYawMT2;
                    this.ftcRange = ftcRangeMT2;
                    this.ftcBearing = ftcBearingMT2;
                }
            } else {
                this.ftcYaw = Double.NaN;
                this.ftcRange = Double.NaN;
                this.ftcBearing = Double.NaN;
            }
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

        public Optional<Pose> getRobotPosePedroMT1() {
            return Optional.ofNullable(pedroPoseMT1);
        }

        public Optional<Pose> getRobotPosePeroMT2() {
            return Optional.ofNullable(pedroPoseMT2) ;
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

    // ========================================================================
    // Coordinate conversion helpers
    // ========================================================================

    public static Pose convertFtcToPedroPose(double ftcX, double ftcY, double headingDeg) {
        double halfField = FieldConstants.FIELD_WIDTH_INCHES / 2.0;
        double pedroX = ftcY + halfField;
        double pedroY = halfField - ftcX;
        // Fixed 180 degree heading error: inverse of forward conversion
        double pedroHeading = AngleUnit.normalizeRadians(Math.toRadians(headingDeg) + Math.PI / 2.0);
        return new Pose(pedroX, pedroY, pedroHeading);
    }

    private static Pose convertPedroToFtcPose(Pose pedroPose) {
        double halfField = FieldConstants.FIELD_WIDTH_INCHES / 2.0;
        double ftcX = halfField - pedroPose.getY();
        double ftcY = pedroPose.getX() - halfField;
        // Matches periodic() conversion that sends FTC heading to Limelight
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