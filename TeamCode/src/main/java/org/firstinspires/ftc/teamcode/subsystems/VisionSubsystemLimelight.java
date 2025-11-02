package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.util.RobotLogger;
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

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private VisionState state = VisionState.OFF;
    private Alliance activeAlliance = Alliance.UNKNOWN;

    private TagSnapshot lastSnapshot;
    private Pose lastRobotPose;
    private boolean odometryUpdatePending;
    private long lastSnapshotTimestampMs = 0L;
    private int lastSeenTagId = -1;

    private final Inputs inputs = new Inputs();
    private RobotLogger logger;
    private RobotLogger.Source loggerSource;
    private double lastPeriodicMs = 0.0;

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
        // Limelight handles processing internally; no periodic work needed.
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public void stop() {
        limelight.stop();
        state = VisionState.OFF;
        clearSnapshot();
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

    public int getCurrentTagId() {
        refreshSnapshotIfStale();
        return lastSnapshot == null ? -1 : lastSnapshot.getTagId();
    }

    public Optional<Pose> getRobotPoseFromTag() {
        refreshSnapshotIfStale();
        return Optional.ofNullable(lastRobotPose);
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
                return Optional.of(FieldConstants.BLUE_GOAL_TAG);
            case RED:
                return Optional.of(FieldConstants.RED_GOAL_TAG);
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

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        refreshSnapshotIfStale();
        inputs.state = state;
        inputs.hasValidTag = lastSnapshot != null;
        inputs.currentTagId = lastSnapshot == null ? -1 : lastSnapshot.getTagId();
        inputs.lastSeenTagId = lastSeenTagId;
        inputs.odometryUpdatePending = odometryUpdatePending;
        inputs.timeSinceLastSeenMs = lastSnapshotTimestampMs == 0L
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0, System.currentTimeMillis() - lastSnapshotTimestampMs);
        inputs.alliance = activeAlliance.name();

        if (lastRobotPose != null) {
            inputs.lastPoseXInches = lastRobotPose.getX();
            inputs.lastPoseYInches = lastRobotPose.getY();
            inputs.lastPoseHeadingDeg = Math.toDegrees(lastRobotPose.getHeading());
        } else {
            inputs.lastPoseXInches = Double.NaN;
            inputs.lastPoseYInches = Double.NaN;
            inputs.lastPoseHeadingDeg = Double.NaN;
        }

        if (lastSnapshot != null) {
            inputs.lastTxDegrees = lastSnapshot.getTxDegrees();
            inputs.lastTyDegrees = lastSnapshot.getTyDegrees();
            inputs.lastTaPercent = lastSnapshot.getTargetAreaPercent();
        } else {
            inputs.lastTxDegrees = Double.NaN;
            inputs.lastTyDegrees = Double.NaN;
            inputs.lastTaPercent = Double.NaN;
        }
    }

    public void setAlliance(Alliance alliance) {
        activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        RobotState.setAlliance(activeAlliance);
    }

    public Alliance getAlliance() {
        return activeAlliance;
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void attachLogger(RobotLogger robotLogger) {
        if (robotLogger == null || loggerSource != null) {
            return;
        }
        logger = robotLogger;
        loggerSource = new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Vision";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                populateInputs(inputs);
                logger.logInputs("Vision", inputs);
            }
        };
        robotLogger.registerSource(loggerSource);
    }

    public void detachLogger() {
        if (logger != null && loggerSource != null) {
            logger.unregisterSource(loggerSource);
            loggerSource = null;
        }
    }

    private TagSnapshot selectSnapshot(Alliance preferredAlliance) {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        TagSnapshot bestSnapshot = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            Alliance detectionAlliance = mapTagToAlliance(fiducial.getFiducialId());
            if (preferredAlliance != null && preferredAlliance != Alliance.UNKNOWN
                    && detectionAlliance != preferredAlliance) {
                continue;
            }
            if (detectionAlliance == Alliance.UNKNOWN && preferredAlliance != Alliance.UNKNOWN) {
                continue;
            }
            TagSnapshot candidate = new TagSnapshot(detectionAlliance, fiducial, result.getTx(), result.getTy(), result.getTa());
            if (candidate.getDecisionMargin() > bestScore) {
                bestScore = candidate.getDecisionMargin();
                bestSnapshot = candidate;
            }
        }

        if (bestSnapshot == null && preferredAlliance != null && preferredAlliance != Alliance.UNKNOWN) {
            // Retry without alliance filtering so we still capture pose data.
            return selectSnapshot(Alliance.UNKNOWN);
        }
        return bestSnapshot;
    }

    private void onSnapshotUpdated(TagSnapshot snapshot) {
        long now = System.currentTimeMillis();
        boolean stale = lastSnapshotTimestampMs == 0L || (now - lastSnapshotTimestampMs) > ODOMETRY_RESET_TIMEOUT_MS;
        lastSnapshot = snapshot;
        lastSnapshotTimestampMs = now;
        Optional<Pose> poseOpt = snapshot.getRobotPose();
        if (poseOpt.isPresent()) {
            lastRobotPose = poseOpt.get();
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
        long now = System.currentTimeMillis();
        if (now - lastSnapshotTimestampMs > ODOMETRY_RESET_TIMEOUT_MS) {
            odometryUpdatePending = false;
            lastSnapshot = null;
            lastRobotPose = null;
            lastSeenTagId = -1;
        }
    }

    private void clearSnapshot() {
        lastSnapshot = null;
        lastRobotPose = null;
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

    public static final class Inputs {
        public VisionState state = VisionState.OFF;
        public boolean hasValidTag;
        public int currentTagId = -1;
        public int lastSeenTagId = -1;
        public double timeSinceLastSeenMs = Double.POSITIVE_INFINITY;
        public boolean odometryUpdatePending;
        public double lastPoseXInches = Double.NaN;
        public double lastPoseYInches = Double.NaN;
        public double lastPoseHeadingDeg = Double.NaN;
        public double lastTxDegrees = Double.NaN;
        public double lastTyDegrees = Double.NaN;
        public double lastTaPercent = Double.NaN;
        public String alliance = Alliance.UNKNOWN.name();
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
        private final Pose robotPose;
        private final double txDegrees;
        private final double tyDegrees;
        private final double targetAreaPercent;

        TagSnapshot(Alliance alliance,
                    LLResultTypes.FiducialResult fiducial,
                    double tx,
                    double ty,
                    double ta) {
            this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
            this.tagId = fiducial.getFiducialId();
            this.decisionMargin = sanitizeDecisionMetric(fiducial);

            Pose3D pose = fiducial.getRobotPoseFieldSpace();
            if (pose != null && pose.getPosition() != null) {
                double xIn = DistanceUnit.METER.toInches(pose.getPosition().x);
                double yIn = DistanceUnit.METER.toInches(pose.getPosition().y);
                double headingDeg = pose.getOrientation() == null ? Double.NaN : pose.getOrientation().getYaw();
                this.robotPose = Double.isNaN(xIn) || Double.isNaN(yIn) || Double.isNaN(headingDeg)
                        ? null
                        : new Pose(xIn, yIn, Math.toRadians(headingDeg));
                this.ftcX = xIn;
                this.ftcY = yIn;
                this.ftcYaw = headingDeg;
                this.ftcRange = Math.hypot(xIn, yIn);
                this.ftcBearing = Math.toDegrees(Math.atan2(yIn, xIn));
            } else {
                this.robotPose = null;
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
            return Optional.ofNullable(robotPose);
        }

        public double getRobotX() {
            return robotPose == null ? Double.NaN : robotPose.getX();
        }

        public double getRobotY() {
            return robotPose == null ? Double.NaN : robotPose.getY();
        }

        public double getRobotYaw() {
            return robotPose == null ? Double.NaN : Math.toDegrees(robotPose.getHeading());
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
