package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import dev.nextftc.core.subsystems.Subsystem;

import java.util.List;

public class VisionSubsystemLimelight implements Subsystem {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private int lastSeenTagId = -1;
    private long lastSeenTimestamp = 0;
    private boolean odometryUpdatedForCurrentTag = false;
    private static final long ODOMETRY_RESET_TIMEOUT_MS = 3000;
    private final Inputs inputs = new Inputs();
    private RobotLogger logger;
    private RobotLogger.Source loggerSource;

    public VisionSubsystemLimelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public boolean hasValidTag() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return result != null && fiducials != null && !fiducials.isEmpty();
    }

    public int getCurrentTagId() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (result == null || fiducials == null || fiducials.isEmpty()) return -1;
        return fiducials.get(0).getFiducialId();
    }

    public Pose2D getRobotPoseFromTag() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (result == null || fiducials == null || fiducials.isEmpty()) return null;

        Pose3D pose = fiducials.get(0).getRobotPoseFieldSpace();
        double x = DistanceUnit.METER.toInches(pose.getPosition().x);
        double y = DistanceUnit.METER.toInches(pose.getPosition().y);
        double headingRad = Math.toRadians(pose.getOrientation().getYaw());

        Pose2D fieldPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad);

        telemetry.addData("Vision X", x);
        telemetry.addData("Vision Y", y);
        telemetry.addData("Heading (deg)", AngleUnit.DEGREES.fromRadians(headingRad));

        return fieldPose;
    }

    public boolean shouldUpdateOdometry() {
        if (!hasValidTag()) return false;

        int currentTagId = getCurrentTagId();
        long now = System.currentTimeMillis();

        boolean isNewTag = currentTagId != lastSeenTagId;
        boolean isStale = (now - lastSeenTimestamp) > ODOMETRY_RESET_TIMEOUT_MS;

        if (isNewTag || isStale) {
            lastSeenTagId = currentTagId;
            lastSeenTimestamp = now;
            odometryUpdatedForCurrentTag = false;
        }

        return !odometryUpdatedForCurrentTag;
    }

    public void markOdometryUpdated() {
        odometryUpdatedForCurrentTag = true;
    }

    public Pose getTargetGoalPose(Alliance alliance) {
        return (alliance == Alliance.RED)
                ? FieldConstants.RED_GOAL_TAG
                : FieldConstants.BLUE_GOAL_TAG;
    }

    @Override
    public void periodic() {}

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result == null ? null : result.getFiducialResults();
        boolean hasValid = result != null && fiducials != null && !fiducials.isEmpty();
        inputs.hasValidTag = hasValid;
        inputs.currentTagId = hasValid ? fiducials.get(0).getFiducialId() : -1;
        inputs.lastSeenTagId = lastSeenTagId;
        inputs.odometryUpdatedForCurrentTag = odometryUpdatedForCurrentTag;
        inputs.timeSinceLastSeenMs = lastSeenTimestamp == 0
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0, System.currentTimeMillis() - lastSeenTimestamp);

        if (hasValid) {
            LLResultTypes.FiducialResult fiducial = fiducials.get(0);
            Pose3D pose = fiducial.getRobotPoseFieldSpace();
            if (pose != null) {
                inputs.lastPoseXInches = DistanceUnit.METER.toInches(pose.getPosition().x);
                inputs.lastPoseYInches = DistanceUnit.METER.toInches(pose.getPosition().y);
                inputs.lastPoseHeadingDeg = AngleUnit.normalizeDegrees(pose.getOrientation().getYaw());
            } else {
                inputs.lastPoseXInches = Double.NaN;
                inputs.lastPoseYInches = Double.NaN;
                inputs.lastPoseHeadingDeg = Double.NaN;
            }
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            inputs.lastTxDegrees = Double.isNaN(tx) ? Double.NaN : tx;
            inputs.lastTyDegrees = Double.isNaN(ty) ? Double.NaN : ty;
            inputs.lastTaPercent = Double.isNaN(ta) ? Double.NaN : ta;
        } else {
            inputs.lastPoseXInches = Double.NaN;
            inputs.lastPoseYInches = Double.NaN;
            inputs.lastPoseHeadingDeg = Double.NaN;
            inputs.lastTxDegrees = Double.NaN;
            inputs.lastTyDegrees = Double.NaN;
            inputs.lastTaPercent = Double.NaN;
        }
    }

    public void attachLogger(RobotLogger robotLogger) {
        if (robotLogger == null || loggerSource != null) {
            return;
        }
        logger = robotLogger;
        loggerSource = new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "VisionLimelight";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                populateInputs(inputs);
                logger.logInputs("VisionLimelight", inputs);
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

    public static final class Inputs {
        public boolean hasValidTag;
        public int currentTagId = -1;
        public int lastSeenTagId = -1;
        public double timeSinceLastSeenMs = Double.POSITIVE_INFINITY;
        public boolean odometryUpdatedForCurrentTag;
        public double lastPoseXInches = Double.NaN;
        public double lastPoseYInches = Double.NaN;
        public double lastPoseHeadingDeg = Double.NaN;
        public double lastTxDegrees = Double.NaN;
        public double lastTyDegrees = Double.NaN;
        public double lastTaPercent = Double.NaN;
    }
}
