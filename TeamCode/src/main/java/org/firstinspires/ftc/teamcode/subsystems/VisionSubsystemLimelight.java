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

import dev.nextftc.core.subsystems.Subsystem;

import java.util.List;

public class VisionSubsystemLimelight implements Subsystem {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private int lastSeenTagId = -1;
    private long lastSeenTimestamp = 0;
    private boolean odometryUpdatedForCurrentTag = false;
    private static final long ODOMETRY_RESET_TIMEOUT_MS = 3000;

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
}