package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AprilTagPoseUtil;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * Wrapper around the FTC AprilTag pipeline so OpModes can query detections
 * without performing hardware setup directly.
 */
@Configurable
public class VisionSubsystem implements Subsystem {

    public static Position CAMERA_POSITION = new Position(
            DistanceUnit.INCH,
            3.0,   // X right
            2.0,   // Y forward
            9.2,   // Z up
            0
    );
    public static YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0.0,   // yaw facing forward
            -90.0, // pitch to horizontal
            0.0,
            0
    );


    private final HardwareMap hardwareMap;

    public enum VisionState {
        OFF,
        STREAMING
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private VisionState state = VisionState.OFF;
    private boolean odometryUpdatePending;
    private Pose lastRobotPoseFromTag;
    private TagSnapshot lastSnapshot;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initialize() {
        if (visionPortal != null) {
            return;
        }
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder().addProcessor(aprilTagProcessor);
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcam != null) {
            builder.setCamera(webcam);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        visionPortal = builder.build();
        state = VisionState.STREAMING;
    }

    @Override
    public void periodic() {
        // Nothing to do – the SDK handles processing on a background thread.
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTagProcessor = null;
        state = VisionState.OFF;
    }

    public VisionState getState() {
        return state;
    }

    public boolean isStreaming() {
        return state == VisionState.STREAMING;
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) {
            return Collections.emptyList();
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return Collections.emptyList();
        }
        return Collections.unmodifiableList(detections);
    }

    public Optional<Double> getAimAngle() {
        return Optional.empty();
    }

    public boolean shouldUpdateOdometry() {
        return odometryUpdatePending;
    }

    public Optional<Pose> getRobotPoseFromTag() {
        return Optional.ofNullable(lastRobotPoseFromTag);
    }

    public void markOdometryUpdated() {
        odometryUpdatePending = false;
    }

    public Optional<TagSnapshot> findAllianceSnapshot(Alliance preferredAlliance) {
        List<AprilTagDetection> detections = getDetections();
        if (detections.isEmpty()) {
            lastSnapshot = null;
            odometryUpdatePending = false;
            return Optional.empty();
        }

        double bestDecision = Double.NEGATIVE_INFINITY;
        TagSnapshot bestSnapshot = null;

        for (AprilTagDetection detection : detections) {
            Alliance detectionAlliance = mapTagToAlliance(detection.id);
            if (detectionAlliance == Alliance.UNKNOWN) {
                continue;
            }
            if (preferredAlliance != null && preferredAlliance != Alliance.UNKNOWN
                    && detectionAlliance != preferredAlliance) {
                continue;
            }
            if (detection.decisionMargin <= bestDecision) {
                continue;
            }

            Pose pose = AprilTagPoseUtil.toPedroPose(detection);
            if (pose != null && (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading()))) {
                pose = null;
            }
            bestDecision = detection.decisionMargin;
            bestSnapshot = new TagSnapshot(detectionAlliance, detection, pose);
        }

        if (bestSnapshot == null) {
            return Optional.empty();
        }

        boolean sameTagAsLast = lastSnapshot != null && lastSnapshot.getTagId() == bestSnapshot.getTagId();
        lastSnapshot = bestSnapshot;

        Pose poseForOdometry = bestSnapshot.getRobotPose().orElse(null);
        if (poseForOdometry != null && !sameTagAsLast) {
            lastRobotPoseFromTag = poseForOdometry;
            odometryUpdatePending = true;
        }

        return Optional.of(bestSnapshot);
    }

    public Optional<TagSnapshot> getLastSnapshot() {
        return Optional.ofNullable(lastSnapshot);
    }

    public void setAlliance(Alliance alliance) {
        RobotState.setAlliance(alliance);
    }

    public Alliance getAlliance() {
        return RobotState.getAlliance();
    }

    public Optional<Pose> getTargetGoalPose() {
        Alliance currentAlliance = RobotState.getAlliance();
        switch (currentAlliance) {
            case BLUE:
                return Optional.of(FieldConstants.BLUE_GOAL_TAG);
            case RED:
                return Optional.of(FieldConstants.RED_GOAL_TAG);
            default:
                return Optional.empty();
        }
    }

    public void submitTagPoseForOdometry(Pose pose) {
        lastRobotPoseFromTag = pose;
        odometryUpdatePending = pose != null;
    }

    private Alliance mapTagToAlliance(int tagId) {
        if (tagId == FieldConstants.BLUE_GOAL_TAG_ID) {
            return Alliance.BLUE;
        }
        if (tagId == FieldConstants.RED_GOAL_TAG_ID) {
            return Alliance.RED;
        }
        return Alliance.UNKNOWN;
    }

    public static final class TagSnapshot {
        private final Alliance alliance;
        private final AprilTagDetection detection;
        private final Pose robotPose;
        private final double ftcRange;
        private final double ftcBearing;
        private final double ftcYaw;
        private final double ftcX;
        private final double ftcY;

        private TagSnapshot(Alliance alliance, AprilTagDetection detection, Pose robotPose) {
            this.alliance = alliance;
            this.detection = detection;
            this.robotPose = robotPose;
            if (detection.ftcPose != null) {
                this.ftcRange = detection.ftcPose.range;
                this.ftcBearing = detection.ftcPose.bearing;
                this.ftcYaw = AngleUnit.normalizeDegrees(detection.ftcPose.yaw);
                this.ftcX = detection.ftcPose.x;
                this.ftcY = detection.ftcPose.y;
            } else {
                this.ftcRange = Double.NaN;
                this.ftcBearing = Double.NaN;
                this.ftcYaw = Double.NaN;
                this.ftcX = Double.NaN;
                this.ftcY = Double.NaN;
            }
        }

        public Alliance getAlliance() {
            return alliance;
        }

        public int getTagId() {
            return detection.id;
        }

        public double getDecisionMargin() {
            return detection.decisionMargin;
        }

        public Optional<Pose> getRobotPose() {
            return Optional.ofNullable(robotPose);
        }

        public double getFtcX() {
            return ftcX;
        }

        public double getFtcY() {
            return ftcY;
        }

        public double getFtcYaw() {
            return ftcYaw;
        }

        public double getRobotX() {
            if (detection.robotPose == null) {
                return Double.NaN;
            }
            return detection.robotPose.getPosition().x;
        }

        public double getRobotY() {
            if (detection.robotPose == null) {
                return Double.NaN;
            }
            return detection.robotPose.getPosition().y;
        }

        public double getRobotYaw() {
            if (detection.robotPose == null) {
                return Double.NaN;
            }
            return AngleUnit.normalizeDegrees(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        public double getFtcRange() {
            return ftcRange;
        }

        public double getFtcBearing() {
            return ftcBearing;
        }
    }

}
