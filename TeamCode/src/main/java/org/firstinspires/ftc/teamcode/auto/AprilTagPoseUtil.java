package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Utility methods for converting FTC AprilTag pose estimates into Pedro Pathing poses.
 */
public final class AprilTagPoseUtil {

    private static final double FIELD_SIZE_IN = 144.0;
    private static final double FIELD_HALF_IN = FIELD_SIZE_IN / 2.0;

    private AprilTagPoseUtil() {
        // Utility class
    }

    /**
     * Converts the FTC AprilTag pose estimate into a Pedro {@link Pose}. Prefers the field-centric
     * {@link AprilTagDetection#ftcPose} metadata when available, falling back to the camera-centric
     * {@link AprilTagDetection#robotPose}.
     *
     * @param detection the detection to translate.
     * @return a {@link Pose} in inches/radians, or {@code null} if the detection lacks pose data.
     */
    public static Pose toPedroPose(AprilTagDetection detection) {
        if (detection == null) {
            return null;
        }

        double ftcX;
        double ftcY;
        double yawDeg;

        if (detection.ftcPose != null) {
            ftcX = detection.ftcPose.x;
            ftcY = detection.ftcPose.y;
            yawDeg = detection.ftcPose.yaw;
        } else if (detection.robotPose != null) {
            Position position = detection.robotPose.getPosition();
            YawPitchRollAngles orientation = detection.robotPose.getOrientation();
            ftcX = position.x;
            ftcY = position.y;
            yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        } else {
            return null;
        }

        double pedroX = FIELD_HALF_IN + ftcX;
        double pedroY = FIELD_HALF_IN - ftcY;
        double headingRad = AngleUnit.normalizeRadians(Math.toRadians(yawDeg));

        return new Pose(pedroX, pedroY, headingRad);
    }
}
