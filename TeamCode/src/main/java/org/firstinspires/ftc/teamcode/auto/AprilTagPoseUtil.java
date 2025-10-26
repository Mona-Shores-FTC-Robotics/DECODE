package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.ftc.FTCCoordinates;
import com.pedropathing.geometry.pedro.PedroCoordinates;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Utility methods for converting FTC AprilTag pose estimates into Pedro Pathing poses.
 */
public final class AprilTagPoseUtil {

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

        Pose ftcPose;

        if (detection.ftcPose != null) {
            double headingRad = AngleUnit.normalizeRadians(Math.toRadians(detection.ftcPose.yaw));
            ftcPose = new Pose(detection.ftcPose.x, detection.ftcPose.y, headingRad, FTCCoordinates.INSTANCE);
        } else if (detection.robotPose != null) {
            Position position = detection.robotPose.getPosition();
            YawPitchRollAngles orientation = detection.robotPose.getOrientation();
            double headingRad = AngleUnit.normalizeRadians(orientation.getYaw(AngleUnit.RADIANS));
            ftcPose = new Pose(position.x, position.y, headingRad, FTCCoordinates.INSTANCE);
        } else {
            return null;
        }

        Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return new Pose(
                pedroPose.getX(),
                pedroPose.getY(),
                AngleUnit.normalizeRadians(pedroPose.getHeading())
        );
    }
}
