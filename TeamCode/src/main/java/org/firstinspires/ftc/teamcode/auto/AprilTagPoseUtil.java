package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

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
     * Converts the {@link AprilTagDetection#robotPose} value into a Pedro {@link Pose}.
     *
     * @param detection the detection to translate.
     * @return a {@link Pose} in inches/radians, or {@code null} if the detection lacks pose data.
     */
    public static Pose toPedroPose(AprilTagDetection detection) {
        if (detection == null || detection.robotPose == null) {
            return null;
        }

        Position position = detection.robotPose.getPosition();
        YawPitchRollAngles orientation = detection.robotPose.getOrientation();

        double heading = orientation.getYaw(AngleUnit.RADIANS);
        Pose ftcPose = new Pose(position.x, position.y, heading, FTCCoordinates.INSTANCE);
        return ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}