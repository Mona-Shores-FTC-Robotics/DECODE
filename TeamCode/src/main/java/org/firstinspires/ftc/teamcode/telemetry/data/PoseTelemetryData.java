package org.firstinspires.ftc.teamcode.telemetry.data;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseTransforms;

/**
 * Robot pose data for telemetry.
 * Includes both Pedro pose and FTC pose representations.
 */
public class PoseTelemetryData {
    // Pedro pose (primary)
    public final boolean poseValid;
    public final double poseXIn;
    public final double poseYIn;
    public final double headingRad;
    public final double headingDeg;

    // FTC pose (for compatibility)
    public final double ftcXIn;
    public final double ftcYIn;
    public final double ftcHeadingRad;

    // Vision pose (when available)
    public final boolean visionPoseValid;
    public final double visionPoseXIn;
    public final double visionPoseYIn;
    public final double visionHeadingRad;

    public PoseTelemetryData(Pose2D pose, Pose pedroPose, Pose visionPose) {
        // Pedro pose
        if (pose != null) {
            this.poseValid = true;
            this.poseXIn = pose.getX(DistanceUnit.INCH);
            this.poseYIn = pose.getY(DistanceUnit.INCH);
            this.headingRad = pose.getHeading(AngleUnit.RADIANS);
            this.headingDeg = pose.getHeading(AngleUnit.DEGREES);
        } else {
            this.poseValid = false;
            this.poseXIn = 0.0;
            this.poseYIn = 0.0;
            this.headingRad = 0.0;
            this.headingDeg = 0.0;
        }

        // FTC pose
        Pose ftcPose = PoseTransforms.toFtcPose(pedroPose);
        if (ftcPose != null) {
            this.ftcXIn = ftcPose.getX();
            this.ftcYIn = ftcPose.getY();
            this.ftcHeadingRad = ftcPose.getHeading();
        } else {
            this.ftcXIn = Double.NaN;
            this.ftcYIn = Double.NaN;
            this.ftcHeadingRad = Double.NaN;
        }

        // Vision pose (FTC Pose type from vision subsystem)
        if (visionPose != null) {
            this.visionPoseValid = true;
            this.visionPoseXIn = visionPose.getX();      // FTC Pose uses inches directly
            this.visionPoseYIn = visionPose.getY();      // FTC Pose uses inches directly
            this.visionHeadingRad = visionPose.getHeading();  // FTC Pose uses radians directly
        } else {
            this.visionPoseValid = false;
            this.visionPoseXIn = Double.NaN;
            this.visionPoseYIn = Double.NaN;
            this.visionHeadingRad = Double.NaN;
        }
    }

    public static PoseTelemetryData capture(DriveSubsystem drive, Pose visionPose) {
        Pose2D pose = drive.getPose();
        Pose pedroPose = drive.getFollowerPose();
        return new PoseTelemetryData(pose, pedroPose, visionPose);
    }
}
