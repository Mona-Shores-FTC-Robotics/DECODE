package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public final class PoseFrames {

    private PoseFrames() {
        // Utility only
    }

    // ============================================
    // ==========  MT1  →  FTC_Field  =============
    // ============================================

    /**
     * Converts a Limelight MT1 Pose3D (meters, degrees) into the
     * canonical FTC_Field frame (inches, radians).
     *
     * MT1 is treated as authoritative because it matches the
     * DECODE field map visualization when camera yaw = 0.
     */
    public static Pose mt1ToFtc(Pose3D mt1Pose) {

        if (mt1Pose == null || mt1Pose.getPosition() == null || mt1Pose.getOrientation() == null) {
            return null;
        }

        double xIn = DistanceUnit.METER.toInches(mt1Pose.getPosition().x);
        double yIn = DistanceUnit.METER.toInches(mt1Pose.getPosition().y);

        double headingRad = Math.toRadians(mt1Pose.getOrientation().getYaw());

        headingRad = AngleUnit.normalizeRadians(headingRad);

        return new Pose(xIn, yIn, headingRad);
    }


    // ============================================
    // ==========  MT2  →  FTC_Field  =============
    // ============================================

    /**
     * Converts a Limelight MT2 Pose3D into FTC_Field coordinates.
     *
     * Based on empirical observation from your robot:
     * MT2 requires a 180 degree rotation to match MT1 when the
     * camera yaw in the Limelight UI is set to 0.
     */
    public static Pose mt2ToFtc(Pose3D mt2Pose) {

        if (mt2Pose == null || mt2Pose.getPosition() == null || mt2Pose.getOrientation() == null) {
            return null;
        }

        double xIn = DistanceUnit.METER.toInches(mt2Pose.getPosition().x);
        double yIn = DistanceUnit.METER.toInches(mt2Pose.getPosition().y);
        double headingRad = Math.toRadians(mt2Pose.getOrientation().getYaw());

        headingRad = AngleUnit.normalizeRadians(headingRad);

        return new Pose(xIn, yIn, headingRad);
    }



    // ============================================
    // ==========  FTC_Field  ↔  Pedro  ============
    // ============================================

    /**
     * Converts from FTC field frame (origin at field center)
     * to Pedro frame (origin at Blue-left corner).
     */
    public static Pose ftcToPedro(Pose ftcPose) {

        if (ftcPose == null) {
            return null;
        }

        double half = FieldConstants.FIELD_WIDTH_INCHES / 2.0;

        double pedroX = half - ftcPose.getX();
        double pedroY = ftcPose.getY() + half;

        double pedroHeading = AngleUnit.normalizeRadians(ftcPose.getHeading()-Math.PI/2);

        return new Pose(pedroY, pedroX, pedroHeading);
    }

    /**
     * Converts from Pedro frame (Blue-left corner) back to
     * FTC_Field frame (field center).
     */
    public static Pose pedroToFtc(Pose pedroPose) {

        if (pedroPose == null) {
            return null;
        }

        double half = FieldConstants.FIELD_WIDTH_INCHES / 2.0;

        double xFTC = half - pedroPose.getX(); // 72 - 56 = 16
        double yFTC = pedroPose.getY() - half; // 8 - 72 = -65
        double headingFTC = AngleUnit.normalizeRadians(pedroPose.getHeading()+Math.PI/2);

        return new Pose(-yFTC, -xFTC, headingFTC);
    }


    // ============================================
    // ==========  Convenience Functions  ==========
    // ============================================

    /**
     * MT1 → Pedro
     */
    public static Pose mt1ToPedro(Pose3D mt1Pose) {
        Pose ftc = mt1ToFtc(mt1Pose);
        return (ftc != null)
                ? ftcToPedro(ftc)
                : null;
    }

    /**
     * MT2 → Pedro
     */
    public static Pose mt2ToPedro(Pose3D mt2Pose) {
        Pose ftc = mt2ToFtc(mt2Pose);
        return (ftc != null)
                ? ftcToPedro(ftc)
                : null;
    }
}
