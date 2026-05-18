package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Utility for alliance-aware pose mirroring. Blue poses pass through unchanged;
 * Red poses are mirrored across the field's X-axis center via Pedro's library
 * method {@link Pose#mirror(double)} with {@link FieldConstants#MIRROR_X_SUM}
 * as the width. To correct for a wider/narrower field, change MIRROR_X_SUM —
 * there's no separate fudge knob, the width itself is the only knob.
 */
public final class AutoField {

    private AutoField() {
        // Utility class
    }

    public static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
        Pose base = new Pose(x, y, Math.toRadians(headingDeg));
        return alliance == Alliance.RED
                ? base.mirror(FieldConstants.MIRROR_X_SUM)
                : base;
    }
}
