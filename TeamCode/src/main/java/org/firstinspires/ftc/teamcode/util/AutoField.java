package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Utility for alliance-aware pose mirroring.
 *
 * Blue alliance poses are defined in "blue coordinates" (origin at blue corner).
 * Red alliance poses are mirrored across the field center by delegating to
 * Pedro's library method {@link Pose#mirror(double)} with the configured
 * field-interior width ({@link FieldConstants#MIRROR_X_SUM}). A small additive
 * fudge ({@link FieldConstants.AllianceMirrorFudge}) is applied on top in case
 * the practice field deviates slightly from the configured width.
 */
public final class AutoField {

    private AutoField() {
        // Utility class
    }

    /**
     * Creates a pose for the given alliance, mirroring X and heading for red.
     *
     * @param x X coordinate in blue alliance frame
     * @param y Y coordinate (same for both alliances)
     * @param headingDeg Heading in degrees (0 = +X, 90 = +Y) in blue alliance frame
     * @param alliance Target alliance
     * @return Pose in world coordinates for the specified alliance
     */
    public static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
        Pose base = new Pose(x, y, Math.toRadians(headingDeg));
        if (alliance != Alliance.RED) return base;

        // Pedro's Pose.mirror(width): x' = width - x, y' = y, heading' = normalize(PI - heading).
        Pose mirrored = base.mirror(FieldConstants.MIRROR_X_SUM);
        return new Pose(
                mirrored.getX() + FieldConstants.AllianceMirrorFudge.redMirrorXInches,
                mirrored.getY() + FieldConstants.AllianceMirrorFudge.redMirrorYInches,
                mirrored.getHeading());
    }
}
