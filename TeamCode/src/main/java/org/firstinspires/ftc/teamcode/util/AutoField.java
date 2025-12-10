package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Utility for alliance-aware pose mirroring.
 *
 * Blue alliance poses are defined in "blue coordinates" (origin at blue corner).
 * Red alliance poses are automatically mirrored across the field center.
 *
 * NOTE: Waypoint values should be defined in the Command classes (e.g., FarTogetherCommand.Waypoints)
 * which are the source of truth for autonomous paths. Use poseForAlliance() to mirror those values.
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
        if (alliance == Alliance.RED) {
            double mirroredX = FieldConstants.FIELD_WIDTH_INCHES - base.getX();
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - base.getHeading());
            return new Pose(mirroredX, base.getY(), mirroredHeading);
        }
        return base;
    }
}
