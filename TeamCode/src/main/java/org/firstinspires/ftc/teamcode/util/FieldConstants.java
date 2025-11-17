package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Field reference points shared across subsystems.
 * Coordinates pulled from the official FTC field CAD and aligned with PedroPathing's coordinate system.
 *
 * This version uses the incenter (center of the inscribed circle) of the goal triangle
 * as the point to aim at when shooting. The incenter is equidistant from all sides of the
 * triangular opening and gives a more centered and safer aim point than the centroid.
 */
@Configurable
public final class FieldConstants {

    private FieldConstants() {
        // Utility only.
    }

    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

    /**
     * AprilTag locations mounted on the face of the baskets.
     */
    public static final Pose BLUE_GOAL_TAG_APRILTAG = new Pose(7.0, 140.0, 0.0);
    public static final Pose RED_GOAL_TAG_APRILTAG = new Pose(137.0, 140.0, 0.0);

    /**
     * Triangle corner locations of the goal openings (in inches) from field CAD.
     */
    public static final Pose BLUE_GOAL_LOWER  = new Pose(0.0,   115.0, 0.0);
    public static final Pose BLUE_GOAL_CORNER = new Pose(0.0,   144.0, 0.0);
    public static final Pose BLUE_GOAL_FAR    = new Pose(25.0,  144.0, 0.0);

    public static final Pose RED_GOAL_LOWER   = new Pose(144.0, 115.0, 0.0);
    public static final Pose RED_GOAL_CORNER  = new Pose(144.0, 144.0, 0.0);
    public static final Pose RED_GOAL_FAR     = new Pose(119.0, 144.0, 0.0);

    /**
     * Incenter of the triangular openings.
     * The incenter is the center of the circle that touches all three sides of the triangle.
     *
     * This produces a better aiming point than the centroid because it sits deeper inside
     * the opening and is more centered with respect to all three faces.
     *
     * For Blue: incenter ≈ (5.95, 131.82)
     * For Red: incenter ≈ (138.05, 131.82)
     */
    public static final Pose BLUE_GOAL_CENTER = incenter(BLUE_GOAL_LOWER, BLUE_GOAL_CORNER, BLUE_GOAL_FAR);
    public static final Pose RED_GOAL_CENTER = incenter(RED_GOAL_LOWER, RED_GOAL_CORNER, RED_GOAL_FAR);

    /**
     * Configurable offset for basket aiming (tunable via FTC Dashboard).
     * Allows fine-tuning the aim point based on projectile arc without changing code.
     * Positive deltaY moves aim point further from driver station.
     */
    @Configurable
    public static class BasketTargets {
        /** Y offset from incenter (inches) - adjust based on projectile arc */
        public static double deltaY = 0.0;
        /** X offset from incenter (inches) - typically leave at 0 */
        public static double deltaX = 0.0;
    }

    public static final int DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID = 21;
    public static final int DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID = 22;
    public static final int DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID = 23;

    /**
     * Gets the current blue basket aiming target with configurable offset.
     */
    public static Pose getBlueBasketTarget() {
        return new Pose(
            BLUE_GOAL_CENTER.getX() + BasketTargets.deltaX,
            BLUE_GOAL_CENTER.getY() + BasketTargets.deltaY,
            0.0
        );
    }

    /**
     * Gets the current red basket aiming target with configurable offset.
     */
    public static Pose getRedBasketTarget() {
        return new Pose(
            RED_GOAL_CENTER.getX() + BasketTargets.deltaX,
            RED_GOAL_CENTER.getY() + BasketTargets.deltaY,
            0.0
        );
    }

    /**
     * Computes the aim angle from a robot pose to a target pose.
     */
    public static double getAimAngleTo(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Computes the incenter of a triangle defined by points a, b, and c.
     *
     * The incenter is the weighted average of the vertices using the lengths of their
     * opposite sides as the weights. This produces the point that is centered relative
     * to all three sides.
     *
     * Formula:
     *  Let A = length of side opposite vertex a (between b and c)
     *  Let B = length of side opposite vertex b (between a and c)
     *  Let C = length of side opposite vertex c (between a and b)
     *
     *  Incenter = (A*a + B*b + C*c) / (A + B + C)
     */
    private static Pose incenter(Pose a, Pose b, Pose c) {
        double A = distance(b, c);
        double B = distance(a, c);
        double C = distance(a, b);

        double x = (A * a.getX() + B * b.getX() + C * c.getX()) / (A + B + C);
        double y = (A * a.getY() + B * b.getY() + C * c.getY()) / (A + B + C);

        return new Pose(x, y, 0.0);
    }

    /**
     * Distance between two poses, ignoring heading.
     */
    private static double distance(Pose p1, Pose p2) {
        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
