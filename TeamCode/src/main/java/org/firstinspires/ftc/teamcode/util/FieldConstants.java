package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Field reference points shared across subsystems.
 * Coordinates pulled from the official FTC field CAD and aligned with PedroPathing's coordinate system.
 */
@Configurable
public final class FieldConstants {

    private FieldConstants() {
        // Utility only.
    }

    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

    /**
     * Basket geometry and aiming targets.
     * Blue basket vertices: (0, 115), (25, 144), (0, 144)
     * Red basket vertices: (144, 115), (144, 144), (119, 144)
     * Centroids: Blue (8.33, 134.33), Red (135.67, 134.33)
     */
    @Configurable
    public static class BasketTargets {
        /** X coordinate for blue basket aim point (inches) */
        public static double blueX = 8.33;
        /** Y coordinate for blue basket aim point (inches) - adjust based on arc */
        public static double blueY = 134.33;
        /** X coordinate for red basket aim point (inches) */
        public static double redX = 135.67;
        /** Y coordinate for red basket aim point (inches) - adjust based on arc */
        public static double redY = 134.33;
    }

    // AprilTag locations (on the face of the basket)
    public static final Pose BLUE_GOAL_TAG_APRILTAG = new Pose(13.679, 126.647, 0.0);
    public static final Pose RED_GOAL_TAG_APRILTAG = new Pose(130.321, 126.647, 0.0);

    // Deprecated - use getBlueBasketTarget() or getRedBasketTarget() instead for live config updates
    @Deprecated
    public static final Pose BLUE_GOAL_TAG = new Pose(8.33, 134.33, 0.0);
    @Deprecated
    public static final Pose RED_GOAL_TAG = new Pose(135.67, 134.33, 0.0);

    public static final int DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID = 21;
    public static final int DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID = 22;
    public static final int DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID = 23;

    /**
     * Gets the current blue basket aiming target (reads from tunable config).
     */
    public static Pose getBlueBasketTarget() {
        return new Pose(BasketTargets.blueX, BasketTargets.blueY, 0.0);
    }

    /**
     * Gets the current red basket aiming target (reads from tunable config).
     */
    public static Pose getRedBasketTarget() {
        return new Pose(BasketTargets.redX, BasketTargets.redY, 0.0);
    }

    public static double getAimAngleTo(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.atan2(dy, dx);
    }
}
