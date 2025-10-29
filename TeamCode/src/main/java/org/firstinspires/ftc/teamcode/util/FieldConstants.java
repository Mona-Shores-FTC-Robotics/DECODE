package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Field reference points shared across subsystems.
 * Coordinates pulled from the official FTC field CAD and aligned with PedroPathing's coordinate system.
 */
public final class FieldConstants {

    private FieldConstants() {
        // Utility only.
    }

    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;
    public static final Pose BLUE_GOAL_TAG = new Pose(13.679, 126.647, 0.0);
    public static final Pose RED_GOAL_TAG = new Pose(130.321, 126.647, 0.0);

    public static final int DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID = 21;
    public static final int DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID = 22;
    public static final int DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID = 23;

    public static double getAimAngleTo(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.atan2(dy, dx);
    }
}
