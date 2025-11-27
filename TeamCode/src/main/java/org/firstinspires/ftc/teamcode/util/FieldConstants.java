package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.GoalGeometry.chebyshevCenter;
import static org.firstinspires.ftc.teamcode.util.GoalGeometry.incenter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

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
    public static BasketTargetOffsets  basketTargetOffsets = new BasketTargetOffsets();

    @Configurable
    public static class BasketTargetOffsets {
        /** Blue basket X offset from incenter (inches). Negative = left toward corner */
        public double blueDeltaX = 0;
        /** Blue basket Y offset from incenter (inches). Positive = up toward corner */
        public double blueDeltaY = 0;
        /** Red basket X offset from incenter (inches). Positive = right toward corner */
        public double redDeltaX = 0.0;
        /** Red basket Y offset from incenter (inches). Positive = up toward corner */
        public double redDeltaY = 0.0;
    }


    public static final double FIELD_WIDTH_INCHES = 144.0;

    /**
     * Goal AprilTags
     */
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final Pose BLUE_GOAL_TAG_APRILTAG = new Pose(7.0, 140.0, 0.0);

    public static final int RED_GOAL_TAG_ID = 24;
    public static final Pose RED_GOAL_TAG_APRILTAG = new Pose(137.0, 140.0, 0.0);

    /**
     * Motif AprilTags
     */
    public static final int DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID = 21;
    public static final int DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID = 22;
    public static final int DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID = 23;

    /**
     * Triangle corner locations of the goal openings (in inches) PEDRO Coordinate Frame.
     */
    public static final Pose BLUE_GOAL_LOWER_PEDRO = new Pose(0.0,   115.0, 0.0);
    public static final Pose BLUE_GOAL_CORNER_PEDRO = new Pose(0.0,   144.0, 0.0);
    public static final Pose BLUE_GOAL_FAR_PEDRO = new Pose(25.0,  144.0, 0.0);

    public static final Pose RED_GOAL_LOWER_PEDRO = new Pose(144.0, 115.0, 0.0);
    public static final Pose RED_GOAL_CORNER_PEDRO = new Pose(144.0, 144.0, 0.0);
    public static final Pose RED_GOAL_FAR_PEDRO = new Pose(119.0, 144.0, 0.0);

    /**
     * Incenter of the triangular openings.
     * The incenter is the center of the circle that touches all three sides of the triangle.
     **/
    public static final Pose BLUE_GOAL_CENTER = incenter(BLUE_GOAL_LOWER_PEDRO, BLUE_GOAL_CORNER_PEDRO, BLUE_GOAL_FAR_PEDRO);
    public static final Pose RED_GOAL_CENTER = incenter(RED_GOAL_LOWER_PEDRO, RED_GOAL_CORNER_PEDRO, RED_GOAL_FAR_PEDRO);

    /**
     * Polygon coordinates of the goal openings (in inches) from field CAD.
     * NOTE: FTC coordinate values swapped from original CAD to match Pedro coordinate system naming.
     * Blue goal (left side in Pedro) uses positive X in FTC after ftcToPedro conversion.
     * Red goal (right side in Pedro) uses negative X in FTC after ftcToPedro conversion.
     */
    public static final Pose BLUE_GOAL_CORNER_FTC = new Pose(-70.191315,   -70.160065, 0.0);
    public static final Pose BLUE_GOAL_FAR_FTC = new Pose(-47.581315,   -70.160065, 0.0);
    public static final Pose BLUE_GOAL_LOWER_INNER_FTC = new Pose(-70.191315,   -48.792542, 0.0);
    public static final Pose BLUE_GOAL_LOWER_EDGE_FTC = new Pose(-63.102536,   -48.792542, 0.0);

    public static final Pose RED_GOAL_CORNER_FTC = new Pose(-70.191315,   70.160065, 0.0);
    public static final Pose RED_GOAL_FAR_FTC = new Pose(-47.581315,   70.160065, 0.0);
    public static final Pose RED_GOAL_LOWER_INNER_FTC = new Pose(-70.191315,   48.792542, 0.0);
    public static final Pose RED_GOAL_LOWER_EDGE_FTC = new Pose(-63.102536,   48.792542, 0.0);

    public static final List<Pose> BLUE_GOAL_POLY = Arrays.asList(
            BLUE_GOAL_CORNER_FTC,
            BLUE_GOAL_FAR_FTC,
            BLUE_GOAL_LOWER_EDGE_FTC,
            BLUE_GOAL_LOWER_INNER_FTC
    );

    public static final Pose BLUE_GOAL_CHEBYSHEV_FTC = chebyshevCenter(BLUE_GOAL_POLY);
    public static final Pose BLUE_GOAL_CHEBYSHEV_PEDRO = PoseFrames.ftcToPedro(BLUE_GOAL_CHEBYSHEV_FTC);

    public static final List<Pose> RED_GOAL_POLY = Arrays.asList(
            RED_GOAL_CORNER_FTC,
            RED_GOAL_FAR_FTC,
            RED_GOAL_LOWER_EDGE_FTC,
            RED_GOAL_LOWER_INNER_FTC
    );

    public static final Pose RED_GOAL_CHEBYSHEV_FTC = chebyshevCenter(RED_GOAL_POLY);
    public static final Pose RED_GOAL_CHEBYSHEV_PEDRO = PoseFrames.ftcToPedro(RED_GOAL_CHEBYSHEV_FTC);

    /**
     * Gets the current blue basket aiming target with configurable offset.
     */
    public static Pose getBlueBasketTarget() {
        Pose triangleIncenterGoalPose =  new Pose(
                BLUE_GOAL_CENTER.getX() + basketTargetOffsets.blueDeltaX,
                BLUE_GOAL_CENTER.getY() + basketTargetOffsets.blueDeltaY,
                0.0
        );
        RobotState.putPose("Goal Poses/BLUE_INCENTER_GOAL", triangleIncenterGoalPose);

        Pose chebyshevGoalPedroPose = new Pose(
                BLUE_GOAL_CHEBYSHEV_PEDRO.getX() + basketTargetOffsets.blueDeltaX,
                BLUE_GOAL_CHEBYSHEV_PEDRO.getY() + basketTargetOffsets.blueDeltaY,
                0.0
        );
        Pose chebyshevGoalFTCPose = PoseFrames.pedroToFtc(chebyshevGoalPedroPose);
        RobotState.putPose("Goal Poses/BLUE_CHEBYSHEV_GOAL", chebyshevGoalFTCPose); //Publish the chebyshev pose for AS

        return chebyshevGoalPedroPose;
    }

    /**
     * Gets the current red basket aiming target with configurable offset.
     */
    public static Pose getRedBasketTarget() {
        Pose triangleIncenterGoalPose =  new Pose(
                RED_GOAL_CENTER.getX() + basketTargetOffsets.redDeltaX,
                RED_GOAL_CENTER.getY() + basketTargetOffsets.redDeltaY,
                0.0
        );
        RobotState.putPose("Goal Poses/RED_INCENTER_GOAL", triangleIncenterGoalPose);

        Pose chebyshevGoalPedroPose = new Pose(
                RED_GOAL_CHEBYSHEV_PEDRO.getX() + basketTargetOffsets.redDeltaX,
                RED_GOAL_CHEBYSHEV_PEDRO.getY() + basketTargetOffsets.redDeltaY,
                0.0
        );
        Pose chebyshevGoalFTCPose = PoseFrames.pedroToFtc(chebyshevGoalPedroPose);
        RobotState.putPose("Goal Poses/RED_CHEBYSHEV_GOAL", chebyshevGoalFTCPose); //Publish the chebyshev pose for AS

        return chebyshevGoalPedroPose;
    }

    /**
     * Computes the aim angle from a robot pose to a target pose.
     */
    public static double getAimAngleTo(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.atan2(dy, dx);
    }

}
