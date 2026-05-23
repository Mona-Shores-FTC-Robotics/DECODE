package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.GoalGeometry.chebyshevCenter;
import static org.firstinspires.ftc.teamcode.util.GoalGeometry.incenter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

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
public final class FieldConstants {

    private FieldConstants() {
        // Utility only.
    }
    /**
     * Live-tunable offsets added to the goal Chebyshev center to produce the
     * actual aim target. Exposed via Panels FTC Dashboard for in-match tuning.
     */
    @Configurable
    public static class BasketTargetOffsets {
        /** Blue basket X offset from Chebyshev center (inches). Negative = left toward corner. */
        public static double blueDeltaX = 0.0;
        /** Blue basket Y offset from Chebyshev center (inches). Positive = up toward corner. */
        public static double blueDeltaY = 0.0;
        /** Red basket X offset from Chebyshev center (inches). Positive = right toward corner. */
        public static double redDeltaX = 0.0;
        /** Red basket Y offset from Chebyshev center (inches). Positive = up toward corner. */
        public static double redDeltaY = 0.0;
    }


    /**
     * Field interior width and depth in inches — the single source of truth for
     * field dimensions across the codebase. Used by:
     * <ul>
     *   <li>{@link AutoField#poseForAlliance} (X mirror)</li>
     *   <li>{@link PoseFrames#ftcToPedro} / {@link PoseFrames#pedroToFtc}
     *       (half-field offset between FTC center-origin and Pedro corner-origin frames)</li>
     *   <li>Goal corner constants below ({@code BLUE_GOAL_CORNER_PEDRO}, etc.)
     *       — the corner is at the walls, so its coordinate equals the field width/depth</li>
     *   <li>Vision goal-side classification in {@code VisionSubsystemLimelight}</li>
     *   <li>Pose-validity bounds checks in OpModes and DriveSubsystem</li>
     * </ul>
     *
     * 141.5 matches Pedro's library hardcoded default in {@code Pose.mirror()}
     * and is consistent with an installed FTC field: 6 tiles × ~23.625" (foam
     * tile with interlock loss) = 141.75". The 2025 game manual cites a nominal
     * 144" "approximately ... bounded by the inside surface of the walls" — that
     * is the design target, not the installed reality, which is consistently
     * smaller after the interlocks lose ~0.4" each.
     *
     * If you measure a different field width at competition, change this one
     * constant and every dimension-dependent calculation in the codebase
     * updates consistently.
     */
    public static final double FIELD_WIDTH_INCHES = 141.5;

    /**
     * Goal AprilTags. The Pedro-frame positions are derived from the FTC SDK's
     * {@link AprilTagGameDatabase#getDecodeTagLibrary()} — the authoritative
     * source for the 2025 DECODE field, which matches the .fmap Limelight uses
     * for MT2 localization. When FIRST publishes corrections in a future SDK
     * release, the constants pick them up automatically.
     *
     * <p>The SDK stores 3D positions in FTC frame (origin at field center,
     * with a Z height); we project to 2D and convert to Pedro frame.
     */
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final Pose BLUE_GOAL_TAG_APRILTAG = aprilTagPedroFromSdk(BLUE_GOAL_TAG_ID);

    public static final int RED_GOAL_TAG_ID = 24;
    public static final Pose RED_GOAL_TAG_APRILTAG = aprilTagPedroFromSdk(RED_GOAL_TAG_ID);

    /**
     * Looks up a tag in the FTC SDK's DECODE library and returns its position
     * in Pedro frame (2D, ignoring Z height). Returns null if the tag has no
     * fixed position in the database (e.g. obelisk motif tags 21–23).
     */
    private static Pose aprilTagPedroFromSdk(int tagId) {
        AprilTagMetadata m = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(tagId);
        if (m == null || m.fieldPosition == null) return null;
        double xInches = m.distanceUnit.toInches(m.fieldPosition.get(0));
        double yInches = m.distanceUnit.toInches(m.fieldPosition.get(1));
        return PoseFrames.ftcToPedro(new Pose(xInches, yInches, 0.0));
    }

    /**
     * Motif AprilTags
     */
    public static final int DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID = 21;
    public static final int DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID = 22;
    public static final int DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID = 23;

    /**
     * Triangle corner locations of the goal openings (in inches) PEDRO Coordinate Frame.
     * The corner sits at the intersection of the side wall and the back wall, so its X/Y
     * coordinates derive from {@link #FIELD_WIDTH_INCHES}. The "FAR" point is 25" along
     * the back wall from the corner.
     */
    public static final Pose BLUE_GOAL_LOWER_PEDRO = new Pose(0.0,   115.0, 0.0);
    public static final Pose BLUE_GOAL_CORNER_PEDRO = new Pose(0.0,   FIELD_WIDTH_INCHES, 0.0);
    public static final Pose BLUE_GOAL_FAR_PEDRO = new Pose(25.0,  FIELD_WIDTH_INCHES, 0.0);

    public static final Pose RED_GOAL_LOWER_PEDRO = new Pose(FIELD_WIDTH_INCHES, 115.0, 0.0);
    public static final Pose RED_GOAL_CORNER_PEDRO = new Pose(FIELD_WIDTH_INCHES, FIELD_WIDTH_INCHES, 0.0);
    public static final Pose RED_GOAL_FAR_PEDRO = new Pose(FIELD_WIDTH_INCHES - 25.0, FIELD_WIDTH_INCHES, 0.0);

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
                BLUE_GOAL_CENTER.getX() + BasketTargetOffsets.blueDeltaX,
                BLUE_GOAL_CENTER.getY() + BasketTargetOffsets.blueDeltaY,
                0.0
        );
        RobotState.putPose("Pose/Goal/BLUE_INCENTER_GOAL", triangleIncenterGoalPose);

        Pose chebyshevGoalPedroPose = new Pose(
                BLUE_GOAL_CHEBYSHEV_PEDRO.getX() + BasketTargetOffsets.blueDeltaX,
                BLUE_GOAL_CHEBYSHEV_PEDRO.getY() + BasketTargetOffsets.blueDeltaY,
                0.0
        );
        Pose chebyshevGoalFTCPose = PoseFrames.pedroToFtc(chebyshevGoalPedroPose);
        RobotState.putPose("Pose/Goal/BLUE_CHEBYSHEV_GOAL", chebyshevGoalFTCPose); //Publish the chebyshev pose for AS

        return chebyshevGoalPedroPose;
    }

    /**
     * Gets the current red basket aiming target with configurable offset.
     */
    public static Pose getRedBasketTarget() {
        Pose triangleIncenterGoalPose =  new Pose(
                RED_GOAL_CENTER.getX() + BasketTargetOffsets.redDeltaX,
                RED_GOAL_CENTER.getY() + BasketTargetOffsets.redDeltaY,
                0.0
        );
        RobotState.putPose("Pose/Goal/RED_INCENTER_GOAL", triangleIncenterGoalPose);

        Pose chebyshevGoalPedroPose = new Pose(
                RED_GOAL_CHEBYSHEV_PEDRO.getX() + BasketTargetOffsets.redDeltaX,
                RED_GOAL_CHEBYSHEV_PEDRO.getY() + BasketTargetOffsets.redDeltaY,
                0.0
        );
        Pose chebyshevGoalFTCPose = PoseFrames.pedroToFtc(chebyshevGoalPedroPose);
        RobotState.putPose("Pose/Goal/RED_CHEBYSHEV_GOAL", chebyshevGoalFTCPose); //Publish the chebyshev pose for AS

        return chebyshevGoalPedroPose;
    }

    @Configurable
    public static class AimOffsets {
        /** Fudge factor added to the computed aim angle (degrees). Positive = CCW (aim further left). */
        public static double aimAngleOffsetDeg = 0.0;
    }

    /**
     * Computes the aim angle from a robot pose to a target pose.
     */
    public static double getAimAngleTo(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.atan2(dy, dx) + Math.toRadians(AimOffsets.aimAngleOffsetDeg);
    }

    /** Straight-line distance (inches) between two field poses. */
    public static double getDistanceTo(Pose robotPose, Pose targetPose) {
        return Math.hypot(
                targetPose.getX() - robotPose.getX(),
                targetPose.getY() - robotPose.getY());
    }

}
