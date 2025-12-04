package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Utility hooks that expose Pedro's internal Panels drawing helpers to the rest of the codebase.
 * <p>
 * The {@link Tuning} OpModes already wire Panels up for us, but autonomous routines live in a
 * different package and cannot access the package-private {@code Drawing} class directly. This
 * bridge keeps the setup and drawing logic in one place so match OpModes can mirror the behaviour
 * of the working tuning flows.
 * </p>
 * <p>
 * <b>Pose Visualization:</b>
 * Multiple poses can be drawn with different styles:
 * <ul>
 *   <li><b>Odometry (Blue)</b> - Primary robot pose from Pinpoint odometry</li>
 *   <li><b>Vision (Lime)</b> - AprilTag-derived pose when available</li>
 *   <li><b>Fused (Yellow)</b> - Pose fusion output (if using PoseFusion)</li>
 *   <li><b>Target (Orange)</b> - Path following target point</li>
 * </ul>
 * </p>
 */
public final class PanelsBridge {

    private PanelsBridge() {}


    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    // Path preview styles
    private static final Style PREVIEW_BLUE = new Style("preview-blue", "#2196F3", 0.7);
    private static final Style PREVIEW_RED = new Style("preview-red", "#F44336", 0.7);

    // Pose visualization styles (match AdvantageScope colors for consistency)
    /** Primary odometry pose - blue, solid */
    public static final Style POSE_ODOMETRY = new Style("pose-odom", "#3F51B5", 0.9);
    /** Vision/AprilTag pose - lime green */
    public static final Style POSE_VISION = new Style("pose-vision", "#4CAF50", 0.8);
    /** Fused pose - yellow/gold */
    public static final Style POSE_FUSED = new Style("pose-fused", "#FFC107", 0.85);
    /** Target/goal pose - orange */
    public static final Style POSE_TARGET = new Style("pose-target", "#FF9800", 0.7);
    /** Pose history trail - lighter blue */
    public static final Style POSE_HISTORY = new Style("pose-history", "#90CAF9", 0.5);
    /** Frame outline - light gray, semi-transparent */
    public static final Style FRAME_OUTLINE = new Style("frame-outline", "#9E9E9E", 0.4);

    // =========================================================================
    // ROBOT DIMENSIONS (Configurable via Panels/FTC Dashboard)
    // Configure these to match your actual robot size for accurate visualization.
    // Standard FTC robots are 18x18 inches max.
    // =========================================================================

    /**
     * Robot visualization configuration.
     * Adjust these values in Panels Configurables or FTC Dashboard to match your robot.
     *
     * <p><b>IMPORTANT:</b> These dimensions should match your <b>drivetrain footprint</b>,
     * not your frame dimensions. The pose from odometry and Limelight represents the
     * center of the drivetrain, so visualization should show the drivetrain.</p>
     *
     * <p>DECODE robot specifics:
     * <ul>
     *   <li>Frame: ~17.5" x 18" (physical robot)</li>
     *   <li>Drivetrain: 14.25" wide x 4.75" long (what odometry tracks)</li>
     *   <li>Limelight configured to return drivetrain center</li>
     * </ul>
     * </p>
     */
    @Configurable
    public static class RobotVisualization {
        /**
         * Robot length in inches (front-to-back, direction of travel).
         * This is the WHEELBASE - front-to-back wheel spacing.
         * DECODE: 4.75" wheelbase, but we add some buffer for visualization.
         */
        public static double length = 10.0;

        /**
         * Robot width in inches (left-to-right).
         * This is the TRACK WIDTH - left-to-right wheel spacing.
         * DECODE: 14.25" track width.
         */
        public static double width = 14.25;

        /** Whether to draw robot as rectangle (true) or circle (false) */
        public static boolean drawRectangle = true;

        /** Show heading arrow inside robot */
        public static boolean showHeadingArrow = true;

        /**
         * Optional: Also draw frame outline (larger rectangle showing physical robot bounds).
         * Useful during development to see both drivetrain and frame.
         */
        public static boolean showFrameOutline = false;

        /** Frame length in inches (physical robot front-to-back) */
        public static double frameLength = 18.0;

        /** Frame width in inches (physical robot left-to-right) */
        public static double frameWidth = 17.5;
    }

    /** Legacy radius for circle mode - computed from dimensions */
    private static double getRobotRadius() {
        return Math.min(RobotVisualization.length, RobotVisualization.width) / 2.0;
    }

    /**
     * Connects to the FTControl Panels process and applies Pedro's field offsets. Call this once
     * when the OpMode is initialised so that subsequent draw calls render in the correct coordinate
     * space.
     *
     * @return the {@link TelemetryManager} instance Panels uses for combined field/DS telemetry
     */
    public static TelemetryManager preparePanels() {
        TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        return telemetryManager;
    }

    /**
     * Mirrors the {@link Drawing#drawDebug(Follower)} helper so autonomous OpModes can render the
     * current path, pose history, and robot pose on the Panels field view.
     *
     * @param follower active Pedro follower instance
     */
    public static void drawFollowerDebug(Follower follower) {
        if (follower == null) {
            return;
        }

        Drawing.drawDebug(follower);
    }

    /**
     * Draws the robot at its current pose on Panels, mirroring the behaviour used by the Pedro
     * localization tuning OpModes.
     *
     * @param follower active Pedro follower instance
     */
    public static void drawCurrentPose(Follower follower) {
        if (follower == null) {
            return;
        }

        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
    }

    /**
     * Draws the robot and its pose history on Panels so the motion trail matches the tuning tools.
     *
     * @param follower active Pedro follower instance
     */
    public static void drawCurrentPoseWithHistory(Follower follower) {
        if (follower == null) {
            return;
        }

        if (follower.getPoseHistory() != null) {
            Drawing.drawPoseHistory(follower.getPoseHistory());
        }
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
    }

    // =========================================================================
    // MULTI-POSE VISUALIZATION
    // These methods allow drawing multiple poses with different styles,
    // similar to AdvantageScope's multi-pose field view.
    // =========================================================================

    /**
     * Draw a pose on the Panels field with a specific style.
     * Does NOT call sendPacket() - call {@link #sendPacket()} after all draws are complete.
     *
     * @param pose  the pose to draw (may be null - will be skipped)
     * @param style the visual style to use
     */
    public static void drawPose(Pose pose, Style style) {
        if (pose == null || !isValidPose(pose)) {
            return;
        }
        drawRobotInternal(pose, style);
    }

    /**
     * Draw a pose on the Panels field using the default odometry style.
     * Does NOT call sendPacket().
     *
     * @param pose the pose to draw
     */
    public static void drawOdometryPose(Pose pose) {
        drawPose(pose, POSE_ODOMETRY);
    }

    /**
     * Draw a vision-derived pose on the Panels field (lime green).
     * Does NOT call sendPacket().
     *
     * @param pose the vision pose to draw
     */
    public static void drawVisionPose(Pose pose) {
        drawPose(pose, POSE_VISION);
    }

    /**
     * Draw a fused pose on the Panels field (yellow/gold).
     * Does NOT call sendPacket().
     *
     * @param pose the fused pose to draw
     */
    public static void drawFusedPose(Pose pose) {
        drawPose(pose, POSE_FUSED);
    }

    /**
     * Draw a target/goal pose on the Panels field (orange).
     * Does NOT call sendPacket().
     *
     * @param pose the target pose to draw
     */
    public static void drawTargetPose(Pose pose) {
        drawPose(pose, POSE_TARGET);
    }

    /**
     * Draw a marker (small circle without heading line) at a position.
     * Useful for waypoints or specific field locations.
     *
     * @param x     x coordinate in inches
     * @param y     y coordinate in inches
     * @param style the visual style to use
     */
    public static void drawMarker(double x, double y, Style style) {
        if (Double.isNaN(x) || Double.isNaN(y)) {
            return;
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(3.0); // Small marker
    }

    /**
     * Draw a line between two points.
     * Useful for showing vectors or connections.
     *
     * @param x1    start x
     * @param y1    start y
     * @param x2    end x
     * @param y2    end y
     * @param style the visual style to use
     */
    public static void drawLine(double x1, double y1, double x2, double y2, Style style) {
        if (Double.isNaN(x1) || Double.isNaN(y1) || Double.isNaN(x2) || Double.isNaN(y2)) {
            return;
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Send the accumulated field drawing commands to Panels.
     * Call this once after all poses and shapes have been drawn for the current frame.
     */
    public static void sendPacket() {
        panelsField.update();
    }

    /**
     * Comprehensive field drawing for telemetry.
     * Draws odometry pose, optional vision pose, and optional pose history.
     * Automatically calls sendPacket().
     *
     * @param odometryPose    primary robot pose from odometry (required)
     * @param visionPose      vision-derived pose (may be null)
     * @param includeHistory  whether to draw pose history trail
     * @param follower        follower for pose history (may be null if includeHistory is false)
     */
    public static void drawFieldView(
            Pose odometryPose,
            Pose visionPose,
            boolean includeHistory,
            Follower follower
    ) {
        // Draw pose history first (so it's behind the robot)
        if (includeHistory && follower != null && follower.getPoseHistory() != null) {
            Drawing.drawPoseHistory(follower.getPoseHistory(), POSE_HISTORY);
        }

        // Draw vision pose (behind odometry pose)
        if (visionPose != null && isValidPose(visionPose)) {
            drawRobotInternal(visionPose, POSE_VISION);
        }

        // Draw primary odometry pose (on top)
        if (odometryPose != null && isValidPose(odometryPose)) {
            drawRobotInternal(odometryPose, POSE_ODOMETRY);
        }

        // Send to Panels
        sendPacket();
    }

    /**
     * Draw the current autonomous path and target point (if path following is active).
     *
     * @param follower the Pedro follower
     */
    public static void drawPathFollowing(Follower follower) {
        if (follower == null || follower.getCurrentPath() == null) {
            return;
        }

        Path currentPath = follower.getCurrentPath();

        // Draw the path itself
        Drawing.drawPath(currentPath, POSE_TARGET);

        // Draw the closest point on path (target)
        double t = currentPath.getClosestPointTValue();
        Pose targetPose = follower.getPointFromPath(t);
        if (targetPose != null) {
            double heading = currentPath.getHeadingGoal(t);
            drawRobotInternal(new Pose(targetPose.getX(), targetPose.getY(), heading), POSE_TARGET);
        }
    }

    // =========================================================================
    // INTERNAL HELPERS
    // =========================================================================

    /**
     * Internal robot drawing - rectangle or circle with heading line.
     * Rectangle mode shows actual robot dimensions; circle mode is simpler.
     */
    private static void drawRobotInternal(Pose pose, Style style) {
        // Optionally draw frame outline first (so it's behind drivetrain)
        if (RobotVisualization.showFrameOutline) {
            drawFrameOutline(pose);
        }

        panelsField.setStyle(style);

        if (RobotVisualization.drawRectangle) {
            drawRobotRectangle(pose, style);
        } else {
            drawRobotCircle(pose, style);
        }
    }

    /**
     * Draw the physical robot frame outline (larger than drivetrain).
     * This shows where the actual robot body extends beyond the wheel positions.
     * Useful for understanding clearance and collision zones.
     */
    private static void drawFrameOutline(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double halfLength = RobotVisualization.frameLength / 2.0;
        double halfWidth = RobotVisualization.frameWidth / 2.0;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Corner positions (rotated by heading)
        double frX = x + halfLength * cos - halfWidth * sin;
        double frY = y + halfLength * sin + halfWidth * cos;
        double flX = x + halfLength * cos + halfWidth * sin;
        double flY = y + halfLength * sin - halfWidth * cos;
        double blX = x - halfLength * cos + halfWidth * sin;
        double blY = y - halfLength * sin - halfWidth * cos;
        double brX = x - halfLength * cos - halfWidth * sin;
        double brY = y - halfLength * sin + halfWidth * cos;

        // Draw frame outline (gray, semi-transparent)
        panelsField.setStyle(FRAME_OUTLINE);
        panelsField.moveCursor(frX, frY);
        panelsField.line(flX, flY);
        panelsField.line(blX, blY);
        panelsField.line(brX, brY);
        panelsField.line(frX, frY);
    }

    /**
     * Draw robot as a rectangle with heading indicator.
     * The rectangle is centered on the pose with the front facing the heading direction.
     */
    private static void drawRobotRectangle(Pose pose, Style style) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        // Half dimensions (read from configurable)
        double halfLength = RobotVisualization.length / 2.0;
        double halfWidth = RobotVisualization.width / 2.0;

        // Calculate corner offsets relative to robot center
        // Front-right, front-left, back-left, back-right (counter-clockwise)
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Corner positions (rotated by heading)
        // Front-right
        double frX = x + halfLength * cos - halfWidth * sin;
        double frY = y + halfLength * sin + halfWidth * cos;
        // Front-left
        double flX = x + halfLength * cos + halfWidth * sin;
        double flY = y + halfLength * sin - halfWidth * cos;
        // Back-left
        double blX = x - halfLength * cos + halfWidth * sin;
        double blY = y - halfLength * sin - halfWidth * cos;
        // Back-right
        double brX = x - halfLength * cos - halfWidth * sin;
        double brY = y - halfLength * sin + halfWidth * cos;

        // Draw rectangle outline
        panelsField.setStyle(style);
        panelsField.moveCursor(frX, frY);
        panelsField.line(flX, flY);  // Front edge
        panelsField.line(blX, blY);  // Left edge
        panelsField.line(brX, brY);  // Back edge
        panelsField.line(frX, frY);  // Right edge (close the rectangle)

        // Draw heading indicator (arrow from center to front)
        if (RobotVisualization.showHeadingArrow) {
            double arrowLength = halfLength * 0.8;
            double arrowEndX = x + arrowLength * cos;
            double arrowEndY = y + arrowLength * sin;

            panelsField.moveCursor(x, y);
            panelsField.line(arrowEndX, arrowEndY);

            // Draw arrowhead
            double arrowheadSize = 2.5;
            double arrowAngle = Math.toRadians(150); // 30 degrees from line

            double ah1X = arrowEndX + arrowheadSize * Math.cos(heading + arrowAngle);
            double ah1Y = arrowEndY + arrowheadSize * Math.sin(heading + arrowAngle);
            double ah2X = arrowEndX + arrowheadSize * Math.cos(heading - arrowAngle);
            double ah2Y = arrowEndY + arrowheadSize * Math.sin(heading - arrowAngle);

            panelsField.moveCursor(arrowEndX, arrowEndY);
            panelsField.line(ah1X, ah1Y);
            panelsField.moveCursor(arrowEndX, arrowEndY);
            panelsField.line(ah2X, ah2Y);
        }
    }

    /**
     * Draw robot as a circle with heading line (legacy mode).
     */
    private static void drawRobotCircle(Pose pose, Style style) {
        double radius = getRobotRadius();

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(radius);

        // Draw heading line
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * radius);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Check if a pose has valid (non-NaN) coordinates.
     */
    private static boolean isValidPose(Pose pose) {
        return !Double.isNaN(pose.getX()) &&
               !Double.isNaN(pose.getY()) &&
               !Double.isNaN(pose.getHeading());
    }

    public static void drawPreview(PathChain[] pathChains, Pose startPose, boolean isRed) {
        if (pathChains == null) {
            return;
        }
        Style style = isRed ? PREVIEW_RED : PREVIEW_BLUE;
        for (PathChain chain : pathChains) {
            if (chain == null) {
                continue;
            }
            drawPreviewChain(chain, style);
        }
        if (startPose != null) {
            Drawing.drawRobot(startPose, style);
        }
        Drawing.sendPacket();
    }

    private static void drawPreviewChain(PathChain chain, Style style) {
        for (int i = 0; i < chain.size(); i++) {
            drawPreviewPath(chain.getPath(i), style);
        }
    }

    private static void drawPreviewPath(Path path, Style style) {
        if (path == null) {
            return;
        }

        double[][] points = path.getPanelsDrawingPoints();
        if (points == null || points.length == 0) {
            return;
        }

        if (isComponentArray(points)) {
            drawFromComponents(points, style);
        } else {
            drawFromPointList(points, style);
        }
    }

    private static void drawFromComponents(double[][] points, Style style) {
        if (points.length < 2 || points[0] == null || points[1] == null) {
            return;
        }

        int count = Math.min(points[0].length, points[1].length);
        if (count < 2) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(sanitize(points[0][0]), sanitize(points[1][0]));
        for (int i = 1; i < count; i++) {
            panelsField.line(sanitize(points[0][i]), sanitize(points[1][i]));
        }
    }

    private static void drawFromPointList(double[][] points, Style style) {
        if (points.length < 2) {
            return;
        }

        double[] first = points[0];
        if (first == null || first.length < 2) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(sanitize(first[0]), sanitize(first[1]));
        for (int i = 1; i < points.length; i++) {
            double[] point = points[i];
            if (point == null || point.length < 2) {
                continue;
            }
            panelsField.line(sanitize(point[0]), sanitize(point[1]));
        }
    }

    private static boolean isComponentArray(double[][] points) {
        if (points.length < 2) {
            return false;
        }

        if (points[0] != null && points[1] != null) {
            int xs = points[0].length;
            int ys = points[1].length;
            if (xs > 1 && ys > 1) {
                return true;
            }
        }

        if (points.length == 2) {
            return true;
        }

        return false;
    }

    private static double sanitize(double value) {
        return Double.isNaN(value) ? 0.0 : value;
    }

}
