package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Utility hooks that expose Pedro's internal Panels drawing helpers to the rest of the codebase.
 * <p>
 * The {@link Tuning} OpModes already wire Panels up for us, but autonomous routines live in a
 * different package and cannot access the package-private {@code Drawing} class directly. This
 * bridge keeps the setup and drawing logic in one place so match OpModes can mirror the behaviour
 * of the working tuning flows.
 */
public final class PanelsBridge {

    private PanelsBridge() {}


    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style PREVIEW_BLUE = new Style("preview-blue", "#2196F3", 0.7);
    private static final Style PREVIEW_RED = new Style("preview-red", "#F44336", 0.7);

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
