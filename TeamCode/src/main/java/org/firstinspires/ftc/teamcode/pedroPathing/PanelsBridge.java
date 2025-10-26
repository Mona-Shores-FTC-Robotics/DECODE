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

        if (drawPointRows(points, style)) {
            return;
        }

        drawComponentColumns(points, style);
    }

    private static boolean drawPointRows(double[][] points, Style style) {
        int count = points.length;
        if (count < 2) {
            return false;
        }

        double[] first = points[0];
        if (first == null || first.length < 2) {
            return false;
        }

        double startX = sanitize(first[0]);
        double startY = sanitize(first[1]);

        panelsField.setStyle(style);
        panelsField.moveCursor(startX, startY);

        boolean drewSegment = false;
        for (int i = 1; i < count; i++) {
            double[] point = points[i];
            if (point == null || point.length < 2) {
                continue;
            }

            double x = sanitize(point[0]);
            double y = sanitize(point[1]);
            panelsField.line(x, y);
            drewSegment = true;
        }

        return drewSegment;
    }

    private static boolean drawComponentColumns(double[][] points, Style style) {
        if (points.length < 2) {
            return false;
        }

        double[] xs = points[0];
        double[] ys = points[1];
        if (xs == null || ys == null) {
            return false;
        }

        int limit = Math.min(xs.length, ys.length);
        if (limit < 2) {
            return false;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(sanitize(xs[0]), sanitize(ys[0]));

        boolean drewSegment = false;
        for (int i = 1; i < limit; i++) {
            double x = sanitize(xs[i]);
            double y = sanitize(ys[i]);
            panelsField.line(x, y);
            drewSegment = true;
        }

        return drewSegment;
    }

    private static double sanitize(double value) {
        return Double.isNaN(value) ? 0.0 : value;
    }

}
