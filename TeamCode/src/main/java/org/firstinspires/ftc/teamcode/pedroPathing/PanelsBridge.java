package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.Style;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;

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
            Drawing.drawPath(chain, style);
        }
        if (startPose != null) {
            Drawing.drawRobot(startPose, style);
        }
        Drawing.sendPacket();
    }

}