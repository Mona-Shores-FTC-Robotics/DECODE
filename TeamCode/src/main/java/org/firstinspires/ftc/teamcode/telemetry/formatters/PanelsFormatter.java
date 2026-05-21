package org.firstinspires.ftc.teamcode.telemetry.formatters;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.telemetry.data.LauncherTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

/**
 * Formats telemetry for the Bylazar Panels dashboard.
 *
 * Feeds three Panels widget types:
 *   - Telemetry widget: human-readable status lines via TelemetryManager.debug()
 *   - Field widget:     live robot + vision pose overlay via PanelsField
 *
 * Never touches the FTC SDK Telemetry object — uses update() with no args so
 * the Driver Station output is unaffected. Safe to call alongside DashboardFormatter.
 *
 * MATCH mode: skipped entirely (same guard as DashboardFormatter).
 * PRACTICE:   pose, launcher RPM, intake state, loop time, field overlay.
 * VERBOSE:    adds per-subsystem timing, vision details, vision pose overlay.
 */
public class PanelsFormatter {

    private static final Style ROBOT_STYLE = new Style("robot", "#3F51B5", 1.5);
    private static final Style VISION_STYLE = new Style("vision", "#4CAF50", 1.5);
    private static final double ROBOT_RADIUS = 9.0;

    private final TelemetryManager telemetry;
    private final FieldManager field;

    public PanelsFormatter() {
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public void publish(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        if (level == TelemetrySettings.TelemetryLevel.MATCH) {
            return;
        }
        publishText(data, level);
        publishField(data, level);
    }

    private void publishText(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        // --- Numeric values via addData() → appear as named series in Graph widget ---

        // Pose
        telemetry.addData("pose_x", data.pose.poseXIn);
        telemetry.addData("pose_y", data.pose.poseYIn);
        telemetry.addData("pose_hdg_deg", Math.toDegrees(data.pose.headingRad));

        // Launcher RPM per lane
        telemetry.addData("left_rpm", data.launcher.left.currentRpm);
        telemetry.addData("left_target", data.launcher.left.targetRpm);
        telemetry.addData("center_rpm", data.launcher.center.currentRpm);
        telemetry.addData("center_target", data.launcher.center.targetRpm);
        telemetry.addData("right_rpm", data.launcher.right.currentRpm);
        telemetry.addData("right_target", data.launcher.right.targetRpm);

        // Vision range (graphable — useful for distance tuning)
        if (data.vision.hasTag) {
            telemetry.addData("range_in", data.vision.rangeIn);
            telemetry.addData("tx_deg", data.vision.txDeg);
        }

        // Loop time
        telemetry.addData("loop_ms", data.timing.mainLoopMs);

        // --- String/status via debug() → appear as text in Telemetry widget ---

        telemetry.debug(String.format("L: %s  C: %s  R: %s",
                laneLabel(data.launcher.left),
                laneLabel(data.launcher.center),
                laneLabel(data.launcher.right)));

        telemetry.debug(String.format("intake: %s x%d  [%s | %s | %s]",
                data.intake.mode,
                data.intake.artifactCount,
                data.intake.laneLeftSummary.color,
                data.intake.laneCenterSummary.color,
                data.intake.laneRightSummary.color));

        telemetry.debug(data.vision.hasTag
                ? String.format("tag %d @ %.1f\"", data.vision.tagId, data.vision.rangeIn)
                : "no tag");

        if (level == TelemetrySettings.TelemetryLevel.VERBOSE) {
            telemetry.addData("drive_ms", data.timing.driveMs);
            telemetry.addData("intake_ms", data.timing.intakeMs);
            telemetry.addData("launcher_ms", data.timing.launcherMs);
            telemetry.addData("vision_ms", data.timing.visionMs);
            telemetry.addData("bearing_deg", data.vision.bearingDeg);
            telemetry.debug(String.format("alliance: %s  opmode: %s",
                    data.context.alliance.name(), data.context.opMode));
        }

        // update() with no args — sends to Panels only, never touches DS
        telemetry.update();
    }

    private void publishField(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        if (data.pose.poseValid) {
            drawRobot(new Pose(data.pose.poseXIn, data.pose.poseYIn, data.pose.headingRad), ROBOT_STYLE);
        }

        // Vision pose overlay in VERBOSE only
        if (level == TelemetrySettings.TelemetryLevel.VERBOSE && data.pose.visionPoseValid) {
            drawRobot(new Pose(data.pose.visionPoseXIn, data.pose.visionPoseYIn, data.pose.visionHeadingRad), VISION_STYLE);
        }

        field.update();
    }

    private void drawRobot(Pose pose, Style style) {
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())) {
            return;
        }
        field.setStyle(style);
        field.moveCursor(pose.getX(), pose.getY());
        field.circle(ROBOT_RADIUS);

        double dx = Math.cos(pose.getHeading()) * ROBOT_RADIUS;
        double dy = Math.sin(pose.getHeading()) * ROBOT_RADIUS;
        field.moveCursor(pose.getX() + dx / 2.0, pose.getY() + dy / 2.0);
        field.line(pose.getX() + dx, pose.getY() + dy);
    }

    private static String laneLabel(LauncherTelemetryData.LaneData lane) {
        return String.format("%d/%d%s",
                (int) lane.currentRpm,
                (int) lane.targetRpm,
                lane.ready ? "✓" : " ");
    }
}
