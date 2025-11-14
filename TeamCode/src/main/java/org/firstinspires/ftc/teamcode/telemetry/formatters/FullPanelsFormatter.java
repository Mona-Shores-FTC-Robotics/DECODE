package org.firstinspires.ftc.teamcode.telemetry.formatters;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.telemetry.data.LauncherTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

/**
 * Formats telemetry for FullPanels (FTControl Panels).
 * This is the live telemetry system that publishes to panels.debug() calls.
 * <p>
 * Note: "debug" here refers to the FullPanels API method name, not debugging level.
 * FullPanels telemetry is only active in DEBUG mode (disabled in MATCH/PRACTICE for performance).
 * </p>
 */
public class FullPanelsFormatter {

    /**
     * Publish telemetry to FullPanels.
     * This should only be called when TelemetrySettings.shouldSendFullPanels() returns true.
     */
    public void publish(TelemetryManager panels, RobotTelemetryData data) {
        if (panels == null) {
            return;
        }

        // Mode and context
        panels.debug("mode/label", data.context.opMode.toLowerCase());
        panels.debug("mode/drive", data.drive.driveMode);
        panels.debug("mode/aim/enabled", data.drive.aimMode);

        // Drive inputs and state
        panels.debug("drive/lx", data.drive.requestX);
        panels.debug("drive/ly", data.drive.requestY);
        panels.debug("drive/rx", data.drive.requestRot);
        panels.debug("drive/slowMode", data.drive.slowMode);
        panels.debug("drive/aimMode", data.drive.aimMode);
        panels.debug("drive/commandTurn", data.drive.commandTurn);

        // Drive motor powers
        panels.debug("drive/lfPower", data.drive.leftFront.power);
        panels.debug("drive/rfPower", data.drive.rightFront.power);
        panels.debug("drive/lbPower", data.drive.leftBack.power);
        panels.debug("drive/rbPower", data.drive.rightBack.power);

        // Drive motor velocities
        panels.debug("drive/lfVelIps", data.drive.leftFront.velocityIps);
        panels.debug("drive/rfVelIps", data.drive.rightFront.velocityIps);
        panels.debug("drive/lbVelIps", data.drive.leftBack.velocityIps);
        panels.debug("drive/rbVelIps", data.drive.rightBack.velocityIps);

        // Drive pose
        panels.debug("drive/xIn", data.pose.poseXIn);
        panels.debug("drive/yIn", data.pose.poseYIn);
        panels.debug("drive/headingDeg", data.pose.headingDeg);

        // Vision
        panels.debug("vision/tag/visible", data.vision.hasTag);
        panels.debug("vision/tag/id", data.vision.tagId);
        panels.debug("vision/tag/rangeIn", data.vision.rangeIn);
        panels.debug("vision/tag/bearingDeg", data.vision.bearingDeg);
        panels.debug("vision/tag/yawDeg", data.vision.yawDeg);
        panels.debug("vision/odometryPending", data.vision.odometryPending);

        // Launcher high-level
        panels.debug("launcher/control/mode", data.launcher.controlMode);

        // Launcher per-lane (left)
        publishLaneData(panels, "launcher/lanes/left", data.launcher.left);

        // Launcher per-lane (center)
        publishLaneData(panels, "launcher/lanes/center", data.launcher.center);

        // Launcher per-lane (right)
        publishLaneData(panels, "launcher/lanes/right", data.launcher.right);

        // Intake
        panels.debug("intake/mode", data.intake.mode);
        panels.debug("intake/power", data.intake.power);
        panels.debug("intake/artifactCount", data.intake.artifactCount);
        panels.debug("intake/artifactState", data.intake.artifactState);
        panels.debug("intake/roller/active", data.intake.rollerActive);
        panels.debug("intake/roller/position", data.intake.rollerPosition);
    }

    /**
     * Publish per-lane launcher data to FullPanels.
     */
    private void publishLaneData(TelemetryManager panels, String prefix, LauncherTelemetryData.LaneData lane) {
        panels.debug(prefix + "/targetRpm", lane.targetRpm);
        panels.debug(prefix + "/currentRpm", lane.currentRpm);
        panels.debug(prefix + "/power", lane.power);
        panels.debug(prefix + "/ready", lane.ready);
        panels.debug(prefix + "/phase", lane.phase);
        panels.debug(prefix + "/bangToHoldCount", lane.bangToHoldCount);
        panels.debug(prefix + "/hoodPosition", lane.hoodPosition);
        panels.debug(prefix + "/feederPosition", lane.feederPosition);
    }
}
