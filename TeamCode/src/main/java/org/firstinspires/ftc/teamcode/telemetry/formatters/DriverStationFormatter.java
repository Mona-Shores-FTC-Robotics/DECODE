package org.firstinspires.ftc.teamcode.telemetry.formatters;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

import java.util.Locale;

/**
 * Formats telemetry for the driver station display.
 * Provides tiered output based on telemetry level (MATCH/PRACTICE/DEBUG).
 */
public class DriverStationFormatter {

    /**
     * Publish MATCH level telemetry - minimal info for competition.
     * Target: <10ms overhead
     */
    public void publishMatch(Telemetry telemetry, RobotTelemetryData data) {
        if (telemetry == null) {
            return;
        }

        // Alliance
        telemetry.addData("Alliance", data.context.alliance.displayName());

        // Match time
        telemetry.addData("Match Time", formatMatchTime(data.context.matchTimeSec));

        // Pose
        if (data.pose.poseValid) {
            telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                    data.pose.poseXIn,
                    data.pose.poseYIn,
                    data.pose.headingDeg);
        } else {
            telemetry.addData("Pose", "(unavailable)");
        }

        // Launcher ready status (critical for driver)
        telemetry.addData("Launcher", data.launcher.ready ? "READY" : "NOT READY");

        // Vision tag detection (helpful for relocalization awareness)
        if (data.vision.hasTag) {
            telemetry.addData("Vision", "Tag %d", data.vision.tagId);
        }

        // Artifact count (critical for match strategy)
        telemetry.addData("Artifacts", "%d", data.intake.artifactCount);

        telemetry.update();
    }

    /**
     * Publish PRACTICE level telemetry - moderate detail for tuning.
     * Target: <20ms overhead
     */
    public void publishPractice(Telemetry telemetry, RobotTelemetryData data) {
        if (telemetry == null) {
            return;
        }

        // Alliance
        telemetry.addData("Alliance", data.context.alliance.displayName());

        // Match time
        telemetry.addData("Match Time", formatMatchTime(data.context.matchTimeSec));

        // Pose
        if (data.pose.poseValid) {
            telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                    data.pose.poseXIn,
                    data.pose.poseYIn,
                    data.pose.headingDeg);
        } else {
            telemetry.addData("Pose", "(unavailable)");
        }

        // Drive state
        telemetry.addData("Drive", "%s | aim=%s | slow=%s",
                data.drive.driveMode,
                data.drive.aimMode ? "ON" : "OFF",
                data.drive.slowMode ? "ON" : "OFF");

        // Launcher state
        telemetry.addData("Launcher", "%s | ready=%s | mode=%s",
                data.launcher.state,
                data.launcher.ready ? "YES" : "NO",
                data.launcher.spinMode);

        // Vision
        if (data.vision.hasTag) {
            telemetry.addData("Vision", "Tag %d @ %.1f in",
                    data.vision.tagId,
                    data.vision.rangeIn);
        }

        // Intake
        telemetry.addData("Intake", "%s | artifacts=%d",
                data.intake.mode,
                data.intake.artifactCount);

        telemetry.update();
    }

    /**
     * Publish DEBUG level telemetry - full diagnostics for development.
     * No performance target - comprehensive detail.
     */
    public void publishDebug(Telemetry telemetry, RobotTelemetryData data) {
        if (telemetry == null) {
            return;
        }

        // Alliance and time
        telemetry.addData("Alliance", data.context.alliance.displayName());
        telemetry.addData("Match Time", formatMatchTime(data.context.matchTimeSec));

        // Pose
        if (data.pose.poseValid) {
            telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                    data.pose.poseXIn,
                    data.pose.poseYIn,
                    data.pose.headingDeg);
        } else {
            telemetry.addData("Pose", "(unavailable)");
        }

        // Drive detailed
        telemetry.addData("Drive", "%s | aim=%s | slow=%s",
                data.drive.driveMode,
                data.drive.aimMode ? "ON" : "OFF",
                data.drive.slowMode ? "ON" : "OFF");
        telemetry.addData("  Turn Cmd", "%.2f", data.drive.commandTurn);
        telemetry.addData("  LF", "P=%.2f V=%.1f ips",
                data.drive.leftFront.power,
                data.drive.leftFront.velocityIps);
        telemetry.addData("  RF", "P=%.2f V=%.1f ips",
                data.drive.rightFront.power,
                data.drive.rightFront.velocityIps);
        telemetry.addData("  LB", "P=%.2f V=%.1f ips",
                data.drive.leftBack.power,
                data.drive.leftBack.velocityIps);
        telemetry.addData("  RB", "P=%.2f V=%.1f ips",
                data.drive.rightBack.power,
                data.drive.rightBack.velocityIps);

        // Launcher detailed
        telemetry.addData("Launcher", "%s | ready=%s | control=%s",
                data.launcher.state,
                data.launcher.ready ? "YES" : "NO",
                data.launcher.controlMode);
        telemetry.addData("  Left",
                "T=%.0f  C=%.0f  P=%.2f  H=%.2f  F=%.2f  %s  %s",
                data.launcher.left.targetRpm,
                data.launcher.left.currentRpm,
                data.launcher.left.power,
                data.launcher.left.hoodPosition,
                data.launcher.left.feederPosition,
                data.launcher.left.ready ? "READY" : "----",
                data.launcher.left.phase);
        telemetry.addData("  Center",
                "T=%.0f  C=%.0f  P=%.2f  H=%.2f  F=%.2f  %s  %s",
                data.launcher.center.targetRpm,
                data.launcher.center.currentRpm,
                data.launcher.center.power,
                data.launcher.center.hoodPosition,
                data.launcher.center.feederPosition,
                data.launcher.center.ready ? "READY" : "----",
                data.launcher.center.phase);
        telemetry.addData("  Right",
                "T=%.0f  C=%.0f  P=%.2f  H=%.2f  F=%.2f  %s  %s",
                data.launcher.right.targetRpm,
                data.launcher.right.currentRpm,
                data.launcher.right.power,
                data.launcher.right.hoodPosition,
                data.launcher.right.feederPosition,
                data.launcher.right.ready ? "READY" : "----",
                data.launcher.right.phase);

        // Vision detailed
        if (data.vision.hasTag) {
            telemetry.addData("Vision", "Tag %d @ %.1f in | bearing=%.1f° | yaw=%.1f°",
                    data.vision.tagId,
                    data.vision.rangeIn,
                    data.vision.bearingDeg,
                    data.vision.yawDeg);
        }

        // Intake detailed
        telemetry.addData("Intake", "%s | power=%.2f",
                data.intake.mode,
                data.intake.power);
        if (data.intake.rollerPresent) {
            telemetry.addData("  Roller", "%s @ %.2f",
                    data.intake.rollerActive ? "active" : "idle",
                    data.intake.rollerPosition);
        }
        telemetry.addData("  Artifacts", "%d (%s)",
                data.intake.artifactCount,
                data.intake.artifactState);

        telemetry.update();
    }

    /**
     * Format match time as MM:SS.
     */
    private String formatMatchTime(double timeSec) {
        if (timeSec < 0) {
            return "0:00";
        }
        int minutes = (int) (timeSec / 60.0);
        int seconds = (int) (timeSec % 60.0);
        return String.format(Locale.US, "%d:%02d", minutes, seconds);
    }
}
