package org.firstinspires.ftc.teamcode.telemetry.formatters;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

import java.util.Locale;

/**
 * Formats telemetry for the driver station display.
 * Provides tiered output based on telemetry level (MATCH/PRACTICE/DEBUG).
 * <p>
 * In DEBUG mode, supports multiple pages (switch with dpad up/down):
 * - Page 1: Overview (alliance, time, pose, launcher, artifacts)
 * - Page 2: Drive (modes, motor powers/velocities)
 * - Page 3: Launcher (per-lane detailed info)
 * - Page 4: Vision & Intake (tag detection, intake state)
 * </p>
 */
public class DriverStationFormatter {

    /**
     * Telemetry page for DEBUG mode.
     */
    public enum DebugPage {
        OVERVIEW(1, "Overview"),
        DRIVE(2, "Drive"),
        LAUNCHER(3, "Launcher"),
        VISION_INTAKE(4, "Vision/Intake");

        public final int number;
        public final String name;

        DebugPage(int number, String name) {
            this.number = number;
            this.name = name;
        }

        public DebugPage next() {
            DebugPage[] pages = values();
            return pages[(this.ordinal() + 1) % pages.length];
        }

        public DebugPage previous() {
            DebugPage[] pages = values();
            return pages[(this.ordinal() - 1 + pages.length) % pages.length];
        }
    }

    private DebugPage currentPage = DebugPage.OVERVIEW;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    /**
     * Handle page navigation from gamepad dpad input.
     * Call this before publishing DEBUG telemetry.
     *
     * @param dpadUp Current state of dpad up
     * @param dpadDown Current state of dpad down
     */
    public void handlePageNavigation(boolean dpadUp, boolean dpadDown) {
        // Dpad up - next page (rising edge detection)
        if (dpadUp && !lastDpadUp) {
            currentPage = currentPage.next();
        }

        // Dpad down - previous page (rising edge detection)
        if (dpadDown && !lastDpadDown) {
            currentPage = currentPage.previous();
        }

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
    }

    /**
     * Get current debug page (for testing/display).
     */
    public DebugPage getCurrentPage() {
        return currentPage;
    }

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
     * <p>
     * Uses multi-page system to avoid screen blinking. Switch pages with dpad up/down.
     * </p>
     */
    public void publishDebug(Telemetry telemetry, RobotTelemetryData data) {
        if (telemetry == null) {
            return;
        }

        // Show page indicator at top
        telemetry.addData("Page", "%d/%d: %s (dpad ↑↓)",
                currentPage.number,
                DebugPage.values().length,
                currentPage.name);

        // Render current page
        switch (currentPage) {
            case OVERVIEW:
                publishDebugOverview(telemetry, data);
                break;
            case DRIVE:
                publishDebugDrive(telemetry, data);
                break;
            case LAUNCHER:
                publishDebugLauncher(telemetry, data);
                break;
            case VISION_INTAKE:
                publishDebugVisionIntake(telemetry, data);
                break;
        }

        telemetry.update();
    }

    /**
     * Page 1: Overview - key match info.
     */
    private void publishDebugOverview(Telemetry telemetry, RobotTelemetryData data) {
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

        // Launcher ready status
        telemetry.addData("Launcher", "%s | %s",
                data.launcher.ready ? "READY" : "NOT READY",
                data.launcher.spinMode);

        // Vision tag
        if (data.vision.hasTag) {
            telemetry.addData("Vision", "Tag %d @ %.1f in",
                    data.vision.tagId,
                    data.vision.rangeIn);
        } else {
            telemetry.addData("Vision", "No tag");
        }

        // Intake and artifacts
        telemetry.addData("Intake", "%s | artifacts=%d",
                data.intake.mode,
                data.intake.artifactCount);
    }

    /**
     * Page 2: Drive - motor powers and velocities.
     */
    private void publishDebugDrive(Telemetry telemetry, RobotTelemetryData data) {
        // Drive state
        telemetry.addData("Drive Mode", data.drive.driveMode);
        telemetry.addData("Aim", data.drive.aimMode ? "ON" : "OFF");
        telemetry.addData("Slow", data.drive.slowMode ? "ON" : "OFF");
        telemetry.addData("Turn Cmd", "%.2f", data.drive.commandTurn);

        // Motor powers and velocities
        telemetry.addData("LF", "P=%.2f  V=%.1f ips",
                data.drive.leftFront.power,
                data.drive.leftFront.velocityIps);
        telemetry.addData("RF", "P=%.2f  V=%.1f ips",
                data.drive.rightFront.power,
                data.drive.rightFront.velocityIps);
        telemetry.addData("LB", "P=%.2f  V=%.1f ips",
                data.drive.leftBack.power,
                data.drive.leftBack.velocityIps);
        telemetry.addData("RB", "P=%.2f  V=%.1f ips",
                data.drive.rightBack.power,
                data.drive.rightBack.velocityIps);
    }

    /**
     * Page 3: Launcher - per-lane detailed info.
     */
    private void publishDebugLauncher(Telemetry telemetry, RobotTelemetryData data) {
        telemetry.addData("State", "%s | %s | %s",
                data.launcher.state,
                data.launcher.ready ? "READY" : "NOT READY",
                data.launcher.controlMode);

        telemetry.addData("Left",
                "T=%.0f C=%.0f P=%.2f %s",
                data.launcher.left.targetRpm,
                data.launcher.left.currentRpm,
                data.launcher.left.power,
                data.launcher.left.phase);
        telemetry.addData("  H/F", "%.2f / %.2f",
                data.launcher.left.hoodPosition,
                data.launcher.left.feederPosition);

        telemetry.addData("Center",
                "T=%.0f C=%.0f P=%.2f %s",
                data.launcher.center.targetRpm,
                data.launcher.center.currentRpm,
                data.launcher.center.power,
                data.launcher.center.phase);
        telemetry.addData("  H/F", "%.2f / %.2f",
                data.launcher.center.hoodPosition,
                data.launcher.center.feederPosition);

        telemetry.addData("Right",
                "T=%.0f C=%.0f P=%.2f %s",
                data.launcher.right.targetRpm,
                data.launcher.right.currentRpm,
                data.launcher.right.power,
                data.launcher.right.phase);
        telemetry.addData("  H/F", "%.2f / %.2f",
                data.launcher.right.hoodPosition,
                data.launcher.right.feederPosition);
    }

    /**
     * Page 4: Vision & Intake - tag detection and intake state.
     */
    private void publishDebugVisionIntake(Telemetry telemetry, RobotTelemetryData data) {
        // Vision detailed
        if (data.vision.hasTag) {
            telemetry.addData("Vision Tag", "%d", data.vision.tagId);
            telemetry.addData("Range", "%.1f in", data.vision.rangeIn);
            telemetry.addData("Bearing", "%.1f°", data.vision.bearingDeg);
            telemetry.addData("Yaw", "%.1f°", data.vision.yawDeg);
            telemetry.addData("Pose", "x=%.1f  y=%.1f  h=%.1f°",
                    data.vision.poseXIn,
                    data.vision.poseYIn,
                    data.vision.poseHeadingDeg);
        } else {
            telemetry.addData("Vision", "No tag detected");
        }

        // Intake detailed
        telemetry.addData("Intake Mode", data.intake.mode);
        telemetry.addData("Power", "%.2f", data.intake.power);
        if (data.intake.rollerPresent) {
            telemetry.addData("Roller", "%s @ %.2f",
                    data.intake.rollerActive ? "active" : "idle",
                    data.intake.rollerPosition);
        }
        telemetry.addData("Artifacts", "%d (%s)",
                data.intake.artifactCount,
                data.intake.artifactState);
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
