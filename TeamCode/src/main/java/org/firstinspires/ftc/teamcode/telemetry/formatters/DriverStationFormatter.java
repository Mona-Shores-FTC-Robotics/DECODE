package org.firstinspires.ftc.teamcode.telemetry.formatters;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Formats telemetry for the driver station display.
 * Provides tiered output based on telemetry level (MATCH/PRACTICE/DEBUG).
 * <p>
 * In DEBUG mode, supports multiple pages (switch with dpad up/down):
 * - Page 1: Overview (alliance, time, pose, launcher, artifacts)
 * - Page 2: Drive (modes, motor powers/velocities)
 * - Page 3: Launcher (per-lane detailed info)
 * - Page 4: Vision & Intake (tag detection, intake state)
 * - Page 5: Timing (loop performance breakdown)
 * - Page 6: Color Sensors (per-lane RGB/HSV values for tuning)
 * - Page 5: Controls (driver and operator bindings)
 * - Page 6: Timing (loop performance metrics)
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
        VISION_INTAKE(4, "Vision/Intake"),
        CONTROLS(5, "Controls"),
        TIMING(6, "Timing"),
        COLOR_SENSORS(7, "Color Sensors"); // Why was 6 afraid of 7? Because 7 ate 9!

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

        // Robot config (which robot - 19429 or 20245)
        telemetry.addData("Robot", data.context.robotConfig);

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

        // Launcher per-lane readiness (critical for driver)
        telemetry.addData("Launcher", "L:%s C:%s R:%s",
                data.launcher.left.ready ? "✓" : "✗",
                data.launcher.center.ready ? "✓" : "✗",
                data.launcher.right.ready ? "✓" : "✗");

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

        // Robot config (which robot - 19429 or 20245)
        telemetry.addData("Robot", data.context.robotConfig);

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

        // Launcher per-lane readiness
        telemetry.addData("Launcher", "L:%s C:%s R:%s",
                data.launcher.left.ready ? "✓" : "✗",
                data.launcher.center.ready ? "✓" : "✗",
                data.launcher.right.ready ? "✓" : "✗");

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
            case CONTROLS:
                publishDebugControls(telemetry, data);
                break;
            case TIMING:
                publishDebugTiming(telemetry, data);
                break;
            case COLOR_SENSORS:
                publishDebugColorSensors(telemetry, data);
                break;
        }

        telemetry.update();
    }

    /**
     * Page 5: Controls - driver and operator bindings reference.
     * Shows button mappings for quick reference during tuning.
     */
    private void publishDebugControls(Telemetry telemetry, RobotTelemetryData data) {
        // Driver controls (Gamepad 1)
        telemetry.addLine("=== DRIVER (GP1) ===");
        for (String line : org.firstinspires.ftc.teamcode.bindings.DriverBindings.controlsSummary()) {
            telemetry.addLine(line);
        }

        // Operator controls (Gamepad 2)
        telemetry.addLine();
        telemetry.addLine("=== OPERATOR (GP2) ===");
        for (String line : org.firstinspires.ftc.teamcode.bindings.OperatorBindings.controlsSummary()) {
            telemetry.addLine(line);
        }
    }

    /**
     * Page 6: Timing - complete loop performance metrics.
     * Shows timing breakdown from previous loop iteration (N-1).
     */
    private void publishDebugTiming(Telemetry telemetry, RobotTelemetryData data) {
        // Calculate telemetry time, but guard against initial 0 value
        double totalTelemetryMs = (data.timing.telemetryStartNs > 0)
                ? TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - data.timing.telemetryStartNs)
                : 0.0;
        // Total loop time
        telemetry.addData("Total Loop", "%.1f ms", data.timing.totalLoopMs(totalTelemetryMs));

        // Main loop overhead
        telemetry.addData("Main Loop", "%.1f ms", data.timing.mainLoopMs);

        telemetry.addData("Telemetry", "%.1f ms", totalTelemetryMs);

        // Subsystem breakdown
        telemetry.addData("Subsystems", "%.1f ms total", data.timing.totalSubsystemMs());
        telemetry.addData("  Drive", "%.1f ms", data.timing.driveMs);
        telemetry.addData("  Intake", "%.1f ms", data.timing.intakeMs);
        telemetry.addData("  Launcher", "%.1f ms", data.timing.launcherMs);
        telemetry.addData("  Lighting", "%.1f ms", data.timing.lightingMs);
        telemetry.addData("  Vision", "%.1f ms", data.timing.visionMs);
    }

    /**
     * Page 1: Overview - key match info.
     */
    private void publishDebugOverview(Telemetry telemetry, RobotTelemetryData data) {
        // Alliance and time
        telemetry.addData("Alliance", data.context.alliance.displayName());
        telemetry.addData("Robot", data.context.robotConfig);
        telemetry.addData("Launcher Mode", data.context.launcherMode);
        telemetry.addData("Motif Tail", data.context.motifTail);
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

        // Launcher per-lane readiness
        telemetry.addData("Launcher", "L:%s C:%s R:%s",
                data.launcher.left.ready ? "✓" : "✗",
                data.launcher.center.ready ? "✓" : "✗",
                data.launcher.right.ready ? "✓" : "✗");

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

        // Calculate telemetry time, but guard against initial 0 value
        double totalTelemetryMs = (data.timing.telemetryStartNs > 0)
                ? TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - data.timing.telemetryStartNs)
                : 0.0;
        telemetry.addData("Total Loop Time", "%.1f ms", data.timing.totalLoopMs(totalTelemetryMs));
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
        telemetry.addData("Control", "Feedforward + Proportional");

        telemetry.addData("Left",
                "Target=%.0f  Curr=%.0f  Pow=%.2f  %s",
                data.launcher.left.targetRpm,
                data.launcher.left.currentRpm,
                data.launcher.left.power,
                data.launcher.left.ready ? "READY" : "WAIT");
        telemetry.addData("  Hood/Feeder", "%.2f / %.2f",
                data.launcher.left.hoodPosition,
                data.launcher.left.feederPosition);

        telemetry.addData("Center",
                "Target=%.0f  Curr=%.0f  Pow=%.2f  %s",
                data.launcher.center.targetRpm,
                data.launcher.center.currentRpm,
                data.launcher.center.power,
                data.launcher.center.ready ? "READY" : "WAIT");
        telemetry.addData("  Hood/Feeder", "%.2f / %.2f",
                data.launcher.center.hoodPosition,
                data.launcher.center.feederPosition);

        telemetry.addData("Right",
                "Target=%.0f  Curr=%.0f  Pow=%.2f  %s",
                data.launcher.right.targetRpm,
                data.launcher.right.currentRpm,
                data.launcher.right.power,
                data.launcher.right.ready ? "READY" : "WAIT");
        telemetry.addData("  Hood/Feeder", "%.2f / %.2f",
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
                    Math.toDegrees(data.vision.headingRad));
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
     * Page 6: Color Sensors - per-lane RGB/HSV values for tuning.
     */
    private void publishDebugColorSensors(Telemetry telemetry, RobotTelemetryData data) {
        // Show classifier mode at top
        String classifierMode = org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.laneSensorConfig.classifier.mode;
        telemetry.addData("Classifier", classifierMode);

        // Left lane
        IntakeSubsystem.LaneSample leftSample = data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.LEFT);
        telemetry.addData("LEFT", "%s | conf=%.2f",
                data.intake.laneLeftSummary.color,
                leftSample != null ? leftSample.confidence : 0.0);

        if (leftSample != null && leftSample.sensorPresent) {
            telemetry.addData("  HSV", "H=%.0f° S=%.3f V=%.3f",
                    leftSample.hue,
                    leftSample.saturation,
                    leftSample.value);
            telemetry.addData("  RGB", "R=%d G=%d B=%d",
                    leftSample.scaledRed,
                    leftSample.scaledGreen,
                    leftSample.scaledBlue);
        }

        // Center lane
        IntakeSubsystem.LaneSample centerSample = data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.CENTER);
        telemetry.addData("CENTER", "%s | conf=%.2f",
                data.intake.laneCenterSummary.color,
                centerSample != null ? centerSample.confidence : 0.0);

        if (centerSample != null && centerSample.sensorPresent) {
            telemetry.addData("  HSV", "H=%.0f° S=%.3f V=%.3f",
                    centerSample.hue,
                    centerSample.saturation,
                    centerSample.value);
            telemetry.addData("  RGB", "R=%d G=%d B=%d",
                    centerSample.scaledRed,
                    centerSample.scaledGreen,
                    centerSample.scaledBlue);
        }

        // Right lane
        IntakeSubsystem.LaneSample rightSample = data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.RIGHT);
        telemetry.addData("RIGHT", "%s | conf=%.2f",
                data.intake.laneRightSummary.color,
                rightSample != null ? rightSample.confidence : 0.0);

        if (rightSample != null && rightSample.sensorPresent) {
            telemetry.addData("  HSV", "H=%.0f° S=%.3f V=%.3f",
                    rightSample.hue,
                    rightSample.saturation,
                    rightSample.value);
            telemetry.addData("  RGB", "R=%d G=%d B=%d",
                    rightSample.scaledRed,
                    rightSample.scaledGreen,
                    rightSample.scaledBlue);
        }
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
