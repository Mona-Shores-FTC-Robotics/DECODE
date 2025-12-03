package org.firstinspires.ftc.teamcode.telemetry.formatters;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.telemetry.data.IntakeTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.LauncherTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

/**
 * Formats telemetry for FullPanels (FTControl Panels).
 * This is the live telemetry system that publishes to panels.debug() calls.
 * <p>
 * Note: "debug" here refers to the FullPanels API method name, not debugging level.
 * FullPanels telemetry is only active in DEBUG mode (disabled in MATCH/PRACTICE for performance).
 * </p>
 * <p>
 * <b>Panels Plugin Reference:</b>
 * <ul>
 *   <li><b>Telemetry</b> - Text values from debug() calls, organized hierarchically by "/" paths</li>
 *   <li><b>Graph</b> - Line graphs for numeric debug() values (configure in Panels UI)</li>
 *   <li><b>Field</b> - 2D robot visualization (see PanelsBridge.java)</li>
 *   <li><b>Configurables</b> - Live-tune @Configurable parameters</li>
 *   <li><b>Limelight Proxy</b> - Enable in Panels to see camera feed (no code needed)</li>
 *   <li><b>OpMode Control</b> - Start/stop OpModes from Panels (no code needed)</li>
 *   <li><b>Theme</b> - UI appearance customization (no code needed)</li>
 * </ul>
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

        // =====================================================================
        // SECTION 1: SYSTEM STATUS SUMMARY (High-Priority Overview)
        // These are the most critical values drivers/pit crew need at a glance
        // =====================================================================
        publishSystemSummary(panels, data);

        // =====================================================================
        // SECTION 2: MATCH CONTEXT
        // =====================================================================
        panels.debug("match/alliance", data.context.alliance.name());
        panels.debug("match/opMode", data.context.opMode);
        panels.debug("match/runtime_sec", String.format("%.1f", data.context.runtimeSec));
        panels.debug("match/matchTime_sec", String.format("%.1f", data.context.matchTimeSec));
        panels.debug("match/robotConfig", data.context.robotConfig);
        panels.debug("match/launcherMode", data.context.launcherMode);
        panels.debug("match/isAutonomous", data.context.isAutonomous);

        // =====================================================================
        // SECTION 3: LOOP TIMING (Performance Diagnostics)
        // =====================================================================
        panels.debug("timing/mainLoop_ms", String.format("%.2f", data.timing.mainLoopMs));
        panels.debug("timing/drive_ms", String.format("%.2f", data.timing.driveMs));
        panels.debug("timing/intake_ms", String.format("%.2f", data.timing.intakeMs));
        panels.debug("timing/launcher_ms", String.format("%.2f", data.timing.launcherMs));
        panels.debug("timing/vision_ms", String.format("%.2f", data.timing.visionMs));
        panels.debug("timing/lighting_ms", String.format("%.2f", data.timing.lightingMs));
        panels.debug("timing/subsystems_total_ms", String.format("%.2f", data.timing.totalSubsystemMs()));

        // Mode and context (legacy paths maintained for compatibility)
        panels.debug("mode/label", data.context.opMode.toLowerCase());
        panels.debug("mode/drive", data.drive.driveMode);
        panels.debug("mode/aim/enabled", data.drive.aimMode);

        // Gamepad inputs (raw driver inputs) - axes
        // Format: functionName(physicalControl) - gives best of both worlds!
        panels.debug("gamepad/driver/axes/fieldX(leftStickX)", data.gamepad.driver.leftStickX);
        panels.debug("gamepad/driver/axes/fieldY(leftStickY)", data.gamepad.driver.leftStickY);
        panels.debug("gamepad/driver/axes/rotationCcw(rightStickX)", data.gamepad.driver.rightStickX);
        panels.debug("gamepad/driver/axes/rightStickY", data.gamepad.driver.rightStickY);
        panels.debug("gamepad/driver/axes/leftTrigger", data.gamepad.driver.leftTrigger);
        panels.debug("gamepad/driver/axes/rightTrigger", data.gamepad.driver.rightTrigger);

        // Driver buttons (named from DriverBindings)
        panels.debug("gamepad/driver/buttons/relocalizeRequest(A)", data.gamepad.driver.buttonA);
        panels.debug("gamepad/driver/buttons/aimHold(B)", data.gamepad.driver.buttonB);
        panels.debug("gamepad/driver/buttons/aim(X)", data.gamepad.driver.buttonX);
        panels.debug("gamepad/driver/buttons/Y(unmapped)", data.gamepad.driver.buttonY);
        panels.debug("gamepad/driver/buttons/rampHold(leftBumper)", data.gamepad.driver.leftBumper);
        panels.debug("gamepad/driver/buttons/slowHold(rightBumper)", data.gamepad.driver.rightBumper);
        panels.debug("gamepad/driver/buttons/telemetryPageUp(dpadUp)", data.gamepad.driver.dpadUp);
        panels.debug("gamepad/driver/buttons/telemetryPageDown(dpadDown)", data.gamepad.driver.dpadDown);

        // Operator axes
        panels.debug("gamepad/operator/axes/leftStickX", data.gamepad.operator.leftStickX);
        panels.debug("gamepad/operator/axes/leftStickY", data.gamepad.operator.leftStickY);
        panels.debug("gamepad/operator/axes/rightStickX", data.gamepad.operator.rightStickX);
        panels.debug("gamepad/operator/axes/rightStickY", data.gamepad.operator.rightStickY);
        panels.debug("gamepad/operator/axes/leftTrigger", data.gamepad.operator.leftTrigger);
        panels.debug("gamepad/operator/axes/rightTrigger", data.gamepad.operator.rightTrigger);

        // Operator buttons (named from OperatorBindings)
        panels.debug("gamepad/operator/buttons/manualSpinButton(A)", data.gamepad.operator.buttonA);
        panels.debug("gamepad/operator/buttons/fireLongButton(B)", data.gamepad.operator.buttonB);
        panels.debug("gamepad/operator/buttons/fireShortButton(X)", data.gamepad.operator.buttonX);
        panels.debug("gamepad/operator/buttons/fireMidButton(Y)", data.gamepad.operator.buttonY);
        panels.debug("gamepad/operator/buttons/leftBumper(unmapped)", data.gamepad.operator.leftBumper);
        panels.debug("gamepad/operator/buttons/intakeForwardHold(rightBumper)", data.gamepad.operator.rightBumper);

        // Drive commanded behavior (after processing)
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

        // Vision - basic tag detection
        panels.debug("vision/tag/visible", data.vision.hasTag);
        panels.debug("vision/tag/id", data.vision.tagId);
        panels.debug("vision/tag/rangeIn", data.vision.rangeIn);
        panels.debug("vision/tag/bearingDeg", data.vision.bearingDeg);
        panels.debug("vision/tag/yawDeg", data.vision.yawDeg);
        panels.debug("vision/odometryPending", data.vision.odometryPending);

        // Vision - raw Limelight values (useful for aim debugging)
        panels.debug("vision/limelight/txDeg", data.vision.txDeg);
        panels.debug("vision/limelight/tyDeg", data.vision.tyDeg);
        panels.debug("vision/limelight/areaPct", data.vision.targetAreaPercent);

        // Vision - pose from AprilTag
        panels.debug("vision/pose/xIn", data.vision.poseXIn);
        panels.debug("vision/pose/yIn", data.vision.poseYIn);
        panels.debug("vision/pose/headingDeg", Double.isNaN(data.vision.headingRad) ? Double.NaN : Math.toDegrees(data.vision.headingRad));

        // =====================================================================
        // SECTION: LAUNCHER RPM GRAPHS
        // These are formatted for easy graphing in the Panels Graph plugin.
        // Configure Graph plugin to show these keys for real-time RPM visualization.
        // =====================================================================
        publishLauncherGraphData(panels, data);

        // Launcher per-lane detailed data
        publishLaneData(panels, "launcher/lanes/left", data.launcher.left);
        publishLaneData(panels, "launcher/lanes/center", data.launcher.center);
        publishLaneData(panels, "launcher/lanes/right", data.launcher.right);

        // Intake
        panels.debug("intake/mode", data.intake.mode);
        panels.debug("intake/power", data.intake.power);
        panels.debug("intake/artifactCount", data.intake.artifactCount);
        panels.debug("intake/artifactState", data.intake.artifactState);
        panels.debug("intake/roller/active", data.intake.rollerActive);
        panels.debug("intake/roller/position", data.intake.rollerPosition);

        // Color Sensors - per-lane telemetry for tuning
        panels.debug("colorSensor/classifierMode", org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.laneSensorConfig.classifier.mode);
        publishColorSensorData(panels, "colorSensor/left", data.intake.laneLeftSummary, data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.LEFT));
        publishColorSensorData(panels, "colorSensor/center", data.intake.laneCenterSummary, data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.CENTER));
        publishColorSensorData(panels, "colorSensor/right", data.intake.laneRightSummary, data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.RIGHT));

        // =====================================================================
        // SECTION: LIGHTING STATE
        // Publishes current lighting state for the Panels Lights widget.
        // The Lights plugin can visualize goBILDA RGB indicators.
        // =====================================================================
        publishLightingState(panels, data);
    }

    /**
     * Publish lighting state for the Panels Lights widget.
     * The widget mirrors the robot's goBILDA RGB indicator lights.
     *
     * <p>To use the Lights widget in Panels:</p>
     * <ol>
     *   <li>Open Panels and add the Lights plugin</li>
     *   <li>Configure 3 lights (Left, Center, Right) matching your robot</li>
     *   <li>The colors will automatically sync with robot state</li>
     * </ol>
     */
    private void publishLightingState(TelemetryManager panels, RobotTelemetryData data) {
        if (data.lighting == null) {
            return;
        }

        // Current pattern being displayed
        panels.debug("lights/pattern", data.lighting.currentPattern);
        panels.debug("lights/goalPattern", data.lighting.goalPattern);
        panels.debug("lights/baseMode", data.lighting.baseMode);

        // Per-lane colors (what's actually being shown)
        panels.debug("lights/left/color", data.lighting.leftColor);
        panels.debug("lights/center/color", data.lighting.centerColor);
        panels.debug("lights/right/color", data.lighting.rightColor);

        // Per-lane servo positions (for Lights widget synchronization)
        // The Panels Lights widget uses servo positions to determine color
        panels.debug("lights/left/position", data.lighting.leftPosition);
        panels.debug("lights/center/position", data.lighting.centerPosition);
        panels.debug("lights/right/position", data.lighting.rightPosition);

        // Hex colors for visual display (easier to read in telemetry)
        panels.debug("lights/left/hex", artifactColorToHex(data.lighting.leftColor));
        panels.debug("lights/center/hex", artifactColorToHex(data.lighting.centerColor));
        panels.debug("lights/right/hex", artifactColorToHex(data.lighting.rightColor));
    }

    /**
     * Convert ArtifactColor to hex color string for display.
     */
    private String artifactColorToHex(String colorName) {
        if (colorName == null) return "#000000";
        switch (colorName) {
            case "GREEN":   return "#00FF00";
            case "PURPLE":  return "#9C27B0";
            case "RED":     return "#FF0000";
            case "BLUE":    return "#2196F3";
            case "WHITE":   return "#FFFFFF";
            case "YELLOW":  return "#FFEB3B";
            case "UNKNOWN": return "#FFFFFF"; // White for unknown detected
            case "NONE":
            default:        return "#000000"; // Off
        }
    }

    /**
     * Publish per-lane launcher data to FullPanels.
     */
    private void publishLaneData(TelemetryManager panels, String prefix, LauncherTelemetryData.LaneData lane) {
        panels.debug(prefix + "/targetRpm", lane.targetRpm);
        panels.debug(prefix + "/currentRpm", lane.currentRpm);
        panels.debug(prefix + "/power", lane.power);
        panels.debug(prefix + "/ready", lane.ready);
        panels.debug(prefix + "/hoodPosition", lane.hoodPosition);
        panels.debug(prefix + "/feederPosition", lane.feederPosition);
    }

    /**
     * Publish per-lane color sensor data to FullPanels for tuning.
     * Includes all metrics needed to tune color detection thresholds.
     */
    private void publishColorSensorData(TelemetryManager panels, String prefix,
                                        IntakeTelemetryData.LaneTelemetrySummary summary,
                                        org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.LaneSample sample) {
        // Basic status
        panels.debug(prefix + "/sensorPresent", summary.sensorPresent);
        panels.debug(prefix + "/detected", summary.detected);
        panels.debug(prefix + "/distanceCm", summary.distanceCm);

        // Color detection results
        panels.debug(prefix + "/color", summary.color);
        panels.debug(prefix + "/hsvColor", summary.hsvColor);

        // Detailed sample data (if available) for tuning
        if (sample != null && sample.sensorPresent) {
            // Confidence metric (how certain classifier is about the color)
            panels.debug(prefix + "/confidence", sample.confidence);

            // RGB values - normalized (0-1) and scaled (0-255)
            panels.debug(prefix + "/rgb/normalizedR", sample.normalizedRed);
            panels.debug(prefix + "/rgb/normalizedG", sample.normalizedGreen);
            panels.debug(prefix + "/rgb/normalizedB", sample.normalizedBlue);
            panels.debug(prefix + "/rgb/scaledR", sample.scaledRed);
            panels.debug(prefix + "/rgb/scaledG", sample.scaledGreen);
            panels.debug(prefix + "/rgb/scaledB", sample.scaledBlue);

            // HSV values - critical for tuning thresholds
            panels.debug(prefix + "/hsv/hue", sample.hue);
            panels.debug(prefix + "/hsv/saturation", sample.saturation);
            panels.debug(prefix + "/hsv/value", sample.value);

            // Distance sensor data
            panels.debug(prefix + "/distance/available", sample.distanceAvailable);
            panels.debug(prefix + "/distance/withinRange", sample.withinDistance);
        }
    }

    // =========================================================================
    // HELPER METHODS FOR ORGANIZED TELEMETRY SECTIONS
    // =========================================================================

    /**
     * Publish system status summary - the most critical at-a-glance info.
     * This is designed to give drivers and pit crew immediate insight into robot state.
     *
     * <p>These widgets should be pinned to the top of the Panels Telemetry view.</p>
     */
    private void publishSystemSummary(TelemetryManager panels, RobotTelemetryData data) {
        // Launcher ready summary (most important for drivers)
        boolean leftReady = data.launcher.left.ready;
        boolean centerReady = data.launcher.center.ready;
        boolean rightReady = data.launcher.right.ready;
        boolean allReady = leftReady && centerReady && rightReady;
        int readyCount = (leftReady ? 1 : 0) + (centerReady ? 1 : 0) + (rightReady ? 1 : 0);

        panels.debug("status/launcher/allReady", allReady);
        panels.debug("status/launcher/readyCount", readyCount);
        panels.debug("status/launcher/summary", readyCount + "/3 READY");
        panels.debug("status/launcher/left", leftReady ? "✓" : "○");
        panels.debug("status/launcher/center", centerReady ? "✓" : "○");
        panels.debug("status/launcher/right", rightReady ? "✓" : "○");

        // Intake/artifact status
        panels.debug("status/artifacts/count", data.intake.artifactCount);
        panels.debug("status/artifacts/state", data.intake.artifactState);
        panels.debug("status/intake/mode", data.intake.mode);

        // Gate status (roller/prefeed)
        panels.debug("status/gate/rollerActive", data.intake.rollerActive);
        panels.debug("status/gate/position", String.format("%.2f", data.intake.rollerPosition));

        // Vision status
        panels.debug("status/vision/tagVisible", data.vision.hasTag);
        panels.debug("status/vision/tagId", data.vision.hasTag ? data.vision.tagId : -1);
        panels.debug("status/vision/rangeIn", data.vision.hasTag ? String.format("%.1f", data.vision.rangeIn) : "---");

        // Drive status
        panels.debug("status/drive/mode", data.drive.driveMode);
        panels.debug("status/drive/aimActive", data.drive.aimMode);

        // Distance to goal
        panels.debug("status/drive/distanceToGoal_in", String.format("%.1f", data.drive.distanceToGoalIn));

        // Alliance (critical for autonomous)
        panels.debug("status/alliance", data.context.alliance.name());
    }

    /**
     * Publish launcher RPM data formatted for graphing.
     *
     * <p>To visualize these in Panels:</p>
     * <ol>
     *   <li>Open the Graph plugin in Panels</li>
     *   <li>Add a new graph</li>
     *   <li>Select "graph/launcher/left/targetRpm" and "graph/launcher/left/currentRpm"</li>
     *   <li>Repeat for center and right lanes as needed</li>
     * </ol>
     *
     * <p>The target/current pairs allow you to see how well the flywheel is tracking setpoint.</p>
     */
    private void publishLauncherGraphData(TelemetryManager panels, RobotTelemetryData data) {
        // Left lane - graphable
        panels.debug("graph/launcher/left/targetRpm", data.launcher.left.targetRpm);
        panels.debug("graph/launcher/left/currentRpm", data.launcher.left.currentRpm);
        panels.debug("graph/launcher/left/errorRpm", data.launcher.left.targetRpm - data.launcher.left.currentRpm);

        // Center lane - graphable
        panels.debug("graph/launcher/center/targetRpm", data.launcher.center.targetRpm);
        panels.debug("graph/launcher/center/currentRpm", data.launcher.center.currentRpm);
        panels.debug("graph/launcher/center/errorRpm", data.launcher.center.targetRpm - data.launcher.center.currentRpm);

        // Right lane - graphable
        panels.debug("graph/launcher/right/targetRpm", data.launcher.right.targetRpm);
        panels.debug("graph/launcher/right/currentRpm", data.launcher.right.currentRpm);
        panels.debug("graph/launcher/right/errorRpm", data.launcher.right.targetRpm - data.launcher.right.currentRpm);

        // All lanes combined (average) - useful for overall spin-up visualization
        double avgTarget = (data.launcher.left.targetRpm + data.launcher.center.targetRpm + data.launcher.right.targetRpm) / 3.0;
        double avgCurrent = (data.launcher.left.currentRpm + data.launcher.center.currentRpm + data.launcher.right.currentRpm) / 3.0;
        panels.debug("graph/launcher/avg/targetRpm", avgTarget);
        panels.debug("graph/launcher/avg/currentRpm", avgCurrent);
        panels.debug("graph/launcher/avg/errorRpm", avgTarget - avgCurrent);
    }
}
