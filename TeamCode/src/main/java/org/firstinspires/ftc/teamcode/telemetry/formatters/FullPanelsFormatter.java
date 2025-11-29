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

        // Vision
        panels.debug("vision/tag/visible", data.vision.hasTag);
        panels.debug("vision/tag/id", data.vision.tagId);
        panels.debug("vision/tag/rangeIn", data.vision.rangeIn);
        panels.debug("vision/tag/bearingDeg", data.vision.bearingDeg);
        panels.debug("vision/tag/yawDeg", data.vision.yawDeg);
        panels.debug("vision/odometryPending", data.vision.odometryPending);

        // Launcher control (feedforward + proportional per lane)

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

        // Color Sensors - per-lane telemetry for tuning
        panels.debug("colorSensor/classifierMode", org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.laneSensorConfig.classifierMode);
        publishColorSensorData(panels, "colorSensor/left", data.intake.laneLeftSummary, data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.LEFT));
        publishColorSensorData(panels, "colorSensor/center", data.intake.laneCenterSummary, data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.CENTER));
        publishColorSensorData(panels, "colorSensor/right", data.intake.laneRightSummary, data.intake.laneSamples.get(org.firstinspires.ftc.teamcode.util.LauncherLane.RIGHT));
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
}
