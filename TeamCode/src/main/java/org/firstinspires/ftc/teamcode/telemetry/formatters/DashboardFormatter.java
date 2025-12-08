package org.firstinspires.ftc.teamcode.telemetry.formatters;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.telemetry.data.IntakeTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.LauncherTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Formats telemetry packets for FTC Dashboard.
 * Uses hierarchical field naming with underscores for alphabetical grouping.
 * <p>
 * Note: MATCH mode does not send dashboard packets. DEBUG mode sends full diagnostics.
 * </p>
 */
public class DashboardFormatter {

    /**
     * Create a telemetry packet based on the current telemetry level.
     * MATCH mode returns null (no packets sent).
     * DEBUG mode includes comprehensive diagnostics.
     */
    public TelemetryPacket createPacket(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        if (level == TelemetrySettings.TelemetryLevel.MATCH) {
            return null;
        }
        return createDebugPacket(data);
    }

    /**
     * Create DEBUG level packet - comprehensive diagnostics.
     */
    private TelemetryPacket createDebugPacket(RobotTelemetryData data) {
        TelemetryPacket packet = RobotState.packet;

        // Context
        packet.put("context/alliance", data.context.alliance.name());
        packet.put("context/match_time_sec", data.context.matchTimeSec);
        packet.put("context/opmode", data.context.opMode);
        packet.put("context/runtime_sec", data.context.runtimeSec);

        // Pose (all representations)
        addPoseData(packet, data);

        // Drive detailed (shows commanded behavior, not raw inputs)
        packet.put("drive/aim_mode", data.drive.aimMode);
        packet.put("drive/command/drive", data.drive.commandDrive);
        packet.put("drive/command/strafe", data.drive.commandStrafe);
        packet.put("drive/command/turn", data.drive.commandTurn);
        packet.put("drive/mode", data.drive.driveMode);
        packet.put("drive/slow_mode", data.drive.slowMode);

        // Drive motors (organized by motor, underscores group power/velocity)
        packet.put("drive/motors/power/lb", data.drive.leftBack.power);
        packet.put("drive/motors/power/lf", data.drive.leftFront.power);
        packet.put("drive/motors/power/rb", data.drive.rightBack.power);
        packet.put("drive/motors/power/rf", data.drive.rightFront.power);

        packet.put("drive/motors/vel_ips/lb", data.drive.leftBack.velocityIps);
        packet.put("drive/motors/vel_ips/lf", data.drive.leftFront.velocityIps);
        packet.put("drive/motors/vel_ips/rb", data.drive.rightBack.velocityIps);
        packet.put("drive/motors/vel_ips/rf", data.drive.rightFront.velocityIps);

        // Intake detailed
        packet.put("intake/artifact_count", data.intake.artifactCount);
        packet.put("intake/artifact_state", data.intake.artifactState);
        packet.put("intake/motor/mode", data.intake.mode);
        packet.put("intake/motor/power", data.intake.power);
        packet.put("intake/roller_active", data.intake.rollerActive);
        addDebugLaneSamples(packet, data.intake);


        // Launcher control (feedforward + proportional per lane)

        // Launcher per-lane (left)
        addLaneData(packet, "launcher/left", data.launcher.left);

        // Launcher per-lane (center)
        addLaneData(packet, "launcher/center", data.launcher.center);

        // Launcher per-lane (right)
        addLaneData(packet, "launcher/right", data.launcher.right);

        // Vision detailed
        packet.put("vision/alliance", data.vision.alliance.name());
        packet.put("vision/bearing_deg", data.vision.bearingDeg);
        packet.put("vision/has_tag", data.vision.hasTag);
        packet.put("vision/odometry_pending", data.vision.odometryPending);
        packet.put("vision/range_in", data.vision.rangeIn);
        packet.put("vision/tag_id", data.vision.tagId);
        packet.put("vision/target_area_percent", data.vision.targetAreaPercent);
        packet.put("vision/tx_deg", data.vision.txDeg);
        packet.put("vision/ty_deg", data.vision.tyDeg);
        packet.put("vision/yaw_deg", data.vision.yawDeg);

        // Loop timing (DEBUG mode only - full performance diagnostics)
        addLoopTimingData(packet, data);

        // Gamepad inputs (AdvantageScope-friendly naming for potential joystick visualization)
        addGamepadData(packet, data);

        // Field overlay (with vision pose in DEBUG)
        addFieldOverlay(packet, data, true);

        return packet;
    }

    private void addDebugLaneSamples(TelemetryPacket packet, IntakeTelemetryData intake) {
        if (intake == null || intake.laneSamples == null || intake.laneSamples.isEmpty()) {
            return;
        }

        for (LauncherLane lane : LauncherLane.values()) {
            IntakeSubsystem.LaneSample sample = intake.laneSamples.get(lane);
            if (sample == null) {
                continue;
            }

            String prefix = lanePrefix(lane);
            packet.put(prefix + "/sensor_present", sample.sensorPresent);
            packet.put(prefix + "/distance_available", sample.distanceAvailable);
            packet.put(prefix + "/within_distance", sample.withinDistance);
            packet.put(prefix + "/  distance_cm", sample.distanceCm);
            packet.put(prefix + "/color", sample.color.name());
            packet.put(prefix + "/color_hsv", sample.hsvColor.name());
            packet.put(prefix + "/normalized_rgb", formatNormalizedRgb(sample.normalizedRed, sample.normalizedGreen, sample.normalizedBlue));
            packet.put(prefix + "/scaled_rgb", formatRgb(sample.scaledRed, sample.scaledGreen, sample.scaledBlue));
            packet.put(prefix + "/hsv", formatHsv(sample.hue, sample.saturation, sample.value));
        }
    }

    private static String lanePrefix(LauncherLane lane) {
        return "intake/lane_" + lane.name().toLowerCase();
    }

    private static String formatNormalizedRgb(float red, float green, float blue) {
        return String.format(Locale.US, "%.3f,%.3f,%.3f", red, green, blue);
    }

    private static String formatRgb(int red, int green, int blue) {
        return red + "," + green + "," + blue;
    }

    private static String formatHsv(float hue, float saturation, float value) {
        return String.format(Locale.US, "%.1f,%.3f,%.3f", hue, saturation, value);
    }

    /**
     * Add pose data to packet.
     * Includes Pedro pose, FTC pose, and vision pose (when available).
     */
    private void addPoseData(TelemetryPacket packet, RobotTelemetryData data) {
        // Pedro pose (primary)
        packet.put("Pose/Pedro x", data.pose.poseXIn);
        packet.put("Pose/Pedro y", data.pose.poseYIn);
        packet.put("Pose/Pedro heading", data.pose.headingRad);
        packet.put("Pose/Pedro degrees", Math.toDegrees(data.pose.headingRad));

        // FTC pose (for compatibility)
        if (!Double.isNaN(data.pose.ftcXIn) && !Double.isNaN(data.pose.ftcYIn)) {
            packet.put("Pose/FTC x", data.pose.ftcXIn);
            packet.put("Pose/FTC y", data.pose.ftcYIn);
        }
        if (!Double.isNaN(data.pose.ftcHeadingRad)) {
            packet.put("Pose/FTC heading", data.pose.ftcHeadingRad);

        }

        // Vision pose (when available)
        if (data.pose.visionPoseValid) {
            packet.put("Pose/Vision x", data.pose.visionPoseXIn);
            packet.put("Pose/Vision y", data.pose.visionPoseYIn);
            packet.put("Pose/Vision heading", data.pose.visionHeadingRad);
        }
    }

    /**
     * Add per-lane launcher data with underscores for alphabetical grouping.
     */
    private void addLaneData(TelemetryPacket packet, String prefix, LauncherTelemetryData.LaneData lane) {
        packet.put(prefix + "/current_rpm", lane.currentRpm);
        packet.put(prefix + "/feeder_position", lane.feederPosition);
        packet.put(prefix + "/hood_position", lane.hoodPosition);
        packet.put(prefix + "/power", lane.power);
        packet.put(prefix + "/ready", lane.ready);
        packet.put(prefix + "/target_rpm", lane.targetRpm);
    }

    /**
     * Add loop timing data to packet (DEBUG mode only).
     * Provides comprehensive performance diagnostics for loop optimization.
     */
    private void addLoopTimingData(TelemetryPacket packet, RobotTelemetryData data) {
        // Calculate telemetry time, but guard against initial 0 value
        double totalTelemetryMs = (data.timing.telemetryStartNs > 0)
                ? TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - data.timing.telemetryStartNs)
                : 0.0;

        // Total loop time
        packet.put("timing/total_loop_ms", data.timing.totalLoopMs(totalTelemetryMs));

        // Main loop overhead
        packet.put("timing/main_loop_ms", data.timing.mainLoopMs);

        // Subsystem periodic times
        packet.put("timing/subsystem_total_ms", data.timing.totalSubsystemMs());
        packet.put("timing/subsystems/drive_ms", data.timing.driveMs);
        packet.put("timing/subsystems/intake_ms", data.timing.intakeMs);
        packet.put("timing/subsystems/launcher_ms", data.timing.launcherMs);
        packet.put("timing/subsystems/lighting_ms", data.timing.lightingMs);
        packet.put("timing/subsystems/vision_ms", data.timing.visionMs);

        // Telemetry and logging overhead
        packet.put("timing/telemetry_ms", totalTelemetryMs);
    }

    /**
     * Add field overlay showing robot pose and vision pose.
     */
    private void addFieldOverlay(TelemetryPacket packet, RobotTelemetryData data, boolean includeVisionPose) {
        Canvas overlay = packet.fieldOverlay();

        // Robot pose (white)
        if (data.pose.poseValid) {
            double headingLength = 6.0;
            double endX = data.pose.poseXIn + Math.cos(data.pose.headingRad) * headingLength;
            double endY = data.pose.poseYIn + Math.sin(data.pose.headingRad) * headingLength;

            overlay.setStroke("white");
            overlay.setStrokeWidth(2);
            overlay.strokeCircle(data.pose.poseXIn, data.pose.poseYIn, 1.5);
            overlay.strokeLine(data.pose.poseXIn, data.pose.poseYIn, endX, endY);
        }

        // Vision pose (lime green) - only in DEBUG mode
        if (includeVisionPose && data.pose.visionPoseValid) {
            double visionHeadingLength = 6.0;
            double visionEndX = data.pose.visionPoseXIn + Math.cos(data.pose.visionHeadingRad) * visionHeadingLength;
            double visionEndY = data.pose.visionPoseYIn + Math.sin(data.pose.visionHeadingRad) * visionHeadingLength;

            overlay.setStroke("lime");
            overlay.setStrokeWidth(2);
            overlay.strokeCircle(data.pose.visionPoseXIn, data.pose.visionPoseYIn, 2.0);
            overlay.strokeLine(data.pose.visionPoseXIn, data.pose.visionPoseYIn, visionEndX, visionEndY);
        }
    }

    /**
     * Add gamepad telemetry data to packet (DEBUG mode only).
     * Uses meaningful binding names with physical button/stick names in parentheses.
     * Format: functionName(physicalControl) - gives best of both worlds!
     */
    private void addGamepadData(TelemetryPacket packet, RobotTelemetryData data) {
        // Driver gamepad (Gamepad1) - stick axes
        packet.put("DriverStation/Gamepad1/Axes/fieldX(leftStickX)", data.gamepad.driver.leftStickX);
        packet.put("DriverStation/Gamepad1/Axes/fieldY(leftStickY)", data.gamepad.driver.leftStickY);
        packet.put("DriverStation/Gamepad1/Axes/rotationCcw(rightStickX)", data.gamepad.driver.rightStickX);
        packet.put("DriverStation/Gamepad1/Axes/Unmapped/rightStickY", data.gamepad.driver.rightStickY);
        packet.put("DriverStation/Gamepad1/Axes/Unmapped/leftTrigger", data.gamepad.driver.leftTrigger);
        packet.put("DriverStation/Gamepad1/Axes/Unmapped/rightTrigger", data.gamepad.driver.rightTrigger);

        // Driver buttons (named from DriverBindings)
        packet.put("DriverStation/Gamepad1/Buttons/relocalizeRequest(A)", data.gamepad.driver.buttonA);
        packet.put("DriverStation/Gamepad1/Buttons/aimHold(B)", data.gamepad.driver.buttonB);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/X", data.gamepad.driver.buttonX);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/Y", data.gamepad.driver.buttonY);
        packet.put("DriverStation/Gamepad1/Buttons/rampHold(leftBumper)", data.gamepad.driver.leftBumper);
        packet.put("DriverStation/Gamepad1/Buttons/slowHold(rightBumper)", data.gamepad.driver.rightBumper);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/back", data.gamepad.driver.back);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/start", data.gamepad.driver.start);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/leftStickButton", data.gamepad.driver.leftStickButton);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/rightStickButton", data.gamepad.driver.rightStickButton);
        packet.put("DriverStation/Gamepad1/Buttons/telemetryPageUp(dpadUp)", data.gamepad.driver.dpadUp);
        packet.put("DriverStation/Gamepad1/Buttons/telemetryPageDown(dpadDown)", data.gamepad.driver.dpadDown);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/dpadLeft", data.gamepad.driver.dpadLeft);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/dpadRight", data.gamepad.driver.dpadRight);
        packet.put("DriverStation/Gamepad1/Buttons/Unmapped/guide", data.gamepad.driver.guide);

        // Operator gamepad (Gamepad2) - stick axes
        packet.put("DriverStation/Gamepad2/Axes/Unmapped/leftStickX", data.gamepad.operator.leftStickX);
        packet.put("DriverStation/Gamepad2/Axes/Unmapped/leftStickY", data.gamepad.operator.leftStickY);
        packet.put("DriverStation/Gamepad2/Axes/Unmapped/rightStickX", data.gamepad.operator.rightStickX);
        packet.put("DriverStation/Gamepad2/Axes/Unmapped/rightStickY", data.gamepad.operator.rightStickY);
        packet.put("DriverStation/Gamepad2/Axes/Unmapped/leftTrigger", data.gamepad.operator.leftTrigger);
        packet.put("DriverStation/Gamepad2/Axes/Unmapped/rightTrigger", data.gamepad.operator.rightTrigger);

        // Operator buttons (named from OperatorBindings)
        packet.put("DriverStation/Gamepad2/Buttons/manualSpinButton(A)", data.gamepad.operator.buttonA);
        packet.put("DriverStation/Gamepad2/Buttons/fireLongButton(B)", data.gamepad.operator.buttonB);
        packet.put("DriverStation/Gamepad2/Buttons/fireShortButton(X)", data.gamepad.operator.buttonX);
        packet.put("DriverStation/Gamepad2/Buttons/fireMidButton(Y)", data.gamepad.operator.buttonY);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/leftBumper", data.gamepad.operator.leftBumper);
        packet.put("DriverStation/Gamepad2/Buttons/intakeForwardHold(rightBumper)", data.gamepad.operator.rightBumper);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/back", data.gamepad.operator.back);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/start", data.gamepad.operator.start);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/leftStickButton", data.gamepad.operator.leftStickButton);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/rightStickButton", data.gamepad.operator.rightStickButton);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/dpadUp", data.gamepad.operator.dpadUp);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/dpadDown", data.gamepad.operator.dpadDown);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/dpadLeft", data.gamepad.operator.dpadLeft);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/dpadRight", data.gamepad.operator.dpadRight);
        packet.put("DriverStation/Gamepad2/Buttons/Unmapped/guide", data.gamepad.operator.guide);
    }
}
