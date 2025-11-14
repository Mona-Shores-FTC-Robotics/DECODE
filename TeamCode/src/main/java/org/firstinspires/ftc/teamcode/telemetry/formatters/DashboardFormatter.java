package org.firstinspires.ftc.teamcode.telemetry.formatters;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.telemetry.data.LauncherTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;

/**
 * Formats telemetry packets for FTC Dashboard.
 * Uses hierarchical field naming with underscores for alphabetical grouping.
 * Provides tiered output based on telemetry level (PRACTICE/DEBUG).
 * <p>
 * Note: MATCH mode does not send dashboard packets for performance.
 * </p>
 */
public class DashboardFormatter {

    /**
     * Create a telemetry packet based on the current telemetry level.
     * MATCH mode returns null (no packets sent).
     * PRACTICE mode includes essential metrics for tuning.
     * DEBUG mode includes comprehensive diagnostics.
     */
    public TelemetryPacket createPacket(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        switch (level) {
            case MATCH:
                return null; // No packets in MATCH mode
            case PRACTICE:
                return createPracticePacket(data);
            case DEBUG:
            default:
                return createDebugPacket(data);
        }
    }

    /**
     * Create PRACTICE level packet - essential metrics for tuning.
     * Target: 10 Hz / 100ms interval, ~20 fields
     */
    private TelemetryPacket createPracticePacket(RobotTelemetryData data) {
        TelemetryPacket packet = new TelemetryPacket();

        // Context
        packet.put("alliance/id", data.context.alliance.name());
        packet.put("context/match_time_sec", data.context.matchTimeSec);
        packet.put("context/runtime_sec", data.context.runtimeSec);

        // Pose (always critical)
        addPoseData(packet, data);

        // Drive essentials
        packet.put("drive/aim_mode", data.drive.aimMode);
        packet.put("drive/mode", data.drive.driveMode);
        packet.put("drive/slow_mode", data.drive.slowMode);

        // Launcher high-level
        packet.put("launcher/ready", data.launcher.ready);
        packet.put("launcher/spin_mode", data.launcher.spinMode);
        packet.put("launcher/state", data.launcher.state);

        // Vision essentials
        packet.put("vision/has_tag", data.vision.hasTag);
        packet.put("vision/range_in", data.vision.rangeIn);
        packet.put("vision/tag_id", data.vision.tagId);

        // Intake
        packet.put("intake/artifact_count", data.intake.artifactCount);
        packet.put("intake/mode", data.intake.mode);

        // Field overlay
        addFieldOverlay(packet, data, false);

        return packet;
    }

    /**
     * Create DEBUG level packet - comprehensive diagnostics.
     * Target: 20 Hz / 50ms interval, ~80 fields
     */
    private TelemetryPacket createDebugPacket(RobotTelemetryData data) {
        TelemetryPacket packet = new TelemetryPacket();

        // Context
        packet.put("alliance/id", data.context.alliance.name());
        packet.put("context/match_time_sec", data.context.matchTimeSec);
        packet.put("context/opmode", data.context.opMode);
        packet.put("context/runtime_sec", data.context.runtimeSec);

        // Pose (all representations)
        addPoseData(packet, data);

        // Drive detailed (shows commanded behavior, not raw inputs)
        packet.put("drive/aim_mode", data.drive.aimMode);
        packet.put("drive/command_turn", data.drive.commandTurn);
        packet.put("drive/mode", data.drive.driveMode);
        packet.put("drive/slow_mode", data.drive.slowMode);

        // Drive motors (organized by motor, underscores group power/velocity)
        packet.put("drive/motor_lb_power", data.drive.leftBack.power);
        packet.put("drive/motor_lb_vel_ips", data.drive.leftBack.velocityIps);
        packet.put("drive/motor_lf_power", data.drive.leftFront.power);
        packet.put("drive/motor_lf_vel_ips", data.drive.leftFront.velocityIps);
        packet.put("drive/motor_rb_power", data.drive.rightBack.power);
        packet.put("drive/motor_rb_vel_ips", data.drive.rightBack.velocityIps);
        packet.put("drive/motor_rf_power", data.drive.rightFront.power);
        packet.put("drive/motor_rf_vel_ips", data.drive.rightFront.velocityIps);

        // Intake detailed
        packet.put("intake/artifact_count", data.intake.artifactCount);
        packet.put("intake/artifact_state", data.intake.artifactState);
        packet.put("intake/mode", data.intake.mode);
        packet.put("intake/power", data.intake.power);
        packet.put("intake/roller_active", data.intake.rollerActive);
        packet.put("intake/roller_position", data.intake.rollerPosition);

        // Launcher high-level
        packet.put("launcher/control_mode", data.launcher.controlMode);
        packet.put("launcher/ready", data.launcher.ready);
        packet.put("launcher/spin_mode", data.launcher.spinMode);
        packet.put("launcher/state", data.launcher.state);

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

        // Gamepad inputs (AdvantageScope-friendly naming for potential joystick visualization)
        addGamepadData(packet, data);

        // Field overlay (with vision pose in DEBUG)
        addFieldOverlay(packet, data, true);

        return packet;
    }

    /**
     * Add pose data to packet.
     * Includes Pedro pose, FTC pose, and vision pose (when available).
     */
    private void addPoseData(TelemetryPacket packet, RobotTelemetryData data) {
        // Pedro pose (primary)
        packet.put("Pose/Pose x", data.pose.poseXIn);
        packet.put("Pose/Pose y", data.pose.poseYIn);
        packet.put("Pose/Pose heading", data.pose.headingRad);

        // FTC pose (for compatibility)
        if (!Double.isNaN(data.pose.ftcXIn) && !Double.isNaN(data.pose.ftcYIn)) {
            packet.put("Pose/FTC Pose x", data.pose.ftcXIn);
            packet.put("Pose/FTC Pose y", data.pose.ftcYIn);
        }
        if (!Double.isNaN(data.pose.ftcHeadingRad)) {
            packet.put("Pose/FTC Pose heading", data.pose.ftcHeadingRad);
        }

        // Vision pose (when available)
        if (data.pose.visionPoseValid) {
            packet.put("Pose/Vision Pose x", data.pose.visionPoseXIn);
            packet.put("Pose/Vision Pose y", data.pose.visionPoseYIn);
            packet.put("Pose/Vision Pose heading", data.pose.visionHeadingRad);
        }
    }

    /**
     * Add per-lane launcher data with underscores for alphabetical grouping.
     */
    private void addLaneData(TelemetryPacket packet, String prefix, LauncherTelemetryData.LaneData lane) {
        packet.put(prefix + "/bang_to_hold_count", lane.bangToHoldCount);
        packet.put(prefix + "/current_rpm", lane.currentRpm);
        packet.put(prefix + "/feeder_position", lane.feederPosition);
        packet.put(prefix + "/hood_position", lane.hoodPosition);
        packet.put(prefix + "/phase", lane.phase);
        packet.put(prefix + "/phase_bang", lane.phaseBang);
        packet.put(prefix + "/phase_hold", lane.phaseHold);
        packet.put(prefix + "/phase_hybrid", lane.phaseHybrid);
        packet.put(prefix + "/power", lane.power);
        packet.put(prefix + "/ready", lane.ready);
        packet.put(prefix + "/target_rpm", lane.targetRpm);
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
     * Uses AdvantageScope/DriverStation-compatible naming to potentially enable joystick visualization.
     * Format: DriverStation/Gamepad{1|2}/Axes/{0-5} and Buttons/{0-14}
     */
    private void addGamepadData(TelemetryPacket packet, RobotTelemetryData data) {
        // Driver gamepad (Gamepad1)
        packet.put("DriverStation/Gamepad1/Axes/0", data.gamepad.driver.leftStickX);
        packet.put("DriverStation/Gamepad1/Axes/1", data.gamepad.driver.leftStickY);
        packet.put("DriverStation/Gamepad1/Axes/2", data.gamepad.driver.leftTrigger);
        packet.put("DriverStation/Gamepad1/Axes/3", data.gamepad.driver.rightTrigger);
        packet.put("DriverStation/Gamepad1/Axes/4", data.gamepad.driver.rightStickX);
        packet.put("DriverStation/Gamepad1/Axes/5", data.gamepad.driver.rightStickY);

        packet.put("DriverStation/Gamepad1/Buttons/0", data.gamepad.driver.buttonA);
        packet.put("DriverStation/Gamepad1/Buttons/1", data.gamepad.driver.buttonB);
        packet.put("DriverStation/Gamepad1/Buttons/2", data.gamepad.driver.buttonX);
        packet.put("DriverStation/Gamepad1/Buttons/3", data.gamepad.driver.buttonY);
        packet.put("DriverStation/Gamepad1/Buttons/4", data.gamepad.driver.leftBumper);
        packet.put("DriverStation/Gamepad1/Buttons/5", data.gamepad.driver.rightBumper);
        packet.put("DriverStation/Gamepad1/Buttons/6", data.gamepad.driver.back);
        packet.put("DriverStation/Gamepad1/Buttons/7", data.gamepad.driver.start);
        packet.put("DriverStation/Gamepad1/Buttons/8", data.gamepad.driver.leftStickButton);
        packet.put("DriverStation/Gamepad1/Buttons/9", data.gamepad.driver.rightStickButton);
        packet.put("DriverStation/Gamepad1/Buttons/10", data.gamepad.driver.dpadUp);
        packet.put("DriverStation/Gamepad1/Buttons/11", data.gamepad.driver.dpadDown);
        packet.put("DriverStation/Gamepad1/Buttons/12", data.gamepad.driver.dpadLeft);
        packet.put("DriverStation/Gamepad1/Buttons/13", data.gamepad.driver.dpadRight);
        packet.put("DriverStation/Gamepad1/Buttons/14", data.gamepad.driver.guide);

        // Operator gamepad (Gamepad2)
        packet.put("DriverStation/Gamepad2/Axes/0", data.gamepad.operator.leftStickX);
        packet.put("DriverStation/Gamepad2/Axes/1", data.gamepad.operator.leftStickY);
        packet.put("DriverStation/Gamepad2/Axes/2", data.gamepad.operator.leftTrigger);
        packet.put("DriverStation/Gamepad2/Axes/3", data.gamepad.operator.rightTrigger);
        packet.put("DriverStation/Gamepad2/Axes/4", data.gamepad.operator.rightStickX);
        packet.put("DriverStation/Gamepad2/Axes/5", data.gamepad.operator.rightStickY);

        packet.put("DriverStation/Gamepad2/Buttons/0", data.gamepad.operator.buttonA);
        packet.put("DriverStation/Gamepad2/Buttons/1", data.gamepad.operator.buttonB);
        packet.put("DriverStation/Gamepad2/Buttons/2", data.gamepad.operator.buttonX);
        packet.put("DriverStation/Gamepad2/Buttons/3", data.gamepad.operator.buttonY);
        packet.put("DriverStation/Gamepad2/Buttons/4", data.gamepad.operator.leftBumper);
        packet.put("DriverStation/Gamepad2/Buttons/5", data.gamepad.operator.rightBumper);
        packet.put("DriverStation/Gamepad2/Buttons/6", data.gamepad.operator.back);
        packet.put("DriverStation/Gamepad2/Buttons/7", data.gamepad.operator.start);
        packet.put("DriverStation/Gamepad2/Buttons/8", data.gamepad.operator.leftStickButton);
        packet.put("DriverStation/Gamepad2/Buttons/9", data.gamepad.operator.rightStickButton);
        packet.put("DriverStation/Gamepad2/Buttons/10", data.gamepad.operator.dpadUp);
        packet.put("DriverStation/Gamepad2/Buttons/11", data.gamepad.operator.dpadDown);
        packet.put("DriverStation/Gamepad2/Buttons/12", data.gamepad.operator.dpadLeft);
        packet.put("DriverStation/Gamepad2/Buttons/13", data.gamepad.operator.dpadRight);
        packet.put("DriverStation/Gamepad2/Buttons/14", data.gamepad.operator.guide);
    }
}
