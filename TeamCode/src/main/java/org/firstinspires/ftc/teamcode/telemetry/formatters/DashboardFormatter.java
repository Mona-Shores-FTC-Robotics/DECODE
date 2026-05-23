package org.firstinspires.ftc.teamcode.telemetry.formatters;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.util.PoseFrames;
import org.firstinspires.ftc.teamcode.util.RobotFrameConfig;
import org.firstinspires.ftc.teamcode.telemetry.data.LauncherTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.concurrent.TimeUnit;

/**
 * Formats telemetry packets for FTC Dashboard.
 * Uses hierarchical field naming with underscores for alphabetical grouping.
 * <p>
 * Note: MATCH mode does not send dashboard packets. PRACTICE mode sends lean packets.
 * VERBOSE mode sends full diagnostics.
 * </p>
 */
public class DashboardFormatter {

    /**
     * Seed packet sent once at init so all Dashboard channels appear immediately
     * without needing to run commands first. Keys are set to zero/empty defaults.
     */
    public TelemetryPacket createSeedPacket(TelemetrySettings.TelemetryLevel level) {
        if (level == TelemetrySettings.TelemetryLevel.MATCH) {
            return null;
        }
        TelemetryPacket seed = new TelemetryPacket();

        // Practice keys
        seed.put("Pose/x", 0.0);
        seed.put("Pose/y", 0.0);
        seed.put("Pose/heading_deg", 0.0);
        seed.put("Pose/Robot x", 0.0);
        seed.put("Pose/Robot y", 0.0);
        seed.put("Pose/Robot heading", 0.0);
        seed.put("vision/has_tag", false);
        seed.put("vision/tag_id", 0);
        seed.put("vision/range_in", 0.0);
        seed.put("vision/tx_deg", 0.0);
        seed.put("intake/count", 0);
        seed.put("intake/mode", "");
        seed.put("intake/left_color", "");
        seed.put("intake/center_color", "");
        seed.put("intake/right_color", "");
        for (String lane : new String[]{"left", "center", "right"}) {
            seed.put("launcher/" + lane + "/ready", false);
            seed.put("launcher/" + lane + "/target_rpm", 0.0);
            seed.put("launcher/" + lane + "/rpm", 0.0);
        }
        seed.put("timing/loop_ms", 0.0);

        if (level == TelemetrySettings.TelemetryLevel.VERBOSE) {
            // Context
            seed.put("context/alliance", "");
            seed.put("context/match_time_sec", 0.0);
            seed.put("context/opmode", "");
            seed.put("context/runtime_sec", 0.0);

            // Pose (verbose representations)
            seed.put("Pose/Pedro x", 0.0);
            seed.put("Pose/Pedro y", 0.0);
            seed.put("Pose/Pedro heading", 0.0);
            seed.put("Pose/Pedro degrees", 0.0);
            seed.put("Pose/FTC x", 0.0);
            seed.put("Pose/FTC y", 0.0);
            seed.put("Pose/FTC heading", 0.0);
            seed.put("Pose/Vision x", 0.0);
            seed.put("Pose/Vision y", 0.0);
            seed.put("Pose/Vision heading", 0.0);

            // Drive
            seed.put("drive/aim_mode", false);
            seed.put("drive/command/drive", 0.0);
            seed.put("drive/command/strafe", 0.0);
            seed.put("drive/command/turn", 0.0);
            seed.put("drive/mode", "");
            seed.put("drive/slow_mode", false);
            for (String motor : new String[]{"lb", "lf", "rb", "rf"}) {
                seed.put("drive/motors/power/" + motor, 0.0);
                seed.put("drive/motors/vel_ips/" + motor, 0.0);
            }

            // Intake (verbose)
            seed.put("intake/artifact_count", 0);
            seed.put("intake/artifact_state", "");
            seed.put("intake/motor/mode", "");
            seed.put("intake/motor/power", 0.0);
            seed.put("intake/roller_active", false);

            // Launcher (verbose per lane)
            for (String lane : new String[]{"left", "center", "right"}) {
                seed.put("launcher/" + lane + "/current_rpm", 0.0);
                seed.put("launcher/" + lane + "/feeder_position", 0.0);
                seed.put("launcher/" + lane + "/hood_position", 0.0);
                seed.put("launcher/" + lane + "/power", 0.0);
                seed.put("launcher/" + lane + "/ready", false);
                seed.put("launcher/" + lane + "/target_rpm", 0.0);
            }

            // Vision (verbose)
            seed.put("vision/alliance", "");
            seed.put("vision/bearing_deg", 0.0);
            seed.put("vision/has_tag", false);
            seed.put("vision/odometry_pending", false);
            seed.put("vision/range_in", 0.0);
            seed.put("vision/tag_id", 0);
            seed.put("vision/target_area_percent", 0.0);
            seed.put("vision/tx_deg", 0.0);
            seed.put("vision/ty_deg", 0.0);
            seed.put("vision/yaw_deg", 0.0);

            // Timing (verbose)
            seed.put("timing/total_loop_ms", 0.0);
            seed.put("timing/main_loop_ms", 0.0);
            seed.put("timing/subsystem_total_ms", 0.0);
            seed.put("timing/subsystems/drive_ms", 0.0);
            seed.put("timing/subsystems/intake_ms", 0.0);
            seed.put("timing/subsystems/launcher_ms", 0.0);
            seed.put("timing/subsystems/lighting_ms", 0.0);
            seed.put("timing/subsystems/vision_ms", 0.0);
            seed.put("timing/telemetry_ms", 0.0);
        }

        return seed;
    }

    /**
     * Create a telemetry packet based on the current telemetry level.
     * MATCH mode returns null (no packets sent).
     * PRACTICE mode includes lean diagnostics.
     * VERBOSE mode includes comprehensive diagnostics.
     */
    public TelemetryPacket createPacket(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        if (level == TelemetrySettings.TelemetryLevel.MATCH) {
            return null;
        }
        if (level == TelemetrySettings.TelemetryLevel.PRACTICE) {
            return createPracticePacket(data);
        }
        return createVerbosePacket(data);
    }

    private TelemetryPacket createPracticePacket(RobotTelemetryData data) {
        TelemetryPacket packet = RobotState.packet;

        // Pose
        packet.put("Pose/x", data.pose.poseXIn);
        packet.put("Pose/y", data.pose.poseYIn);
        packet.put("Pose/heading_deg", Math.toDegrees(data.pose.headingRad));
        publishRobotFramePose(data);

        // Vision
        packet.put("vision/has_tag", data.vision.hasTag);
        packet.put("vision/tag_id", data.vision.tagId);
        packet.put("vision/range_in", data.vision.rangeIn);
        packet.put("vision/tx_deg", data.vision.txDeg);

        // Intake
        packet.put("intake/count", data.intake.artifactCount);
        packet.put("intake/mode", data.intake.mode);
        packet.put("intake/left_color", data.intake.laneLeftSummary.color);
        packet.put("intake/center_color", data.intake.laneCenterSummary.color);
        packet.put("intake/right_color", data.intake.laneRightSummary.color);

        // Launcher (per lane: ready + RPM)
        addPracticeLaneData(packet, "launcher/left", data.launcher.left);
        addPracticeLaneData(packet, "launcher/center", data.launcher.center);
        addPracticeLaneData(packet, "launcher/right", data.launcher.right);

        // Timing
        packet.put("timing/loop_ms", data.timing.mainLoopMs);

        // Field overlay
        addFieldOverlay(packet, data, false);

        return packet;
    }

    private void addPracticeLaneData(TelemetryPacket packet, String prefix, LauncherTelemetryData.LaneData lane) {
        packet.put(prefix + "/ready", lane.ready);
        packet.put(prefix + "/target_rpm", lane.targetRpm);
        packet.put(prefix + "/rpm", lane.currentRpm);
    }

    /**
     * Create VERBOSE level packet - comprehensive diagnostics.
     */
    private TelemetryPacket createVerbosePacket(RobotTelemetryData data) {
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

        // Field overlay (with vision pose in DEBUG)
        addFieldOverlay(packet, data, true);

        return packet;
    }

    /**
     * Add pose data to packet.
     * Includes Pedro pose, FTC pose, and vision pose (when available).
     */
    private void addPoseData(TelemetryPacket packet, RobotTelemetryData data) {
        // Pedro pose (primary). putPose emits the x/y/heading triplet AdvantageScope
        // Lite synthesises into a draggable Pose2d; degrees stays separate for graphs.
        RobotState.putPose("Pose/Pedro", data.pose.poseXIn, data.pose.poseYIn, data.pose.headingRad);
        packet.put("Pose/Pedro degrees", Math.toDegrees(data.pose.headingRad));

        if (!Double.isNaN(data.pose.ftcXIn) && !Double.isNaN(data.pose.ftcYIn) && !Double.isNaN(data.pose.ftcHeadingRad)) {
            RobotState.putPose("Pose/FTC", data.pose.ftcXIn, data.pose.ftcYIn, data.pose.ftcHeadingRad);
        }

        if (data.pose.visionPoseValid) {
            RobotState.putPose("Pose/Vision", data.pose.visionPoseXIn, data.pose.visionPoseYIn, data.pose.visionHeadingRad);
        }

        publishRobotFramePose(data);
    }

    /**
     * Publishes the AS-draggable {@code Pose/Robot} in the FTC frame, shifted from
     * the drivetrain/odometry center to the robot's frame center (see
     * {@link RobotFrameConfig}) so the AdvantageScope box sits on the real body
     * instead of misrepresenting where the robot is. Visualization only.
     */
    private void publishRobotFramePose(RobotTelemetryData data) {
        if (Double.isNaN(data.pose.poseXIn)
                || Double.isNaN(data.pose.poseYIn)
                || Double.isNaN(data.pose.headingRad)) {
            return;
        }
        Pose framePedro = PoseFrames.shiftToFrameCenter(
                new Pose(data.pose.poseXIn, data.pose.poseYIn, data.pose.headingRad),
                RobotFrameConfig.frameCenterForwardIn,
                RobotFrameConfig.frameCenterLeftIn);
        Pose frameFtc = PoseFrames.pedroToFtc(framePedro);
        RobotState.putPose("Pose/Robot", frameFtc.getX(), frameFtc.getY(), frameFtc.getHeading());
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
}
