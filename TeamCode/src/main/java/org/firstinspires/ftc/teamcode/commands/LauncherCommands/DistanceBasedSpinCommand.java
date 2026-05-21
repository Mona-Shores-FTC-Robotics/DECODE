package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.DistanceCalibrationConfig;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.HoodThresholdsConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotProfile;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Continuously calculates distance to goal and updates launcher RPM while held.
 * Ported from NextFTC to an Ivy static factory.
 */
@Configurable
public final class DistanceBasedSpinCommand {

    /** Exponential smoothing weight for the live distance reading (higher = trust new sample more, less smoothing). */
    private static final double DISTANCE_SMOOTHING_FACTOR = 0.7;
    /** Lane is "ready" once its actual RPM reaches this fraction of the target RPM. */
    private static final double RPM_READY_THRESHOLD_PERCENT = 0.90;

    /** Runtime diagnostic snapshot — read by telemetry, not a tuning surface. */
    public static class DiagnosticData {
        public double lastCalculatedDistanceIn = 0.0;
        public double lastLeftTargetRpm = 0.0;
        public double lastCenterTargetRpm = 0.0;
        public double lastRightTargetRpm = 0.0;
        public double lastHoodPosition = 0.0;
        public String lastSource = "none";
        public int updateCount = 0;
        public boolean robotPoseAvailable = false;
        public boolean goalPoseAvailable = false;
    }
    public static DiagnosticData diagnostics = new DiagnosticData();

    public static DistanceCalibrationConfig distanceCalibration = new DistanceCalibrationConfig();
    public static HoodThresholdsConfig hoodThresholds = new HoodThresholdsConfig();

    private DistanceBasedSpinCommand() {}

    public static CommandRangeConfig rangeConfig() {
        return RobotProfile.forCurrent().commandRange;
    }

    public static Command create(LauncherSubsystem launcher,
                                  VisionSubsystemLimelight vision,
                                  DriveSubsystem drive,
                                  LightingSubsystem lighting,
                                  Gamepad gamepad) {
        Objects.requireNonNull(launcher, "launcher required");
        Objects.requireNonNull(vision, "vision required");
        Objects.requireNonNull(drive, "drive required");

        final double[] lastSmoothedDistanceIn = {0.0};
        final boolean[] feedbackTriggered = {false};
        final ElapsedTime readyLossTimer = new ElapsedTime();

        return new CommandBuilder()
                .setStart(() -> {
                    diagnostics.updateCount = 0;
                    lastSmoothedDistanceIn[0] = 0.0;
                    feedbackTriggered[0] = false;
                    readyLossTimer.reset();
                    launcher.clearRecoveryDeadlines();
                    CommandRangeConfig cfg = rangeConfig();
                    launcher.setLaunchRpm(LauncherLane.LEFT, cfg.midLeftRpm);
                    launcher.setLaunchRpm(LauncherLane.CENTER, cfg.midCenterRpm);
                    launcher.setLaunchRpm(LauncherLane.RIGHT, cfg.midRightRpm);
                    launcher.setAllHoodPositions(cfg.midHoodPosition);
                    launcher.spinUpAllLanesToLaunch();
                })
                .setExecute(() -> {
                    diagnostics.updateCount++;
                    double rawDistance = calculateDistanceToGoal(vision, drive);
                    double smoothedDistance;
                    if (rawDistance > 0.0) {
                        if (lastSmoothedDistanceIn[0] > 0.0) {
                            smoothedDistance = DISTANCE_SMOOTHING_FACTOR * rawDistance
                                    + (1.0 - DISTANCE_SMOOTHING_FACTOR) * lastSmoothedDistanceIn[0];
                        } else {
                            smoothedDistance = rawDistance;
                        }
                        lastSmoothedDistanceIn[0] = smoothedDistance;
                    } else {
                        smoothedDistance = 0.0;
                        lastSmoothedDistanceIn[0] = 0.0;
                    }
                    if (smoothedDistance > 0.0) {
                        setRpmsForDistance(launcher, smoothedDistance);
                        setHoodForDistance(launcher, smoothedDistance);
                        checkAndTriggerReadyFeedback(launcher, drive, lighting, gamepad,
                                feedbackTriggered, readyLossTimer);
                    }
                    diagnostics.lastCalculatedDistanceIn = smoothedDistance;

                    if (TelemetrySettings.isVerbose()) {
                        RobotState.packet.put("Commands/Distance-Based-Spin/Update Count", diagnostics.updateCount);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Distance (in)", diagnostics.lastCalculatedDistanceIn);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Left Target RPM", diagnostics.lastLeftTargetRpm);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Center Target RPM", diagnostics.lastCenterTargetRpm);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Right Target RPM", diagnostics.lastRightTargetRpm);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Hood Position", diagnostics.lastHoodPosition);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Data Source", diagnostics.lastSource);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Robot Pose Available", diagnostics.robotPoseAvailable);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Goal Pose Available", diagnostics.goalPoseAvailable);
                        RobotState.packet.put("Commands/Distance-Based-Spin/Feedback Triggered", feedbackTriggered[0]);
                    }
                })
                .setDone(() -> false)
                .requiring(launcher);
    }

    private static void checkAndTriggerReadyFeedback(LauncherSubsystem launcher,
                                                       DriveSubsystem drive,
                                                       LightingSubsystem lighting,
                                                       Gamepad gamepad,
                                                       boolean[] feedbackTriggered,
                                                       ElapsedTime readyLossTimer) {
        double maxTargetRpm = 0.0;
        double maxCurrentRpm = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            double targetRpm = launcher.getLaunchRpm(lane);
            double currentRpm = launcher.getCurrentRpm(lane);
            if (targetRpm > 0.0) {
                maxTargetRpm = Math.max(maxTargetRpm, targetRpm);
                maxCurrentRpm = Math.max(maxCurrentRpm, currentRpm);
            }
        }
        boolean rpmReady = maxTargetRpm > 0.0 && maxCurrentRpm >= maxTargetRpm * RPM_READY_THRESHOLD_PERCENT;
        boolean aimReady = drive.isAimSettled(2.0);
        boolean stationary = drive.getRobotSpeedInchesPerSecond() <= DriveSubsystem.STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
        boolean readyWithAim = rpmReady && aimReady && stationary;

        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("Commands/Distance-Based-Spin/RPM Ready", rpmReady);
            RobotState.packet.put("Commands/Distance-Based-Spin/Aim Ready", aimReady);
            RobotState.packet.put("Commands/Distance-Based-Spin/Stationary", stationary);
            RobotState.packet.put("Commands/Distance-Based-Spin/Ready With Aim", readyWithAim);
        }
        if (readyWithAim) {
            readyLossTimer.reset();
            if (feedbackTriggered[0]) return;
            feedbackTriggered[0] = true;
            if (gamepad != null) gamepad.rumble(200);
            if (lighting != null) lighting.flashAimAligned();
            return;
        }
        if (readyLossTimer.milliseconds() >= LauncherSubsystem.READY_LOSS_DEBOUNCE_MS) {
            feedbackTriggered[0] = false;
        }
    }

    private static double calculateDistanceToGoal(VisionSubsystemLimelight vision, DriveSubsystem drive) {
        diagnostics.robotPoseAvailable = false;
        diagnostics.goalPoseAvailable = false;
        Pose odometryPose = drive.getFollower().getPose();
        if (odometryPose == null) {
            diagnostics.lastSource = "none";
            return 0.0;
        }
        diagnostics.lastSource = "odometry";
        diagnostics.robotPoseAvailable = true;
        Pose goalPose = vision.getTargetGoalPose();
        diagnostics.goalPoseAvailable = goalPose != null;
        if (goalPose == null) {
            diagnostics.lastSource = "none";
            return 0.0;
        }
        double dx = goalPose.getX() - odometryPose.getX();
        double dy = goalPose.getY() - odometryPose.getY();
        return Math.hypot(dx, dy);
    }

    private static void setRpmsForDistance(LauncherSubsystem launcher, double distanceIn) {
        CommandRangeConfig cfg = rangeConfig();
        double leftRpm = interpolateRpm(distanceIn,
                distanceCalibration.shortRangeDistanceIn, cfg.shortLeftRpm,
                distanceCalibration.midRangeDistanceIn, cfg.midLeftRpm,
                distanceCalibration.longRangeMinDistanceIn, cfg.longMinLeftRpm,
                distanceCalibration.longRangeMaxDistanceIn, cfg.longMaxLeftRpm);
        double centerRpm = interpolateRpm(distanceIn,
                distanceCalibration.shortRangeDistanceIn, cfg.shortCenterRpm,
                distanceCalibration.midRangeDistanceIn, cfg.midCenterRpm,
                distanceCalibration.longRangeMinDistanceIn, cfg.longMinCenterRpm,
                distanceCalibration.longRangeMaxDistanceIn, cfg.longMaxCenterRpm);
        double rightRpm = interpolateRpm(distanceIn,
                distanceCalibration.shortRangeDistanceIn, cfg.shortRightRpm,
                distanceCalibration.midRangeDistanceIn, cfg.midRightRpm,
                distanceCalibration.longRangeMinDistanceIn, cfg.longMinRightRpm,
                distanceCalibration.longRangeMaxDistanceIn, cfg.longMaxRightRpm);

        launcher.setLaunchRpm(LauncherLane.LEFT, leftRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, centerRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, rightRpm);
        launcher.spinUpLaneToLaunch(LauncherLane.LEFT);
        launcher.spinUpLaneToLaunch(LauncherLane.CENTER);
        launcher.spinUpLaneToLaunch(LauncherLane.RIGHT);

        diagnostics.lastLeftTargetRpm = leftRpm;
        diagnostics.lastCenterTargetRpm = centerRpm;
        diagnostics.lastRightTargetRpm = rightRpm;
    }

    private static double interpolateRpm(double distance,
                                           double shortDist, double shortRpm,
                                           double midDist, double midRpm,
                                           double longMinDist, double longMinRpm,
                                           double longMaxDist, double longMaxRpm) {
        distance = Range.clip(distance, shortDist * 0.5, longMaxDist * 1.5);
        if (distance <= shortDist) return shortRpm;
        if (distance <= midDist) {
            double t = (distance - shortDist) / (midDist - shortDist);
            return shortRpm + t * (midRpm - shortRpm);
        }
        if (distance <= longMaxDist) {
            double t = (distance - longMinDist) / (longMaxDist - longMinDist);
            return longMinRpm + t * (longMaxRpm - longMinRpm);
        }
        return longMaxRpm;
    }

    private static void setHoodForDistance(LauncherSubsystem launcher, double distanceIn) {
        CommandRangeConfig cfg = rangeConfig();
        double hoodPosition = interpolateHood(distanceIn,
                hoodThresholds.shortRangeDistanceIn, cfg.shortHoodPosition,
                hoodThresholds.midRangeDistanceIn, cfg.midHoodPosition,
                cfg.longHoodPosition);
        launcher.setAllHoodPositions(hoodPosition);
        diagnostics.lastHoodPosition = hoodPosition;
    }

    private static double interpolateHood(double distance,
                                            double shortDist, double shortPos,
                                            double midDist, double midPos,
                                            double longPos) {
        distance = Range.clip(distance, shortDist * 0.5, midDist * 1.5);
        if (distance <= shortDist) return shortPos;
        if (distance <= midDist) {
            double t = (distance - shortDist) / (midDist - shortDist);
            return shortPos + t * (midPos - shortPos);
        }
        return longPos;
    }
}
