package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;
import java.util.Optional;

/**
 * Continuously calculates distance to goal and updates launcher RPM while held.
 *
 * This command is designed for hold-to-spin, release-to-fire button behavior:
 * 1. While held: Continuously calculates distance and updates RPM targets
 * 2. On release: Caller should trigger fire command
 *
 * Uses AprilTag vision for distance measurement, falls back to odometry if unavailable.
 * RPM is interpolated based on configurable distance/RPM calibration points.
 */
@Configurable
public class ContinuousDistanceBasedSpinCommand extends Command {

    @Configurable
    public static class DiagnosticData {
        /** Last calculated distance to goal in inches (updates every loop) */
        public double lastCalculatedDistanceIn = 0.0;
        /** Last calculated left lane RPM */
        public double lastLeftRpm = 0.0;
        /** Last calculated center lane RPM */
        public double lastCenterRpm = 0.0;
        /** Last calculated right lane RPM */
        public double lastRightRpm = 0.0;
        /** Last calculated hood position */
        public double lastHoodPosition = 0.0;
        /** Data source for distance: "vision", "odometry", or "none" */
        public String lastSource = "none";
        /** Number of update cycles performed */
        public int updateCount = 0;
        /** Robot pose available from vision or odometry */
        public boolean robotPoseAvailable = false;
        /** Goal pose available from vision */
        public boolean goalPoseAvailable = false;
        /** Vision pose available */
        public boolean visionPoseAvailable = false;
        /** Odometry pose available */
        public boolean odometryPoseAvailable = false;
    }

    public static DiagnosticData diagnostics = new DiagnosticData();

    @Configurable
    public static class DistanceRpmCalibration {
        /** Distance in inches for short range reference point */
        public double shortRangeDistanceIn = 36.0;
        /** RPM for left lane at short range */
        public double shortLeftRpm = 2100;
        /** RPM for center lane at short range */
        public double shortCenterRpm = 2100;
        /** RPM for right lane at short range */
        public double shortRightRpm = 2100;

        /** Distance in inches for mid range reference point */
        public double midRangeDistanceIn = 72.0;
        /** RPM for left lane at mid range */
        public double midLeftRpm = 2550;
        /** RPM for center lane at mid range */
        public double midCenterRpm = 2550;
        /** RPM for right lane at mid range */
        public double midRightRpm = 2550;

        /** Distance in inches for long range reference point */
        public double longRangeDistanceIn = 108.0;
        /** RPM for left lane at long range */
        public double longLeftRpm = 3000;
        /** RPM for center lane at long range */
        public double longCenterRpm = 3000;
        /** RPM for right lane at long range */
        public double longRightRpm = 3000;
    }

    @Configurable
    public static class HoodCalibration {
        /** Hood position for distances below short range threshold */
        public double shortHoodPosition = 0.45;
        /** Hood position for distances in mid range */
        public double midHoodPosition = 0.0;
        /** Hood position for distances above long range threshold */
        public double longHoodPosition = 0.0;
        /** Distance threshold (inches) between short and mid range */
        public double shortToMidThresholdIn = 54.0;
        /** Distance threshold (inches) between mid and long range */
        public double midToLongThresholdIn = 90.0;
    }

    public static DistanceRpmCalibration calibration = new DistanceRpmCalibration();
    public static HoodCalibration hoodCalibration = new HoodCalibration();

    private final LauncherSubsystem launcher;
    private final VisionSubsystemLimelight vision;
    private final DriveSubsystem drive;
    private final LightingSubsystem lighting;
    private final Gamepad gamepad;

    private double lastSmoothedDistanceIn = 0.0;
    private static final double DISTANCE_SMOOTHING_FACTOR = 0.7;  // 0-1: higher = more filtering
    private static final double RPM_READY_THRESHOLD_PERCENT = 0.95;  // Trigger feedback at 95% of target
    private boolean feedbackTriggered = false;

    /**
     * Creates command that continuously updates launcher RPM based on distance to goal.
     *
     * @param launcher The launcher subsystem
     * @param vision The vision subsystem (for AprilTag distance)
     * @param drive The drive subsystem (for odometry fallback)
     * @param lighting The lighting subsystem (for ready feedback)
     * @param gamepad The operator gamepad (for haptic feedback)
     */
    public ContinuousDistanceBasedSpinCommand(LauncherSubsystem launcher,
                                               VisionSubsystemLimelight vision,
                                               DriveSubsystem drive,
                                               LightingSubsystem lighting,
                                               Gamepad gamepad) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.vision = Objects.requireNonNull(vision, "vision required");
        this.drive = Objects.requireNonNull(drive, "drive required");
        this.lighting = lighting;  // Can be null
        this.gamepad = gamepad;    // Can be null
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        // Initialize diagnostics
        diagnostics.lastSource = "none";
        diagnostics.updateCount = 0;
        lastSmoothedDistanceIn = 0.0;
        feedbackTriggered = false;

        // Set spin mode to FULL to start spinning up
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
    }

    @Override
    public void update() {
        diagnostics.updateCount++;

        // Calculate raw distance to goal
        double rawDistance = calculateDistanceToGoal();

        // Apply exponential smoothing to reduce jitter from noisy pose estimates
        double smoothedDistance;
        if (rawDistance > 0.0) {
            if (lastSmoothedDistanceIn > 0.0) {
                // Exponential moving average: smooth_new = factor * raw + (1 - factor) * smooth_old
                smoothedDistance = DISTANCE_SMOOTHING_FACTOR * rawDistance + (1.0 - DISTANCE_SMOOTHING_FACTOR) * lastSmoothedDistanceIn;
            } else {
                // First measurement - use raw value
                smoothedDistance = rawDistance;
            }
            lastSmoothedDistanceIn = smoothedDistance;
        } else {
            smoothedDistance = 0.0;
            lastSmoothedDistanceIn = 0.0;
        }

        if (smoothedDistance > 0.0) {
            // Set RPMs based on smoothed distance
            setRpmsForDistance(smoothedDistance);

            // Set hood positions based on smoothed distance
            setHoodForDistance(smoothedDistance);

            // Check if any lane has reached 95% of target RPM for haptic/light feedback
            checkAndTriggerReadyFeedback();
        }

        // Update diagnostics
        diagnostics.lastCalculatedDistanceIn = smoothedDistance;

        // Push diagnostics to telemetry packet
        RobotState.packet.put("X Button Distance-Based/Update Count", diagnostics.updateCount);
        RobotState.packet.put("X Button Distance-Based/Distance (in)", diagnostics.lastCalculatedDistanceIn);
        RobotState.packet.put("X Button Distance-Based/Left RPM", diagnostics.lastLeftRpm);
        RobotState.packet.put("X Button Distance-Based/Center RPM", diagnostics.lastCenterRpm);
        RobotState.packet.put("X Button Distance-Based/Right RPM", diagnostics.lastRightRpm);
        RobotState.packet.put("X Button Distance-Based/Hood Position", diagnostics.lastHoodPosition);
        RobotState.packet.put("X Button Distance-Based/Data Source", diagnostics.lastSource);
        RobotState.packet.put("X Button Distance-Based/Robot Pose Available", diagnostics.robotPoseAvailable);
        RobotState.packet.put("X Button Distance-Based/Goal Pose Available", diagnostics.goalPoseAvailable);
        RobotState.packet.put("X Button Distance-Based/Vision Pose Available", diagnostics.visionPoseAvailable);
        RobotState.packet.put("X Button Distance-Based/Odometry Pose Available", diagnostics.odometryPoseAvailable);
    }

    @Override
    public boolean isDone() {
        // This command runs continuously until interrupted (button release)
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        // Preserve the calculated RPMs so LaunchAllCommand can use them immediately
        // Do NOT clear overrides - the fire command needs these RPMs to determine readiness
        // The launcher stays spun up at the calculated RPM for immediate firing
    }

    /**
     * Checks if any launcher lane has reached 95% of target RPM and triggers feedback.
     * Feedback is only triggered once per button press (when first crossing the threshold).
     */
    private void checkAndTriggerReadyFeedback() {
        if (feedbackTriggered) {
            return;  // Only trigger once
        }

        // Check each lane to see if it's at 95% of target RPM
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

        // Trigger feedback if current RPM >= 95% of target RPM
        if (maxTargetRpm > 0.0 && maxCurrentRpm >= maxTargetRpm * RPM_READY_THRESHOLD_PERCENT) {
            feedbackTriggered = true;

            // Haptic feedback: rumble controller for 200ms
            if (gamepad != null) {
                gamepad.rumble(200);  // 200ms rumble
            }

            // Light feedback: flash yellow to indicate launcher ready
            if (lighting != null) {
                lighting.flashAimAligned();
            }
        }
    }

    /**
     * Calculates distance to the goal tag in inches.
     * Uses vision pose if available, falls back to odometry pose.
     *
     * @return Distance to goal in inches, or 0.0 if unable to determine
     */
    private double calculateDistanceToGoal() {
        // Reset diagnostic flags
        diagnostics.robotPoseAvailable = false;
        diagnostics.goalPoseAvailable = false;
        diagnostics.visionPoseAvailable = false;
        diagnostics.odometryPoseAvailable = false;

        // Try to get robot pose from vision first
        Optional<Pose> poseOpt = vision.getRobotPoseFromTag();
        diagnostics.visionPoseAvailable = poseOpt.isPresent();

        // Fall back to odometry if vision unavailable
        if (!poseOpt.isPresent()) {
            Pose odometryPose = drive.getFollower().getPose();
            diagnostics.odometryPoseAvailable = (odometryPose != null);
            if (odometryPose != null) {
                poseOpt = Optional.of(odometryPose);
                diagnostics.lastSource = "odometry";
                diagnostics.robotPoseAvailable = true;
            } else {
                diagnostics.lastSource = "none";
                return 0.0;
            }
        } else {
            diagnostics.lastSource = "vision";
            diagnostics.robotPoseAvailable = true;
        }

        // Get goal pose based on alliance
        Optional<Pose> goalOpt = vision.getTargetGoalPose();
        diagnostics.goalPoseAvailable = goalOpt.isPresent();

        if (!poseOpt.isPresent() || !goalOpt.isPresent()) {
            diagnostics.lastSource = "none";
            return 0.0;
        }

        Pose robotPose = poseOpt.get();
        Pose goalPose = goalOpt.get();

        // Calculate Euclidean distance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double distance = Math.hypot(dx, dy);

        return distance;
    }

    /**
     * Sets RPM targets for all lanes based on distance using linear interpolation.
     * Uses calibration points from LaunchAllAtPresetRangeCommand.
     */
    private void setRpmsForDistance(double distanceIn) {
        double leftRpm = interpolateRpm(distanceIn,
            calibration.shortRangeDistanceIn, calibration.shortLeftRpm,
            calibration.midRangeDistanceIn, calibration.midLeftRpm,
            calibration.longRangeDistanceIn, calibration.longLeftRpm);

        double centerRpm = interpolateRpm(distanceIn,
            calibration.shortRangeDistanceIn, calibration.shortCenterRpm,
            calibration.midRangeDistanceIn, calibration.midCenterRpm,
            calibration.longRangeDistanceIn, calibration.longCenterRpm);

        double rightRpm = interpolateRpm(distanceIn,
            calibration.shortRangeDistanceIn, calibration.shortRightRpm,
            calibration.midRangeDistanceIn, calibration.midRightRpm,
            calibration.longRangeDistanceIn, calibration.longRightRpm);

        launcher.setLaunchRpm(LauncherLane.LEFT, leftRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, centerRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, rightRpm);

        // Update diagnostics
        diagnostics.lastLeftRpm = leftRpm;
        diagnostics.lastCenterRpm = centerRpm;
        diagnostics.lastRightRpm = rightRpm;
    }

    /**
     * Interpolates RPM based on distance using piecewise linear interpolation.
     *
     * @param distance Current distance to goal
     * @param shortDist Short range distance calibration point
     * @param shortRpm Short range RPM
     * @param midDist Mid range distance calibration point
     * @param midRpm Mid range RPM
     * @param longDist Long range distance calibration point
     * @param longRpm Long range RPM
     * @return Interpolated RPM value
     */
    private double interpolateRpm(double distance,
                                   double shortDist, double shortRpm,
                                   double midDist, double midRpm,
                                   double longDist, double longRpm) {
        // Clamp distance to reasonable range
        distance = Range.clip(distance, shortDist * 0.5, longDist * 1.5);

        // Piecewise linear interpolation
        if (distance <= shortDist) {
            // Below short range - use short RPM
            return shortRpm;
        } else if (distance <= midDist) {
            // Between short and mid - interpolate
            double t = (distance - shortDist) / (midDist - shortDist);
            return shortRpm + t * (midRpm - shortRpm);
        } else if (distance <= longDist) {
            // Between mid and long - interpolate
            double t = (distance - midDist) / (longDist - midDist);
            return midRpm + t * (longRpm - midRpm);
        } else {
            // Above long range - use long RPM
            return longRpm;
        }
    }

    /**
     * Sets hood position for all lanes based on distance thresholds.
     */
    private void setHoodForDistance(double distanceIn) {
        double hoodPosition;

        if (distanceIn < hoodCalibration.shortToMidThresholdIn) {
            hoodPosition = hoodCalibration.shortHoodPosition;
        } else if (distanceIn < hoodCalibration.midToLongThresholdIn) {
            hoodPosition = hoodCalibration.midHoodPosition;
        } else {
            hoodPosition = hoodCalibration.longHoodPosition;
        }

        launcher.setAllHoodPositions(hoodPosition);

        // Update diagnostics
        diagnostics.lastHoodPosition = hoodPosition;
    }
}
