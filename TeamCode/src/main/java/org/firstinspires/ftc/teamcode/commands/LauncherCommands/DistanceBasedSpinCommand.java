package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotConfigs;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;
import java.util.Optional;

import dev.nextftc.core.commands.Command;

/**
 * Continuously calculates distance to goal and updates launcher RPM while held.
 * This command is designed for hold-to-spin, release-to-fire button behavior:
 * 1. While held: Continuously calculates distance and updates RPM targets
 * 2. On release: Caller should trigger fire command
 * Uses AprilTag vision for distance measurement, falls back to odometry if unavailable.
 * RPM is interpolated using the shared preset short/mid/long RPM anchors and configurable distance breakpoints.
 */
@Configurable
public class DistanceBasedSpinCommand extends Command {

    @Configurable
    public static class DiagnosticData {
        /** Last calculated distance to goal in inches (updates every loop) */
        public double lastCalculatedDistanceIn = 0.0;
        /** Last calculated left lane RPM */
        public double lastLeftTargetRpm = 0.0;
        /** Last calculated center lane RPM */
        public double lastCenterTargetRpm = 0.0;
        /** Last calculated right lane RPM */
        public double lastRightTargetRpm = 0.0;
        /** Last calculated hood position */
        public double lastHoodPosition = 0.0;
        /** Data source for distance: "vision", "odometry", or "none" */
        public String lastSource = "none";
        /** Number of update cycles performed */
        public int updateCount = 0;
        /** Robot pose available */
        public boolean robotPoseAvailable = false;
        /** Goal pose available */
        public boolean goalPoseAvailable = false;
    }

    public static DiagnosticData diagnostics = new DiagnosticData();

    @Configurable
    public static class DistanceCalibration {
        /** Distance in inches for short range reference point */
        public double shortRangeDistanceIn = 18.4;

        /** Distance in inches for mid range reference point */
        public double midRangeDistanceIn = 96.0;

        /** Distance in inches for long range reference point */
        public double longRangeMinDistanceIn = 125.4;
        public double longRangeMaxDistanceIn = 153;
    }

    @Configurable
    public static class HoodThresholds {
        /** Distance threshold (inches) between short and mid range */
        public double shortRangeDistanceIn = 0;
        /** Distance threshold (inches) between mid and long range */
        public double midRangeDistanceIn = 90;
    }

    public static DistanceCalibration distanceCalibration = new DistanceCalibration();
    public static HoodThresholds hoodThresholds = new HoodThresholds();

    private final LauncherSubsystem launcher;
    private final VisionSubsystemLimelight vision;
    private final DriveSubsystem drive;
    private final LightingSubsystem lighting;
    private final Gamepad gamepad;
    public static CommandRangeConfig rangeConfig() {
        return RobotConfigs.getCommandRangeConfig();
    }

    private double lastSmoothedDistanceIn = 0.0;
    private static final double DISTANCE_SMOOTHING_FACTOR = 0.7;  // 0-1: higher = more filtering
    private static final double RPM_READY_THRESHOLD_PERCENT = 0.95;  // Trigger feedback at 95% of target
    private static final double READY_LOSS_DEBOUNCE_MS = 250.0;  // How long to be "not ready" before re-arming
    private boolean feedbackTriggered = false;
    private final ElapsedTime readyLossTimer = new ElapsedTime();

    /**
     * Creates command that continuously updates launcher RPM based on distance to goal.
     *
     * @param launcher The launcher subsystem
     * @param vision The vision subsystem (for AprilTag distance)
     * @param drive The drive subsystem (for odometry fallback)
     * @param lighting The lighting subsystem (for ready feedback)
     * @param gamepad The operator gamepad (for haptic feedback)
     */
    public DistanceBasedSpinCommand(LauncherSubsystem launcher,
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
        diagnostics.updateCount = 0;
        lastSmoothedDistanceIn = 0.0;
        feedbackTriggered = false;
        readyLossTimer.reset();

        // Clear recovery deadlines so lanes can immediately respond to new RPM targets
        // Without this, lanes in recovery from previous shots won't update their target RPM
        launcher.clearRecoveryDeadlines();

        // Set initial RPMs to mid-range values so launcher starts spinning immediately
        // update() will refine these based on actual distance calculations
        launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().midLeftRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().midCenterRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().midRightRpm);
        launcher.setAllHoodPositions(rangeConfig().midHoodPosition);
        launcher.spinUpAllLanesToLaunch();
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
        RobotState.packet.put("Commands/Distance-Based-Spin/Update Count", diagnostics.updateCount);
        RobotState.packet.put("Commands/Distance-Based-Spin/Distance (in)", diagnostics.lastCalculatedDistanceIn);
        RobotState.packet.put("Commands/Distance-Based-Spin/Left Target RPM", diagnostics.lastLeftTargetRpm);
        RobotState.packet.put("Commands/Distance-Based-Spin/Center Target RPM", diagnostics.lastCenterTargetRpm);
        RobotState.packet.put("Commands/Distance-Based-Spin/Right Target RPM", diagnostics.lastRightTargetRpm);
        RobotState.packet.put("Commands/Distance-Based-Spin/Hood Position", diagnostics.lastHoodPosition);
        RobotState.packet.put("Commands/Distance-Based-Spin/Data Source", diagnostics.lastSource);
        RobotState.packet.put("Commands/Distance-Based-Spin/Robot Pose Available", diagnostics.robotPoseAvailable);
        RobotState.packet.put("Commands/Distance-Based-Spin/Goal Pose Available", diagnostics.goalPoseAvailable);
        RobotState.packet.put("Commands/Distance-Based-Spin/Feedback Triggered", feedbackTriggered);
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

        boolean rpmReady = maxTargetRpm > 0.0 && maxCurrentRpm >= maxTargetRpm * RPM_READY_THRESHOLD_PERCENT;
        boolean aimReady = drive.isAimSettled(2.0); // tight deadband for firing
        boolean stationary = drive.getRobotSpeedInchesPerSecond() <= DriveSubsystem.STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
        boolean readyWithAim = rpmReady && aimReady && stationary;

        // Publish readiness axes for dashboard / lighting debug
        RobotState.packet.put("Commands/Distance-Based-Spin/RPM Ready", rpmReady);
        RobotState.packet.put("Commands/Distance-Based-Spin/Aim Ready", aimReady);
        RobotState.packet.put("Commands/Distance-Based-Spin/Stationary", stationary);
        RobotState.packet.put("Commands/Distance-Based-Spin/Ready With Aim", readyWithAim);

        if (readyWithAim) {
            readyLossTimer.reset();
            if (feedbackTriggered) {
                return;
            }

            // Trigger feedback only when both launcher and aim are ready
            feedbackTriggered = true;

            // Haptic feedback: rumble controller for 200ms
            if (gamepad != null) {
                gamepad.rumble(200);  // 200ms rumble
            }

            // Light feedback: flash white to indicate launcher ready
            if (lighting != null) {
                lighting.flashAimAligned();
            }
            return;
        }

        // Re-arm once we've been out of the ready window for long enough
        if (readyLossTimer.milliseconds() >= READY_LOSS_DEBOUNCE_MS) {
            feedbackTriggered = false;
        }
    }

    /**
     * Calculates distance to the goal tag in inches.
     * Uses odometry pose only (vision only used if relocalized previously)
     * @return Distance to goal in inches, or 0.0 if unable to determine
     */
    private double calculateDistanceToGoal() {
        // Reset diagnostic flags
        diagnostics.robotPoseAvailable = false;
        diagnostics.goalPoseAvailable = false;

        // Use odometry pose only
        Pose odometryPose = drive.getFollower().getPose();
        Optional<Pose> poseOpt = Optional.empty();
        if (odometryPose != null) {
            poseOpt = Optional.of(odometryPose);
            diagnostics.lastSource = "odometry";
            diagnostics.robotPoseAvailable = true;
        } else {
            diagnostics.lastSource = "none";
            return 0.0;
        }

        // Get goal pose based on alliance
        Optional<Pose> goalOpt = vision.getTargetGoalPose();
        diagnostics.goalPoseAvailable = goalOpt.isPresent();

        if (!goalOpt.isPresent()) {
            diagnostics.lastSource = "none";
            return 0.0;
        }

        Pose robotPose = poseOpt.get();
        Pose goalPose = goalOpt.get();

        // Calculate Euclidean distance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy);
    }

    /**
     * Sets RPM targets for all lanes based on distance using linear interpolation.
     * Uses the shared preset range RPM anchors combined with tunable distance breakpoints.
     */
    private void setRpmsForDistance(double distanceIn) {
        double leftRpm = interpolateRpm(distanceIn,
                distanceCalibration.shortRangeDistanceIn, rangeConfig().shortLeftRpm,
                distanceCalibration.midRangeDistanceIn, rangeConfig().midLeftRpm,
                distanceCalibration.longRangeMinDistanceIn, rangeConfig().longMinLeftRpm,
                distanceCalibration.longRangeMaxDistanceIn , rangeConfig().longMaxLeftRpm);

        double centerRpm = interpolateRpm(distanceIn,
                distanceCalibration.shortRangeDistanceIn, rangeConfig().shortCenterRpm,
                distanceCalibration.midRangeDistanceIn, rangeConfig().midCenterRpm,
                distanceCalibration.longRangeMinDistanceIn, rangeConfig().longMinCenterRpm,
                distanceCalibration.longRangeMaxDistanceIn , rangeConfig().longMaxCenterRpm);

        double rightRpm = interpolateRpm(distanceIn,
                distanceCalibration.shortRangeDistanceIn, rangeConfig().shortRightRpm,
                distanceCalibration.midRangeDistanceIn, rangeConfig().midRightRpm,
                distanceCalibration.longRangeMinDistanceIn, rangeConfig().longMinRightRpm,
                distanceCalibration.longRangeMaxDistanceIn , rangeConfig().longMaxRightRpm);

        launcher.setLaunchRpm(LauncherLane.LEFT, leftRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, centerRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, rightRpm);

        // Update diagnostics
        diagnostics.lastLeftTargetRpm = leftRpm;
        diagnostics.lastCenterTargetRpm = centerRpm;
        diagnostics.lastRightTargetRpm = rightRpm;
    }

    private double interpolateRpm(double distance,
                                   double shortDist, double shortRpm,
                                   double midDist, double midRpm,
                                   double longMinDist, double longMinRpm,
                                   double longMaxDist, double longMaxRpm) {
        // Clamp distance to reasonable range
        distance = Range.clip(distance, shortDist * 0.5, longMaxDist * 1.5);

        // Piecewise linear interpolation
        if (distance <= shortDist) {
            // Below short range - use short RPM
            return shortRpm;
        } else if (distance <= midDist) {
            // Between short and mid - interpolate
            double t = (distance - shortDist) / (midDist - shortDist);
            return shortRpm + t * (midRpm - shortRpm);
        } else if (distance <= longMaxDist) {
            // Between minLong and maxLong - interpolate
            double t = (distance - longMinDist) / (longMaxDist - longMinDist);
            return longMinRpm + t * (longMaxRpm - longMinRpm);
        } else {
            // Above long range - use long RPM
            return longMaxRpm;
        }
    }

    private void setHoodForDistance(double distanceIn) {
        double hoodPosition = interpolateHood(
                distanceIn,
                hoodThresholds.shortRangeDistanceIn, rangeConfig().shortHoodPosition,
                hoodThresholds.midRangeDistanceIn, rangeConfig().midHoodPosition,
                rangeConfig().longHoodPosition
        );

        launcher.setAllHoodPositions(hoodPosition);

        diagnostics.lastHoodPosition = hoodPosition;
    }

    /**
     * Interpolates hood position based on distance using a single linear segment
     * between the short and mid calibration points.
     *
     * @param distance Current distance to goal (inches)
     * @param shortDist Distance at which the short hood position is used
     * @param shortPos Hood position for short range
     * @param midDist Distance at which the mid hood position is used
     * @param midPos Hood position for mid range
     * @param longPos Hood position to use for distances beyond midDist
     * @return Interpolated hood position
     */
    private double interpolateHood(double distance,
                                   double shortDist, double shortPos,
                                   double midDist,   double midPos,
                                   double longPos) {

        // Clamp distance to a reasonable range
        distance = Range.clip(distance, shortDist * 0.5, midDist * 1.5);

        if (distance <= shortDist) {
            return shortPos;

        } else if (distance <= midDist) {
            double t = (distance - shortDist) / (midDist - shortDist);
            return shortPos + t * (midPos - shortPos);

        } else {
            return longPos;
        }
    }


}
