package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumSet;
import java.util.Objects;
import java.util.Optional;

/**
 * Fires all three launcher lanes with RPM and hood position calculated from distance to goal.
 *
 * Uses AprilTag vision to determine robot pose and calculate distance to the goal tag.
 * Falls back to odometry pose if vision is unavailable. RPM is interpolated based on
 * configurable distance/RPM calibration points from LaunchAllAtPresetRangeCommand.
 *
 * The command:
 * 1. Calculates distance to goal using vision (or odometry fallback)
 * 2. Interpolates RPM based on distance using calibrated data points
 * 3. Calculates hood position based on distance
 * 4. Spins up flywheels to calculated RPM
 * 5. Fires all lanes in sequence when ready
 * 6. Optionally spins down after firing
 */
@Configurable
public class LaunchAllAtDistanceBasedRPMCommand extends Command {

    @Configurable
    public static class DiagnosticData {
        /** Last calculated distance to goal in inches (updates during command execution) */
        public double lastCalculatedDistanceIn = 0.0;
        /** Last calculated left lane RPM */
        public double lastLeftRpm = 0.0;
        /** Last calculated center lane RPM */
        public double lastCenterRpm = 0.0;
        /** Last calculated right lane RPM */
        public double lastRightRpm = 0.0;
        /** Last calculated hood position */
        public double lastHoodPosition = 0.0;
        /** Whether vision was successfully used (true) or fell back to odometry (false) */
        public boolean usedVision = false;
        /** Data source for distance: "vision", "odometry", or "none" */
        public String lastSource = "none";
        /** Current command stage */
        public String currentStage = "not_started";
    }

    public static DiagnosticData diagnostics = new DiagnosticData();

    @Configurable
    public static class DistanceRpmCalibration {
        /** Distance in inches for short range reference point */
        public double shortRangeDistanceIn = 36.0;
        /** RPM for left lane at short range */
        public double shortLeftRpm = 1000;
        /** RPM for center lane at short range */
        public double shortCenterRpm = 1000;
        /** RPM for right lane at short range */
        public double shortRightRpm = 1000;

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
        public double longLeftRpm = 2900;
        /** RPM for center lane at long range */
        public double longCenterRpm = 2850;
        /** RPM for right lane at long range */
        public double longRightRpm = 3000;

        /** Timeout in seconds before giving up on spin-up */
        public double timeoutSeconds = 8.0;
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

    private enum Stage {
        CALCULATING_DISTANCE,
        SPINNING_UP,
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final VisionSubsystemLimelight vision;
    private final DriveSubsystem drive;
    private final boolean spinDownAfterShot;

    private final EnumSet<LauncherLane> queuedLanes = EnumSet.noneOf(LauncherLane.class);
    private final ElapsedTime timer = new ElapsedTime();

    private Stage stage = Stage.CALCULATING_DISTANCE;
    private boolean spinDownApplied = false;
    private double calculatedDistance = 0.0;

    /**
     * Creates command that fires all lanes at RPM calculated from distance to goal.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed roller control, nullable)
     * @param vision The vision subsystem (for AprilTag distance)
     * @param drive The drive subsystem (for odometry fallback)
     * @param spinDownAfterShot Whether to spin down to idle after firing
     */
    public LaunchAllAtDistanceBasedRPMCommand(LauncherSubsystem launcher,
                                               IntakeSubsystem intake,
                                               VisionSubsystemLimelight vision,
                                               DriveSubsystem drive,
                                               boolean spinDownAfterShot) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake; // Nullable - robot may not have prefeed roller
        this.vision = Objects.requireNonNull(vision, "vision required");
        this.drive = Objects.requireNonNull(drive, "drive required");
        this.spinDownAfterShot = spinDownAfterShot;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.CALCULATING_DISTANCE;
        queuedLanes.clear();
        spinDownApplied = false;
        calculatedDistance = 0.0;

        // Initialize diagnostics
        diagnostics.currentStage = stage.name();
        diagnostics.lastSource = "none";
        diagnostics.usedVision = false;
    }

    @Override
    public void update() {
        diagnostics.currentStage = stage.name();

        switch (stage) {
            case CALCULATING_DISTANCE:
                // Calculate distance to goal and set RPMs
                calculatedDistance = calculateDistanceToGoal();
                if (calculatedDistance > 0.0) {
                    // Activate prefeed roller in forward direction to help feed
                    if (intake != null) {
                        intake.setPrefeedForward();
                    }

                    // Set RPMs based on calculated distance
                    setRpmsForDistance(calculatedDistance);

                    // Set hood positions based on distance
                    setHoodForDistance(calculatedDistance);

                    // Spin up to target
                    launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);

                    stage = Stage.SPINNING_UP;
                } else {
                    // If we can't get distance, abort
                    stage = Stage.COMPLETED;
                }
                break;

            case SPINNING_UP:
                // Wait briefly for flywheels to start ramping
                if (timer.milliseconds() > 100) {
                    stage = Stage.WAITING_FOR_READY;
                }
                break;

            case WAITING_FOR_READY:
                checkLaneReadiness();
                if (areAllEnabledLanesQueued()) {
                    stage = Stage.SHOTS_QUEUED;
                }
                // Timeout safety
                if (timer.seconds() >= calibration.timeoutSeconds) {
                    // Queue whatever lanes are ready to prevent hanging
                    queueRemainingLanes();
                    stage = Stage.SHOTS_QUEUED;
                }
                break;

            case SHOTS_QUEUED:
                if (!launcher.isBusy() && launcher.getQueuedShots() == 0) {
                    stage = Stage.COMPLETED;
                    if (spinDownAfterShot && !spinDownApplied) {
                        launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
                        spinDownApplied = true;
                    }
                }
                break;

            case COMPLETED:
            default:
                // nothing
                break;
        }
    }

    @Override
    public boolean isDone() {
        return stage == Stage.COMPLETED;
    }

    @Override
    public void stop(boolean interrupted) {
        // Deactivate prefeed roller (returns to not spinning)
        if (intake != null) {
            intake.setPrefeedReverse();
        }

        // Clear RPM overrides to return to default values
        launcher.clearOverrides();

        // Clear queue if interrupted
        if (interrupted && !queuedLanes.isEmpty()) {
            launcher.clearQueue();
        }

        // Spin down if configured and not already done
        if (interrupted && spinDownAfterShot && !spinDownApplied) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
            spinDownApplied = true;
        }
    }

    /**
     * Calculates distance to the goal tag in inches.
     * Uses vision pose if available, falls back to odometry pose.
     *
     * @return Distance to goal in inches, or 0.0 if unable to determine
     */
    private double calculateDistanceToGoal() {
        // Try to get robot pose from vision first
        Optional<Pose> poseOpt = vision.getRobotPoseFromTagPedro();
        boolean visionUsed = false;

        // Fall back to odometry if vision unavailable
        if (!poseOpt.isPresent()) {
            Pose odometryPose = drive.getFollower().getPose();
            if (odometryPose != null) {
                poseOpt = Optional.of(odometryPose);
                diagnostics.lastSource = "odometry";
                diagnostics.usedVision = false;
            } else {
                diagnostics.lastSource = "none";
                diagnostics.usedVision = false;
                diagnostics.lastCalculatedDistanceIn = 0.0;
                return 0.0;
            }
        } else {
            diagnostics.lastSource = "vision";
            diagnostics.usedVision = true;
            visionUsed = true;
        }

        // Get goal pose based on alliance
        Optional<Pose> goalOpt = vision.getTargetGoalPose();

        if (!poseOpt.isPresent() || !goalOpt.isPresent()) {
            diagnostics.lastSource = "none";
            diagnostics.usedVision = false;
            diagnostics.lastCalculatedDistanceIn = 0.0;
            return 0.0;
        }

        Pose robotPose = poseOpt.get();
        Pose goalPose = goalOpt.get();

        // Calculate Euclidean distance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double distance = Math.hypot(dx, dy);

        // Update diagnostics
        diagnostics.lastCalculatedDistanceIn = distance;

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
            // Above long range - use long RPM (or extrapolate if desired)
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

    /**
     * Checks each lane for readiness and queues shots immediately when ready.
     */
    private void checkLaneReadiness() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) {
                continue;
            }

            // Skip disabled lanes (RPM = 0)
            if (launcher.getLaunchRpm(lane) <= 0.0) {
                queuedLanes.add(lane); // Mark as "queued" to skip in future checks
                continue;
            }

            // Queue shot immediately when lane is ready - no stability wait
            if (launcher.isLaneReady(lane)) {
                launcher.queueShot(lane);
                queuedLanes.add(lane);
            }
        }
    }

    /**
     * Checks if all enabled lanes have been queued.
     */
    private boolean areAllEnabledLanesQueued() {
        for (LauncherLane lane : LauncherLane.values()) {
            // Skip disabled lanes
            if (launcher.getLaunchRpm(lane) <= 0.0) {
                continue;
            }
            // If any enabled lane isn't queued, we're not done
            if (!queuedLanes.contains(lane)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Queues all remaining enabled lanes (used on timeout).
     */
    private void queueRemainingLanes() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) {
                continue;
            }
            if (launcher.getLaunchRpm(lane) > 0.0) {
                launcher.queueShot(lane);
                queuedLanes.add(lane);
            }
        }
    }

    /**
     * Gets the calculated distance for telemetry/logging.
     */
    public double getCalculatedDistance() {
        return calculatedDistance;
    }
}
