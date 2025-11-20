package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;
import java.util.Optional;

/**
 * Ultimate smart shot command that combines distance-based RPM with mode-aware firing.
 *
 * This is the "one button does everything" command that:
 * 1. Calculates distance to goal using AprilTag vision (falls back to odometry)
 * 2. Interpolates RPM and hood position based on distance
 * 3. Spins up to calculated RPM with haptic/light feedback when ready
 * 4. Fires based on current launcher mode:
 *    - THROUGHPUT: All lanes fire rapidly for maximum scoring rate
 *    - DECODE: Fires in obelisk pattern sequence with motif tail offset
 *
 * Designed to replace multiple separate fire buttons with a single intelligent button.
 *
 * Usage: Bind to a single button (e.g., X) - operator just presses when ready to shoot,
 * command handles distance calculation, RPM selection, mode detection, and firing strategy.
 */
@Configurable
public class UniversalSmartShotCommand extends Command {

    @Configurable
    public static class DiagnosticData {
        /** Last calculated distance to goal in inches */
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
        /** Current launcher mode used for firing */
        public String launcherMode = "unknown";
        /** Current command stage */
        public String currentStage = "not_started";
        /** Whether haptic feedback was triggered */
        public boolean hapticTriggered = false;
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

    private static final double RPM_READY_THRESHOLD_PERCENT = 0.95;

    private enum Stage {
        CALCULATING_DISTANCE,
        SPINNING_UP,
        WAITING_FOR_READY,
        FIRING,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final VisionSubsystemLimelight vision;
    private final DriveSubsystem drive;
    private final LightingSubsystem lighting;
    private final Gamepad gamepad;

    private Stage stage = Stage.CALCULATING_DISTANCE;
    private Command delegateFireCommand;
    private boolean feedbackTriggered = false;
    private boolean spinDownApplied = false;

    /**
     * Creates the universal smart shot command.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed and artifact tracking)
     * @param vision The vision subsystem (for AprilTag distance)
     * @param drive The drive subsystem (for odometry fallback)
     * @param lighting The lighting subsystem (for ready feedback, nullable)
     * @param gamepad The operator gamepad (for haptic feedback, nullable)
     */
    public UniversalSmartShotCommand(LauncherSubsystem launcher,
                                      IntakeSubsystem intake,
                                      VisionSubsystemLimelight vision,
                                      DriveSubsystem drive,
                                      LightingSubsystem lighting,
                                      Gamepad gamepad) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = Objects.requireNonNull(intake, "intake required");
        this.vision = Objects.requireNonNull(vision, "vision required");
        this.drive = Objects.requireNonNull(drive, "drive required");
        this.lighting = lighting;  // Can be null
        this.gamepad = gamepad;    // Can be null
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        stage = Stage.CALCULATING_DISTANCE;
        delegateFireCommand = null;
        feedbackTriggered = false;
        spinDownApplied = false;

        // Initialize diagnostics
        diagnostics.currentStage = stage.name();
        diagnostics.lastSource = "none";
        diagnostics.hapticTriggered = false;

        // Calculate distance and set RPMs
        double distance = calculateDistanceToGoal();

        if (distance > 0.0) {
            // Set RPMs based on calculated distance
            setRpmsForDistance(distance);

            // Set hood positions based on distance
            setHoodForDistance(distance);

            // Activate prefeed roller
            if (intake != null) {
                intake.setGatePreventArtifact();
            }

            // Spin up to target
            launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);

            stage = Stage.SPINNING_UP;
        } else {
            // If we can't get distance, abort
            stage = Stage.COMPLETED;
        }
    }

    @Override
    public void update() {
        diagnostics.currentStage = stage.name();

        switch (stage) {
            case CALCULATING_DISTANCE:
                // Should not reach here - handled in start()
                stage = Stage.COMPLETED;
                break;

            case SPINNING_UP:
                // Check if launcher is ready and provide feedback
                checkAndTriggerReadyFeedback();

                // Once ready, create mode-aware fire command
                if (isReadyToFire()) {
                    createAndStartFireCommand();
                    stage = Stage.FIRING;
                }
                break;

            case WAITING_FOR_READY:
                // Legacy stage - not used in current flow
                if (isReadyToFire()) {
                    createAndStartFireCommand();
                    stage = Stage.FIRING;
                }
                break;

            case FIRING:
                // Update delegate command
                if (delegateFireCommand != null) {
                    delegateFireCommand.update();

                    // Check if firing is complete
                    if (delegateFireCommand.isDone()) {
                        stage = Stage.COMPLETED;
                        if (!spinDownApplied) {
                            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
                            spinDownApplied = true;
                        }
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
        // Stop delegate command if running
        if (delegateFireCommand != null) {
            delegateFireCommand.stop(interrupted);
        }

        // Deactivate prefeed roller
        if (intake != null) {
            intake.setGatePreventArtifact();
        }

        // Clear RPM overrides
        launcher.clearOverrides();

        // Spin down if not already done
        if (interrupted && !spinDownApplied) {
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

        // Fall back to odometry if vision unavailable
        if (!poseOpt.isPresent()) {
            Pose odometryPose = drive.getFollower().getPose();
            if (odometryPose != null) {
                poseOpt = Optional.of(odometryPose);
                diagnostics.lastSource = "odometry";
            } else {
                diagnostics.lastSource = "none";
                return 0.0;
            }
        } else {
            diagnostics.lastSource = "vision";
        }

        // Get goal pose based on alliance
        Optional<Pose> goalOpt = vision.getTargetGoalPose();

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

        // Update diagnostics
        diagnostics.lastCalculatedDistanceIn = distance;

        // Push to telemetry
        RobotState.packet.put("Universal Smart Shot/Distance (in)", distance);
        RobotState.packet.put("Universal Smart Shot/Data Source", diagnostics.lastSource);

        return distance;
    }

    /**
     * Sets RPM targets for all lanes based on distance using linear interpolation.
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

        // Push to telemetry
        RobotState.packet.put("Universal Smart Shot/Left RPM", leftRpm);
        RobotState.packet.put("Universal Smart Shot/Center RPM", centerRpm);
        RobotState.packet.put("Universal Smart Shot/Right RPM", rightRpm);
    }

    /**
     * Interpolates RPM based on distance using piecewise linear interpolation.
     */
    private double interpolateRpm(double distance,
                                   double shortDist, double shortRpm,
                                   double midDist, double midRpm,
                                   double longDist, double longRpm) {
        // Clamp distance to reasonable range
        distance = Range.clip(distance, shortDist * 0.5, longDist * 1.5);

        // Piecewise linear interpolation
        if (distance <= shortDist) {
            return shortRpm;
        } else if (distance <= midDist) {
            double t = (distance - shortDist) / (midDist - shortDist);
            return shortRpm + t * (midRpm - shortRpm);
        } else if (distance <= longDist) {
            double t = (distance - midDist) / (longDist - midDist);
            return midRpm + t * (longRpm - midRpm);
        } else {
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

        // Push to telemetry
        RobotState.packet.put("Universal Smart Shot/Hood Position", hoodPosition);
    }

    /**
     * Checks if launcher is ready and triggers haptic/light feedback.
     */
    private void checkAndTriggerReadyFeedback() {
        if (feedbackTriggered) {
            return;
        }

        // Check if any lane has reached 95% of target RPM
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
            diagnostics.hapticTriggered = true;

            // Haptic feedback
            if (gamepad != null) {
                gamepad.rumble(200);  // 200ms rumble
            }

            // Light feedback
            if (lighting != null) {
                lighting.flashAimAligned();
            }
        }
    }

    /**
     * Checks if launcher is ready to fire (at least one lane ready).
     */
    private boolean isReadyToFire() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (launcher.getLaunchRpm(lane) > 0.0 && launcher.isLaneReady(lane)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Creates and starts the appropriate fire command based on current launcher mode.
     */
    private void createAndStartFireCommand() {
        // Get current mode from RobotState
        LauncherMode mode = RobotState.getLauncherMode();
        if (mode == null) {
            mode = LauncherMode.THROUGHPUT; // Default to throughput if not set
        }

        diagnostics.launcherMode = mode.name();
        RobotState.packet.put("Universal Smart Shot/Mode", mode.name());

        // Create appropriate fire command based on mode
        switch (mode) {
            case DECODE:
                // DECODE mode: Fire in obelisk pattern sequence with motif tail offset
                delegateFireCommand = new LaunchInSequenceCommand(launcher, intake);
                break;

            case THROUGHPUT:
            default:
                // THROUGHPUT mode: Fire all lanes rapidly
                delegateFireCommand = new LaunchAllCommand(launcher, intake, true);
                break;
        }

        // Start the delegate command
        if (delegateFireCommand != null) {
            delegateFireCommand.start();
        }
    }
}
