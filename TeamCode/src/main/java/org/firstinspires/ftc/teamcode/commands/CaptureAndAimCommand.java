package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

import java.util.Objects;
import java.util.Optional;

/**
 * Command that captures the target aim angle ONCE at start, then turns to that fixed heading.
 * Unlike continuous tracking (aimAndDrive), this command:
 * 1. Samples the target angle at the beginning (from vision or odometry)
 * 2. Optionally averages over multiple frames for robustness
 * 3. Uses Pedro Pathing's Follower to execute the turn with full PIDF control
 * 4. Assumes robot is stationary during the turn
 *
 * INTENDED USAGE:
 * - **Autonomous**: Primary use case - predictable, repeatable turns with no driver
 * - **TeleOp testing**: X button binding for bench testing and parameter tuning
 * - **TeleOp match play**: NOT recommended - use B button (aimAndDrive) instead
 *
 * IMPORTANT - Driver Control Lockout:
 * During execution, Pedro's Follower takes full control of drive motors. Driver translation
 * inputs (left stick) are ignored until the turn completes. This is fine in autonomous but
 * can feel "locked up" in TeleOp. For match play, use DriveSubsystem.aimAndDrive() instead,
 * which allows driver translation during aiming.
 *
 * Benefits of using Pedro's Follower:
 * - Leverages Pedro Pathing's tuned heading PIDF (both primary and secondary)
 * - Consistent behavior with autonomous path following
 * - Tune once in Pedro's Constants, used everywhere
 */
@Configurable
public class CaptureAndAimCommand extends Command {

    /**
     * Configuration for capture-and-aim command behavior.
     * Tunable via FTC Dashboard at runtime.
     */
    @Configurable
    public static class CaptureAimConfig {
        /**
         * Number of frames to average for robustness (1 = no averaging).
         * Averaging multiple vision samples helps filter out single-frame noise.
         */
        public static int sampleFrames = 3;

        /**
         * Time between frame samples (milliseconds).
         * Spreading samples over time improves robustness to transient vision errors.
         */
        public static double frameSampleIntervalMs = 50.0;

        /**
         * Maximum execution time before giving up (milliseconds).
         * Includes both sampling time and turn execution time.
         */
        public static double timeoutMs = 2000.0;
    }

    private final DriveSubsystem drive;
    private final VisionSubsystemLimelight vision;
    private final Follower follower;
    private final long timeoutMs;
    private final int sampleFrames;

    private Double capturedTargetHeadingRad = null;
    private Path turnPath = null;
    private long startTimeMs = -1L;
    private int framesSampled = 0;
    private double summedHeadings = 0.0;
    private long lastSampleMs = -1L;

    /**
     * Creates a capture-and-aim command with default parameters from CaptureAimConfig.
     */
    public CaptureAndAimCommand(DriveSubsystem drive, VisionSubsystemLimelight vision) {
        this(drive, vision, (long) CaptureAimConfig.timeoutMs, CaptureAimConfig.sampleFrames);
    }

    /**
     * Creates a capture-and-aim command with custom parameters.
     *
     * @param drive Drive subsystem
     * @param vision Vision subsystem
     * @param timeoutMs Maximum execution time before giving up
     * @param sampleFrames Number of frames to average for robustness (1 = no averaging)
     */
    public CaptureAndAimCommand(DriveSubsystem drive,
                                VisionSubsystemLimelight vision,
                                long timeoutMs,
                                int sampleFrames) {
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        this.vision = Objects.requireNonNull(vision, "vision subsystem required");
        this.follower = drive.getFollower();
        this.timeoutMs = Math.max(0L, timeoutMs);
        this.sampleFrames = Math.max(1, sampleFrames);
        requires(this.drive);
        setInterruptible(true);
    }

    @Override
    public void start() {
        capturedTargetHeadingRad = null;
        turnPath = null;
        startTimeMs = System.currentTimeMillis();
        framesSampled = 0;
        summedHeadings = 0.0;
        lastSampleMs = -1L;

        // Start sampling immediately
        sampleTargetHeading();
    }

    @Override
    public void update() {
        // Phase 1: Sample target heading
        if (capturedTargetHeadingRad == null) {
            long nowMs = System.currentTimeMillis();
            if (framesSampled < sampleFrames && nowMs - lastSampleMs >= CaptureAimConfig.frameSampleIntervalMs) {
                sampleTargetHeading();
            }

            // Once we have enough samples, compute the average
            if (framesSampled >= sampleFrames) {
                capturedTargetHeadingRad = normalizeAngle(summedHeadings / sampleFrames);
            }

            // Don't proceed until we've captured the target
            return;
        }

        // Phase 2: Execute turn using Pedro's Follower
        if (turnPath == null) {
            // Create a zero-length path from current pose to same position with target heading
            // This is the same pattern used in autonomous for pure rotations
            Pose currentPose = follower.getPose();
            Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), capturedTargetHeadingRad);

            // Use BezierLine for instant heading interpolation (no translation)
            turnPath = new Path(new BezierLine(currentPose, targetPose));
            turnPath.setConstantHeadingInterpolation(capturedTargetHeadingRad);
            PathConstraints turnConstraints = new PathConstraints(
                    0.0,
                    PathConstraints.defaultConstraints.getVelocityConstraint(),
                    PathConstraints.defaultConstraints.getTranslationalConstraint(),
                    PathConstraints.defaultConstraints.getHeadingConstraint(),
                    PathConstraints.defaultConstraints.getTimeoutConstraint(),
                    PathConstraints.defaultConstraints.getBrakingStrength(),
                    PathConstraints.defaultConstraints.getBEZIER_CURVE_SEARCH_LIMIT(),
                    PathConstraints.defaultConstraints.getBrakingStart()
            );
            turnPath.setConstraints(turnConstraints);

            // Start following the path - Pedro's PIDF takes over
            follower.followPath(turnPath, true);  // holdPositionAtEnd = true
        }

        // Pedro's follower.update() is called by DriveSubsystem.periodic()
        // which runs automatically via SubsystemComponent
    }

    @Override
    public boolean isDone() {
        // Timeout check
        if (startTimeMs >= 0L && System.currentTimeMillis() - startTimeMs >= timeoutMs) {
            return true;
        }

        // If we haven't started turning yet, not done
        if (turnPath == null) {
            return false;
        }

        // Done when Follower finishes the path
        // isBusy() returns false when path is complete and robot is settled
        return !follower.isBusy();
    }

    @Override
    public void stop(boolean interrupted) {
        capturedTargetHeadingRad = null;
        turnPath = null;
        startTimeMs = -1L;
        drive.stop();
    }

    /**
     * Samples the target heading and accumulates it for averaging.
     */
    private void sampleTargetHeading() {
        double targetHeading = computeTargetHeading();
        summedHeadings += targetHeading;
        framesSampled++;
        lastSampleMs = System.currentTimeMillis();
    }

    /**
     * Computes the target heading from vision or odometry fallback.
     * This logic matches DriveSubsystem.aimAndDrive() fallback behavior.
     */
    private double computeTargetHeading() {
        // First, try to get aim angle from vision
        Optional<Double> visionAngle = vision.getAimAngle();
        if (visionAngle.isPresent()) {
            return visionAngle.get();
        }

        // Fallback: calculate from odometry + known goal position
        Pose currentPose = follower.getPose();
        Pose targetPose = vision.getTargetGoalPose().orElse(FieldConstants.BLUE_GOAL_TAG);
        return FieldConstants.getAimAngleTo(currentPose, targetPose);
    }

    /**
     * Normalizes angle to [-π, π] range.
     */
    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Gets the captured target heading in radians (null if not yet captured).
     */
    public Double getCapturedTargetHeading() {
        return capturedTargetHeadingRad;
    }
}
