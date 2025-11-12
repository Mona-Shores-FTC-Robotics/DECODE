package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.TurnTo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

import java.util.Objects;
import java.util.Optional;

/**
 * Simplified capture-and-aim command using NextFTC's Pedro TurnTo extension.
 * Similar to AimAndDrive but uses a discrete turn command instead of continuous tracking.
 *
 * When activated:
 * 1. Determines target heading from AprilTag vision or odometry fallback
 * 2. Respects alliance color to point at the correct goal (Red/Blue)
 * 3. Uses TurnTo command to execute the rotation with Pedro's PIDF control
 *
 * This approach is simpler than the Path-based CaptureAndAimCommand and leverages
 * NextFTC's built-in turn commands for cleaner code.
 */
@Configurable
public class CaptureAndAim2Command extends Command {

    @Configurable
    public static class Config {
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
         * Maximum time for sampling phase before giving up (milliseconds).
         */
        public static double samplingTimeoutMs = 500.0;
    }

    private final DriveSubsystem drive;
    private final VisionSubsystemLimelight vision;

    private Double capturedTargetHeadingRad = null;
    private Command turnCommand = null;
    private long startTimeMs = -1L;
    private int framesSampled = 0;
    private double summedCos = 0.0;
    private double summedSin = 0.0;
    private long lastSampleMs = -1L;

    /**
     * Creates a capture-and-aim command using NextFTC's TurnTo.
     */
    public CaptureAndAim2Command(DriveSubsystem drive, VisionSubsystemLimelight vision) {
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        this.vision = Objects.requireNonNull(vision, "vision subsystem required");
        requires(this.drive);
        setInterruptible(true);
    }

    @Override
    public void start() {
        capturedTargetHeadingRad = null;
        turnCommand = null;
        startTimeMs = System.currentTimeMillis();
        framesSampled = 0;
        summedCos = 0.0;
        summedSin = 0.0;
        lastSampleMs = -1L;

        // Start sampling immediately
        sampleTargetHeading();
    }

    @Override
    public void update() {
        // Phase 1: Sample target heading
        if (capturedTargetHeadingRad == null) {
            long nowMs = System.currentTimeMillis();

            // Check sampling timeout
            if (nowMs - startTimeMs >= Config.samplingTimeoutMs) {
                // Timeout during sampling - use whatever we've collected so far
                if (framesSampled > 0) {
                    capturedTargetHeadingRad = Math.atan2(summedSin / framesSampled, summedCos / framesSampled);
                }
                return;
            }

            // Continue sampling if needed
            if (framesSampled < Config.sampleFrames
                    && nowMs - lastSampleMs >= Config.frameSampleIntervalMs) {
                sampleTargetHeading();
            }

            // Once we have enough samples, compute the circular mean
            if (framesSampled >= Config.sampleFrames) {
                // Use circular mean instead of arithmetic mean to handle angle wrapping correctly.
                // Arithmetic mean fails for angles near 0°/360° boundary (e.g., mean of [1°, 359°] = 180°).
                // Circular mean: compute mean of sin/cos components, then use atan2 to recover the angle.
                // This properly handles wraparound and gives the expected result for all angle pairs.
                capturedTargetHeadingRad = Math.atan2(summedSin / framesSampled, summedCos / framesSampled);
            }

            // Don't proceed until we've captured the target
            return;
        }

        // Phase 2: Execute turn using NextFTC's TurnTo command
        if (turnCommand == null) {
            // Create and schedule the TurnTo command with the captured heading
            turnCommand = new TurnTo(drive.getFollower(), capturedTargetHeadingRad, AngleUnit.RADIANS);
            turnCommand.schedule();
        }
    }

    @Override
    public boolean isDone() {
        // If sampling timed out without capturing any frames, we're done (failed)
        if (capturedTargetHeadingRad == null
                && framesSampled == 0
                && System.currentTimeMillis() - startTimeMs >= Config.samplingTimeoutMs) {
            return true;
        }

        // If we haven't started turning yet, not done
        if (turnCommand == null) {
            return false;
        }

        // Done when the TurnTo command finishes
        return turnCommand.isFinished();
    }

    @Override
    public void stop(boolean interrupted) {
        if (turnCommand != null && !turnCommand.isFinished()) {
            turnCommand.cancel();
        }
        capturedTargetHeadingRad = null;
        turnCommand = null;
        startTimeMs = -1L;
        drive.stop();
    }

    /**
     * Samples the target heading and accumulates sin/cos components for circular mean.
     */
    private void sampleTargetHeading() {
        double targetHeading = computeTargetHeading();
        // Accumulate sin and cos components for circular mean calculation
        summedCos += Math.cos(targetHeading);
        summedSin += Math.sin(targetHeading);
        framesSampled++;
        lastSampleMs = System.currentTimeMillis();
    }

    /**
     * Computes the target heading from vision or odometry fallback.
     * Respects alliance color to aim at the correct goal (Red or Blue).
     */
    private double computeTargetHeading() {
        // First, try to get aim angle from vision
        // Vision system already handles alliance-specific goal selection
        Optional<Double> visionAngle = vision.getAimAngle();
        if (visionAngle.isPresent()) {
            return visionAngle.get();
        }

        // Fallback: calculate from odometry + alliance-specific goal position
        Pose currentPose = drive.getFollowerPose();
        if (currentPose == null) {
            // No pose available, default to current heading
            return drive.getFollower().getHeading();
        }

        // Get alliance-specific goal pose
        Pose targetPose = vision.getTargetGoalPose().orElse(FieldConstants.BLUE_GOAL_TAG);
        return FieldConstants.getAimAngleTo(currentPose, targetPose);
    }

    /**
     * Gets the captured target heading in radians (null if not yet captured).
     */
    public Double getCapturedTargetHeading() {
        return capturedTargetHeadingRad;
    }
}
