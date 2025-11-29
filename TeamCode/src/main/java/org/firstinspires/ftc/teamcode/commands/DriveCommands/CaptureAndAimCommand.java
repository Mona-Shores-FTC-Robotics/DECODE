package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.config.CaptureAndAimConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

import java.util.Objects;

/**
 * Simplified capture-and-aim command using Pedro Follower's turnTo() method.
 * Similar to AimAndDrive but uses a discrete turn command instead of continuous tracking.
 *
 * When activated:
 * 1. Determines target heading from AprilTag vision or odometry fallback
 * 2. Respects alliance color to point at the correct goal (Red/Blue)
 * 3. Uses follower.turnTo() to execute the rotation with Pedro's PIDF control
 *
 * This approach is simpler than the Path-based CaptureAndAimCommand and directly
 * uses Pedro Pathing's built-in turn functionality for cleaner code.
 */
@Configurable
public class CaptureAndAimCommand extends Command {

    public static CaptureAndAimConfig config = new CaptureAndAimConfig();

    private final DriveSubsystem drive;
    private final VisionSubsystemLimelight vision;

    private Double capturedTargetHeadingRad = null;
    private boolean breakRequested = false;
    private boolean turnStarted = false;
    private boolean alreadyAimed = false;
    private long startTimeMs = -1L;
    private int framesSampled = 0;
    private double summedCos = 0.0;
    private double summedSin = 0.0;
    private long lastSampleMs = -1L;

    /**
     * Creates a capture-and-aim command using Pedro Follower's turnTo() method.
     */
    public CaptureAndAimCommand(DriveSubsystem drive, VisionSubsystemLimelight vision) {
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        this.vision = Objects.requireNonNull(vision, "vision subsystem required");
        requires(this.drive);
        setInterruptible(true);
    }

    @Override
    public void start() {
        // Validate configuration to prevent division by zero
        if (config.sampleFrames < 1) {
            throw new IllegalStateException(
                "CaptureAndAimCommand: config.sampleFrames must be >= 1, got " + config.sampleFrames);
        }

        capturedTargetHeadingRad = null;
        breakRequested = false;
        turnStarted = false;
        alreadyAimed = false;
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
            if (nowMs - startTimeMs >= config.samplingTimeoutMs) {
                // Timeout during sampling - use whatever we've collected so far
                // Defensive check: only compute mean if we have at least one valid sample
                if (framesSampled > 0) {
                    capturedTargetHeadingRad = Math.atan2(summedSin / framesSampled, summedCos / framesSampled);
                }
                return;
            }

            // Continue sampling if needed
            if (framesSampled < config.sampleFrames
                    && nowMs - lastSampleMs >= config.frameSampleIntervalMs) {
                sampleTargetHeading();
            }

            // Once we have enough samples, compute the circular mean
            // Defensive check: ensure we have at least one sample before dividing
            if (framesSampled >= config.sampleFrames && framesSampled > 0) {
                // Use circular mean instead of arithmetic mean to handle angle wrapping correctly.
                // Arithmetic mean fails for angles near 0°/360° boundary (e.g., mean of [1°, 359°] = 180°).
                // Circular mean: compute mean of sin/cos components, then use atan2 to recover the angle.
                // This properly handles wraparound and gives the expected result for all angle pairs.
                capturedTargetHeadingRad = Math.atan2(summedSin / framesSampled, summedCos / framesSampled);
            }

            // Don't proceed until we've captured the target
            return;
        }

        // Phase 2: Execute turn using Pedro Follower directly
        if (!turnStarted && !alreadyAimed) {
            // Check if we're already aimed within tolerance - if so, skip the turn
            double currentHeading = drive.getFollower().getHeading();
            double headingError = normalizeAngle(capturedTargetHeadingRad - currentHeading);
            double headingErrorDeg = Math.toDegrees(Math.abs(headingError));

            if (headingErrorDeg <= config.headingToleranceDeg) {
                // Already aimed - no turn needed
                alreadyAimed = true;
                return;
            }

            // If follower is still busy from previous command, break it first
            if (drive.getFollower().isBusy()) {
                if (!breakRequested) {
                    drive.getFollower().breakFollowing();
                    breakRequested = true;
                }
                // Wait for follower to process the break (need one update cycle)
                return;
            }

            // Follower is now ready - command the turn using autonomous PIDF control
            drive.getFollower().turnTo(capturedTargetHeadingRad);
            turnStarted = true;
        }
    }

    @Override
    public boolean isDone() {
        // If sampling timed out without capturing any frames, we're done (failed)
        if (capturedTargetHeadingRad == null
                && framesSampled == 0
                && System.currentTimeMillis() - startTimeMs >= config.samplingTimeoutMs) {
            return true;
        }

        // If already aimed within tolerance, we're done (no turn needed)
        if (alreadyAimed) {
            return true;
        }

        // If we haven't started turning yet, not done
        if (!turnStarted) {
            return false;
        }

        // Done when the follower finishes the turn (isBusy returns false when complete)
        return !drive.getFollower().isBusy();
    }

    @Override
    public void stop(boolean interrupted) {
        capturedTargetHeadingRad = null;
        breakRequested = false;
        turnStarted = false;
        alreadyAimed = false;
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
     * Computes the target heading using pinpoint odometry.
     * Respects alliance color to aim at the correct goal (Red or Blue).
     */
    private double computeTargetHeading() {
        // Calculate from odometry + alliance-specific goal position
        Pose currentPose = drive.getFollowerPose();
        if (currentPose == null) {
            // No pose available, default to current heading
            return drive.getFollower().getHeading();
        }

        // Get alliance-specific goal pose
        Alliance alliance = vision.getAlliance();
        Pose targetPose = (alliance == Alliance.RED)
            ? FieldConstants.getRedBasketTarget()
            : FieldConstants.getBlueBasketTarget();
        return FieldConstants.getAimAngleTo(currentPose, targetPose);
    }

    /**
     * Gets the captured target heading in radians (null if not yet captured).
     */
    public Double getCapturedTargetHeading() {
        return capturedTargetHeadingRad;
    }

    /**
     * Normalizes an angle to [-π, π] range.
     */
    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
