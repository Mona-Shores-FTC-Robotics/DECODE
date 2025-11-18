package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import java.util.Objects;
import java.util.Optional;

/**
 * Smart fire command that automatically selects SHORT/MID/LONG range based on distance to goal.
 *
 * Uses AprilTag vision to measure distance, falls back to odometry if vision unavailable.
 * Selects discrete range (SHORT/MID/LONG) based on configurable distance thresholds,
 * then fires using the calibrated RPMs for that range.
 *
 * This simplifies operator controls - one button instead of three range buttons.
 */
@Configurable
public class FireAllAtAutoRangeCommand extends Command {

    @Configurable
    public static class RangeThresholds {
        /** Distance threshold (inches) - closer than this = SHORT range */
        public double shortMaxDistanceIn = 48.0;  // 4 feet
        /** Distance threshold (inches) - between short and this = MID range */
        public double midMaxDistanceIn = 90.0;   // 7.5 feet
        /** Distance beyond mid threshold = LONG range */
        // (anything > midMaxDistanceIn is LONG)
    }

    public static RangeThresholds thresholds = new RangeThresholds();

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final VisionSubsystemLimelight vision;
    private final DriveSubsystem drive;
    private final ManualSpinController manualSpinController;
    private final boolean spinDownAfterShot;

    private Command delegateCommand;
    private LauncherRange selectedRange;
    private double calculatedDistance;

    /**
     * Creates auto-range fire command that selects SHORT/MID/LONG based on distance.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (nullable)
     * @param vision The vision subsystem (for distance measurement)
     * @param drive The drive subsystem (for odometry fallback)
     * @param manualSpinController Controller for manual spin state
     * @param spinDownAfterShot Whether to spin down after firing
     */
    public FireAllAtAutoRangeCommand(LauncherSubsystem launcher,
                                      IntakeSubsystem intake,
                                      VisionSubsystemLimelight vision,
                                      DriveSubsystem drive,
                                      ManualSpinController manualSpinController,
                                      boolean spinDownAfterShot) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake;
        this.vision = Objects.requireNonNull(vision, "vision required");
        this.drive = Objects.requireNonNull(drive, "drive required");
        this.manualSpinController = Objects.requireNonNull(manualSpinController, "manualSpinController required");
        this.spinDownAfterShot = spinDownAfterShot;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        // Calculate distance and select range
        calculatedDistance = calculateDistanceToGoal();
        selectedRange = selectRangeForDistance(calculatedDistance);

        // Create delegate command for the selected range
        delegateCommand = new FireAllAtRangeCommand(
            launcher,
            intake,
            selectedRange,
            spinDownAfterShot,
            manualSpinController
        );

        // Start the delegate
        delegateCommand.start();
    }

    @Override
    public void update() {
        if (delegateCommand != null) {
            delegateCommand.update();
        }
    }

    @Override
    public boolean isDone() {
        return delegateCommand != null && delegateCommand.isDone();
    }

    @Override
    public void stop(boolean interrupted) {
        if (delegateCommand != null) {
            delegateCommand.stop(interrupted);
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
        Optional<Pose> poseOpt = vision.getRobotPoseFromTag();

        // Fall back to odometry if vision unavailable
        if (!poseOpt.isPresent()) {
            Pose odometryPose = drive.getFollower().getPose();
            if (odometryPose != null) {
                poseOpt = Optional.of(odometryPose);
            }
        }

        // Get goal pose based on alliance
        Optional<Pose> goalOpt = vision.getTargetGoalPose();

        if (!poseOpt.isPresent() || !goalOpt.isPresent()) {
            // Default to MID range if can't determine distance
            return thresholds.shortMaxDistanceIn + 10.0;  // Just above SHORT threshold
        }

        Pose robotPose = poseOpt.get();
        Pose goalPose = goalOpt.get();

        // Calculate Euclidean distance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Selects the appropriate range based on distance.
     *
     * @param distanceIn Distance to goal in inches
     * @return Selected range (SHORT, MID, or LONG)
     */
    private LauncherRange selectRangeForDistance(double distanceIn) {
        if (distanceIn <= thresholds.shortMaxDistanceIn) {
            return LauncherRange.SHORT;
        } else if (distanceIn <= thresholds.midMaxDistanceIn) {
            return LauncherRange.MID;
        } else {
            return LauncherRange.LONG;
        }
    }

    /**
     * Gets the calculated distance for telemetry/logging.
     */
    public double getCalculatedDistance() {
        return calculatedDistance;
    }

    /**
     * Gets the selected range for telemetry/logging.
     */
    public LauncherRange getSelectedRange() {
        return selectedRange;
    }
}
