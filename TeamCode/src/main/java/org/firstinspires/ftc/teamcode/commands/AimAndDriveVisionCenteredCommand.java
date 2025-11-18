package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Vision-centered aiming command that uses Limelight tx (horizontal offset) to center the AprilTag.
 * This approach directly uses the camera's measurement without coordinate calculations,
 * similar to the original FTC RobotAutoDriveToAprilTagOmni example.
 *
 * INTENDED USAGE:
 * - Alternative aiming method for testing and comparison
 * - Uses direct camera feedback to center the target
 * - No dependency on odometry pose or field coordinates
 *
 * Contrast with AimAndDriveCommand (geometry-based):
 * - This: Uses tx angle from camera, centers tag in view
 * - Geometry: Uses atan2 from robot pose to basket centroid
 */
public class AimAndDriveVisionCenteredCommand extends Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier fieldXSupplier;
    private final DoubleSupplier fieldYSupplier;
    private final BooleanSupplier slowModeSupplier;
    private static final double TRANSLATION_IDLE_THRESHOLD = 0.05;

    /**
     * Creates a vision-centered aim-and-drive command.
     *
     * @param fieldXSupplier Supplier for field X input (strafe, usually left stick X)
     * @param fieldYSupplier Supplier for field Y input (forward, usually left stick Y)
     * @param slowModeSupplier Supplier for slow mode (usually right bumper)
     * @param drive Drive subsystem
     */
    public AimAndDriveVisionCenteredCommand(DoubleSupplier fieldXSupplier,
                                            DoubleSupplier fieldYSupplier,
                                            BooleanSupplier slowModeSupplier,
                                            DriveSubsystem drive) {
        this.fieldXSupplier = Objects.requireNonNull(fieldXSupplier, "fieldXSupplier required");
        this.fieldYSupplier = Objects.requireNonNull(fieldYSupplier, "fieldYSupplier required");
        this.slowModeSupplier = Objects.requireNonNull(slowModeSupplier, "slowModeSupplier required");
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        requires(this.drive);
        setInterruptible(true);
    }

    @Override
    public void start() {
        // No initialization needed
    }

    @Override
    public void update() {
        // Sample driver inputs
        double fieldX = fieldXSupplier.getAsDouble();
        double fieldY = fieldYSupplier.getAsDouble();
        boolean slowMode = slowModeSupplier.getAsBoolean();

        fieldX = applyTranslationDeadband(fieldX);
        fieldY = applyTranslationDeadband(fieldY);

        // Vision-centered aim and drive - rotation uses Limelight tx to center tag
        drive.aimAndDriveVisionCentered(fieldX, fieldY, slowMode);
    }

    @Override
    public boolean isDone() {
        // Command runs while button is held, never finishes on its own
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        // Don't stop motors - let default command resume smoothly
    }

    private static double applyTranslationDeadband(double value) {
        return Math.abs(value) < TRANSLATION_IDLE_THRESHOLD ? 0.0 : value;
    }
}
