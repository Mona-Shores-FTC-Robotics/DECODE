package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Default drive command for TeleOp. Reads driver inputs and passes them to DriveSubsystem.
 * This command runs whenever no other command is using the drive subsystem.
 *
 * Handles:
 * - Field-centric drive (default) or robot-centric via configuration
 * - Slow mode (right bumper)
 * - Ramp mode (left bumper) for smooth acceleration
 * - Rotation override via right stick
 */
public class DefaultDriveCommand extends Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier fieldXSupplier;
    private final DoubleSupplier fieldYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier slowModeSupplier;
    private final BooleanSupplier rampModeSupplier;

    /**
     * Creates a default drive command.
     *
     * @param fieldXSupplier Supplier for field X input (strafe, usually left stick X)
     * @param fieldYSupplier Supplier for field Y input (forward, usually left stick Y)
     * @param rotationSupplier Supplier for rotation input (turn, usually right stick X)
     * @param slowModeSupplier Supplier for slow mode (usually right bumper)
     * @param rampModeSupplier Supplier for ramp mode (usually left bumper)
     * @param drive Drive subsystem
     */
    public DefaultDriveCommand(DoubleSupplier fieldXSupplier,
                               DoubleSupplier fieldYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier slowModeSupplier,
                               BooleanSupplier rampModeSupplier,
                               DriveSubsystem drive) {
        this.fieldXSupplier = Objects.requireNonNull(fieldXSupplier, "fieldXSupplier required");
        this.fieldYSupplier = Objects.requireNonNull(fieldYSupplier, "fieldYSupplier required");
        this.rotationSupplier = Objects.requireNonNull(rotationSupplier, "rotationSupplier required");
        this.slowModeSupplier = Objects.requireNonNull(slowModeSupplier, "slowModeSupplier required");
        this.rampModeSupplier = Objects.requireNonNull(rampModeSupplier, "rampModeSupplier required");
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
        double rotation = rotationSupplier.getAsDouble();
        boolean slowMode = slowModeSupplier.getAsBoolean();
        boolean rampMode = rampModeSupplier.getAsBoolean();

        // Pass to drive subsystem
        drive.driveTeleOp(fieldX, fieldY, rotation, slowMode, rampMode);
    }

    @Override
    public boolean isDone() {
        // Default command never finishes on its own
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        // Don't stop motors when interrupted - let the new command take over
        // This allows smooth transitions between default drive and aim commands
    }
}