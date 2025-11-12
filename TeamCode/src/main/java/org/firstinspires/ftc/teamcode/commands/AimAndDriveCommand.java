package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command that continuously aims at the goal while allowing driver translation.
 * This uses DriveSubsystem.aimAndDrive() which:
 * - Overrides rotation to aim at vision target (or odometry-based fallback)
 * - Continuously tracks target (updates every loop)
 * - Allows driver to translate (left stick) while aiming
 * - Falls back to odometry + known goal position if no vision
 *
 * INTENDED USAGE:
 * - **TeleOp match play**: B button (hold) - primary aiming method for drivers
 * - Allows driver to strafe/move while staying aimed at goal
 * - No control lockout - driver can always translate
 *
 * Contrast with CaptureAndAimCommand:
 * - This: Continuous tracking, driver can translate
 * - Capture: One-time snap, driver locked out, better for autonomous
 */
public class AimAndDriveCommand extends Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier fieldXSupplier;
    private final DoubleSupplier fieldYSupplier;
    private final BooleanSupplier slowModeSupplier;
    private static final double TRANSLATION_IDLE_THRESHOLD = 0.05;

    /**
     * Creates an aim-and-drive command.
     *
     * @param fieldXSupplier Supplier for field X input (strafe, usually left stick X)
     * @param fieldYSupplier Supplier for field Y input (forward, usually left stick Y)
     * @param slowModeSupplier Supplier for slow mode (usually right bumper)
     * @param drive Drive subsystem
     */
    public AimAndDriveCommand(DoubleSupplier fieldXSupplier,
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

        // Aim and drive - rotation is overridden by vision/odometry targeting
        drive.aimAndDrive(fieldX, fieldY, slowMode);
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
