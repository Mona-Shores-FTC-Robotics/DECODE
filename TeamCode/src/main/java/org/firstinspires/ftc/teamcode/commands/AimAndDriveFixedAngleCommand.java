package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Fixed-angle aiming command that rotates to a predetermined heading based on alliance.
 * Simple and predictable - works well from specific field positions (e.g., launch line).
 *
 * INTENDED USAGE:
 * - Simple aiming from known field positions (e.g., launch line)
 * - No dependency on vision or precise pose
 * - Driver just needs to be in the right general area
 * - Tunable angles via FTC Dashboard (e.g., 60° blue, 120° red)
 *
 * Contrast with other aiming methods:
 * - Geometry: Uses pose + coordinates (complex, requires accurate odometry)
 * - Vision-centered: Uses camera tx (requires tag visibility)
 * - This: Simple fixed angle (no vision or pose needed)
 */
public class AimAndDriveFixedAngleCommand extends Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier fieldXSupplier;
    private final DoubleSupplier fieldYSupplier;
    private final BooleanSupplier slowModeSupplier;
    private static final double TRANSLATION_IDLE_THRESHOLD = 0.05;

    /**
     * Creates a fixed-angle aim-and-drive command.
     *
     * @param fieldXSupplier Supplier for field X input (strafe, usually left stick X)
     * @param fieldYSupplier Supplier for field Y input (forward, usually left stick Y)
     * @param slowModeSupplier Supplier for slow mode (usually right bumper)
     * @param drive Drive subsystem
     */
    public AimAndDriveFixedAngleCommand(DoubleSupplier fieldXSupplier,
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

        // Fixed-angle aim and drive - rotation to fixed heading based on alliance
        drive.aimAndDriveFixedAngle(fieldX, fieldY, slowMode);
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
