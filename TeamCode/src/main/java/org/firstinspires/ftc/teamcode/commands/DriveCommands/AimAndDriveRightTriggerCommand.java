package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Right trigger fixed-angle aiming command that rotates to a predetermined heading based on alliance.
 * Uses different target angles than triangle button (265° blue / 275° red).
 * Driver retains full translation control (left stick) while holding right trigger.
 * Slow mode (right bumper) remains functional.
 *
 * INTENDED USAGE:
 * - Hold right trigger to maintain fixed heading while driving
 * - Driver controls forward/backward/strafe with left stick
 * - Slow mode button (right bumper) still works
 * - Release right trigger to regain manual rotation control
 *
 * Contrast with triangle button:
 * - Triangle: 135.2° blue / 45.2° red (aiming at basket from launch line)
 * - Right trigger: 265° blue / 275° red (configurable via Dashboard)
 */
public class AimAndDriveRightTriggerCommand extends Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier fieldXSupplier;
    private final DoubleSupplier fieldYSupplier;
    private final BooleanSupplier slowModeSupplier;
    private static final double TRANSLATION_IDLE_THRESHOLD = 0.05;

    /**
     * Creates a right trigger fixed-angle aim-and-drive command.
     *
     * @param fieldXSupplier Supplier for field X input (strafe, usually left stick X)
     * @param fieldYSupplier Supplier for field Y input (forward, usually left stick Y)
     * @param slowModeSupplier Supplier for slow mode (usually right bumper)
     * @param drive Drive subsystem
     */
    public AimAndDriveRightTriggerCommand(DoubleSupplier fieldXSupplier,
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

        // Right trigger fixed-angle aim and drive - rotation to fixed heading based on alliance
        drive.aimAndDriveRightTriggerFixedAngle(fieldX, fieldY, slowMode);

    }

    @Override
    public boolean isDone() {
        // Command runs while right trigger is held, never finishes on its own
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
