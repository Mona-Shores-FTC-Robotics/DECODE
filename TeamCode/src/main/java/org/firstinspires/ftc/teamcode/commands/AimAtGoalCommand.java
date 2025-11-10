package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command that keeps the drive aimed at the alliance goal using {@link DriveSubsystem#aimAndDrive}.
 * Translation requests are passed straight through so the driver or calling routine can continue
 * to command forward/strafe motion while the command manages heading.
 */
public class AimAtGoalCommand extends Command {

    private static final double DEFAULT_HEADING_TOLERANCE_DEG = 1.5;
    private static final long DEFAULT_SETTLE_TIME_MS = 120L;

    private final DriveSubsystem drive;
    private final DoubleSupplier fieldXSupplier;
    private final DoubleSupplier fieldYSupplier;
    private final BooleanSupplier slowModeSupplier;
    private static final long DEFAULT_MAX_DURATION_MS = 0L;

    private final double headingToleranceRad;
    private final long settleTimeMs;
    private final boolean requireStationary;
    private final boolean finishWhenAligned;
    private final long maxDurationMs;

    private long alignedSinceMs = -1L;
    private long startTimeMs = -1L;

    public AimAtGoalCommand(DriveSubsystem drive,
                            DoubleSupplier fieldXSupplier,
                            DoubleSupplier fieldYSupplier,
                            BooleanSupplier slowModeSupplier) {
        this(drive,
                fieldXSupplier,
                fieldYSupplier,
                slowModeSupplier,
                DEFAULT_HEADING_TOLERANCE_DEG,
                DEFAULT_SETTLE_TIME_MS,
                true,
                true,
                DEFAULT_MAX_DURATION_MS);
    }

    public AimAtGoalCommand(DriveSubsystem drive,
                            DoubleSupplier fieldXSupplier,
                            DoubleSupplier fieldYSupplier,
                            BooleanSupplier slowModeSupplier,
                            double headingToleranceDeg,
                            long settleTimeMs,
                            boolean requireStationary,
                            boolean finishWhenAligned,
                            long timeoutMs) {
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        this.fieldXSupplier = fieldXSupplier != null ? fieldXSupplier : () -> 0.0;
        this.fieldYSupplier = fieldYSupplier != null ? fieldYSupplier : () -> 0.0;
        this.slowModeSupplier = slowModeSupplier != null ? slowModeSupplier : () -> false;
        this.headingToleranceRad = Math.toRadians(Math.max(0.0, headingToleranceDeg));
        this.settleTimeMs = Math.max(0L, settleTimeMs);
        this.requireStationary = requireStationary;
        this.finishWhenAligned = finishWhenAligned;
        this.maxDurationMs = Math.max(0L, timeoutMs);
        requires(this.drive);
        setInterruptible(true);
    }

    @Override
    public void start() {
        alignedSinceMs = -1L;
        startTimeMs = System.currentTimeMillis();
        drive.aimAndDrive(fieldXSupplier.getAsDouble(),
                fieldYSupplier.getAsDouble(),
                slowModeSupplier.getAsBoolean());
        if (finishWhenAligned) {
            updateAlignmentWindow();
        }
    }

    @Override
    public void update() {
        drive.aimAndDrive(fieldXSupplier.getAsDouble(),
                fieldYSupplier.getAsDouble(),
                slowModeSupplier.getAsBoolean());
        if (finishWhenAligned) {
            updateAlignmentWindow();
        }
    }

    private void updateAlignmentWindow() {
        double headingError = drive.getLastAimErrorRadians();
        boolean hasValidError = Double.isFinite(headingError);
        boolean withinTolerance = hasValidError && Math.abs(headingError) <= headingToleranceRad;
        boolean stationaryOk = !requireStationary || drive.isRobotStationary();
        if (withinTolerance && stationaryOk) {
            if (alignedSinceMs < 0L) {
                alignedSinceMs = System.currentTimeMillis();
            }
        } else {
            alignedSinceMs = -1L;
        }
    }

    @Override
    public boolean isDone() {
        if (maxDurationMs > 0 && startTimeMs >= 0L) {
            if (System.currentTimeMillis() - startTimeMs >= maxDurationMs) {
                return true;
            }
        }
        if (!finishWhenAligned) {
            return false;
        }
        if (alignedSinceMs < 0L) {
            return false;
        }
        return System.currentTimeMillis() - alignedSinceMs >= settleTimeMs;
    }

    @Override
    public void stop(boolean interrupted) {
        alignedSinceMs = -1L;
        startTimeMs = -1L;
        drive.stop();
    }

    public double getHeadingToleranceDegrees() {
        return Math.toDegrees(headingToleranceRad);
    }

    public boolean finishesWhenAligned() {
        return finishWhenAligned;
    }
}
