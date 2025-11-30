package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;

import java.util.Objects;

/**
 * Lightweight helper that repeatedly requests a vision-based relocalization
 * while another command (like AimAtGoalCommand) is running as the deadline.
 */
public class TryRelocalizeForShotCommand extends Command {

    private final DriveSubsystem drive;
    private final VisionSubsystemLimelight vision;
    private final long timeoutMs;
    private long startTimeMs;

    public TryRelocalizeForShotCommand(DriveSubsystem drive, VisionSubsystemLimelight vision) {
        this(drive, vision, 500);
    }

    public TryRelocalizeForShotCommand(DriveSubsystem drive, VisionSubsystemLimelight vision, long timeoutMs) {
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        this.vision = Objects.requireNonNull(vision, "vision subsystem required");
        this.timeoutMs = timeoutMs;
        setInterruptible(true);
    }

    @Override
    public void start() {
        startTimeMs = System.currentTimeMillis();
    }

    @Override
    public void update() {
        if (vision.hasValidTag()) {
            drive.tryRelocalizeForShot();
        }
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis() - startTimeMs >= timeoutMs;
    }
}
