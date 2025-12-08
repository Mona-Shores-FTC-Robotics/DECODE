package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command that runs the eject sequence (aggressive reverse) for a specified duration.
 * Stops the intake when complete.
 */
public class TimedEjectCommand extends IntakeCommand {

    private static final double DEFAULT_DURATION_MS = 400.0;

    private final double durationMs;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;

    /**
     * Creates a timed eject command with the default duration (400ms).
     *
     * @param intake the intake subsystem
     */
    public TimedEjectCommand(IntakeSubsystem intake) {
        this(intake, DEFAULT_DURATION_MS);
    }

    /**
     * Creates a timed eject command with a custom duration.
     *
     * @param intake     the intake subsystem
     * @param durationMs how long to run the eject in milliseconds
     */
    public TimedEjectCommand(IntakeSubsystem intake, double durationMs) {
        super(intake);
        this.durationMs = durationMs;
    }

    @Override
    public void start() {
        getIntake().setMode(IntakeSubsystem.IntakeMode.AGGRESSIVE_REVERSE);
        timer.reset();
        started = true;
    }

    @Override
    public boolean isDone() {
        return started && timer.milliseconds() >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        getIntake().setMode(IntakeSubsystem.IntakeMode.STOPPED);
    }
}
