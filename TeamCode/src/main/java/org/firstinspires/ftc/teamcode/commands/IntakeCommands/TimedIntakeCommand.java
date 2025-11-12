package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake forward for a fixed duration, then returns to passive reverse.
 */
public class TimedIntakeCommand extends IntakeCommand {

    private final ElapsedTime timer = new ElapsedTime();
    private final double durationSeconds;

    public TimedIntakeCommand(IntakeSubsystem intake, double durationSeconds) {
        super(intake);
        this.durationSeconds = durationSeconds;
    }

    @Override
    public void start() {
        getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        timer.reset();
    }

    @Override
    public boolean isDone() {
        return timer.seconds() >= durationSeconds;
    }

    @Override
    public void stop(boolean interrupted) {
        getIntake().setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
    }
}
