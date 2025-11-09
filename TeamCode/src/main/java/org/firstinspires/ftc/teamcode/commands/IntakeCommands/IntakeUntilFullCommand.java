package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake until all lanes are filled (or until a timeout expires).
 */
public class IntakeUntilFullCommand extends IntakeCommand {

    private final ElapsedTime timer = new ElapsedTime();
    private final double timeoutSeconds;

    public IntakeUntilFullCommand(IntakeSubsystem intake, double timeoutSeconds) {
        super(intake);
        this.timeoutSeconds = timeoutSeconds;
    }

    @Override
    public void start() {
        getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        timer.reset();
    }

    @Override
    public boolean isDone() {
        // For now, timeout only
        if (timer.seconds() >= timeoutSeconds) return true;

        return getIntake().isFull();
    }

    @Override
    public void stop(boolean interrupted) {
        getIntake().setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
    }
}
