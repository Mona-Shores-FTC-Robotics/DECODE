package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Keeps the intake in passive reverse mode indefinitely.
 */
public class ForwardIntakeCommand extends IntakeCommand {

    public ForwardIntakeCommand(IntakeSubsystem intake) {
        super(intake);
    }

    @Override
    public void start() {
        getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
    }

    @Override
    public boolean isDone() {
        return false; // never ends until interrupted
    }

    @Override
    public void stop(boolean interrupted) {
        getIntake().setMode(IntakeSubsystem.IntakeMode.STOPPED);
    }
}
