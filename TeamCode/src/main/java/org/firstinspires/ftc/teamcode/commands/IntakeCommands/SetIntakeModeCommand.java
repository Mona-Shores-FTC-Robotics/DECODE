package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Simple one-shot command that sets the intake mode.
 */
public class SetIntakeModeCommand extends IntakeCommand {

    private final IntakeSubsystem.IntakeMode mode;
    private boolean started = false;

    public SetIntakeModeCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeMode mode) {
        super(intake);
        this.mode = mode == null ? IntakeSubsystem.IntakeMode.STOPPED : mode;
    }

    @Override
    public void start() {
        getIntake().setMode(mode);
        started = true;
    }

    @Override
    public boolean isDone() {
        return started;
    }
}
