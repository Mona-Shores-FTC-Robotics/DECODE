package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.Objects;

/**
 * Keeps the launcher spinning at full power while a button is held.
 */
public class SpinHoldCommand extends Command {

    private final LauncherSubsystem launcher;

    public SpinHoldCommand(LauncherSubsystem launcher) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
    }

    @Override
    public void update() {
        // Stay in spin mode while held.
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
    }
}
