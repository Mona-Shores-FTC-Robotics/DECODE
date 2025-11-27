package org.firstinspires.ftc.teamcode.commands.LauncherCommands.testing;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.Objects;

/**
 * Keeps the launcher spinning at full power while the driver holds a lane button.
 * The command never completes on its own; the release logic handles firing via
 * a paired {@link FireLaneCommand}.
 */
public class SpinLaneCommand extends Command {

    private final LauncherSubsystem launcher;

    public SpinLaneCommand(LauncherSubsystem launcher) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        launcher.spinUpAllLanesToLaunch();
    }

    @Override
    public void update() {
        // Nothing to do while the button is held; we just stay in spin mode.
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        // Spin mode will be managed by next command or default to idle
    }
}
