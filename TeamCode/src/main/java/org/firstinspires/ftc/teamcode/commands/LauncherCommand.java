package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/**
 * Base class for launcher-oriented commands. Handles shooter requirements and spin mode management
 * so concrete commands only need to describe which shots to queue.
 */
abstract class LauncherCommand extends Command {

    private final LauncherSubsystem launcher;
    private final boolean manageSpinMode;

    private LauncherSubsystem.SpinMode previousSpinMode = LauncherSubsystem.SpinMode.OFF;
    private boolean queuedAnyShot = false;

    protected LauncherCommand(LauncherSubsystem launcher ,
                               boolean manageSpinMode,
                               Subsystem... additionalRequirements) {
        this.launcher = launcher;
        this.manageSpinMode = manageSpinMode;
        requires(launcher);
        if (additionalRequirements != null) {
            for (Subsystem requirement : additionalRequirements) {
                if (requirement != null && requirement != launcher) {
                    requires(requirement);
                }
            }
        }
        setInterruptible(true);
    }

    protected LauncherSubsystem getLauncher() {
        return launcher;
    }

    @Override
    public void start() {
        previousSpinMode = launcher.getRequestedSpinMode();
        if (manageSpinMode && previousSpinMode != LauncherSubsystem.SpinMode.FULL) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        }
        queuedAnyShot = queueShots();
    }

    @Override
    public void update() {
        // Shooter commands only need to wait on subsystem state, so nothing to do each cycle.
    }

    @Override
    public boolean isDone() {
        if (!queuedAnyShot) {
            return true;
        }
        return launcher.getQueuedShots() == 0 && ! launcher.isBusy();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted && queuedAnyShot) {
            launcher.clearQueue();
        }
        if (manageSpinMode && previousSpinMode != LauncherSubsystem.SpinMode.FULL) {
            launcher.setSpinMode(previousSpinMode);
        }
    }

    /**
     * Implementations should enqueue the desired shots and report whether any work was scheduled.
     */
    protected abstract boolean queueShots();
}
