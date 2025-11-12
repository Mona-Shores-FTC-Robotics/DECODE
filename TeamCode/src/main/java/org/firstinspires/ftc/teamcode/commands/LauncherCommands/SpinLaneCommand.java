package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

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
    private final ManualSpinController manualSpinController;

    private boolean manualSpinActive = false;

    public SpinLaneCommand(LauncherSubsystem launcher, ManualSpinController manualSpinController) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.manualSpinController = Objects.requireNonNull(manualSpinController, "manualSpinController required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        manualSpinController.enterManualSpin();
        manualSpinActive = true;
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
        if (manualSpinActive) {
            manualSpinController.exitManualSpin();
            manualSpinActive = false;
        }
    }
}
