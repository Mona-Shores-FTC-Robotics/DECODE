package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Mode-aware launch command that checks launcher mode at RUNTIME (not binding time).
 *
 * This fixes the bug where launchAccordingToMode() was called once at teleop start,
 * causing mode switches to have no effect on launching behavior.
 *
 * Now the mode is checked in start() when the command actually executes:
 * - DECODE mode: Delegates to LaunchInSequenceCommand (pattern-based sequential firing)
 * - THROUGHPUT mode: Delegates to LaunchAllCommand (fire all lanes ASAP)
 */
public class ModeAwareLaunchCommand extends Command {

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final boolean spinDownAfterShot;

    private Command delegate;

    public ModeAwareLaunchCommand(LauncherSubsystem launcher,
                                   IntakeSubsystem intake,
                                   boolean spinDownAfterShot) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = Objects.requireNonNull(intake, "intake required");
        this.spinDownAfterShot = spinDownAfterShot;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        // Check mode NOW (at execution time), not at binding configuration time
        LauncherMode currentMode = RobotState.getLauncherMode();

        if (currentMode == LauncherMode.DECODE) {
            delegate = new LaunchInSequenceCommand(launcher, intake, spinDownAfterShot);
        } else {
            delegate = new LaunchAllCommand(launcher, intake, spinDownAfterShot);
        }

        delegate.start();
    }

    @Override
    public void update() {
        if (delegate != null) {
            delegate.update();
        }
    }

    @Override
    public boolean isDone() {
        return delegate != null && delegate.isDone();
    }

    @Override
    public void stop(boolean interrupted) {
        if (delegate != null) {
            delegate.stop(interrupted);
        }
    }
}
