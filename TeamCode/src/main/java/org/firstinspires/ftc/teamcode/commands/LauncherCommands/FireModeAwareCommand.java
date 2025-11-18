package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Smart fire command that adapts behavior based on current launcher mode.
 *
 * THROUGHPUT mode: Fires all lanes rapidly at MID range for maximum scoring rate
 * DECODE mode: Fires in obelisk pattern sequence with motif tail offset for precise scoring
 *
 * The mode is controlled by RobotState.launcherMode and can be changed:
 * - Manually by operator via dpad button
 * - Automatically when 30 seconds remain in teleop (switches to DECODE)
 * - Set to DECODE by default in autonomous
 */
public class FireModeAwareCommand extends Command {

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;

    private Command delegateCommand;

    /**
     * Creates a mode-aware fire command that delegates to the appropriate firing strategy.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed roller control and artifact tracking)
     */
    public FireModeAwareCommand(LauncherSubsystem launcher,
                                 IntakeSubsystem intake) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = Objects.requireNonNull(intake, "intake required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        // Get current mode from RobotState
        LauncherMode mode = RobotState.getLauncherMode();
        if (mode == null) {
            mode = LauncherMode.THROUGHPUT; // Default to throughput if not set
        }

        // Create appropriate delegate command based on mode
        switch (mode) {
            case DECODE:
                // DECODE mode: Fire in obelisk pattern sequence with motif tail offset
                delegateCommand = new FireInSequenceCommand(
                    launcher,
                    intake
                );
                break;

            case THROUGHPUT:
            default:
                // THROUGHPUT mode: Fire all lanes rapidly at MID range
                delegateCommand = new FireAllAtRangeCommand(
                    launcher,
                    intake,
                    LauncherRange.MID,
                    true // spin down after shot
                );
                break;
        }

        // Start the delegate command
        delegateCommand.start();
    }

    @Override
    public void update() {
        if (delegateCommand != null) {
            delegateCommand.update();
        }
    }

    @Override
    public boolean isDone() {
        return delegateCommand != null && delegateCommand.isDone();
    }

    @Override
    public void stop(boolean interrupted) {
        if (delegateCommand != null) {
            delegateCommand.stop(interrupted);
        }
    }
}
