package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.Objects;

/**
 * Toggle command for human loading station operations.
 *
 * Press once to start:
 * - Reverses flywheel for loading
 * - Reverses prefeed roller
 * - Retracts all hoods
 *
 * Press again to stop:
 * - Stops flywheel reverse
 * - Returns prefeed to forward
 * - Extends all hoods
 *
 * This is a toggle - the command remembers its state and alternates behavior on each press.
 */
public class HumanLoadingCommand extends Command {

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;

    private static boolean isLoading = false;  // Static state persists between command instances

    /**
     * Creates a toggle command for human loading operations.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed control)
     */
    public HumanLoadingCommand(LauncherSubsystem launcher, IntakeSubsystem intake) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake;  // Can be null
        setInterruptible(true);
    }

    @Override
    public void start() {
        // Toggle the loading state
        isLoading = !isLoading;

        if (isLoading) {
            // START LOADING: Reverse everything, retract hoods
            launcher.runReverseFlywheelForHumanLoading();
            if (intake != null) {
                intake.setGateAllowArtifacts();
            }
            launcher.setAllHoodsRetracted();
        } else {
            // STOP LOADING: Stop reverse, return to normal
            launcher.stopReverseFlywheelForHumanLoading();
            if (intake != null) {
                intake.setGatePreventArtifact();
            }
            launcher.setAllHoodsExtended();
        }
    }

    @Override
    public void update() {
        // Nothing to do - toggle happens instantly on start
    }

    @Override
    public boolean isDone() {
        // Complete immediately - toggle is instant
        return true;
    }

    @Override
    public void stop(boolean interrupted) {
        // Nothing to clean up - state persists intentionally
    }

    /**
     * Gets the current loading state.
     * Useful for telemetry/debugging.
     *
     * @return true if currently in loading mode, false otherwise
     */
    public static boolean isLoading() {
        return isLoading;
    }

    /**
     * Resets the loading state to stopped.
     * Should be called at match start to ensure clean state.
     */
    public static void reset() {
        isLoading = false;
    }
}
