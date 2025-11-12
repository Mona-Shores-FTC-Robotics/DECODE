package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.Objects;

/**
 * Spins up the launcher to target RPM and completes once all ENABLED launchers are ready.
 * This is designed for autonomous sequences where you need to ensure launchers
 * are at target speed before firing.
 *
 * Only checks launchers that have non-zero launch RPM configured.
 */
@Configurable
public class SpinUpUntilReadyCommand extends Command {

    @Configurable
    public static class SpinUpConfig {
        /** Timeout in seconds before giving up and completing anyway */
        public static double timeoutSeconds = 3.0;
    }

    private final LauncherSubsystem launcher;
    private double startTime = 0.0;

    public SpinUpUntilReadyCommand(LauncherSubsystem launcher) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void update() {
        // Continuously monitor launcher status
    }

    @Override
    public boolean isDone() {
        // Check if all ENABLED launchers (non-zero launch RPM) are ready
        if (areEnabledLaunchersReady()) {
            return true;
        }

        // Safety timeout to prevent hanging
        double elapsedSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
        return elapsedSeconds >= SpinUpConfig.timeoutSeconds;
    }

    @Override
    public void stop(boolean interrupted) {
        // Keep launchers spinning - the fire command will manage them
        // Only stop if interrupted
        if (interrupted) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
        }
    }

    /**
     * Checks if all enabled launchers (those with non-zero launch RPM) are at target.
     * This handles the case where some launchers may be disabled (0 RPM).
     */
    private boolean areEnabledLaunchersReady() {
        for (LauncherLane lane : LauncherLane.values()) {
            double launchRpm = launcher.getLaunchRpm(lane);

            // Skip disabled launchers (0 RPM)
            if (launchRpm <= 0.0) {
                continue;
            }

            // If this launcher is enabled but not ready, we're not done
            if (!launcher.isLaneReady(lane)) {
                return false;
            }
        }

        // All enabled launchers are ready
        return true;
    }
}
