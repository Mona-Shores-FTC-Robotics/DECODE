package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.Objects;

/**
 * Spins up launchers to position-specific RPM targets and waits until ready.
 *
 * Phase 1: Uses tunable RPM values per field position
 * Phase 3 upgrade path: Can switch to distance-based formula
 */
@Configurable
public class LaunchAtPositionCommand extends Command {

    @Configurable
    public static class PositionRpmConfig {
        /** RPM for shots from LAUNCH_FAR position */
        public double farLaunchRpm = 2900;

        /** RPM for shots from LAUNCH_CLOSE position */
        public double closeLaunchRpm = 2150;

        /** Default RPM if position unknown */
        public double defaultLaunchRpm = 2400;

        /** Timeout in seconds before giving up */
        public double timeoutSeconds = 3.0;
    }

    public static PositionRpmConfig positionRpmConfig = new PositionRpmConfig();

    private final LauncherSubsystem launcher;
    private final FieldPoint position;
    private final double targetRpm;
    private double startTime = 0.0;

    /**
     * Creates command that spins up to position-specific RPM
     * @param launcher The launcher subsystem
     * @param position The field position we're launching from
     */
    public LaunchAtPositionCommand(LauncherSubsystem launcher, FieldPoint position) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.position = position;
        this.targetRpm = getRpmForPosition(position);
        requires(launcher);
        setInterruptible(true);
    }

    /**
     * Creates command with explicit RPM override (for testing/tuning)
     * @param launcher The launcher subsystem
     * @param targetRpm Explicit RPM to use
     */
    public LaunchAtPositionCommand(LauncherSubsystem launcher, double targetRpm) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.position = null;
        this.targetRpm = targetRpm;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {

        launcher.setLaunchRpm(LauncherLane.LEFT, targetRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, targetRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, targetRpm);
        launcher.setAllHoodsExtended();

        // Spin up to target
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void update() {
        // Monitor launcher status
    }

    @Override
    public boolean isDone() {
        // Check if all enabled launchers are at target
        if (areEnabledLaunchersReady()) {
            return true;
        }

        // Safety timeout
        double elapsedSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
        return elapsedSeconds >= positionRpmConfig.timeoutSeconds;
    }

    @Override
    public void stop(boolean interrupted) {
        // Keep launchers spinning unless interrupted
        if (interrupted) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
            launcher.clearOverrides(); // Reset to default RPMs
        }
    }

    /**
     * Checks if all enabled launchers are at target RPM
     */
    private boolean areEnabledLaunchersReady() {
        for (LauncherLane lane : LauncherLane.values()) {
            double launchRpm = launcher.getLaunchRpm(lane);

            if (launchRpm <= 0.0) {
                continue; // Skip disabled launchers
            }

            if (!launcher.isLaneReady(lane)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Maps field position to RPM target
     * Phase 3: Replace with distance-based formula
     */
    private static double getRpmForPosition(FieldPoint position) {
        if (position == null) {
            return positionRpmConfig.defaultLaunchRpm;
        }

        switch (position) {
            case LAUNCH_FAR:
                return positionRpmConfig.farLaunchRpm;
            case LAUNCH_CLOSE:
                return positionRpmConfig.closeLaunchRpm;
            default:
                return positionRpmConfig.defaultLaunchRpm;
        }
    }

    /**
     * Gets the RPM this command will use (for logging)
     */
    public double getTargetRpm() {
        return targetRpm;
    }
}
