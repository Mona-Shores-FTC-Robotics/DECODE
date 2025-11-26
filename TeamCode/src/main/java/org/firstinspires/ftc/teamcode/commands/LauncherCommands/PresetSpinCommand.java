package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import java.util.Objects;

/**
 * Spins up launchers to position-specific RPM targets and waits until ready.
 *
 * Phase 1: Uses tunable RPM values per field position
 * Phase 3 upgrade path: Can switch to distance-based formula
 */
@Configurable
public class PresetSpinCommand extends Command {

    private final LauncherSubsystem launcher;
    private final double range;
    private final double targetRpm;
    private double startTime = 0.0;

    /**
     * Creates command that spins up to position-specific RPM
     * @param launcher The launcher subsystem
     */
    public PresetSpinCommand(LauncherSubsystem launcher, LauncherRange range) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.range = range;
        this.targetRpm = getRpmForPosition(launcher);
        this.targetHood = get(position);
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        launcher.setLaunchRpm(LauncherLane.LEFT, targetRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, targetRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, targetRpm);
        launcher.setAllHoodsForRange();

        // Spin up to target
        launcher.spinUpAllLanesToLaunch();


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
            launcher.spinUpAllLanesToLaunch();
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

}
