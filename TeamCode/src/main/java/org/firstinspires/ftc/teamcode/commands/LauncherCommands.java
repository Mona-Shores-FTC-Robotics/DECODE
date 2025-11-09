package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Convenience factory for launcher-related commands alongside immediate queue helpers used by
 * legacy binding paths.
 */
public class LauncherCommands {

    public static final double DEFAULT_BURST_SPACING_MS = 150.0;

    private final LauncherSubsystem launcher;
    private final LauncherCoordinator launcherCoordinator;

    public LauncherCommands(LauncherSubsystem launcher, LauncherCoordinator launcherCoordinator) {
        this.launcher = launcher;
        this.launcherCoordinator = launcherCoordinator;
    }

    public LaunchLaneCommand launchLane(LauncherLane lane) {
        return new LaunchLaneCommand(launcher , lane);
    }

    public LaunchLaneCommand launchLeft() {
        return launchLane(LauncherLane.LEFT);
    }

    public LaunchLaneCommand launchCenter() {
        return launchLane(LauncherLane.CENTER);
    }

    public LaunchLaneCommand launchRight() {
        return launchLane(LauncherLane.RIGHT);
    }

    public LaunchBurstCommand launchAll() {
        return launchAll(DEFAULT_BURST_SPACING_MS);
    }

    public LaunchBurstCommand launchAll(double spacingMs) {
        return new LaunchBurstCommand(launcher , spacingMs);
    }

    public LaunchDetectedBurstCommand launchDetectedBurst() {
        return launchDetectedBurst(DEFAULT_BURST_SPACING_MS);
    }

    public LaunchDetectedBurstCommand launchDetectedBurst(double spacingMs) {
        return new LaunchDetectedBurstCommand(launcher , launcherCoordinator, spacingMs);
    }

    /**
     * Immediate helper used by bindings that still trigger launcher actions imperatively.
     */
    public void queueLane(LauncherLane lane) {
        if (lane == null) {
            return;
        }
        launcher.queueShot(lane);
    }

    public void queueDetectedBurst(double spacingMs) {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(spacingMs);
        } else {
            launcher.queueBurstAll();
        }
    }

    public void cancelAll() {
        launcher.clearQueue();
    }

}
