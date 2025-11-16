package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Command that queues launcher lanes to fire sequentially with configurable spacing.
 *
 * IMPORTANT: This command assumes launch RPMs have already been set via setLaunchRpm().
 * If no RPMs are set, lanes will be treated as disabled (RPM = 0) and won't fire.
 *
 * Typically used after FireAllAtRangeCommand or other commands that set RPMs explicitly.
 */
public class LaunchSequentialCommand extends LauncherCommand {

    private final double spacingMs;

    public LaunchSequentialCommand(LauncherSubsystem launcher,
                                    double spacingMs) {
        super(launcher, true);
        this.spacingMs = Math.max(0.0, spacingMs);
    }

    @Override
    protected boolean queueShots() {
        if (LauncherLane.DEFAULT_BURST_ORDER.length == 0) {
            return false;
        }
        boolean queued = false;
        double delay = 0.0;
        for (LauncherLane lane : LauncherLane.DEFAULT_BURST_ORDER) {
            if (lane == null) {
                delay += spacingMs;
                continue;
            }
            if (delay > 0.0) {
                getLauncher().queueShot(lane, delay);
            } else {
                getLauncher().queueShot(lane);
            }
            queued = true;
            delay += spacingMs;
        }
        return queued;
    }
}
