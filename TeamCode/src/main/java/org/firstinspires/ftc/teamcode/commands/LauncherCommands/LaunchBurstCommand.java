package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Command that queues a predefined burst of launcher lanes with fixed spacing.
 */
public class LaunchBurstCommand extends LauncherCommand {

    private final double spacingMs;

    public LaunchBurstCommand(LauncherSubsystem launcher,
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
