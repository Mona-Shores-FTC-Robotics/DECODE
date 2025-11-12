package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Command that queues a single launcher lane for firing.
 */
public class LaunchLaneCommand extends LauncherCommand {

    private final LauncherLane lane;
    private final double delayMs;

    public LaunchLaneCommand(LauncherSubsystem launcher, LauncherLane lane) {
        this(launcher, lane, 0.0);
    }

    public LaunchLaneCommand(LauncherSubsystem launcher, LauncherLane lane, double delayMs) {
        super(launcher, true);
        this.lane = lane;
        this.delayMs = Math.max(0.0, delayMs);
    }

    @Override
    protected boolean queueShots() {
        if (lane == null) {
            return false;
        }
        if (delayMs > 0.0) {
            getLauncher().queueShot(lane, delayMs);
        } else {
            getLauncher().queueShot(lane);
        }
        return true;
    }
}
