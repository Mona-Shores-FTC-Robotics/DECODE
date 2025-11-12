package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Diagnostic command that moves a single launcher feeder between its configured load and fire positions.
 */
public class SetFeederPositionCommand extends Command {

    private final LauncherSubsystem launcher;
    private final LauncherLane lane;
    private final boolean toLoad;

    public SetFeederPositionCommand(LauncherSubsystem launcher, LauncherLane lane, boolean toLoad) {
        this.launcher = launcher;
        this.lane = lane;
        this.toLoad = toLoad;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        if (lane == null) {
            return;
        }
        if (toLoad) {
            launcher.moveFeederToLoad(lane);
        } else {
            launcher.moveFeederToFire(lane);
        }
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
