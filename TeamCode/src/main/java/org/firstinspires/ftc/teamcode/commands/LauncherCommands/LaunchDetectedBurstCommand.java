package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Command that fires every lane currently holding an artifact according to the launcher coordinator.
 */
public class LaunchDetectedBurstCommand extends LauncherCommand {

    private final LauncherCoordinator launcherCoordinator;
    private final double spacingMs;

    public LaunchDetectedBurstCommand(LauncherSubsystem launcher,
                                      LauncherCoordinator launcherCoordinator,
                                      double spacingMs) {
        super(launcher, true, launcherCoordinator);
        this.launcherCoordinator = launcherCoordinator;
        this.spacingMs = Math.max(0.0, spacingMs);
    }

    @Override
    protected boolean queueShots() {
        if (launcherCoordinator == null) {
            return false;
        }
        boolean queued = false;
        double delay = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = launcherCoordinator.getLaneColor(lane);
            if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
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
