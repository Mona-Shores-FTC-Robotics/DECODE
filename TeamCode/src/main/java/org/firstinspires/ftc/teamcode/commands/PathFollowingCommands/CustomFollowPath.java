package org.firstinspires.ftc.teamcode.commands.PathFollowingCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.FollowerHolder;
import org.firstinspires.ftc.teamcode.util.RobotState;

import dev.nextftc.core.commands.Command;

/**
 * Custom path following command that bypasses NextFTC's PedroComponent.
 * Gets follower directly from FollowerHolder instead.
 */
public class CustomFollowPath extends Command {
    private final PathChain path;
    private final boolean holdEnd;
    private final double maxPower;
    private Follower follower;

    public CustomFollowPath(PathChain path, boolean holdEnd, double maxPower) {
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    @Override
    public void start() {
        follower = FollowerHolder.getFollower();
        if (follower == null) {
            throw new IllegalStateException("Follower not initialized in FollowerHolder");
        }
        follower.followPath(path, holdEnd);
        RobotState.packet.put("Auto/PathFollowing", "Started");
        RobotState.packet.put("Auto/PathHoldEnd", holdEnd);
        RobotState.packet.put("Auto/PathMaxPower", maxPower);
    }

    @Override
    public void execute() {
        // Follower update is handled by DriveSubsystem.periodic()
        if (follower != null) {
            RobotState.packet.put("Auto/PathFollowing", "Running");
            RobotState.packet.put("Auto/FollowerBusy", follower.isBusy());
        }
    }

    @Override
    public boolean isDone() {
        return follower != null && !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && follower != null) {
            follower.breakFollowing();
            RobotState.packet.put("Auto/PathFollowing", "Interrupted");
        } else {
            RobotState.packet.put("Auto/PathFollowing", "Completed");
        }
    }
}
