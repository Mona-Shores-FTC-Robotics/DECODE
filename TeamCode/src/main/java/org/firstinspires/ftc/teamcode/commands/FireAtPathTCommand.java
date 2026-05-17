package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Fires all launcher lanes when the Pedro Pathing follower's T value reaches a threshold.
 *
 * Run this as a non-deadline inside a ParallelDeadlineGroup where the deadline is a FollowPath
 * command. The fire trigger at fireAtT (e.g. 0.85) should come AFTER headingInterpEnd so
 * heading is already settled when shots are queued.
 *
 * Reads progress via follower.getCurrentPath().getClosestPointTValue() — the confirmed Pedro API.
 * Assumes flywheels are already at launch speed (spinDownAfterShot=false from the previous shot).
 * Calls spinUpAllLanesToLaunch() as a safety re-confirm before queuing.
 */
public class FireAtPathTCommand extends Command {

    private final Follower follower;
    private final double fireAtT;
    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;

    private boolean fired = false;

    public FireAtPathTCommand(Follower follower,
                              double fireAtT,
                              LauncherSubsystem launcher,
                              IntakeSubsystem intake) {
        this.follower = follower;
        this.fireAtT = fireAtT;
        this.launcher = launcher;
        this.intake = intake;
        setInterruptible(true);
    }

    @Override
    public void start() {
        fired = false;
    }

    @Override
    public void update() {
        if (fired) return;
        Path currentPath = follower.getCurrentPath();
        if (currentPath != null && currentPath.getClosestPointTValue() >= fireAtT) {
            launcher.spinUpAllLanesToLaunch();
            for (LauncherLane lane : LauncherLane.values()) {
                launcher.queueShot(lane);
            }
            if (intake != null) {
                intake.setGateAllowArtifacts();
            }
            fired = true;
        }
    }

    @Override
    public boolean isDone() {
        // Finish once fired, or if path ended before reaching the threshold (timeout/early stop)
        return fired || !follower.isBusy();
    }

    @Override
    public void stop(boolean interrupted) {
        // Shots are queued directly in the subsystem - no cleanup needed
    }
}
