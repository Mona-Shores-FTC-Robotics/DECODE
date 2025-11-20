package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumSet;
import java.util.Objects;

/**
 * Launches every lane as soon as the flywheel speeds reach target.
 *
 * IMPORTANT: This command assumes launch RPMs have already been set via setLaunchRpm().
 * If no RPMs are set, lanes will be treated as disabled (RPM = 0) and won't fire.
 *
 * For range-based or vision-based shooting, use LaunchAllAtPresetRangeCommand or a custom command
 * that sets RPMs explicitly.
 *
 * Designed to be triggered when the operator releases the "launch all" button.
 */
public class LaunchAllCommand extends Command {

    private enum Stage {
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final boolean spinDownAfterShot;

    private final EnumSet<LauncherLane> queuedLanes = EnumSet.noneOf(LauncherLane.class);
    private final ElapsedTime timer = new ElapsedTime();

    private Stage stage = Stage.WAITING_FOR_READY;
    private boolean spinDownApplied = false;
    private final IntakeSubsystem intake;


    public LaunchAllCommand(LauncherSubsystem launcher,
                            IntakeSubsystem intake,
                            boolean spinDownAfterShot) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake; // Nullable - robot may not have prefeed roller
        this.spinDownAfterShot = spinDownAfterShot;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.WAITING_FOR_READY;
        queuedLanes.clear();
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        spinDownApplied = false;
        // Activate gate in forward direction to help feed
        if (intake != null) {
            intake.setGateAllowArtifacts();
        }
    }

    @Override
    public void update() {
        switch (stage) {
            case WAITING_FOR_READY:
                checkLaneReadiness();
                if (queuedLanes.size() == LauncherLane.values().length) {
                    stage = Stage.SHOTS_QUEUED;
                }
                break;
            case SHOTS_QUEUED:
                if (!launcher.isBusy() && launcher.getQueuedShots() == 0) {
                    stage = Stage.COMPLETED;
                    if (spinDownAfterShot && !spinDownApplied) {
                        launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
                        spinDownApplied = true;
                    }
                }
                break;
            case COMPLETED:
            default:
                // nothing
                break;
        }
    }

    private void checkLaneReadiness() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) {
                continue;
            }
            // Queue shot immediately when lane is ready - no stability wait
            if (launcher.isLaneReady(lane)) {
                launcher.queueShot(lane);
                queuedLanes.add(lane);
            }
        }
    }

    @Override
    public boolean isDone() {
        return stage == Stage.COMPLETED;
    }

    @Override
    public void stop(boolean interrupted) {
        // put gate down
        if (intake != null) {
            intake.setGatePreventArtifact();
        }
        if (interrupted && !queuedLanes.isEmpty()) {
            launcher.clearQueue();
        }
        if (interrupted && spinDownAfterShot && !spinDownApplied) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
            spinDownApplied = true;
        }
    }
}
