package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.Objects;

/**
 * Handles the launch sequence for a single lane:
 *   1. Spin up the flywheel.
 *   2. Wait for the lane to report ready (with a small stability window).
 *   3. Queue the shot and wait for completion.
 *   4. Optionally spin down.
 */
public class FireLaneCommand extends Command {

    private static final double READY_STABLE_MS = 5;

    private enum Stage {
        WAITING_FOR_READY,
        SHOT_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final LauncherLane lane;
    private final boolean spinDownAfterShot;
    private final ManualSpinController manualSpinController;

    private final ElapsedTime timer = new ElapsedTime();

    private Stage stage = Stage.WAITING_FOR_READY;
    private double readySinceMs = -1.0;
    private boolean manualSpinActive = false;
    private boolean shotQueued = false;
    private boolean spinDownApplied = false;

    public FireLaneCommand(LauncherSubsystem launcher,
                          LauncherLane lane,
                          boolean spinDownAfterShot,
                          ManualSpinController manualSpinController) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.lane = Objects.requireNonNull(lane, "lane required");
        this.spinDownAfterShot = spinDownAfterShot;
        this.manualSpinController = Objects.requireNonNull(manualSpinController, "manualSpinController required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.WAITING_FOR_READY;
        readySinceMs = -1.0;
        shotQueued = false;
        manualSpinActive = true;
        manualSpinController.enterManualSpin();
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        spinDownApplied = false;
    }

    @Override
    public void update() {
        switch (stage) {
            case WAITING_FOR_READY:
                if (launcher.isLaneReady(lane)) {
                    if (readySinceMs < 0.0) {
                        readySinceMs = timer.milliseconds();
                    } else if (timer.milliseconds() - readySinceMs >= READY_STABLE_MS) {
                        launcher.queueShot(lane);
                        shotQueued = true;
                        stage = Stage.SHOT_QUEUED;
                    }
                } else {
                    readySinceMs = -1.0;
                }
                break;
            case SHOT_QUEUED:
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

    @Override
    public boolean isDone() {
        return stage == Stage.COMPLETED;
    }

    @Override
    public void stop(boolean interrupted) {
        if (manualSpinActive) {
            manualSpinController.exitManualSpin();
            manualSpinActive = false;
        }
        if (interrupted && shotQueued) {
            launcher.clearQueue();
        }
        if (interrupted && spinDownAfterShot && !spinDownApplied) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
            spinDownApplied = true;
        }
    }
}
