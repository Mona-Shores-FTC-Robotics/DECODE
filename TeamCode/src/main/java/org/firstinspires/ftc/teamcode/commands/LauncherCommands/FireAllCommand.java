package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Objects;

/**
 * Launches every lane once the flywheel speeds have stabilized.
 * Designed to be triggered when the operator releases the "launch all" button.
 */
public class FireAllCommand extends Command {

    private static final double READY_STABLE_MS = 50.0;

    private enum Stage {
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final ManualSpinController manualSpinController;
    private final boolean spinDownAfterShot;

    private final Map<LauncherLane, Double> readySinceMs = new EnumMap<>(LauncherLane.class);
    private final EnumSet<LauncherLane> queuedLanes = EnumSet.noneOf(LauncherLane.class);
    private final ElapsedTime timer = new ElapsedTime();

    private Stage stage = Stage.WAITING_FOR_READY;
    private boolean manualSpinActive = false;
    private boolean spinDownApplied = false;

    public FireAllCommand(LauncherSubsystem launcher,
                          boolean spinDownAfterShot,
                          ManualSpinController manualSpinController) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.spinDownAfterShot = spinDownAfterShot;
        this.manualSpinController = Objects.requireNonNull(manualSpinController, "manualSpinController required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.WAITING_FOR_READY;
        queuedLanes.clear();
        readySinceMs.clear();
        manualSpinActive = true;
        manualSpinController.enterManualSpin();
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        spinDownApplied = false;
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
        double now = timer.milliseconds();
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) {
                continue;
            }
            if (launcher.isLaneReady(lane)) {
                double since = readySinceMs.getOrDefault(lane, -1.0);
                if (since < 0.0) {
                    readySinceMs.put(lane, now);
                } else if (now - since >= READY_STABLE_MS) {
                    launcher.queueShot(lane);
                    queuedLanes.add(lane);
                }
            } else {
                readySinceMs.put(lane, -1.0);
            }
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
        if (interrupted && !queuedLanes.isEmpty()) {
            launcher.clearQueue();
        }
        if (interrupted && spinDownAfterShot && !spinDownApplied) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
            spinDownApplied = true;
        }
    }
}
