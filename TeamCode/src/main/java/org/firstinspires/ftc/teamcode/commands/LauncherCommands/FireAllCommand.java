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
 * Designed to be triggered when the operator releases the "launch all" button.
 */
public class FireAllCommand extends Command {

    private enum Stage {
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final ManualSpinController manualSpinController;
    private final boolean spinDownAfterShot;

    private final EnumSet<LauncherLane> queuedLanes = EnumSet.noneOf(LauncherLane.class);
    private final ElapsedTime timer = new ElapsedTime();

    private Stage stage = Stage.WAITING_FOR_READY;
    private boolean manualSpinActive = false;
    private boolean spinDownApplied = false;
    private final IntakeSubsystem intake;


    public FireAllCommand(LauncherSubsystem launcher,
                          IntakeSubsystem intake,
                          boolean spinDownAfterShot,
                          ManualSpinController manualSpinController) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake; // Nullable - robot may not have prefeed roller
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
        manualSpinActive = true;
        manualSpinController.enterManualSpin();
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
        spinDownApplied = false;
        // Activate prefeed roller in forward direction to help feed
        if (intake != null) {
            intake.setPrefeedForward();
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
            // Deactivate prefeed roller (returns to not spinning)
            //TODO maybe this should be sooner?
            if (intake != null) {
                intake.deactivatePrefeed();
            }


        }
    }
}
