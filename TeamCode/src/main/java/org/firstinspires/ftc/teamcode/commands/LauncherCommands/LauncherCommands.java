package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

/**
 * Convenience factory for launcher-related commands alongside immediate queue helpers used by
 * legacy binding paths.
 */
@SuppressWarnings("UnusedReturnValue")
@Configurable
public class LauncherCommands {

    public static final double DEFAULT_BURST_SPACING_MS = 150.0;

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final LauncherCoordinator launcherCoordinator;
    private final ManualSpinController manualSpinController;

    public LauncherCommands(LauncherSubsystem launcher,
                           IntakeSubsystem intake,
                           LauncherCoordinator launcherCoordinator,
                           ManualSpinController manualSpinController) {
        this.launcher = launcher;
        this.intake = intake;
        this.launcherCoordinator = launcherCoordinator;
        this.manualSpinController = manualSpinController;
    }

    public LaunchLaneCommand launchLane(LauncherLane lane) {
        return new LaunchLaneCommand(launcher , lane);
    }

    public LaunchLaneCommand launchLeft() {
        return launchLane(LauncherLane.LEFT);
    }

    public LaunchLaneCommand launchCenter() {
        return launchLane(LauncherLane.CENTER);
    }

    public LaunchLaneCommand launchRight() {
        return launchLane(LauncherLane.RIGHT);
    }

    public LaunchBurstCommand launchAllInSequence() {
        return launchAllInSequence(DEFAULT_BURST_SPACING_MS);
    }

    public LaunchBurstCommand launchAllInSequence(double spacingMs) {
        return new LaunchBurstCommand(launcher , spacingMs);
    }

    public LaunchDetectedBurstCommand launchDetectedBurst() {
        return launchDetectedBurst(DEFAULT_BURST_SPACING_MS);
    }

    public LaunchDetectedBurstCommand launchDetectedBurst(double spacingMs) {
        return new LaunchDetectedBurstCommand(launcher , launcherCoordinator, spacingMs);
    }

    /**
     * Immediate helper used by bindings that still trigger launcher actions imperatively.
     */
    public void queueLane(LauncherLane lane) {
        if (lane == null) {
            return;
        }
        launcher.queueShot(lane);
    }

    public void queueDetectedBurst(double spacingMs) {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(spacingMs);
        } else {
            launcher.queueBurstAll();
        }
    }

    public void cancelAll() {
        launcher.clearQueue();
    }

    public void setSpinModeToFull() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
    }

    public void setSpinModeToIdle() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
    }

    public void setSpinModeToOff() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.OFF);
    }

    public SetFeederPositionCommand setLeftFeederToLoad() {
        return new SetFeederPositionCommand(launcher, LauncherLane.LEFT, true);
    }

    public SetFeederPositionCommand setLeftFeederToFire() {
        return new SetFeederPositionCommand(launcher, LauncherLane.LEFT, false);
    }

    public SpinUpUntilReadyCommand spinUpUntilReady() {
        return new SpinUpUntilReadyCommand(launcher);
    }

    /**
     * Phase 1: Spin up to position-specific RPM (tunable in Dashboard)
     * @param position Field position we're launching from
     */
    public LaunchAtPositionCommand spinUpForPosition(org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint position) {
        return new LaunchAtPositionCommand(launcher, position);
    }

    /**
     * Phase 1: Spin up to explicit RPM (for testing)
     * @param targetRpm Desired RPM for both enabled launchers
     */
    public LaunchAtPositionCommand spinUpToRpm(double targetRpm) {
        return new LaunchAtPositionCommand(launcher, targetRpm);
    }

    /**
     * Fires all lanes at SHORT range (~2700 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a short-range shot
     */
    public FireAllAtRangeCommand fireAllShortRange() {
        return new FireAllAtRangeCommand(launcher, intake, LauncherRange.SHORT, true, manualSpinController);
    }

    /**
     * Fires all lanes at MID range (~3600 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a mid-range shot
     */
    public FireAllAtRangeCommand fireAllMidRange() {
        return new FireAllAtRangeCommand(launcher, intake, LauncherRange.MID, true, manualSpinController);
    }

    /**
     * Fires all lanes at LONG range (~4200 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a long-range shot
     */
    public FireAllAtRangeCommand fireAllLongRange() {
        return new FireAllAtRangeCommand(launcher, intake, LauncherRange.LONG, true, manualSpinController);
    }

    /**
     * Generic range-based firing command.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @param range The shooting range (SHORT, MID, or LONG)
     * @param spinDownAfterShot Whether to spin down to idle after firing
     * @return Command that executes the range-based shot
     */
    public FireAllAtRangeCommand fireAllAtRange(LauncherRange range, boolean spinDownAfterShot) {
        return new FireAllAtRangeCommand(launcher, intake, range, spinDownAfterShot, manualSpinController);
    }

    /**
     * Fires all lanes at SHORT range (~2700 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a short-range shot
     */
    public FireAllCommand fireAll(boolean spinDownAfterShot) {
        return new FireAllCommand(launcher, intake, spinDownAfterShot, manualSpinController);
    }

}
