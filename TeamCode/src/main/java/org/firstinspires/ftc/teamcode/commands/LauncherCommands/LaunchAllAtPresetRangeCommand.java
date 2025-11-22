package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import java.util.EnumSet;
import java.util.Objects;

/**
 * Fires all three launcher lanes at a specific range configuration.
 *
 * Sets RPM targets and hood angles for all lanes based on the selected range (SHORT, MID, LONG),
 * spins up flywheels to target speed, fires all lanes in sequence, then spins down.
 *
 * RPM values and hood positions are configurable via FTC Dashboard for field tuning.
 * Each lane's hood can have different servo values to account for mounting differences.
 */
@Configurable
public class LaunchAllAtPresetRangeCommand extends Command {

    @Configurable
    public static class RangeRpmConfig {
        /** Short range configuration */
        public double shortLeftRpm = 2000;
        public double shortCenterRpm = 2000; // Center disabled by default
        public double shortRightRpm = 2000;

        /** Mid range configuration (default/current values) */
        public double midLeftRpm = 2400;
        public double midCenterRpm = 2400; // Center disabled by default
        public double midRightRpm = 2400;

        /** Long range configuration */
        public double longLeftRpm = 2900;
        public double longCenterRpm = 2900; // Center disabled by default
        public double longRightRpm = 2900;

        /** Timeout in seconds before giving up on spin-up */
        public double timeoutSeconds = 8.0;
    }

    public static RangeRpmConfig rangeRpmConfig = new RangeRpmConfig();

    private enum Stage {
        SPINNING_UP,
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final LauncherRange range;
    private final boolean spinDownAfterShot;

    private final EnumSet<LauncherLane> queuedLanes = EnumSet.noneOf(LauncherLane.class);
    private final ElapsedTime timer = new ElapsedTime();

    private Stage stage = Stage.SPINNING_UP;
    private boolean spinDownApplied = false;

    /**
     * Creates command that fires all lanes at the specified range.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed roller control)
     * @param range The shooting range (SHORT, MID, or LONG)
     * @param spinDownAfterShot Whether to spin down to idle after firing
     */
    public LaunchAllAtPresetRangeCommand(LauncherSubsystem launcher,
                                          IntakeSubsystem intake,
                                          LauncherRange range,
                                          boolean spinDownAfterShot) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake; // Nullable - robot may not have prefeed roller
        this.range = Objects.requireNonNull(range, "range required");
        this.spinDownAfterShot = spinDownAfterShot;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.SPINNING_UP;
        queuedLanes.clear();
        spinDownApplied = false;

        // Activate prefeed roller in forward direction to help feed
        if (intake != null) {
            intake.setGateAllowArtifacts();
        }

        // Set RPMs and hood angles for all lanes based on range
        setRpmsForRange();
        launcher.setAllHoodsForRange(range);

        // Spin up to target
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
    }

    @Override
    public void update() {
        switch (stage) {
            case SPINNING_UP:
                // Wait briefly for flywheels to start ramping
                if (timer.milliseconds() > 100) {
                    stage = Stage.WAITING_FOR_READY;
                }
                break;

            case WAITING_FOR_READY:
                checkLaneReadiness();
                if (areAllEnabledLanesQueued()) {
                    stage = Stage.SHOTS_QUEUED;
                }
                // Timeout safety
                if (timer.seconds() >= rangeRpmConfig.timeoutSeconds) {
                    // Queue whatever lanes are ready to prevent hanging
                    queueRemainingLanes();
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

    @Override
    public boolean isDone() {
        return stage == Stage.COMPLETED;
    }

    @Override
    public void stop(boolean interrupted) {
        if (intake != null) {
            intake.setGatePreventArtifact();
        }

        // Clear RPM overrides to return to default values
        launcher.clearOverrides();

        // Clear queue if interrupted
        if (interrupted && !queuedLanes.isEmpty()) {
            launcher.clearQueue();
        }

        // Spin down if configured and not already done
        if (interrupted && spinDownAfterShot && !spinDownApplied) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
            spinDownApplied = true;
        }
    }

    /**
     * Sets RPM targets for all lanes based on the selected range.
     */
    private void setRpmsForRange() {
        switch (range) {
            case SHORT:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeRpmConfig.shortLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeRpmConfig.shortCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeRpmConfig.shortRightRpm);
                break;

            case MID:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeRpmConfig.midLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeRpmConfig.midCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeRpmConfig.midRightRpm);
                break;

            case LONG:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeRpmConfig.longLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeRpmConfig.longCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeRpmConfig.longRightRpm);
                break;
        }
    }

    /**
     * Checks each lane for readiness and queues shots immediately when ready.
     */
    private void checkLaneReadiness() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) {
                continue;
            }

            // Skip disabled lanes (RPM = 0)
            if (launcher.getLaunchRpm(lane) <= 0.0) {
                queuedLanes.add(lane); // Mark as "queued" to skip in future checks
                continue;
            }

            // Queue shot immediately when lane is ready - no stability wait
            if (launcher.isLaneReady(lane)) {
                launcher.queueShot(lane);
                queuedLanes.add(lane);
            }
        }
    }

    /**
     * Checks if all enabled lanes have been queued.
     */
    private boolean areAllEnabledLanesQueued() {
        for (LauncherLane lane : LauncherLane.values()) {
            // Skip disabled lanes
            if (launcher.getLaunchRpm(lane) <= 0.0) {
                continue;
            }
            // If any enabled lane isn't queued, we're not done
            if (!queuedLanes.contains(lane)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Queues all remaining enabled lanes (used on timeout).
     */
    private void queueRemainingLanes() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) {
                continue;
            }
            if (launcher.getLaunchRpm(lane) > 0.0) {
                launcher.queueShot(lane);
                queuedLanes.add(lane);
            }
        }
    }

    /**
     * Gets the range this command is configured for (for logging/debugging).
     */
    public LauncherRange getRange() {
        return range;
    }
}
