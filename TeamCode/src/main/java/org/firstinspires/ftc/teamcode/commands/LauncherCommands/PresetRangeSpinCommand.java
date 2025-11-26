package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeLaunchAllCommand.RangeRpmConfig;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import java.util.Objects;

/**
 * Spins up launchers to preset (short/mid/long) RPM targets and waits until ready.
 * Uses the same tunable RPM table as LaunchAllAtPresetRangeCommand so operators can
 * reuse dashboard values for spin-up-only behaviour.
 */
@Configurable
public class PresetRangeSpinCommand extends Command {

    private final LauncherSubsystem launcher;
    private final LauncherRange range;
    private final RangeRpmConfig rpmConfig = PresetRangeLaunchAllCommand.rangeRpmConfig;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Creates a command that spins up to a preset range (SHORT, MID, LONG).
     * @param launcher The launcher subsystem
     * @param range The preset range to use
     */
    public PresetRangeSpinCommand(LauncherSubsystem launcher, LauncherRange range) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.range = Objects.requireNonNull(range, "range required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        setRpmsForRange();
        launcher.setAllHoodsForRange(range);
        launcher.spinUpAllLanesToLaunch();
    }

    @Override
    public void update() {
        // Monitor launcher status
    }

    @Override
    public boolean isDone() {
        if (areEnabledLaunchersReady()) {
            return true;
        }
        return timer.seconds() >= rpmConfig.timeoutSeconds;
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            launcher.clearOverrides();
        }
    }

    /**
     * Checks if all enabled launchers are at target RPM
     */
    private boolean areEnabledLaunchersReady() {
        for (LauncherLane lane : LauncherLane.values()) {
            double launchRpm = launcher.getLaunchRpm(lane);

            if (launchRpm <= 0.0) {
                continue; // Skip disabled launchers
            }

            if (!launcher.isLaneReady(lane)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Sets RPM targets for all lanes based on the selected range.
     */
    private void setRpmsForRange() {
        switch (range) {
            case SHORT:
                launcher.setLaunchRpm(LauncherLane.LEFT, rpmConfig.shortLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rpmConfig.shortCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rpmConfig.shortRightRpm);
                break;

            case MID:
                launcher.setLaunchRpm(LauncherLane.LEFT, rpmConfig.midLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rpmConfig.midCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rpmConfig.midRightRpm);
                break;

            case LONG:
                launcher.setLaunchRpm(LauncherLane.LEFT, rpmConfig.longLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rpmConfig.longCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rpmConfig.longRightRpm);
                break;
        }
    }

}
