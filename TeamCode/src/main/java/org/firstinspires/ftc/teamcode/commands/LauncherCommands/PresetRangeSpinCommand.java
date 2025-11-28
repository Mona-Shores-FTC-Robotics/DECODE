package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotConfigs;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Spins up launchers to preset (short/mid/long) RPM and hood targets and waits until ready.
 * Uses the shared CommandRangeConfig so dashboard-tuned short/mid/long values carry across
 * both preset and distance-based flows.
 */
@Configurable
public class PresetRangeSpinCommand extends Command {

    private final LauncherSubsystem launcher;
    private final LauncherRange range;
    public static CommandRangeConfig rangeConfig() {
        return RobotConfigs.getCommandRangeConfig();
    }
    private final ElapsedTime timer = new ElapsedTime();
    private final boolean finishWhenReady;

    /**
     * Creates a command that spins up to a preset range (SHORT, MID, LONG).
     * @param launcher The launcher subsystem
     * @param range The preset range to use
     */
    public PresetRangeSpinCommand(LauncherSubsystem launcher, LauncherRange range) {
        this(launcher, range, true);
    }

    /**
     * Creates a command that spins up to a preset range (SHORT, MID, LONG).
     * @param launcher The launcher subsystem
     * @param range The preset range to use
     * @param finishWhenReady Whether the command should end automatically once RPMs are reached
     */
    public PresetRangeSpinCommand(LauncherSubsystem launcher, LauncherRange range, boolean finishWhenReady) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.range = Objects.requireNonNull(range, "range required");
        this.finishWhenReady = finishWhenReady;
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        launcher.clearRecoveryDeadlines();
        timer.reset();
        setRpmsForRange();
        launcher.setAllHoodPositions(hoodPositionForRange());
        launcher.spinUpAllLanesToLaunch();
    }

    @Override
    public void update() {
        boolean allEnabledReady = true;
        boolean anyEnabled = false;

        for (LauncherLane lane : LauncherLane.values()) {
            double launchRpm = launcher.getLaunchRpm(lane);
            boolean enabled = launchRpm > 0.0;
            boolean ready = enabled && launcher.isLaneReady(lane);

            if (enabled) {
                anyEnabled = true;
                allEnabledReady &= ready;
            }

            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Enabled", enabled);
            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Target RPM", launchRpm);
            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Ready", ready);
            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Current RPM", launcher.getCurrentRpm(lane));
        }

        RobotState.packet.put("Preset-Spin/Range", range.name());
        RobotState.packet.put("Preset-Spin/Finish When Ready", finishWhenReady);
        RobotState.packet.put("Preset-Spin/All Enabled Ready", anyEnabled && allEnabledReady);
    }

    @Override
    public boolean isDone() {
        if (!finishWhenReady) {
            return false; // Hold-to-spin behaviour; caller interrupts when ready to fire
        }

        return areEnabledLaunchersReady() || timer.seconds() >= rangeConfig().timeoutSeconds;
    }

    @Override
    public void stop(boolean interrupted) {
        // Preserve RPM/hood overrides so LaunchAllCommand can fire immediately after cancel/interruption.
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
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().shortLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().shortCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().shortRightRpm);
                break;

            case MID:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().midLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().midCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().midRightRpm);
                break;

            case LONG:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().longLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().longCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().longRightRpm);
                break;

            case SHORTSTOP:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().shortstopLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().shortstopCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().shortstopRightRpm);
                break;
        }
    }

    private double hoodPositionForRange() {
        switch (range) {
            case SHORT:
                return rangeConfig().shortHoodPosition;
            case MID:
                return rangeConfig().midHoodPosition;
            case LONG:
                return rangeConfig().longHoodPosition;
            case SHORTSTOP:
                return rangeConfig().shortstopHoodPosition;
            default:
                return rangeConfig().midHoodPosition;
        }
    }

}
