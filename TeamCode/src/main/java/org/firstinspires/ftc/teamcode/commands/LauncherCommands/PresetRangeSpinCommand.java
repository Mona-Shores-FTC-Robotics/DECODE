package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
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
    private DriveSubsystem drive;
    private LightingSubsystem lighting;
    private Gamepad gamepad;
    private boolean feedbackTriggered = false;

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

    public void setDriveSubsystem(DriveSubsystem drive) {
        this.drive = drive;
    }

    public void setReadyFeedbackTargets(LightingSubsystem lighting, Gamepad gamepad) {
        this.lighting = lighting;
        this.gamepad = gamepad;
    }

    @Override
    public void start() {
        launcher.clearRecoveryDeadlines();
        timer.reset();
        setRpmsForRange();
        launcher.setAllHoodPositions(hoodPositionForRange());
        launcher.spinUpAllLanesToLaunch();
        feedbackTriggered = false;
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
        boolean rpmReady = anyEnabled && allEnabledReady;
        boolean aimReady = drive != null && drive.isAimSettled(2.0);
        boolean stable = drive != null && drive.getRobotSpeedInchesPerSecond() <= DriveSubsystem.STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
        boolean readyWithAim = rpmReady && aimReady && stable;

        RobotState.packet.put("Preset-Spin/All Enabled Ready", rpmReady);
        RobotState.packet.put("Preset-Spin/Aim Ready", aimReady);
        RobotState.packet.put("Preset-Spin/Stable", stable);
        RobotState.packet.put("Preset-Spin/Ready With Aim", readyWithAim);

        triggerReadyFeedback(readyWithAim);
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
     * Optional feedback trigger when RPMs are ready and aim is settled.
     */
    public boolean isReadyWithAim() {
        boolean rpmReady = areEnabledLaunchersReady();
        boolean aimReady = drive != null && drive.isAimSettled(2.0);
        boolean stable = drive != null && drive.getRobotSpeedInchesPerSecond() <= DriveSubsystem.STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
        return rpmReady && aimReady && stable;
    }

    private void triggerReadyFeedback(boolean readyWithAim) {
        if (!readyWithAim || feedbackTriggered) {
            return;
        }
        feedbackTriggered = true;

        if (gamepad != null) {
            gamepad.rumble(200);
        }
        if (lighting != null) {
            lighting.flashAimAligned();
        }
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

            case SHORT_AUTO:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().shortAutoLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().shortAutoCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().shortAutoRightRpm);
                break;

            case FAR_AUTO:
                launcher.setLaunchRpm(LauncherLane.LEFT, rangeConfig().farAutoLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, rangeConfig().farAutoCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, rangeConfig().farAutoRightRpm);
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
            case SHORT_AUTO:
                return rangeConfig().shortAutoHoodPosition;
            case FAR_AUTO:
                return rangeConfig().farAutoHoodPosition;
            default:
                return rangeConfig().midHoodPosition;
        }
    }

}
