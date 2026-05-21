package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotProfile;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Spins up launchers to preset (short/mid/long) RPM and hood targets and waits until ready.
 * Ported from NextFTC to an Ivy static factory.
 */
@Configurable
public final class PresetRangeSpinCommand {

    private PresetRangeSpinCommand() {}

    public static CommandRangeConfig rangeConfig() {
        return RobotProfile.forCurrent().commandRange;
    }

    public static Command create(LauncherSubsystem launcher,
                                  LauncherRange range,
                                  boolean finishWhenReady,
                                  DriveSubsystem drive,
                                  LightingSubsystem lighting,
                                  Gamepad gamepad) {
        Objects.requireNonNull(launcher, "launcher required");
        Objects.requireNonNull(range, "range required");

        final ElapsedTime timer = new ElapsedTime();
        final ElapsedTime readyLossTimer = new ElapsedTime();
        // Array wraps a mutable value so the lambda below can update it (Java requires final captures).
        final boolean[] feedbackTriggered = {false};

        return new CommandBuilder()
                .setStart(() -> {
                    launcher.clearRecoveryDeadlines();
                    timer.reset();
                    readyLossTimer.reset();
                    setRpmsForRange(launcher, range);
                    launcher.setAllHoodPositions(hoodPositionForRange(range));
                    launcher.spinUpAllLanesToLaunch();
                    feedbackTriggered[0] = false;
                })
                .setExecute(() -> {
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
                        if (TelemetrySettings.isVerbose()) {
                            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Enabled", enabled);
                            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Target RPM", launchRpm);
                            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Ready", ready);
                            RobotState.packet.put("Preset-Spin/Lane " + lane.name() + "/Current RPM", launcher.getCurrentRpm(lane));
                        }
                    }
                    boolean rpmReady = anyEnabled && allEnabledReady;
                    boolean aimReady = drive != null && drive.isAimSettled(2.0);
                    boolean stable = drive != null
                            && drive.getRobotSpeedInchesPerSecond() <= DriveSubsystem.STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
                    boolean readyWithAim = rpmReady && aimReady && stable;

                    if (TelemetrySettings.isVerbose()) {
                        RobotState.packet.put("Preset-Spin/Range", range.name());
                        RobotState.packet.put("Preset-Spin/Finish When Ready", finishWhenReady);
                        RobotState.packet.put("Preset-Spin/All Enabled Ready", rpmReady);
                        RobotState.packet.put("Preset-Spin/Aim Ready", aimReady);
                        RobotState.packet.put("Preset-Spin/Stable", stable);
                        RobotState.packet.put("Preset-Spin/Ready With Aim", readyWithAim);
                    }
                    triggerReadyFeedback(readyWithAim, feedbackTriggered, readyLossTimer, lighting, gamepad);
                })
                .setDone(() -> {
                    if (!finishWhenReady) return false;
                    return areEnabledLaunchersReady(launcher) || timer.seconds() >= rangeConfig().timeoutSeconds;
                })
                .requiring(launcher);
    }

    private static boolean areEnabledLaunchersReady(LauncherSubsystem launcher) {
        for (LauncherLane lane : LauncherLane.values()) {
            double launchRpm = launcher.getLaunchRpm(lane);
            if (launchRpm <= 0.0) continue;
            if (!launcher.isLaneReady(lane)) return false;
        }
        return true;
    }

    private static void triggerReadyFeedback(boolean readyWithAim,
                                              boolean[] feedbackTriggered,
                                              ElapsedTime readyLossTimer,
                                              LightingSubsystem lighting,
                                              Gamepad gamepad) {
        if (readyWithAim) {
            readyLossTimer.reset();
            if (feedbackTriggered[0]) return;
            feedbackTriggered[0] = true;
            if (gamepad != null) gamepad.rumble(200);
            if (lighting != null) lighting.flashAimAligned();
            return;
        }
        if (readyLossTimer.milliseconds() >= LauncherSubsystem.READY_LOSS_DEBOUNCE_MS) {
            feedbackTriggered[0] = false;
        }
    }

    private static void setRpmsForRange(LauncherSubsystem launcher, LauncherRange range) {
        CommandRangeConfig cfg = rangeConfig();
        switch (range) {
            case SHORT:
                launcher.setLaunchRpm(LauncherLane.LEFT, cfg.shortLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, cfg.shortCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, cfg.shortRightRpm);
                break;
            case MID:
                launcher.setLaunchRpm(LauncherLane.LEFT, cfg.midLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, cfg.midCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, cfg.midRightRpm);
                break;
            case LONG:
                launcher.setLaunchRpm(LauncherLane.LEFT, cfg.longLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, cfg.longCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, cfg.longRightRpm);
                break;
            case SHORT_AUTO:
                launcher.setLaunchRpm(LauncherLane.LEFT, cfg.shortAutoLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, cfg.shortAutoCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, cfg.shortAutoRightRpm);
                break;
            case FAR_AUTO:
                launcher.setLaunchRpm(LauncherLane.LEFT, cfg.farAutoLeftRpm);
                launcher.setLaunchRpm(LauncherLane.CENTER, cfg.farAutoCenterRpm);
                launcher.setLaunchRpm(LauncherLane.RIGHT, cfg.farAutoRightRpm);
                break;
        }
    }

    private static double hoodPositionForRange(LauncherRange range) {
        CommandRangeConfig cfg = rangeConfig();
        switch (range) {
            case SHORT: return cfg.shortHoodPosition;
            case MID: return cfg.midHoodPosition;
            case LONG: return cfg.longHoodPosition;
            case SHORT_AUTO: return cfg.shortAutoHoodPosition;
            case FAR_AUTO: return cfg.farAutoHoodPosition;
            default: return cfg.midHoodPosition;
        }
    }
}
