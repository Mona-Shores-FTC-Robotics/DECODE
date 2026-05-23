package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

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
 */
// TODO (post-competition): revisit this class design. finishWhenReady=true works as a
// sequential gate in teleop (blocks until RPM reached before next command runs), and
// finishWhenReady=false works as a background RPM holder in deadline groups. These are
// conceptually different enough that they may warrant two separate factory methods
// (e.g. spinUpAndWait vs holdRpm) rather than one boolean parameter. Also reconsider
// whether drive/lighting/gamepad feedback belongs here or in a wrapper.
public final class PresetRangeSpinCommand {

    private static final double READY_LOSS_DEBOUNCE_MS = 300.0;

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
        if (readyLossTimer.milliseconds() >= READY_LOSS_DEBOUNCE_MS) {
            feedbackTriggered[0] = false;
        }
    }

    private static void setRpmsForRange(LauncherSubsystem launcher, LauncherRange range) {
        CommandRangeConfig cfg = rangeConfig();
        switch (range) {
            case SHORT:      applyRpms(launcher, cfg.teleop.shortRange); break;
            case MID:        applyRpms(launcher, cfg.teleop.midRange);   break;
            case LONG: {
                // LONG preset sits at the midpoint of the distance-based
                // interpolation window — one fewer constant to tune.
                CommandRangeConfig.LongRangeSettings l = cfg.teleop.longRange;
                launcher.setLaunchRpm(LauncherLane.LEFT, (l.minLeft + l.maxLeft) / 2.0);
                launcher.setLaunchRpm(LauncherLane.CENTER, (l.minCenter + l.maxCenter) / 2.0);
                launcher.setLaunchRpm(LauncherLane.RIGHT, (l.minRight + l.maxRight) / 2.0);
                break;
            }
            case SHORT_AUTO: applyRpms(launcher, cfg.auto.shortRange); break;
            case MID_AUTO:   applyRpms(launcher, cfg.auto.midRange);   break;
            case FAR_AUTO:   applyRpms(launcher, cfg.auto.farRange);   break;
        }
    }

    /** Pushes a range's three lane RPMs to the launcher. */
    private static void applyRpms(LauncherSubsystem launcher, CommandRangeConfig.RangeSettings r) {
        launcher.setLaunchRpm(LauncherLane.LEFT, r.left);
        launcher.setLaunchRpm(LauncherLane.CENTER, r.center);
        launcher.setLaunchRpm(LauncherLane.RIGHT, r.right);
    }

    private static double hoodPositionForRange(LauncherRange range) {
        CommandRangeConfig cfg = rangeConfig();
        switch (range) {
            case SHORT: return cfg.teleop.shortRange.hood;
            case MID: return cfg.teleop.midRange.hood;
            case LONG: return cfg.teleop.longRange.hood;
            case SHORT_AUTO: return cfg.auto.shortRange.hood;
            case MID_AUTO: return cfg.auto.midRange.hood;
            case FAR_AUTO: return cfg.auto.farRange.hood;
            default: return cfg.teleop.midRange.hood;
        }
    }
}
