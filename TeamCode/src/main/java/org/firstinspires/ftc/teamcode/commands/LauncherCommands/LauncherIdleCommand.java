package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotProfile;

import java.util.Objects;

/**
 * Persistent background command that manages launcher idle state between shots.
 *
 * <p>Idles at {@link LauncherSubsystem#launcherIdleConfig}.lowIdleRpm when no artifacts
 * are loaded. Automatically spins up to the last-shot RPM (or MID range as fallback)
 * when the intake loads artifacts into any lane — so the robot arrives at the shooting
 * position already at speed.
 *
 * <p>Wire with {@code bindings.when(() -> true).whileTrue(...)} so it reschedules
 * automatically after being preempted by aim or fire commands.
 */
public final class LauncherIdleCommand {

    private LauncherIdleCommand() {}

    public static Command create(LauncherSubsystem launcher, IntakeSubsystem intake) {
        Objects.requireNonNull(launcher, "launcher required");
        Objects.requireNonNull(intake, "intake required");

        final boolean[] spinningUp = {false};

        return new CommandBuilder()
                .setStart(() -> {
                    spinningUp[0] = false;
                    applyLowIdle(launcher);
                })
                .setExecute(() -> {
                    boolean anyLoaded = anyLaneLoaded(intake);
                    if (anyLoaded && !spinningUp[0]) {
                        spinningUp[0] = true;
                        spinUpToLastShot(launcher);
                    } else if (!anyLoaded && spinningUp[0]) {
                        spinningUp[0] = false;
                        applyLowIdle(launcher);
                    }
                })
                .setDone(() -> false)
                .requiring(launcher);
    }

    private static boolean anyLaneLoaded(IntakeSubsystem intake) {
        for (LauncherLane lane : LauncherLane.values()) {
            if (intake.getLaneColor(lane).isArtifact()) return true;
        }
        return false;
    }

    private static void spinUpToLastShot(LauncherSubsystem launcher) {
        CommandRangeConfig cfg = RobotProfile.forCurrent().commandRange;
        for (LauncherLane lane : LauncherLane.values()) {
            double rpm = launcher.getLastShotRpm(lane);
            if (rpm <= 0.0) {
                switch (lane) {
                    case LEFT:   rpm = cfg.midLeftRpm;   break;
                    case CENTER: rpm = cfg.midCenterRpm; break;
                    default:     rpm = cfg.midRightRpm;  break;
                }
            }
            launcher.setLaunchRpm(lane, rpm);
        }
        launcher.spinUpAllLanesToLaunch();
    }

    private static void applyLowIdle(LauncherSubsystem launcher) {
        double low = LauncherSubsystem.launcherIdleConfig.lowIdleRpm;
        for (LauncherLane lane : LauncherLane.values()) {
            launcher.setIdleRpm(lane, low);
        }
        launcher.setAllLanesToIdle();
    }
}
