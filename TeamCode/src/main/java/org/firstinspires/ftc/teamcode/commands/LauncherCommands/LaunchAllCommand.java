package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.EnumSet;
import java.util.Objects;

/**
 * Launches every lane as soon as the flywheel speeds reach target.
 *
 * Assumes launch RPMs are already set via {@link LauncherSubsystem#setLaunchRpm}.
 * If no RPMs are set, lanes are treated as disabled (RPM = 0) and won't fire.
 *
 * Ported from NextFTC {@code extends Command} to an Ivy static factory.
 */
public final class LaunchAllCommand {

    // Tracks where we are in the launch sequence.
    private enum Stage { WAITING, SHOTS_QUEUED, COMPLETED }

    /** Safety timeout (ms): if the launch never completes (a lane never reports ready, or a
     *  queued shot never clears), give up after this long so an auto can't hang on the launch.
     *  Set longer than a normal launch (spin-up + fire all lanes + recovery). */
    public static double launchTimeoutMs = 5000;

    /** Number of lanes that must reach target RPM before the launch is treated as ready: once
     *  this many are at speed, ALL enabled lanes fire (a lane that can't quite hit target still
     *  fires, at its current speed). With one weak flywheel, 2 means the launch no longer stalls
     *  waiting on it. Set to 3 to require every lane (the old all-must-be-ready behavior). */
    public static int minLanesReady = 2;

    private LaunchAllCommand() {}

    public static Command create(LauncherSubsystem launcher,
                                  IntakeSubsystem intake,
                                  boolean spinDownAfterShot) {
        Objects.requireNonNull(launcher, "launcher required");

        // Arrays wrap mutable values so lambdas (which require final captures) can update them.
        final Stage[] stage = {Stage.WAITING};
        final EnumSet<LauncherLane> queuedLanes = EnumSet.noneOf(LauncherLane.class);
        final boolean[] spinDownApplied = {false};
        final ElapsedTime timer = new ElapsedTime();

        return new CommandBuilder()
                .setStart(() -> {
                    timer.reset();
                    stage[0] = Stage.WAITING;
                    queuedLanes.clear();
                    spinDownApplied[0] = false;
                    launcher.recordLastShotRpms();
                    launcher.spinUpAllLanesToLaunch();
                    if (intake != null) intake.setGateAllowArtifacts();
                })
                .setExecute(() -> {
                    // Safety timeout: if the launch never completes (a lane never reaches target
                    // RPM, or a queued shot never clears), give up so the auto can't hang here.
                    // Clear pending shots so they don't fire during the next move.
                    if (stage[0] != Stage.COMPLETED && timer.milliseconds() >= launchTimeoutMs) {
                        launcher.clearQueue();
                        stage[0] = Stage.COMPLETED;
                        if (spinDownAfterShot && !spinDownApplied[0]) {
                            launcher.clearOverrides();
                            launcher.setAllLanesToIdle();
                            spinDownApplied[0] = true;
                        }
                        return;
                    }
                    switch (stage[0]) {
                        case WAITING: {
                            checkLaneReadiness(launcher, queuedLanes);
                            if (queuedLanes.size() >= minLanesReady) {
                                // Enough lanes at target → treat the whole launch as ready and
                                // fire EVERY enabled lane, including one that hasn't reached target
                                // (it still fires, at its current speed) so no shot is skipped.
                                for (LauncherLane lane : LauncherLane.values()) {
                                    if (launcher.getLaunchRpm(lane) > 0.0 && !queuedLanes.contains(lane)) {
                                        launcher.queueShot(lane);
                                        queuedLanes.add(lane);
                                    }
                                }
                                stage[0] = Stage.SHOTS_QUEUED;
                            }
                            break;
                        }
                        case SHOTS_QUEUED:
                            if (!launcher.isBusy() && launcher.getQueuedShots() == 0) {
                                stage[0] = Stage.COMPLETED;
                                if (spinDownAfterShot && !spinDownApplied[0]) {
                                    launcher.clearOverrides();
                                    launcher.setAllLanesToIdle();
                                    spinDownApplied[0] = true;
                                }
                            }
                            break;
                        default:
                            break;
                    }
                })
                .setDone(() -> stage[0] == Stage.COMPLETED)
                .setEnd(endCondition -> {
                    if (intake != null) intake.setGatePreventArtifact();
                    if (endCondition == EndCondition.INTERRUPTED && !queuedLanes.isEmpty()) {
                        launcher.clearQueue();
                    }
                    if (endCondition == EndCondition.INTERRUPTED && spinDownAfterShot && !spinDownApplied[0]) {
                        launcher.clearOverrides();
                        launcher.setAllLanesToIdle();
                        spinDownApplied[0] = true;
                    }
                })
                .requiring(launcher);
    }

    private static void checkLaneReadiness(LauncherSubsystem launcher, EnumSet<LauncherLane> queuedLanes) {
        for (LauncherLane lane : LauncherLane.values()) {
            if (queuedLanes.contains(lane)) continue;
            boolean ready = launcher.isLaneReady(lane);
            if (TelemetrySettings.isVerbose()) {
                double launchRpm = launcher.getLaunchRpm(lane);
                double currentRpm = launcher.getCurrentRpm(lane);
                double targetRpm = launcher.getTargetRpm(lane);
                RobotState.packet.put("Commands/LaunchAll/Lane " + lane.name() + "/Ready", ready);
                RobotState.packet.put("Commands/LaunchAll/Lane " + lane.name() + "/Launch RPM", launchRpm);
                RobotState.packet.put("Commands/LaunchAll/Lane " + lane.name() + "/Current RPM", currentRpm);
                RobotState.packet.put("Commands/LaunchAll/Lane " + lane.name() + "/Target RPM", targetRpm);
            }
            if (ready) {
                launcher.queueShot(lane);
                queuedLanes.add(lane);
            }
        }
    }
}
