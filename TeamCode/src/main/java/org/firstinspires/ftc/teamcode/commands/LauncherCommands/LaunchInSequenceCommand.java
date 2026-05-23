package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.LaunchInSequenceConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

/**
 * Fires artifacts in the detected obelisk pattern sequence with simultaneous-group optimization.
 *
 * <p>Lifecycle stages (see {@link Stage}):
 * {@code SPINNING_UP → WAITING_FOR_READY → SHOTS_QUEUED → COMPLETED}.
 *
 * <p>Consecutive same-color positions in the rotated motif pattern fire SIMULTANEOUSLY:
 * <ul>
 *   <li>PPG → [[P,P], [G]] → both purples at t=0, green at t=spacingMs</li>
 *   <li>PGP → [[P], [G], [P]] → uses shorter alternating spacing to match total time</li>
 *   <li>GPP → [[G], [P,P]] → green at t=0, both purples at t=spacingMs</li>
 * </ul>
 *
 * <p>Lane selection within a same-color group prioritizes ready lanes first.
 */
public final class LaunchInSequenceCommand {

    // Tuning lives on LauncherSubsystem so Panels shows it under the launcher tree.
    private static final LaunchInSequenceConfig sequenceConfig = LauncherSubsystem.launchInSequenceConfig;

    /** Stages of a single launch-in-sequence run. */
    private enum Stage {
        /** Flywheels just started spinning up; brief settle delay before checking readiness. */
        SPINNING_UP,
        /** Waiting for at least one lane to reach target RPM (or timeout). */
        WAITING_FOR_READY,
        /** All shots have been handed to the launcher's internal queue. */
        SHOTS_QUEUED,
        /** Launcher reported idle and queue empty — command can end. */
        COMPLETED
    }

    private LaunchInSequenceCommand() {}

    public static Command create(LauncherSubsystem launcher,
                                  IntakeSubsystem intake,
                                  boolean spinDownAfterShot) {
        Objects.requireNonNull(launcher, "launcher required");
        Objects.requireNonNull(intake, "intake required");

        // Arrays wrap mutable values so the lambdas below can update them
        // (Java requires lambda captures to be final, so we mutate state via array slots).
        final Stage[] stage = {Stage.SPINNING_UP};
        final EnumSet<LauncherLane> usedLanes = EnumSet.noneOf(LauncherLane.class);
        final boolean[] spinDownApplied = {false};
        final boolean[] shotsQueued = {false};
        final ElapsedTime timer = new ElapsedTime();

        return new CommandBuilder()
                .setStart(() -> {
                    timer.reset();
                    stage[0] = Stage.SPINNING_UP;
                    usedLanes.clear();
                    spinDownApplied[0] = false;
                    shotsQueued[0] = false;
                    launcher.recordLastShotRpms();
                    intake.setGateAllowArtifacts();
                    launcher.spinUpAllLanesToLaunch();
                })
                .setExecute(() -> {
                    switch (stage[0]) {
                        case SPINNING_UP:
                            if (timer.milliseconds() > 100) {
                                stage[0] = Stage.WAITING_FOR_READY;
                            }
                            break;
                        case WAITING_FOR_READY:
                            if (!shotsQueued[0] && canQueueSequence(launcher)) {
                                queueSequenceShots(launcher, intake, usedLanes);
                                shotsQueued[0] = true;
                                stage[0] = Stage.SHOTS_QUEUED;
                            }
                            if (timer.seconds() >= sequenceConfig.timeoutSeconds) {
                                if (!shotsQueued[0]) {
                                    queueSequenceShots(launcher, intake, usedLanes);
                                    shotsQueued[0] = true;
                                }
                                stage[0] = Stage.SHOTS_QUEUED;
                            }
                            break;
                        case SHOTS_QUEUED:
                            if (!launcher.isBusy() && launcher.getQueuedShots() == 0) {
                                stage[0] = Stage.COMPLETED;
                                if (spinDownAfterShot && !spinDownApplied[0]) {
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
                    intake.setGatePreventArtifact();
                    if (spinDownApplied[0]) {
                        launcher.clearOverrides();
                    }
                    boolean interrupted = endCondition == EndCondition.INTERRUPTED;
                    if (interrupted && shotsQueued[0]) {
                        launcher.clearQueue();
                    }
                    if (interrupted && spinDownAfterShot && !spinDownApplied[0]) {
                        launcher.setAllLanesToIdle();
                        spinDownApplied[0] = true;
                    }
                })
                .requiring(launcher);
    }

    private static boolean canQueueSequence(LauncherSubsystem launcher) {
        for (LauncherLane lane : LauncherLane.values()) {
            if (launcher.getLaunchRpm(lane) > 0.0 && launcher.isLaneReady(lane)) {
                return true;
            }
        }
        return false;
    }

    private static void queueSequenceShots(LauncherSubsystem launcher,
                                            IntakeSubsystem intake,
                                            EnumSet<LauncherLane> usedLanes) {
        MotifPattern motif = RobotState.getMotif();
        if (motif == null || motif == MotifPattern.UNKNOWN) {
            motif = MotifPattern.PPG;
        }
        int motifTail = RobotState.getMotifTail();
        ArtifactColor[] rotatedPattern = motif.getRotatedPattern(motifTail);
        List<List<ArtifactColor>> groups = groupConsecutiveColors(rotatedPattern);

        double spacingMs = (groups.size() >= 3)
                ? sequenceConfig.alternatingSpacingMs
                : sequenceConfig.shotSpacingMs;
        double currentDelay = 0.0;

        for (List<ArtifactColor> group : groups) {
            if (group.isEmpty()) continue;
            ArtifactColor neededColor = group.get(0);
            int neededCount = group.size();
            if (neededColor == null || !neededColor.isArtifact()) continue;

            // Prefer lanes loaded with the right color; fall back to any unused lane if none match.
            List<LauncherLane> matching = findUnusedLanes(launcher, intake, usedLanes, neededColor, neededCount);
            if (matching.isEmpty()) {
                matching = findUnusedLanes(launcher, intake, usedLanes, null, neededCount);
                if (matching.isEmpty()) continue;
            }
            for (LauncherLane lane : matching) {
                if (currentDelay > 0.0) {
                    launcher.queueShot(lane, currentDelay);
                } else {
                    launcher.queueShot(lane);
                }
                usedLanes.add(lane);
            }
            currentDelay += spacingMs;
        }

        List<LauncherLane> remaining = new ArrayList<>();
        for (LauncherLane lane : LauncherLane.values()) {
            if (!usedLanes.contains(lane)) remaining.add(lane);
        }
        if (!remaining.isEmpty()) {
            for (LauncherLane lane : remaining) {
                if (currentDelay > 0.0) {
                    launcher.queueShot(lane, currentDelay);
                } else {
                    launcher.queueShot(lane);
                }
                usedLanes.add(lane);
            }
        }

        if (usedLanes.isEmpty()) {
            for (LauncherLane lane : LauncherLane.values()) {
                launcher.queueShot(lane);
            }
        }
    }

    private static List<List<ArtifactColor>> groupConsecutiveColors(ArtifactColor[] pattern) {
        List<List<ArtifactColor>> groups = new ArrayList<>();
        if (pattern == null || pattern.length == 0) return groups;
        List<ArtifactColor> current = new ArrayList<>();
        current.add(pattern[0]);
        for (int i = 1; i < pattern.length; i++) {
            if (pattern[i] == pattern[i - 1]) {
                current.add(pattern[i]);
            } else {
                groups.add(current);
                current = new ArrayList<>();
                current.add(pattern[i]);
            }
        }
        groups.add(current);
        return groups;
    }

    /**
     * Returns up to {@code maxCount} unused lanes, with ready-to-fire lanes first.
     * If {@code colorFilter} is non-null, only lanes whose current artifact matches
     * that color are returned; pass {@code null} to ignore color.
     */
    private static List<LauncherLane> findUnusedLanes(LauncherSubsystem launcher,
                                                      IntakeSubsystem intake,
                                                      EnumSet<LauncherLane> usedLanes,
                                                      ArtifactColor colorFilter,
                                                      int maxCount) {
        List<LauncherLane> matching = new ArrayList<>();
        for (LauncherLane lane : LauncherLane.values()) {
            if (usedLanes.contains(lane)) continue;
            if (colorFilter != null && intake.getLaneColor(lane) != colorFilter) continue;
            matching.add(lane);
        }
        // Ready-to-fire lanes sort first (Boolean.compare(true, false) == 1, so we flip args).
        matching.sort((a, b) -> Boolean.compare(launcher.isLaneReady(b), launcher.isLaneReady(a)));
        return matching.subList(0, Math.min(maxCount, matching.size()));
    }
}
