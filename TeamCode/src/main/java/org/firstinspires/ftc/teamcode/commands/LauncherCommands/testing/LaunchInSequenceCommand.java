package org.firstinspires.ftc.teamcode.commands.LauncherCommands.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.LaunchInSequenceConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

/**
 * Fires artifacts in the detected obelisk pattern sequence with optimized simultaneous firing.
 *
 * This command:
 * 1. Gets the current motif pattern from RobotState (detected from AprilTag during init)
 * 2. Gets the manually-set motif tail from RobotState (operator visually counts field ramp)
 * 3. Rotates the pattern to complete the existing motif on the field
 * 4. Groups consecutive same-color positions for simultaneous firing
 * 5. Spins up to MID range
 * 6. Fires groups with spacing BETWEEN groups only (not within groups)
 * 7. Spins down to idle
 *
 * KEY OPTIMIZATION: Consecutive same-color artifacts fire SIMULTANEOUSLY!
 * - PPG → Groups: [[P,P], [G]] → Fire both P at t=0, G at t=500ms (50% faster!)
 * - PGP → Groups: [[P], [G], [P]] → Fire P, wait, G, wait, P (1000ms total)
 * - GPP → Groups: [[G], [P,P]] → Fire G at t=0, both P at t=500ms (50% faster!)
 *
 * Lane selection prioritizes ready lanes first for additional speed optimization.
 *
 * IMPORTANT: The motif tail is MANUALLY set by the operator (not automatically calculated).
 * The operator visually counts artifacts in the field ramp and updates the tail using
 * dpad buttons:
 * - Dpad Left (tail=0): 0, 3, 6, ... artifacts in field ramp
 * - Dpad Up (tail=1): 1, 4, 7, ... artifacts in field ramp
 * - Dpad Right (tail=2): 2, 5, 8, ... artifacts in field ramp
 *
 * Example: If detected pattern is PPG and operator set tail=1:
 * - Motif tail = 1 (manually set by operator)
 * - Rotated pattern = PGP (shifted left by 1 from PPG)
 * - Groups: [[P], [G], [P]]
 * - Fires Purple, wait 500ms, Green, wait 500ms, Purple (no simultaneous optimization for PGP)
 */
@Configurable
public class LaunchInSequenceCommand extends Command {

    public static LaunchInSequenceConfig sequenceConfig = new LaunchInSequenceConfig();

    private enum Stage {
        SPINNING_UP,
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;

    private final ElapsedTime timer = new ElapsedTime();
    private final EnumSet<LauncherLane> usedLanes = EnumSet.noneOf(LauncherLane.class);

    private Stage stage = Stage.SPINNING_UP;
    private boolean spinDownApplied = false;
    private boolean shotsQueued = false;

    /**
     * Creates command that fires in obelisk pattern sequence with motif tail offset.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed roller control and artifact tracking)
     */
    public LaunchInSequenceCommand(LauncherSubsystem launcher,
                                    IntakeSubsystem intake) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = Objects.requireNonNull(intake, "intake required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.SPINNING_UP;
        usedLanes.clear();
        spinDownApplied = false;
        shotsQueued = false;

        // gate Allow
        if (intake != null) {
            intake.setGateAllowArtifacts();
        }

        // Set RPMs for sequence firing (MID range by default)
        launcher.setLaunchRpm(LauncherLane.LEFT, sequenceConfig.sequenceLeftRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, sequenceConfig.sequenceCenterRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, sequenceConfig.sequenceRightRpm);

        // Set hoods to MID range
        launcher.setAllHoodsForRange(LauncherRange.MID);

        // Spin up to target
        launcher.spinUpAllLanesToLaunch();
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
                // Check if we can queue shots yet
                if (!shotsQueued && canQueueSequence()) {
                    queueSequenceShots();
                    shotsQueued = true;
                    stage = Stage.SHOTS_QUEUED;
                }

                // Timeout safety
                if (timer.seconds() >= sequenceConfig.timeoutSeconds) {
                    if (!shotsQueued) {
                        queueSequenceShots(); // Try to queue anyway
                        shotsQueued = true;
                    }
                    stage = Stage.SHOTS_QUEUED;
                }
                break;

            case SHOTS_QUEUED:
                if (!launcher.isBusy() && launcher.getQueuedShots() == 0) {
                    stage = Stage.COMPLETED;
                    if (!spinDownApplied) {
                        launcher.setAllLanesToIdle();
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
        if (interrupted && shotsQueued) {
            launcher.clearQueue();
        }

        // Spin down if not already done
        if (interrupted && !spinDownApplied) {
            launcher.setAllLanesToIdle();
            spinDownApplied = true;
        }
    }

    /**
     * Checks if enough lanes are ready to start queuing the sequence.
     * Waits for at least one enabled lane to be ready.
     */
    private boolean canQueueSequence() {
        for (LauncherLane lane : LauncherLane.values()) {
            if (launcher.getLaunchRpm(lane) > 0.0 && launcher.isLaneReady(lane)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Queues shots with optimized simultaneous firing for consecutive same-color artifacts.
     *
     * Key optimizations:
     * - Groups consecutive same-color positions (e.g., PPG → [[P,P], [G]])
     * - Fires all artifacts in each group SIMULTANEOUSLY (no delay within group)
     * - Only adds delay BETWEEN groups
     * - Prioritizes ready lanes when multiple lanes have the same color
     * - Example: PPG fires both purples at t=0, green at t=500ms (50% faster!)
     *
     * Handles imperfect matches gracefully:
     * - If pattern needs 2 purples but only 1 available, fires that 1
     * - Fires all remaining unmatched artifacts simultaneously at the end
     */
    private void queueSequenceShots() {
        // Get current motif pattern from RobotState
        MotifPattern motif = RobotState.getMotif();
        if (motif == null || motif == MotifPattern.UNKNOWN) {
            // Default to PPG if no pattern detected
            motif = MotifPattern.PPG;
        }

        // Get manually-set motif tail from RobotState
        int motifTail = RobotState.getMotifTail();

        // Get rotated pattern based on motif tail
        ArtifactColor[] rotatedPattern = motif.getRotatedPattern(motifTail);

        // Group consecutive same-color positions
        List<List<ArtifactColor>> groups = groupConsecutiveColors(rotatedPattern);

        double currentDelay = 0.0;

        // Phase 1: Match and fire each group simultaneously
        for (List<ArtifactColor> group : groups) {
            if (group.isEmpty()) {
                continue;
            }

            ArtifactColor neededColor = group.get(0); // All same color in group
            int neededCount = group.size();

            // Skip non-artifacts
            if (neededColor == null || !neededColor.isArtifact()) {
                continue;
            }

            // Find unused lanes with this color (prioritize ready lanes, up to neededCount)
            List<LauncherLane> matchingLanes = findUnusedLanesWithColor(neededColor, neededCount);

            if (matchingLanes.isEmpty()) {
                // Can't match this group - continue to try remaining groups
                continue;
            }

            // Fire ALL lanes in this group SIMULTANEOUSLY at currentDelay
            for (LauncherLane lane : matchingLanes) {
                if (currentDelay > 0.0) {
                    launcher.queueShot(lane, currentDelay);
                } else {
                    launcher.queueShot(lane);
                }
                usedLanes.add(lane);
            }

            // Only increment delay AFTER the entire group (not between individual shots)
            currentDelay += sequenceConfig.shotSpacingMs;
        }

        // Phase 2: Fire all remaining unused lanes SIMULTANEOUSLY
        List<LauncherLane> remainingLanes = new ArrayList<>();
        for (LauncherLane lane : LauncherLane.values()) {
            if (!usedLanes.contains(lane)) {
                ArtifactColor laneColor = intake.getLaneColor(lane);
                if (laneColor != null && laneColor.isArtifact()) {
                    remainingLanes.add(lane);
                }
            }
        }

        // Fire all remaining lanes simultaneously (if any)
        if (!remainingLanes.isEmpty()) {
            for (LauncherLane lane : remainingLanes) {
                if (currentDelay > 0.0) {
                    launcher.queueShot(lane, currentDelay);
                } else {
                    launcher.queueShot(lane);
                }
                usedLanes.add(lane);
            }
        }
    }

    /**
     * Groups consecutive same-color positions in the pattern.
     *
     * Example: [P, P, G] → [[P, P], [G]]
     * Example: [P, G, P] → [[P], [G], [P]]
     * Example: [G, P, P] → [[G], [P, P]]
     *
     * @param pattern The obelisk pattern
     * @return List of groups, where each group contains consecutive same-color positions
     */
    private List<List<ArtifactColor>> groupConsecutiveColors(ArtifactColor[] pattern) {
        List<List<ArtifactColor>> groups = new ArrayList<>();
        if (pattern == null || pattern.length == 0) {
            return groups;
        }

        List<ArtifactColor> currentGroup = new ArrayList<>();
        currentGroup.add(pattern[0]);

        for (int i = 1; i < pattern.length; i++) {
            if (pattern[i] == pattern[i - 1]) {
                // Same color - add to current group
                currentGroup.add(pattern[i]);
            } else {
                // Different color - save current group and start new one
                groups.add(currentGroup);
                currentGroup = new ArrayList<>();
                currentGroup.add(pattern[i]);
            }
        }

        // Add final group
        groups.add(currentGroup);
        return groups;
    }

    /**
     * Finds unused lanes with the specified color, prioritizing ready lanes first.
     *
     * This optimization ensures that when we have multiple lanes with the same color,
     * we fire the lanes that are already spun up first, reducing wait time.
     *
     * @param color The artifact color to search for
     * @param maxCount Maximum number of lanes to return
     * @return List of matching lanes (up to maxCount), sorted by readiness
     */
    private List<LauncherLane> findUnusedLanesWithColor(ArtifactColor color, int maxCount) {
        List<LauncherLane> matching = new ArrayList<>();

        // Find all unused lanes with this color
        for (LauncherLane lane : LauncherLane.values()) {
            if (usedLanes.contains(lane)) {
                continue;
            }

            ArtifactColor laneColor = intake.getLaneColor(lane);
            if (laneColor == color) {
                matching.add(lane);
            }
        }

        // Sort by readiness (ready lanes first)
        matching.sort((lane1, lane2) -> {
            boolean ready1 = launcher.isLaneReady(lane1);
            boolean ready2 = launcher.isLaneReady(lane2);
            if (ready1 && !ready2) {
                return -1; // ready1 comes first
            }
            if (!ready1 && ready2) {
                return 1; // ready2 comes first
            }
            return 0; // same readiness
        });

        // Return up to maxCount lanes
        return matching.subList(0, Math.min(maxCount, matching.size()));
    }
}
