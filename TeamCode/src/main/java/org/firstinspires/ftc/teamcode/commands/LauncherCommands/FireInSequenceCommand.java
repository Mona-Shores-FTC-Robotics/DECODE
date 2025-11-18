package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

/**
 * Fires artifacts in the detected obelisk pattern sequence with motif tail offset.
 *
 * This command:
 * 1. Gets the current motif pattern from RobotState (detected from AprilTag during init)
 * 2. Gets the manually-set motif tail from RobotState (operator visually counts field ramp)
 * 3. Rotates the pattern to complete the existing motif on the field
 * 4. Spins up to MID range
 * 5. Fires lanes in the rotated pattern sequence with configurable spacing
 * 6. Spins down to idle
 *
 * If no pattern has been detected, defaults to PPG pattern.
 *
 * IMPORTANT: The motif tail is MANUALLY set by the operator (not automatically calculated).
 * The operator visually counts artifacts in the field ramp and updates the tail using
 * increment/decrement buttons:
 * - Tail = 0 when field ramp has 0, 3, 6, ... artifacts
 * - Tail = 1 when field ramp has 1, 4, 7, ... artifacts
 * - Tail = 2 when field ramp has 2, 5, 8, ... artifacts
 *
 * Example: If detected pattern is PPG and operator set tail=1:
 * - Motif tail = 1 (manually set by operator)
 * - Rotated pattern = PGP (shifted left by 1 from PPG)
 * - Fires Purple, Green, Purple to complete the existing motif
 */
@Configurable
public class FireInSequenceCommand extends Command {

    @Configurable
    public static class SequenceConfig {
        /** Milliseconds between shots in sequence */
        public double shotSpacingMs = 500.0;

        /** RPM for left lane in sequence mode */
        public double sequenceLeftRpm = 2550;

        /** RPM for center lane in sequence mode */
        public double sequenceCenterRpm = 2550;

        /** RPM for right lane in sequence mode */
        public double sequenceRightRpm = 2550;

        /** Timeout in seconds before giving up on spin-up */
        public double timeoutSeconds = 8.0;
    }

    public static SequenceConfig sequenceConfig = new SequenceConfig();

    private enum Stage {
        SPINNING_UP,
        WAITING_FOR_READY,
        SHOTS_QUEUED,
        COMPLETED
    }

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final LauncherCoordinator coordinator;
    private final ManualSpinController manualSpinController;

    private final ElapsedTime timer = new ElapsedTime();
    private final EnumSet<LauncherLane> usedLanes = EnumSet.noneOf(LauncherLane.class);

    private Stage stage = Stage.SPINNING_UP;
    private boolean manualSpinActive = false;
    private boolean spinDownApplied = false;
    private boolean shotsQueued = false;

    /**
     * Creates command that fires in obelisk pattern sequence with motif tail offset.
     *
     * @param launcher The launcher subsystem
     * @param intake The intake subsystem (for prefeed roller control)
     * @param coordinator The launcher coordinator (tracks lane colors and artifact count)
     * @param manualSpinController Controller for manual spin state
     */
    public FireInSequenceCommand(LauncherSubsystem launcher,
                                  IntakeSubsystem intake,
                                  LauncherCoordinator coordinator,
                                  ManualSpinController manualSpinController) {
        this.launcher = Objects.requireNonNull(launcher, "launcher required");
        this.intake = intake; // Nullable - robot may not have prefeed roller
        this.coordinator = Objects.requireNonNull(coordinator, "coordinator required");
        this.manualSpinController = Objects.requireNonNull(manualSpinController, "manualSpinController required");
        requires(launcher);
        setInterruptible(true);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.SPINNING_UP;
        usedLanes.clear();
        manualSpinActive = true;
        spinDownApplied = false;
        shotsQueued = false;

        // Enter manual spin mode to prevent automation from changing RPMs
        manualSpinController.enterManualSpin();

        // Activate prefeed roller in forward direction to help feed
        if (intake != null) {
            intake.setPrefeedForward();
        }

        // Set RPMs for sequence firing (MID range by default)
        launcher.setLaunchRpm(LauncherLane.LEFT, sequenceConfig.sequenceLeftRpm);
        launcher.setLaunchRpm(LauncherLane.CENTER, sequenceConfig.sequenceCenterRpm);
        launcher.setLaunchRpm(LauncherLane.RIGHT, sequenceConfig.sequenceRightRpm);

        // Set hoods to MID range
        launcher.setAllHoodsForRange(LauncherRange.MID);

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
            intake.setPrefeedReverse();
        }

        // Clear RPM overrides to return to default values
        launcher.clearOverrides();

        // Exit manual spin mode
        if (manualSpinActive) {
            manualSpinController.exitManualSpin();
            manualSpinActive = false;
        }

        // Clear queue if interrupted
        if (interrupted && shotsQueued) {
            launcher.clearQueue();
        }

        // Spin down if not already done
        if (interrupted && !spinDownApplied) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
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
     * Queues shots in the obelisk pattern sequence with motif tail offset.
     * Uses the same logic as LaunchObeliskPatternCommand but integrated with spin-up.
     */
    private void queueSequenceShots() {
        // Get current motif pattern from RobotState
        MotifPattern motif = RobotState.getMotif();
        if (motif == null || motif == MotifPattern.UNKNOWN) {
            // Default to PPG if no pattern detected
            motif = MotifPattern.PPG;
        }

        // Get manually-set motif tail from RobotState
        // (Operator visually counts artifacts in field ramp and updates this value)
        int motifTail = RobotState.getMotifTail();

        // Get rotated pattern based on motif tail
        ArtifactColor[] rotatedPattern = motif.getRotatedPattern(motifTail);
        List<ArtifactColor> desiredPattern = Arrays.asList(rotatedPattern);

        // Queue shots in pattern sequence (same logic as LaunchObeliskPatternCommand)
        double currentDelay = 0.0;

        for (ArtifactColor neededColor : desiredPattern) {
            // Skip NONE, UNKNOWN, and BACKGROUND - they're not real artifacts
            if (neededColor == null || !neededColor.isArtifact()) {
                continue;
            }

            // Find a lane that has this color and hasn't been used yet
            LauncherLane matchingLane = findUnusedLaneWithColor(neededColor);

            if (matchingLane == null) {
                // Can't continue the pattern - stop here
                break;
            }

            // Queue this shot with the appropriate delay
            if (currentDelay > 0.0) {
                launcher.queueShot(matchingLane, currentDelay);
            } else {
                launcher.queueShot(matchingLane);
            }

            // Mark this lane as used and increment delay for next shot
            usedLanes.add(matchingLane);
            currentDelay += sequenceConfig.shotSpacingMs;
        }
    }

    /**
     * Finds a lane that contains the specified color and hasn't been used yet.
     *
     * @param color The artifact color to search for
     * @return The matching lane, or null if none found
     */
    private LauncherLane findUnusedLaneWithColor(ArtifactColor color) {
        for (LauncherLane lane : LauncherLane.values()) {
            // Skip lanes we've already used
            if (usedLanes.contains(lane)) {
                continue;
            }

            // Check if this lane has the color we need
            ArtifactColor laneColor = coordinator.getLaneColor(lane);
            if (laneColor == color) {
                return lane;
            }
        }
        return null;
    }
}
