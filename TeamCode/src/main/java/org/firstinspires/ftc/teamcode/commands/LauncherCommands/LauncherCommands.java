package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import java.util.Arrays;
import java.util.List;

/**
 * Convenience factory for launcher-related commands alongside immediate queue helpers used by
 * legacy binding paths.
 */
@SuppressWarnings("UnusedReturnValue")
@Configurable
public class LauncherCommands {

    public static final double DEFAULT_BURST_SPACING_MS = 300;

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;

    public LauncherCommands(LauncherSubsystem launcher,
                           IntakeSubsystem intake) {
        this.launcher = launcher;
        this.intake = intake;
    }

    public LaunchLaneCommand launchLane(LauncherLane lane) {
        return new LaunchLaneCommand(launcher , lane);
    }

    public LaunchLaneCommand launchLeft() {
        return launchLane(LauncherLane.LEFT);
    }

    public LaunchLaneCommand launchCenter() {
        return launchLane(LauncherLane.CENTER);
    }

    public LaunchLaneCommand launchRight() {
        return launchLane(LauncherLane.RIGHT);
    }

    /**
     * Immediate helper used by bindings that still trigger launcher actions imperatively.
     */
    public void queueLane(LauncherLane lane) {
        if (lane == null) {
            return;
        }
        launcher.queueShot(lane);
    }

    public void queueDetectedBurst(double spacingMs) {
        launcher.queueBurstAll();
    }

    public void cancelAll() {
        launcher.clearQueue();
    }

    public void setSpinModeToFull() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
    }

    public void setSpinModeToIdle() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.IDLE);
    }

    public void setSpinModeToOff() {
        launcher.setSpinMode(LauncherSubsystem.SpinMode.OFF);
    }

    public SetFeederPositionCommand setLeftFeederToLoad() {
        return new SetFeederPositionCommand(launcher, LauncherLane.LEFT, true);
    }

    public SetFeederPositionCommand setLeftFeederToFire() {
        return new SetFeederPositionCommand(launcher, LauncherLane.LEFT, false);
    }

    public SpinUpUntilReadyCommand spinUpUntilReady() {
        return new SpinUpUntilReadyCommand(launcher);
    }

    /**
     * Phase 1: Spin up to position-specific RPM (tunable in Dashboard)
     * @param position Field position we're launching from
     */
    public LaunchAtPositionCommand spinUpForPosition(org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint position) {
        return new LaunchAtPositionCommand(launcher, position);
    }

    /**
     * Phase 1: Spin up to explicit RPM (for testing)
     * @param targetRpm Desired RPM for both enabled launchers
     */
    public LaunchAtPositionCommand spinUpToRpm(double targetRpm) {
        return new LaunchAtPositionCommand(launcher, targetRpm);
    }

    /**
     * Fires all lanes at SHORT range (~2700 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a short-range shot
     */
    public LaunchAllAtPresetRangeCommand fireAllShortRange() {
        return new LaunchAllAtPresetRangeCommand(launcher, intake, LauncherRange.SHORT, true);
    }

    /**
     * Fires all lanes at MID range (~3600 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a mid-range shot
     */
    public LaunchAllAtPresetRangeCommand fireAllMidRange() {
        return new LaunchAllAtPresetRangeCommand(launcher, intake, LauncherRange.MID, true);
    }

    /**
     * Fires all lanes at LONG range (~4200 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a long-range shot
     */
    public LaunchAllAtPresetRangeCommand fireAllLongRange() {
        return new LaunchAllAtPresetRangeCommand(launcher, intake, LauncherRange.LONG, true);
    }

    /**
     * Generic range-based firing command.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @param range The shooting range (SHORT, MID, or LONG)
     * @param spinDownAfterShot Whether to spin down to idle after firing
     * @return Command that executes the range-based shot
     */
    public LaunchAllAtPresetRangeCommand launchAllAtRangePreset(LauncherRange range, boolean spinDownAfterShot) {
        return new LaunchAllAtPresetRangeCommand(launcher, intake, range, spinDownAfterShot);
    }

    /**
     * Fires all lanes at SHORT range (~2700 RPM).
     * Spins up, fires all three lanes, then spins down to idle.
     * Activates prefeed roller in forward direction to help feed.
     *
     * @return Command that executes a short-range shot
     */
    public LaunchAllCommand fireAll(boolean spinDownAfterShot) {
        return new LaunchAllCommand(launcher, intake, spinDownAfterShot);
    }

    // ========== Obelisk Pattern Commands ==========

    /**
     * Fires artifacts in a specific color sequence for Obelisk scoring.
     * Looks at what's loaded and fires as many matching artifacts as possible (1-3).
     *
     * @param pattern The desired color sequence (e.g., [PURPLE, PURPLE, GREEN])
     * @param spacingMs Milliseconds between shots
     * @return Command that fires matching artifacts in pattern order
     */
    public LaunchObeliskPatternCommand launchObeliskPattern(List<ArtifactColor> pattern, double spacingMs) {
        return new LaunchObeliskPatternCommand(launcher, intake, pattern, spacingMs);
    }

    /**
     * Fires artifacts in a specific color sequence for Obelisk scoring with default spacing.
     *
     * @param pattern The desired color sequence
     * @return Command that fires matching artifacts in pattern order
     */
    public LaunchObeliskPatternCommand launchObeliskPattern(List<ArtifactColor> pattern) {
        return launchObeliskPattern(pattern, DEFAULT_BURST_SPACING_MS);
    }

    /**
     * Obelisk PPG pattern: Purple, Purple, Green (Gold).
     * Fires matching artifacts in this specific order.
     */
    public LaunchObeliskPatternCommand launchObeliskPPG() {
        return launchObeliskPattern(
            Arrays.asList(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN),
            DEFAULT_BURST_SPACING_MS
        );
    }

    /**
     * Obelisk PGP pattern: Purple, Green (Gold), Purple.
     * Fires matching artifacts in this specific order.
     */
    public LaunchObeliskPatternCommand launchObeliskPGP() {
        return launchObeliskPattern(
            Arrays.asList(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE),
            DEFAULT_BURST_SPACING_MS
        );
    }

    /**
     * Obelisk GPP pattern: Green (Gold), Purple, Purple.
     * Fires matching artifacts in this specific order.
     */
    public LaunchObeliskPatternCommand launchObeliskGPP() {
        return launchObeliskPattern(
            Arrays.asList(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE),
            DEFAULT_BURST_SPACING_MS
        );
    }

    // ========== Sequence Mode Commands ==========

    /**
     * Fires in obelisk pattern sequence with motif tail offset.
     *
     * Gets the detected motif pattern from RobotState, calculates the motif tail
     * offset based on artifacts in ramp, rotates the pattern accordingly, and
     * fires lanes in sequence with configurable spacing.
     *
     * Used for precise obelisk scoring in DECODE mode.
     *
     * @return Command that fires in motif-aware sequence
     */
    public LaunchInSequenceCommand fireInSequence() {
        return new LaunchInSequenceCommand(launcher, intake);
    }

    /**
     * Mode-aware fire command that adapts based on RobotState launcher mode.
     *
     * THROUGHPUT mode: Fires all lanes rapidly at MID range
     * DECODE mode: Fires in obelisk pattern sequence with motif tail offset
     *
     * This is the recommended fire button command for TeleOp as it automatically
     * adapts when the mode switches (e.g., at 30 seconds remaining).
     *
     * @return Command that fires using current launcher mode strategy
     */
    public LaunchModeAwareCommand fireModeAware() {
        return new LaunchModeAwareCommand(launcher, intake);
    }

    // ========== Distance-Based Commands ==========

    /**
     * Smart auto-range fire command - ONE BUTTON SOLUTION!
     * Automatically selects SHORT/MID/LONG range based on distance to goal.
     * Uses AprilTag vision for distance, falls back to odometry.
     *
     * Replaces the need for separate short/mid/long range buttons.
     *
     * @param vision The vision subsystem (for distance measurement)
     * @param drive The drive subsystem (for odometry fallback)
     * @param spinDownAfterShot Whether to spin down to idle after firing
     * @return Command that auto-selects range and fires
     */
    public LaunchAllAtAutoRangeCommand fireAllAutoRange(VisionSubsystemLimelight vision,
                                                         DriveSubsystem drive,
                                                         boolean spinDownAfterShot) {
        return new LaunchAllAtAutoRangeCommand(launcher, intake, vision, drive, spinDownAfterShot);
    }

    /**
     * Smart auto-range fire command with spin-down.
     * Automatically selects SHORT/MID/LONG range based on distance.
     *
     * @param vision The vision subsystem
     * @param drive The drive subsystem
     * @return Command that auto-selects range and fires
     */
    public LaunchAllAtAutoRangeCommand fireAllAutoRange(VisionSubsystemLimelight vision,
                                                         DriveSubsystem drive) {
        return fireAllAutoRange(vision, drive, true);
    }

    /**
     * Fires all lanes with RPM and hood position calculated from distance to goal.
     * Uses AprilTag vision to determine distance, falls back to odometry if unavailable.
     * Interpolates RPM based on configurable distance/RPM calibration points.
     *
     * @param vision The vision subsystem (for AprilTag distance measurement)
     * @param drive The drive subsystem (for odometry fallback)
     * @param spinDownAfterShot Whether to spin down to idle after firing
     * @return Command that fires all lanes at distance-calculated RPM
     */
    public LaunchAllAtDistanceBasedRPMCommand fireAllAtDistance(VisionSubsystemLimelight vision,
                                                                 DriveSubsystem drive,
                                                                 boolean spinDownAfterShot) {
        return new LaunchAllAtDistanceBasedRPMCommand(launcher, intake, vision, drive, spinDownAfterShot);
    }

    /**
     * Fires all lanes with RPM and hood position calculated from distance to goal.
     * Uses AprilTag vision to determine distance, falls back to odometry if unavailable.
     * Spins down to idle after firing.
     *
     * @param vision The vision subsystem (for AprilTag distance measurement)
     * @param drive The drive subsystem (for odometry fallback)
     * @return Command that fires all lanes at distance-calculated RPM
     */
    public LaunchAllAtDistanceBasedRPMCommand fireAllAtDistance(VisionSubsystemLimelight vision,
                                                                 DriveSubsystem drive) {
        return fireAllAtDistance(vision, drive, true);
    }

    /**
     * Continuously calculates distance to goal and updates launcher RPM while held.
     * Hold X button to spin up at calculated RPM, release to fire all lanes.
     *
     * Designed for hold-to-spin, release-to-fire button behavior.
     * While held: Continuously calculates distance and updates RPM targets
     * On release: Caller should trigger fire command
     *
     * Triggers haptic feedback (controller rumble) and light flash when launcher
     * reaches 95% of target RPM.
     *
     * @param vision The vision subsystem (for AprilTag distance measurement)
     * @param drive The drive subsystem (for odometry fallback)
     * @param lighting The lighting subsystem (for ready feedback)
     * @param gamepad The operator gamepad (for haptic feedback)
     * @return Command that continuously adjusts RPM based on distance
     */
    public ContinuousDistanceBasedSpinCommand spinUpAtDistance(VisionSubsystemLimelight vision,
                                                                DriveSubsystem drive,
                                                                LightingSubsystem lighting,
                                                                Gamepad gamepad) {
        return new ContinuousDistanceBasedSpinCommand(launcher, vision, drive, lighting, gamepad);
    }

}
