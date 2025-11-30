package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;

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

    public PresetRangeSpinCommand presetRangeSpinUp(LauncherRange range, boolean finishWhenReady) {
        return new PresetRangeSpinCommand(launcher, range, finishWhenReady);
    }

    public DistanceBasedSpinCommand distanceBasedSpinUp(VisionSubsystemLimelight vision,
                                                        DriveSubsystem drive,
                                                        LightingSubsystem lighting,
                                                        Gamepad gamepad) {
        return new DistanceBasedSpinCommand(launcher, vision, drive, lighting, gamepad);
    }

    public LaunchAllCommand launchAll(boolean spinDownAfterShot) {
        return new LaunchAllCommand(launcher, intake, spinDownAfterShot);
    }

    /**
     * Mode-aware launch: DECODE uses sequence, THROUGHPUT uses all-at-once.
     */
    public Command launchAccordingToMode(boolean spinDownAfterShot) {
        if (org.firstinspires.ftc.teamcode.util.RobotState.getLauncherMode() == org.firstinspires.ftc.teamcode.util.LauncherMode.DECODE) {
            return new LaunchInSequenceCommand(launcher, intake, spinDownAfterShot);
        }
        return new LaunchAllCommand(launcher, intake, spinDownAfterShot);
    }

    /**
     * Launches in obelisk pattern sequence with motif tail offset.
     *
     * Gets the detected motif pattern from RobotState, calculates the motif tail
     * offset based on artifacts in ramp, rotates the pattern accordingly, and
     * fires lanes in sequence with configurable spacing.
     *
     * Used for precise obelisk scoring in DECODE mode.
     *
     * @param spinDownAfterShot Whether to spin down flywheels after shooting (false in autonomous)
     * @return Command that fires in motif-aware sequence
     */
    public LaunchInSequenceCommand launchInSequence(boolean spinDownAfterShot) {
        return new LaunchInSequenceCommand(launcher, intake, spinDownAfterShot);
    }

}
