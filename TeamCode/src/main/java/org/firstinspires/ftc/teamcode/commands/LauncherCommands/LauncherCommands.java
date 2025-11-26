package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.testing.LaunchInSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.testing.LaunchObeliskPatternCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
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




//    /**
//     * Universal smart shot - the ONE BUTTON SOLUTION!
//     * Combines distance-based RPM calculation with mode-aware firing strategy.
//     *
//     * This command automatically:
//     * - Calculates distance to goal (AprilTag vision with odometry fallback)
//     * - Interpolates RPM and hood position based on distance
//     * - Spins up with haptic/light feedback when ready
//     * - Fires using current launcher mode:
//     *   - THROUGHPUT: All lanes fire rapidly for maximum scoring rate
//     *   - DECODE: Fires in obelisk pattern sequence with motif tail offset
//     *
//     * This is the ultimate fire button - operator just presses when ready to shoot,
//     * and the robot handles distance, RPM, mode, and firing strategy automatically.
//     *
//     * @param vision The vision subsystem (for AprilTag distance measurement)
//     * @param drive The drive subsystem (for odometry fallback)
//     * @param lighting The lighting subsystem (for ready feedback)
//     * @param gamepad The operator gamepad (for haptic feedback)
//     * @return Command that does everything automatically
//     */
//    public UniversalSmartShotCommand fireUniversalSmart(VisionSubsystemLimelight vision,
//                                                        DriveSubsystem drive,
//                                                        LightingSubsystem lighting,
//                                                        Gamepad gamepad) {
//        return new UniversalSmartShotCommand(launcher, intake, vision, drive, lighting, gamepad);
//    }

}
