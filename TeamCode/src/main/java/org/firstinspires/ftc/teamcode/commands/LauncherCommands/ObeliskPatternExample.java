package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import org.firstinspires.ftc.teamcode.util.ArtifactColor;

import java.util.Arrays;

/**
 * Example usage of LaunchObeliskPatternCommand.
 *
 * This demonstrates how to use the smart Obelisk pattern command that fires
 * artifacts in the correct color sequence regardless of which lane they're in.
 */
@SuppressWarnings("unused")
public class ObeliskPatternExample {

    /**
     * Example scenario:
     *
     * Loaded lanes:
     * - LEFT: Purple
     * - CENTER: Green
     * - RIGHT: Purple
     *
     * Desired pattern: PPG (Purple, Purple, Green)
     *
     * The command will fire: LEFT → RIGHT → CENTER
     * This gives the correct PPG color sequence even though the lanes are out of order!
     *
     * Usage in TeleOp or Autonomous:
     *
     * <pre>
     * // Option 1: Use convenience methods (recommended)
     * CommandScheduler.scheduleCommand(launcherCommands.launchObeliskPPG());
     * CommandScheduler.scheduleCommand(launcherCommands.launchObeliskPGP());
     * CommandScheduler.scheduleCommand(launcherCommands.launchObeliskGPP());
     *
     * // Option 2: Custom pattern with specific spacing
     * CommandScheduler.scheduleCommand(
     *     launcherCommands.launchObeliskPattern(
     *         Arrays.asList(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN),
     *         200.0  // 200ms spacing between shots
     *     )
     * );
     *
     * // Option 3: Direct instantiation
     * new LaunchObeliskPatternCommand(
     *     launcher,
     *     coordinator,
     *     Arrays.asList(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE),
     *     150.0
     * ).schedule();
     * </pre>
     *
     * IMPORTANT: This command assumes RPMs have already been set via setLaunchRpm().
     * Typically you'll want to precede this with a LaunchAllAtPresetRangeCommand or similar
     * to set the appropriate RPMs for all lanes.
     *
     * Partial sequences are fine:
     * - If pattern is PPG but you only have 2 purples loaded, it fires both purples
     * - If pattern is GPP but you only have 1 green and 1 purple, it fires green then purple
     * - If nothing matches, the command completes immediately without firing
     */
    public static void exampleUsage() {
        // See JavaDoc above for usage patterns
    }
}
