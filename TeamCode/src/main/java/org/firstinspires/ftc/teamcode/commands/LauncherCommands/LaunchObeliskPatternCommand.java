package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumSet;
import java.util.List;

/**
 * Smart command that fires artifacts in a specific color sequence for Obelisk scoring.
 *
 * Valid Obelisk patterns:
 * - PPG: Purple, Purple, Green (Gold)
 * - PGP: Purple, Green (Gold), Purple
 * - GPP: Green (Gold), Purple, Purple
 *
 * This command looks at what's actually loaded in the launcher lanes and fires as many
 * artifacts as possible in the correct sequence. If only 2 artifacts match the pattern,
 * it fires 2. If all 3 match, it fires all 3. If none match, it does nothing.
 *
 * IMPORTANT: This command assumes launch RPMs have already been set via setLaunchRpm().
 * If no RPMs are set, lanes will be treated as disabled (RPM = 0) and won't fire.
 *
 * Example usage:
 * // For PPG pattern (Purple, Purple, Green)
 * new LaunchObeliskPatternCommand(launcher, coordinator,
 *     Arrays.asList(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN),
 *     150.0)
 */
public class LaunchObeliskPatternCommand extends LauncherCommand {

    private final LauncherCoordinator coordinator;
    private final List<ArtifactColor> desiredPattern;
    private final double spacingMs;

    /**
     * Creates a command to fire artifacts matching an Obelisk pattern.
     *
     * @param launcher The launcher subsystem
     * @param coordinator The launcher coordinator (tracks lane colors)
     * @param desiredPattern The desired color sequence (e.g., [PURPLE, PURPLE, GREEN])
     * @param spacingMs Milliseconds between shots
     */
    public LaunchObeliskPatternCommand(LauncherSubsystem launcher,
                                       LauncherCoordinator coordinator,
                                       List<ArtifactColor> desiredPattern,
                                       double spacingMs) {
        super(launcher, true, coordinator);
        this.coordinator = coordinator;
        this.desiredPattern = desiredPattern;
        this.spacingMs = Math.max(0.0, spacingMs);
    }

    @Override
    protected boolean queueShots() {
        if (coordinator == null || desiredPattern == null || desiredPattern.isEmpty()) {
            return false;
        }

        // Track which lanes we've already used in this sequence
        EnumSet<LauncherLane> usedLanes = EnumSet.noneOf(LauncherLane.class);
        boolean queuedAny = false;
        double currentDelay = 0.0;

        // For each color in the desired pattern sequence
        for (ArtifactColor neededColor : desiredPattern) {
            // Skip NONE, UNKNOWN, and BACKGROUND - they're not real artifacts
            if (neededColor == null || !neededColor.isArtifact()) {
                continue;
            }

            // Find a lane that has this color and hasn't been used yet
            LauncherLane matchingLane = findUnusedLaneWithColor(neededColor, usedLanes);

            if (matchingLane == null) {
                // Can't continue the pattern - stop here
                // (Partial sequences are fine, e.g., firing 2 out of 3)
                break;
            }

            // Queue this shot with the appropriate delay
            if (currentDelay > 0.0) {
                getLauncher().queueShot(matchingLane, currentDelay);
            } else {
                getLauncher().queueShot(matchingLane);
            }

            // Mark this lane as used and increment delay for next shot
            usedLanes.add(matchingLane);
            queuedAny = true;
            currentDelay += spacingMs;
        }

        return queuedAny;
    }

    /**
     * Finds a lane that contains the specified color and hasn't been used yet.
     *
     * @param color The artifact color to search for
     * @param usedLanes Set of lanes already queued in this sequence
     * @return The matching lane, or null if none found
     */
    private LauncherLane findUnusedLaneWithColor(ArtifactColor color, EnumSet<LauncherLane> usedLanes) {
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
