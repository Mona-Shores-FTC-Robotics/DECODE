package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.lighting.config.LightingColorPositionConfig;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumMap;

/**
 * Lighting subsystem telemetry data.
 * Captures the current state of all three lane indicator lights.
 * <p>
 * This data is used by:
 * <ul>
 *   <li>Panels Lights widget - mirrors robot's goBILDA RGB indicators</li>
 *   <li>Telemetry display - shows current pattern and colors</li>
 *   <li>Debugging - verifies lighting responds correctly to robot state</li>
 * </ul>
 * </p>
 */
public class LightingTelemetryData {
    // Pattern state
    public final String currentPattern;
    public final String goalPattern;
    public final String baseMode;

    // Per-lane colors (as strings for telemetry)
    public final String leftColor;
    public final String centerColor;
    public final String rightColor;

    // Per-lane servo positions (0.0-1.0)
    public final double leftPosition;
    public final double centerPosition;
    public final double rightPosition;

    public LightingTelemetryData(
            String currentPattern,
            String goalPattern,
            String baseMode,
            String leftColor,
            String centerColor,
            String rightColor,
            double leftPosition,
            double centerPosition,
            double rightPosition
    ) {
        this.currentPattern = currentPattern;
        this.goalPattern = goalPattern;
        this.baseMode = baseMode;
        this.leftColor = leftColor;
        this.centerColor = centerColor;
        this.rightColor = rightColor;
        this.leftPosition = leftPosition;
        this.centerPosition = centerPosition;
        this.rightPosition = rightPosition;
    }

    /**
     * Capture lighting telemetry from the subsystem.
     *
     * @param lighting Lighting subsystem (may be null)
     * @return Captured telemetry data, or default values if lighting is null
     */
    public static LightingTelemetryData capture(LightingSubsystem lighting) {
        if (lighting == null) {
            return new LightingTelemetryData(
                    "OFF", "OFF", "OFF",
                    "NONE", "NONE", "NONE",
                    0.0, 0.0, 0.0
            );
        }

        // Get color snapshot from lighting subsystem
        EnumMap<LauncherLane, ArtifactColor> colors = lighting.getSensorLaneColorSnapshot();

        ArtifactColor left = colors.getOrDefault(LauncherLane.LEFT, ArtifactColor.NONE);
        ArtifactColor center = colors.getOrDefault(LauncherLane.CENTER, ArtifactColor.NONE);
        ArtifactColor right = colors.getOrDefault(LauncherLane.RIGHT, ArtifactColor.NONE);

        // Convert colors to servo positions using the config
        LightingColorPositionConfig config = LightingSubsystem.colorPositionConfig;

        return new LightingTelemetryData(
                "LANE_TRACKING", // Would need subsystem access to get actual pattern
                "LANE_TRACKING",
                "ALLIANCE",
                left.name(),
                center.name(),
                right.name(),
                colorToPosition(left, config),
                colorToPosition(center, config),
                colorToPosition(right, config)
        );
    }

    /**
     * Convert ArtifactColor to servo position.
     */
    private static double colorToPosition(ArtifactColor color, LightingColorPositionConfig config) {
        switch (color) {
            case GREEN:   return config.greenPosition;
            case PURPLE:  return config.purplePosition;
            case UNKNOWN: return config.whitePosition;
            case NONE:
            default:      return config.offPosition;
        }
    }
}
