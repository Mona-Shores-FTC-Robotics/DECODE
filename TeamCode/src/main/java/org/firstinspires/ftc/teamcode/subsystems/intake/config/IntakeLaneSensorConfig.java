package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Lane sensor configuration for artifact color detection.
 *
 * ============================================================================
 * TUNING GUIDE FOR MENTORS
 * ============================================================================
 *
 * The sensor does TWO things:
 *   1. PRESENCE DETECTION - Is there an artifact in the lane? (yes/no)
 *   2. COLOR CLASSIFICATION - Is it GREEN or PURPLE? (uses hue boundary)
 *
 * COLOR CLASSIFICATION is already working. Don't touch it.
 *
 * PRESENCE DETECTION needs tuning. Settings are PER-ROBOT in
 * lanePresenceConfig19429 or lanePresenceConfig20245.
 *
 * You have FOUR detection methods (enable any combination):
 *
 *   Distance (useDistance = true)
 *     - Artifact present when distance <= threshold
 *     - Telemetry: intake/sample/{lane}/distance_cm
 *     - Problem: Background objects cause false positives
 *
 *   Saturation (useSaturation = true)
 *     - Artifact present when sat >= threshold
 *     - Telemetry: intake/sample/{lane}/sat
 *     - Colorful artifact vs dull background
 *
 *   Value (useValue = true)
 *     - Artifact present when val >= threshold
 *     - Telemetry: intake/sample/{lane}/val
 *     - Bright artifact vs dark background
 *
 *   Hue (useHue = true)
 *     - Artifact present when hue >= threshold
 *     - Telemetry: intake/sample/{lane}/hue
 *     - Artifact hue vs background hue
 *
 * When MULTIPLE methods are enabled, ALL must pass (AND logic).
 * This lets you combine for reliability (e.g., sat AND val).
 *
 * QUICK START:
 *   1. Run TeleOp and watch telemetry for all four values
 *   2. Note values when EMPTY vs when ARTIFACT is present
 *   3. Find which has the clearest separation
 *   4. Set threshold halfway between empty and artifact values
 *   5. Enable that method in YOUR ROBOT'S config
 *   6. Test both detection AND clearing speed
 * ============================================================================
 */
@Configurable
public class IntakeLaneSensorConfig {

    /** Number of lanes that must have artifacts to consider intake "full" */
    public int fullCount = 3;

    // Configuration groups (shown in FTC Dashboard)
    public static Polling polling = new Polling();
    public Hardware hardware = new Hardware();
    public static SaturationFilter saturationFilter = new SaturationFilter();
    public static ValueFilter valueFilter = new ValueFilter();
    public static HueFilter hueFilter = new HueFilter();
    public static DistanceFilter distanceFilter = new DistanceFilter();
    public static Gating gating = new Gating();
    public static ColorClassifier colorClassifier = new ColorClassifier();

    // Robot-specific presence detection configs
    public static LanePresenceConfig lanePresenceConfig19429 = createLanePresenceConfig19429();
    public static LanePresenceConfig lanePresenceConfig20245 = createLanePresenceConfig20245();

    // =========================================================================
    // SENSOR POLLING
    // =========================================================================

    public static class Polling {
        public boolean enablePolling = true;
        /** How often to read sensors (ms). Lower = faster but more CPU. */
        public double samplePeriodMs = 150;
        public String leftSensor = "lane_left_color";
        public String centerSensor = "lane_center_color";
        public String rightSensor = "lane_right_color";
    }

    public static class Hardware {
        /** Turn the onboard white LED on/off */
        public boolean enableSensorLight = true;
        /** Override sensor gain (REV Color Sensor V3 typical range ~1-10) */
        public boolean overrideSensorGain = true;
        public double sensorGain = 20.0;
    }

    // =========================================================================
    // PRESENCE DETECTION - Per-robot settings in LanePresenceConfig
    // =========================================================================

    /**
     * Per-robot presence detection settings.
     * Each robot has its own instance (lanePresenceConfig19429, lanePresenceConfig20245).
     * RobotConfigs.getLanePresenceConfig() returns the active robot's config.
     */
    public static class LanePresenceConfig {
        // --- DETECTION METHODS (enable any combination) ---
        // When multiple are enabled, ALL must pass (AND logic)

        /** Use distance sensor for presence detection */
        public boolean useDistance = true;

        /** Use saturation for presence (colorful artifact vs dull background) */
        public boolean useSaturation = false;
        /** Saturation threshold - artifact present when sat >= this */
        public double saturationThreshold = 0.25;

        /** Use brightness for presence (bright artifact vs dark background) */
        public boolean useValue = false;
        /** Value threshold - artifact present when val >= this */
        public double valueThreshold = 0.15;

        /** Use hue for presence (artifact hue vs background hue) */
        public boolean useHue = false;
        /** Hue threshold - artifact present when hue >= this */
        public double hueThreshold = 130.0;

        // --- DISTANCE THRESHOLDS (per-lane, only used when useDistance = true) ---

        public double leftThresholdCm = 5.0;
        public double centerThresholdCm = 5.0;
        public double rightThresholdCm = 5.0;

        // --- QUALITY CHECK ---

        /** Minimum brightness for valid reading */
        public double minValue = 0.02;
    }

    // =========================================================================
    // FILTERS - Smooth out whiffle ball hole flicker
    // =========================================================================

    public static class SaturationFilter {
        public boolean enableFilter = true;
        /** Higher = smoother but slower. Start with 4. */
        public int windowSize = 4;
    }

    public static class ValueFilter {
        public boolean enableFilter = true;
        /** Higher = smoother but slower. Start with 4. */
        public int windowSize = 4;
    }

    public static class HueFilter {
        public boolean enableFilter = true;
        /** Higher = smoother but slower. Start with 4. */
        public int windowSize = 4;
    }

    public static class DistanceFilter {
        public boolean enableFilter = true;
        public int windowSize = 4;
    }

    // =========================================================================
    // GATING - Debounce to prevent flickering
    // =========================================================================

    public static class Gating {
        /** Consecutive samples needed to confirm artifact detected */
        public int consecutiveConfirmationsRequired = 1;
        /** Consecutive samples needed to confirm artifact removed */
        public int consecutiveClearConfirmationsRequired = 2;
        /** Distance margin (cm) for instant clearing when artifact is removed */
        public double distanceClearanceMarginCm = 0.5;
    }

    // =========================================================================
    // COLOR CLASSIFICATION - Already working, probably don't touch
    // =========================================================================

    /**
     * Color classifier settings. Uses hue to distinguish GREEN vs PURPLE.
     * This is already tuned and working well.
     */
    public static class ColorClassifier {
        /**
         * Hue decision boundary (degrees).
         * GREEN if hue < boundary, PURPLE if hue >= boundary.
         * Default 165 works well for most setups.
         */
        public double hueDecisionBoundary = 165.0;

        /** Margin for low-confidence warning (degrees from boundary) */
        public double lowConfidenceMargin = 8.0;
    }

    // =========================================================================
    // ROBOT-SPECIFIC CONFIGS - Tune these for each robot!
    // =========================================================================

    private static LanePresenceConfig createLanePresenceConfig19429() {
        LanePresenceConfig config = new LanePresenceConfig();
        // Detection methods (enable any combination - all enabled must pass)
        config.useDistance = true;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = false;
        config.valueThreshold = 0.15;
        config.useHue = false;
        config.hueThreshold = 130.0;
        // Distance thresholds
        config.leftThresholdCm = 6.5;
        config.centerThresholdCm = 7.0;
        config.rightThresholdCm = 5.0;
        // Quality
        config.minValue = 0.02;
        return config;
    }

    private static LanePresenceConfig createLanePresenceConfig20245() {
        LanePresenceConfig config = new LanePresenceConfig();
        // Detection methods (enable any combination - all enabled must pass)
        config.useDistance = true;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = false;
        config.valueThreshold = 0.15;
        config.useHue = false;
        config.hueThreshold = 130.0;
        // Distance thresholds
        config.leftThresholdCm = 4.0;
        config.centerThresholdCm = 4.0;
        config.rightThresholdCm = 4.5;
        // Quality
        config.minValue = 0.02;
        return config;
    }
}
