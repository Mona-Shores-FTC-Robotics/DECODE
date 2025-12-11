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
 *   2. COLOR CLASSIFICATION - Is it GREEN or PURPLE? (uses hue)
 *
 * COLOR CLASSIFICATION (hueDecisionBoundary) is already working well.
 * You probably don't need to touch it.
 *
 * PRESENCE DETECTION is what needs tuning. Settings are PER-ROBOT in
 * lanePresenceConfig19429 or lanePresenceConfig20245. You have THREE options:
 *
 *   Option A: Distance-based (useDistance = true)
 *     - Uses the distance sensor to detect if something is close
 *     - Problem: Background objects near sensor cause false positives
 *     - Tune: Adjust per-lane thresholds (leftThresholdCm, etc.)
 *
 *   Option B: Saturation-based (useSaturation = true)
 *     - Colorful artifacts have HIGH saturation, dull background has LOW
 *     - Watch telemetry: intake/sample/{lane}/sat (filtered value)
 *     - Tune: Set saturationThreshold between background and artifact values
 *
 *   Option C: Value-based (useValue = true)
 *     - Bright artifacts have HIGH value, dark background has LOW
 *     - Watch telemetry: intake/sample/{lane}/val (filtered value)
 *     - Tune: Set valueThreshold between background and artifact values
 *
 * You can combine options (e.g., useSaturation + useValue requires BOTH).
 *
 * FILTERING (saturationFilter, valueFilter):
 *   - Smooths out flicker from whiffle ball holes
 *   - Higher windowSize = smoother but slower response
 *   - Start with windowSize = 4, increase if flickering persists
 *
 * QUICK START:
 *   1. Run TeleOp and watch telemetry values for sat and val
 *   2. Note the values when empty vs when artifact is present
 *   3. Pick whichever (sat or val) has the clearest separation
 *   4. Set the threshold halfway between empty and artifact values
 *   5. Enable that detection method in YOUR ROBOT'S config
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
        // --- DETECTION METHOD (pick one or combine) ---

        /** Use distance sensor for presence detection */
        public boolean useDistance = true;

        /** Use saturation for presence detection (colorful = artifact, dull = background) */
        public boolean useSaturation = false;

        /** Saturation threshold - artifact present when sat >= this */
        public double saturationThreshold = 0.25;

        /** Use brightness for presence detection (bright = artifact, dark = background) */
        public boolean useValue = false;

        /** Value threshold - artifact present when val >= this */
        public double valueThreshold = 0.15;

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
        /** Minimum confidence to accept a color classification */
        public double minConfidenceToAccept = 0.2;
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
        // Detection method
        config.useDistance = true;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = false;
        config.valueThreshold = 0.15;
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
        // Detection method
        config.useDistance = true;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = false;
        config.valueThreshold = 0.15;
        // Distance thresholds
        config.leftThresholdCm = 4.0;
        config.centerThresholdCm = 4.0;
        config.rightThresholdCm = 4.5;
        // Quality
        config.minValue = 0.02;
        return config;
    }
}
