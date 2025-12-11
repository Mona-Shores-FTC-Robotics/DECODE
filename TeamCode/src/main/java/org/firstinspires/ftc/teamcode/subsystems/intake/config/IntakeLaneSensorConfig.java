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
 * PRESENCE DETECTION is what needs tuning. You have THREE options:
 *
 *   Option A: Distance-based (useDistance = true)
 *     - Uses the distance sensor to detect if something is close
 *     - Problem: Background objects near sensor cause false positives
 *     - Tune: Adjust per-lane thresholds in LanePresenceConfig
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
 *   5. Enable that detection method (useSaturation or useValue)
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
    public static Presence presence = new Presence();
    public static Gating gating = new Gating();
    public static ColorClassifier colorClassifier = new ColorClassifier();
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
    // PRESENCE DETECTION - This is what you need to tune!
    // =========================================================================

    /**
     * Presence detection settings.
     * Determines whether an artifact is in the lane (before classifying its color).
     */
    public static class Presence {
        /**
         * Use distance sensor for presence detection.
         * Artifact present when distance <= per-lane threshold.
         */
        public boolean useDistance = true;

        /**
         * Use saturation for presence detection.
         * Artifact present when filtered saturation >= saturationThreshold.
         * Colorful artifacts = high saturation, dull background = low saturation.
         */
        public boolean useSaturation = false;
        public double saturationThreshold = 0.25;

        /**
         * Use brightness (value) for presence detection.
         * Artifact present when filtered value >= valueThreshold.
         * Bright artifacts = high value, dark background = low value.
         */
        public boolean useValue = false;
        public double valueThreshold = 0.15;

        /** Minimum brightness for valid reading (quality check, not presence) */
        public double minValue = 0.02;
    }

    /**
     * Per-lane distance thresholds (only used when useDistance = true).
     * Robot-specific - each robot has different sensor mounting.
     */
    public static class LanePresenceConfig {
        public double leftThresholdCm;
        public double centerThresholdCm;
        public double rightThresholdCm;
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
    // ROBOT-SPECIFIC CONFIGS
    // =========================================================================

    private static LanePresenceConfig createLanePresenceConfig19429() {
        LanePresenceConfig config = new LanePresenceConfig();
        config.leftThresholdCm = 6.5;
        config.centerThresholdCm = 7.0;
        config.rightThresholdCm = 5.0;
        return config;
    }

    private static LanePresenceConfig createLanePresenceConfig20245() {
        LanePresenceConfig config = new LanePresenceConfig();
        config.leftThresholdCm = 4.0;
        config.centerThresholdCm = 4.0;
        config.rightThresholdCm = 4.5;
        return config;
    }
}
