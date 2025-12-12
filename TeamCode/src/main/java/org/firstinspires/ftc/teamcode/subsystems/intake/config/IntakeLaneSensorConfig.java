package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Lane sensor configuration for artifact color detection.
 *
 * ============================================================================
 * TUNING GUIDE FOR MENTORS
 * ============================================================================
 *
 * HOW IT WORKS (Data Flow):
 *
 *   Raw Sensor Reading
 *         ↓
 *   Moving Average Filter (smooths noise from whiffle ball holes)
 *         ↓
 *   PRESENCE DETECTION (is something there? yes/no)
 *         ↓
 *   COLOR CLASSIFICATION (if present: GREEN or PURPLE based on hue)
 *         ↓
 *   DEBOUNCE (consecutive samples confirm detection/clearing)
 *         ↓
 *   Final Result → getArtifactCount(), isFull(), getLaneColor()
 *
 * IMPORTANT: Once presence is detected, the artifact IS classified as
 * GREEN or PURPLE. There are no hidden gates - if presence = true, you
 * WILL get a color. Misclassification is acceptable; false negatives are not.
 *
 * ============================================================================
 * WHAT TO TUNE
 * ============================================================================
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
 *     - Good: Works in all lighting
 *     - Problem: Background objects cause false positives
 *
 *   Saturation (useSaturation = true)
 *     - Artifact present when sat >= threshold
 *     - Telemetry: intake/sample/{lane}/sat
 *     - Good: Colorful artifact vs dull background
 *     - Problem: Varies with lighting conditions
 *
 *   Value (useValue = true)
 *     - Artifact present when val >= threshold
 *     - Telemetry: intake/sample/{lane}/val
 *     - Good: Bright artifact vs dark background
 *     - Problem: Varies with lighting conditions
 *
 *   Hue (useHue = true)
 *     - Artifact present when hue >= threshold
 *     - Telemetry: intake/sample/{lane}/hue
 *     - Good: Artifact hue differs from background
 *     - Problem: Hue is undefined when saturation is near 0
 *
 * When MULTIPLE methods are enabled, ALL must pass (AND logic).
 * Example: useDistance=true AND useSaturation=true means both must
 * be satisfied to detect presence. This reduces false positives.
 *
 * ============================================================================
 * QUICK START
 * ============================================================================
 *
 *   1. Run TeleOp and watch telemetry for all four values
 *   2. Note values when EMPTY vs when ARTIFACT is present
 *   3. Find which has the clearest separation (biggest gap)
 *   4. Set threshold halfway between empty and artifact values
 *   5. Enable that method in YOUR ROBOT'S config (see bottom of file)
 *   6. Test both detection AND clearing speed
 *
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 *
 * FALSE POSITIVES (detects artifact when empty):
 *   - Lower the threshold (tighter detection)
 *   - Enable additional detection methods (AND logic)
 *   - Increase consecutiveConfirmationsRequired in Gating
 *
 * FALSE NEGATIVES (misses artifact when present):
 *   - Raise the threshold (looser detection)
 *   - Disable detection methods that aren't working well
 *   - Try a different detection method entirely
 *
 * SLOW CLEARING (takes too long to register removal):
 *   - Lower consecutiveClearConfirmationsRequired in Gating
 *   - Reduce filter windowSize (faster response, more noise)
 *
 * FLICKERING (detection rapidly toggles on/off):
 *   - Increase consecutiveConfirmationsRequired in Gating
 *   - Increase filter windowSize (smoother, but slower)
 *
 * TELEMETRY KEYS TO WATCH:
 *   intake/sample/{lane}/presence_detected - Is presence detected? (after filters)
 *   intake/sample/{lane}/distance_cm       - Raw distance reading
 *   intake/sample/{lane}/sat               - Filtered saturation (0.0-1.0)
 *   intake/sample/{lane}/val               - Filtered value/brightness (0.0-1.0)
 *   intake/sample/{lane}/hue               - Filtered hue (0-360 degrees)
 *   intake/sample/{lane}/color             - Final debounced color (NONE/GREEN/PURPLE)
 *
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
     *
     * IMPORTANT: These are just field declarations with safe defaults.
     * Actual values are set in the robot-specific factory methods below:
     *   - createLanePresenceConfig19429()
     *   - createLanePresenceConfig20245()
     *
     * To change settings, edit YOUR ROBOT'S factory method, not these defaults.
     * RobotConfigs.getLanePresenceConfig() returns the active robot's config.
     */
    public static class LanePresenceConfig {
        // --- DETECTION METHODS (enable any combination) ---
        // When multiple are enabled, ALL must pass (AND logic)
        // Defaults are all OFF - robot-specific configs enable what's needed

        /** Use distance sensor for presence detection */
        public boolean useDistance;

        /** Use saturation for presence (colorful artifact vs dull background) */
        public boolean useSaturation;
        /** Saturation threshold - artifact present when sat >= this */
        public double saturationThreshold;

        /** Use brightness for presence (bright artifact vs dark background) */
        public boolean useValue;
        /** Value threshold - artifact present when val >= this */
        public double valueThreshold;

        /** Use hue for presence (artifact hue vs background hue) */
        public boolean useHue;
        /** Hue threshold - artifact present when hue >= this */
        public double hueThreshold;

        // --- DISTANCE THRESHOLDS (per-lane, only used when useDistance = true) ---

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
        /** Consecutive samples needed to confirm artifact detected */
        public int consecutiveConfirmationsRequired = 1;
        /** Consecutive samples needed to confirm artifact removed */
        public int consecutiveClearConfirmationsRequired = 2;
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
    }

    // =========================================================================
    // ROBOT-SPECIFIC CONFIGS - Tune these for each robot!
    // =========================================================================

    private static LanePresenceConfig createLanePresenceConfig19429() {
        LanePresenceConfig config = new LanePresenceConfig();
        // Detection methods (enable any combination - all enabled must pass)
        config.useDistance = false;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = true;
        config.valueThreshold = 0.09;
        config.useHue = false;
        config.hueThreshold = 130.0;
        // Distance thresholds (only used when useDistance = true)
        config.leftThresholdCm = 6.5;
        config.centerThresholdCm = 7.0;
        config.rightThresholdCm = 5.0;
        return config;
    }

    private static LanePresenceConfig createLanePresenceConfig20245() {
        LanePresenceConfig config = new LanePresenceConfig();
        // Detection methods (enable any combination - all enabled must pass)
        config.useDistance = false;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = true;
        config.valueThreshold = .09;
        config.useHue = false;
        config.hueThreshold = 130.0;
        // Distance thresholds
        config.leftThresholdCm = 4.0;
        config.centerThresholdCm = 4.0;
        config.rightThresholdCm = 4.5;
        return config;
    }
}
