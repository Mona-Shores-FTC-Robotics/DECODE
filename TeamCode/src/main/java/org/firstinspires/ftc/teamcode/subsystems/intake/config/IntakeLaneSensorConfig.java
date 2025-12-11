package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Lane sensor configuration for artifact color detection.
 * Configures color sensors, classification algorithms, and presence detection.
 */
@Configurable
public class IntakeLaneSensorConfig {

    public int fullCount = 3;
    /**
     * Artifact color classifier mode.
     * Dtermines which algorithm is used to classify GREEN vs PURPLE.
     */
    public enum ClassifierMode {
        /** Range-based: Independent hue ranges for green and purple (legacy) */
        RANGE_BASED,
        /** Decision boundary: Single hue threshold between green and purple (default, most robust) */
        DECISION_BOUNDARY,
        /** Distance-based: Euclidean distance in HSV space to color targets */
        DISTANCE_BASED
    }

    // Groups below are shown as collapsible sections in @Configurable UI
    public static Polling polling = new Polling();
    public Hardware hardware = new Hardware();
    public static DistanceFilter distanceFilter = new DistanceFilter();
    public static HueFilter hueFilter = new HueFilter();
    public static Gating gating = new Gating();
    public static Quality quality = new Quality();
    public static Presence presence = new Presence();
    public static Classifier classifier = new Classifier();
    public static LanePresenceConfig lanePresenceConfig19429 = createLanePresenceConfig19429();
    public static LanePresenceConfig lanePresenceConfig20245 = createLanePresenceConfig20245();

    public static class Polling {
        public boolean enablePolling = true;
        public double samplePeriodMs = 150;
        public String leftSensor = "lane_left_color";
        public String centerSensor = "lane_center_color";
        public String rightSensor = "lane_right_color";
    }

    public static class Hardware {
        /** Turn the onboard white LED on/off (applied at bind time) */
        public boolean enableSensorLight = true;
        /** If true, override sensor gain with sensorGain value; otherwise leave default */
        public boolean overrideSensorGain = true;
        /** Gain applied when overrideSensorGain is true (REV Color Sensor V3 typical range ~1-10) */
        public double sensorGain = 20.0;
    }

    public static class DistanceFilter {
        /** Enable moving average filtering for distance sensor readings */
        public boolean enableFilter = true;
        /**
         * Number of samples to average for the moving average filter.
         * Higher values = more smoothing but slower response.
         * Recommended: 3-5 for good balance of smoothing and responsiveness.
         * At 150ms sample period: 3 samples = 450ms window, 5 samples = 750ms window.
         */
        public int windowSize = 4;
    }

    public static class HueFilter {
        /** Enable circular moving average filtering for hue values (helps stabilize purple vs green classification) */
        public boolean enableFilter = true;
        /**
         * Number of samples to average for the moving average filter.
         * Higher values = more smoothing but slower response.
         * Recommended: 3-4 for good balance of smoothing and responsiveness.
         * At 50ms sample period: 3 samples = 150ms window, 4 samples = 200ms window.
         * Uses circular averaging to properly handle purple wrap-around (270°-30°).
         */
        public int windowSize = 10;
        /**
         * Jump detection threshold in degrees. If raw hue differs from filtered hue
         * by more than this amount, the filter is reset to the new value immediately.
         * This prevents slow transitions through intermediate hues (e.g., background->purple
         * passing through green). Set to 0 to disable jump detection.
         */
        public double jumpThreshold = 50.0;
        /**
         * Minimum hue value to accept into the filter. Readings below this are ignored.
         * Helps filter out spurious red readings that can corrupt the circular average.
         * Set to 0 to disable. Typical value: 70 to reject reds.
         */
        public double minHue = 70.0;
    }

    public static class Gating {
        /** Minimum confidence required to accept a new artifact color classification */
        public double minConfidenceToAccept = .2;
        /** Number of consecutive confident samples required before updating lane color */
        public int consecutiveConfirmationsRequired = 1;
        /** Number of consecutive non-artifact samples required before clearing lane color (helps with whiffle ball holes) */
        public int consecutiveClearConfirmationsRequired = 2;
        /** Keep-alive duration (ms) - keep artifact detection alive after last good reading (helps with whiffle ball holes) */
        public double keepAliveMs = 0;
        /** Distance clearance margin (cm) - how far beyond threshold to instantly clear (helps artifacts clear quickly when removed) */
        public double distanceClearanceMarginCm = .5;
    }

    public static class Quality {
        /**
         * Quality thresholds - only used when hue-based presence is DISABLED for a lane.
         * When hue-based presence is enabled, these checks are skipped (hue is the authority).
         */
        public double minValue = 0.02;
        public double minSaturation = 0.15;
    }

    public static class Presence {
        /**
         * Enable distance-based presence detection (used when hue-based is disabled for a lane).
         * When true, artifact presence requires distance <= per-lane threshold.
         */
        public boolean useDistance = true;

        /**
         * Global enable for hue-based presence detection.
         * When true, per-lane settings (leftUseHuePresence, etc.) control which lanes use hue.
         * When false, all lanes use distance-based detection.
         */
        public boolean useHuePresence = false;
    }

    /**
     * Per-lane presence detection settings.
     * Robot-specific configs are created in createLanePresenceConfig19429() and createLanePresenceConfig20245().
     *
     * Usage:
     * - Set Presence.useHuePresence = true to enable hue-based detection globally
     * - Then use per-lane flags to control which lanes use hue vs distance
     * - Example: Set rightUseHuePresence = false to use distance-based for right lane only
     */
    public static class LanePresenceConfig {
        // Distance thresholds (used when hue-based detection is disabled for a lane)
        public double leftThresholdCm;
        public double centerThresholdCm;
        public double rightThresholdCm;

        // Per-lane hue presence enable (requires global Presence.useHuePresence = true)
        public boolean leftUseHuePresence = true;
        public boolean centerUseHuePresence = true;
        public boolean rightUseHuePresence = true;

        // Per-lane hue thresholds (hue >= threshold = artifact present)
        public double leftHueThreshold = 130.0;
        public double centerHueThreshold = 130.0;
        public double rightHueThreshold = 130.0;
    }

    public static class Classifier {
        // Classifier mode selector
        public String mode = ClassifierMode.DECISION_BOUNDARY.name();

        public DecisionBoundary decision = new DecisionBoundary();
        public Range range = new Range();
        public DistanceBased distance = new DistanceBased();
    }

    public static class DecisionBoundary {
        /** Hue decision boundary - classify as GREEN if hue < boundary, PURPLE otherwise */
        public double hueDecisionBoundary = 165.0; //20245
        /** Distance from boundary for low confidence warning (degrees) */
        public double lowConfidenceMargin = 8.0;
    }

    public static class Range {
        /** Range-based mode parameters (legacy) */
        public double greenHueMin = 80.0;
        public double greenHueMax = 160.0;
        public double purpleHueMin = 260.0;
        public double purpleHueMax = 330.0;
        public double purpleHueWrapMax = 40.0;
    }

    public static class DistanceBased {
        /** Weight for hue component in distance calculation */
        public double hueWeight = 2.0;
        /** Weight for saturation component in distance calculation */
        public double saturationWeight = 0.5;
        /** Weight for value component in distance calculation */
        public double valueWeight = 0.3;
        /** Target hue/sat/value for green artifacts */
        public double greenHueTarget = 120.0;
        public double greenSatTarget = 0.45;
        public double greenValTarget = 0.50;
        /** Target hue/sat/value for purple artifacts */
        public double purpleHueTarget = 290.0;
        public double purpleSatTarget = 0.40;
        public double purpleValTarget = 0.45;
    }

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
