package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Lane sensor configuration for artifact color detection.
 * Configures color sensors, classification algorithms, and presence detection.
 *
 * Presence detection uses saturation threshold (colorful artifact vs dull background).
 * Color classification uses hue to distinguish GREEN vs PURPLE.
 */
@Configurable
public class IntakeLaneSensorConfig {

    public int fullCount = 3;

    /**
     * Artifact color classifier mode.
     * Determines which algorithm is used to classify GREEN vs PURPLE.
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
    public static SaturationFilter saturationFilter = new SaturationFilter();
    public static ValueFilter valueFilter = new ValueFilter();
    public static HueFilter hueFilter = new HueFilter();
    public static Gating gating = new Gating();
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
        /** Number of samples to average. Higher = smoother but slower response. */
        public int windowSize = 4;
    }

    public static class SaturationFilter {
        /** Enable moving average filtering for saturation (smooths whiffle ball hole flicker) */
        public boolean enableFilter = true;
        /** Number of samples to average. Higher = smoother but slower response. */
        public int windowSize = 4;
    }

    public static class ValueFilter {
        /** Enable moving average filtering for value/brightness (smooths whiffle ball hole flicker) */
        public boolean enableFilter = true;
        /** Number of samples to average. Higher = smoother but slower response. */
        public int windowSize = 4;
    }

    public static class HueFilter {
        /** Enable circular moving average filtering for hue (stabilizes GREEN vs PURPLE classification) */
        public boolean enableFilter = true;
        /** Number of samples to average. Uses circular averaging for proper wrap-around handling. */
        public int windowSize = 4;
    }

    public static class Gating {
        /** Minimum confidence required to accept a new artifact color classification */
        public double minConfidenceToAccept = 0.2;
        /** Number of consecutive confident samples required before updating lane color */
        public int consecutiveConfirmationsRequired = 1;
        /** Number of consecutive non-artifact samples required before clearing lane color */
        public int consecutiveClearConfirmationsRequired = 2;
        /** Distance clearance margin (cm) - how far beyond threshold to instantly clear */
        public double distanceClearanceMarginCm = 0.5;
    }

    public static class Presence {
        /**
         * Enable distance-based presence detection.
         * When true, artifact presence requires distance <= per-lane threshold.
         */
        public boolean useDistance = true;

        /**
         * Enable saturation-based presence detection.
         * When true, artifact presence requires filtered saturation >= saturationThreshold.
         * Artifacts are colorful (high saturation), background is dull (low saturation).
         */
        public boolean useSaturation = false;

        /** Saturation threshold for presence - values >= this indicate artifact present */
        public double saturationThreshold = 0.25;

        /**
         * Enable value/brightness-based presence detection.
         * When true, artifact presence requires filtered value >= valueThreshold.
         * Useful if artifacts are brighter than background.
         */
        public boolean useValue = false;

        /** Value threshold for presence - values >= this indicate artifact present */
        public double valueThreshold = 0.15;

        /** Minimum brightness (value) required for valid color reading (quality check, not presence) */
        public double minValue = 0.02;
    }

    /**
     * Per-lane presence detection settings.
     * Robot-specific configs are created in createLanePresenceConfig19429() and createLanePresenceConfig20245().
     */
    public static class LanePresenceConfig {
        // Distance thresholds per lane (cm) - artifact detected when distance <= threshold
        public double leftThresholdCm;
        public double centerThresholdCm;
        public double rightThresholdCm;
    }

    public static class Classifier {
        public String mode = ClassifierMode.DECISION_BOUNDARY.name();
        public DecisionBoundary decision = new DecisionBoundary();
        public Range range = new Range();
        public DistanceBased distance = new DistanceBased();
    }

    public static class DecisionBoundary {
        /** Hue decision boundary - classify as GREEN if hue < boundary, PURPLE otherwise */
        public double hueDecisionBoundary = 165.0;
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
