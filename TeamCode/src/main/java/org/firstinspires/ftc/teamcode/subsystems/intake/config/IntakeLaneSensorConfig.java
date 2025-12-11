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
    public static Background background = new Background();
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
        public int windowSize = 6;
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
    }

    public static class Gating {
        /** Minimum confidence required to accept a new artifact color classification */
        public double minConfidenceToAccept = .2;
        /** Number of consecutive confident samples required before updating lane color */
        public int consecutiveConfirmationsRequired = 1;
        /** Number of consecutive non-artifact samples required before clearing lane color (helps with whiffle ball holes) */
        public int consecutiveClearConfirmationsRequired = 3;
        /** Keep-alive duration (ms) - keep artifact detection alive after last good reading (helps with whiffle ball holes) */
        public double keepAliveMs = 0;
        /** Distance clearance margin (cm) - how far beyond threshold to instantly clear (helps artifacts clear quickly when removed) */
        public double distanceClearanceMarginCm = .5;
    }

    public static class Quality {
        // Quality thresholds (used by all classifiers)
        public double minValue = 0.02;
        public double minSaturation = 0.15;
    }

    public static class Presence {
        // Distance gating (used by all classifiers)
        public boolean useDistance = true;
        /** Legacy presence distance for scoring (diagnostics only; hysteresis uses per-lane thresholds) */
        public double presenceDistanceCm = 5.5;

        // Multi-factor presence detection - improves artifact vs background discrimination
        /** Enabnicle enhanced presence scoring (not just distance) */
        public boolean enablePresenceScoring = true;
        /** Minimum total RGB intensity for artifact presence (0-765 range) */
        public double minTotalIntensity = 10;
        /** Weight for distance factor in presence score */
        public double presenceDistanceWeight = .55;
        /** Weight for saturation factor in presence score */
        public double presenceSaturationWeight = 0.25;
        /** Weight for value factor in presence score */
        public double presenceValueWeight = 0.15;
        /** Weight for intensity factor in presence score */
        public double presenceIntensityWeight = 0.05;
        /** Minimum presence score (0-1) to consider artifact present */
        public double minPresenceScore = .25;
    }

    public static class LanePresenceConfig {
        /** Distance threshold per lane (cm) - artifact detected when distance <= threshold */
        public double leftThresholdCm;
        public double centerThresholdCm;
        public double rightThresholdCm;
    }

    public static class Background {
        // Background detection - distinguishes empty space from artifacts
        /** Enable background similarity checking */
        public boolean enableBackgroundDetection = true;
        /** Background hue (set from calibration, typically ~40-60° for field mat) */
        public double backgroundHue = 50.0;
        /** Background saturation (set from calibration, typically low ~0.1-0.3) */
        public double backgroundSaturation = 0.20;
        /** Background value (set from calibration, brightness of empty space) */
        public double backgroundValue = 0.30;
        /** Maximum weighted distance to background to classify as BACKGROUND */
        public double maxBackgroundDistance = 40.0;
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
        config.centerThresholdCm = 3.85;
        config.rightThresholdCm = 4.5;
        return config;
    }
}
