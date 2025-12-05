package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Lane sensor configuration for artifact color detection.
 * Configures color sensors, classification algorithms, and presence detection.
 */
@Configurable
public class IntakeLaneSensorConfig {
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
    public Polling polling = new Polling();
    public Hardware hardware = new Hardware();
    public Gating gating = new Gating();
    public Quality quality = new Quality();
    public Presence presence = new Presence();
    public Background background = new Background();
    public Classifier classifier = new Classifier();
    public static LanePresenceConfig lanePresenceConfig19429 = createLanePresenceConfig19429();
    public static LanePresenceConfig lanePresenceConfig20245 = createLanePresenceConfig20245();

    @Configurable
    public static class Polling {
        public boolean enablePolling = true;
        public double samplePeriodMs = 150;
        public String leftSensor = "lane_left_color";
        public String centerSensor = "lane_center_color";
        public String rightSensor = "lane_right_color";
    }

    @Configurable
    public static class Hardware {
        /** Turn the onboard white LED on/off (applied at bind time) */
        public boolean enableSensorLight = true;
        /** If true, override sensor gain with sensorGain value; otherwise leave default */
        public boolean overrideSensorGain = true;
        /** Gain applied when overrideSensorGain is true (REV Color Sensor V3 typical range ~1-10) */
        public double sensorGain = 20.0;
    }

    @Configurable
    public static class Gating {
        /** Minimum confidence required to accept a new artifact color classification */
        public double minConfidenceToAccept = .2;
        /** Number of consecutive confident samples required before updating lane color */
        public int consecutiveConfirmationsRequired = 1;
        /** Number of consecutive non-artifact samples required before clearing lane color (helps with whiffle ball holes) */
        public int consecutiveClearConfirmationsRequired = 2;
        /** Keep-alive duration (ms) - keep artifact detection alive after last good reading (helps with whiffle ball holes) */
        public double keepAliveMs = 400.0;
        /** Distance clearance margin (cm) - how far beyond exit threshold to instantly clear (helps artifacts clear quickly when removed) */
        public double distanceClearanceMarginCm = .7;
    }

    @Configurable
    public static class Quality {
        // Quality thresholds (used by all classifiers)
        public double minValue = 0.02;
        public double minSaturation = 0.15;
    }

    @Configurable
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

    @Configurable
    public static class LanePresenceConfig {
        /** Enter/exit distance thresholds per lane (cm) with hysteresis */
        public double leftEnterDistanceCm;
        public double leftExitDistanceCm;
        public double centerEnterDistanceCm;
        public double centerExitDistanceCm;
        public double rightEnterDistanceCm;
        public double rightExitDistanceCm;
    }

    @Configurable
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

    @Configurable
    public static class Classifier {
        // Classifier mode selector
        public String mode = ClassifierMode.DECISION_BOUNDARY.name();

        public DecisionBoundary decision = new DecisionBoundary();
        public Range range = new Range();
        public DistanceBased distance = new DistanceBased();
    }

    @Configurable
    public static class DecisionBoundary {
        /** Hue decision boundary - classify as GREEN if hue < boundary, PURPLE otherwise */
        public double hueDecisionBoundary = 175.0;
        /** Distance from boundary for low confidence warning (degrees) */
        public double lowConfidenceMargin = 15.0;
    }

    @Configurable
    public static class Range {
        /** Range-based mode parameters (legacy) */
        public double greenHueMin = 80.0;
        public double greenHueMax = 160.0;
        public double purpleHueMin = 260.0;
        public double purpleHueMax = 330.0;
        public double purpleHueWrapMax = 40.0;
    }

    @Configurable
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
        config.leftEnterDistanceCm = 4.9;
        config.leftExitDistanceCm = 5.3;
        config.centerEnterDistanceCm = 6.7;
        config.centerExitDistanceCm = 7.3;
        config.rightEnterDistanceCm = 10;
        config.rightExitDistanceCm = 11;
        return config;
    }

    private static LanePresenceConfig createLanePresenceConfig20245() {
        LanePresenceConfig config = new LanePresenceConfig();
        // Start with the same defaults; tune via Dashboard per robot
        config.leftEnterDistanceCm = 6;
        config.leftExitDistanceCm = 6.3;
        config.centerEnterDistanceCm = 2.8;
        config.centerExitDistanceCm = 3.7;
        config.rightEnterDistanceCm = 3.6;
        config.rightExitDistanceCm = 4;
        return config;
    }
}
