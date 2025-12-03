package org.firstinspires.ftc.teamcode.subsystems.intake.config;

/**
 * Lane sensor configuration for artifact color detection.
 * Configures color sensors, classification algorithms, and presence detection.
 *
 * ========== TUNING GUIDE ==========
 * If you're tuning sensors, focus on these sections IN ORDER:
 *
 * 1. PRESENCE DETECTION - Tune per-lane distance thresholds (robot-specific!)
 *    → lanePresenceConfig19429 or lanePresenceConfig20245 (depending on robot)
 *    → Each lane has enter/exit thresholds with hysteresis
 *    → This is usually the main issue - getting reliable presence detection
 *
 * 2. COLOR CLASSIFICATION (classifier.decision) - Tune hue decision boundary
 *    → Only needs tuning if colors are misclassified
 *    → Run ArtifactColorCalibration OpMode to find optimal boundary
 *
 * Leave everything else at defaults unless you have a specific issue.
 */
public class IntakeLaneSensorConfig {
    /**
     * Artifact color classifier mode.
     * Determines which algorithm is used to classify GREEN vs PURPLE.
     */
    public enum ClassifierMode {
        /** Decision boundary: Single hue threshold between green and purple (RECOMMENDED - simple and robust) */
        DECISION_BOUNDARY,
        /** Distance-based: Euclidean distance in HSV space to color targets (ADVANCED - backup option) */
        DISTANCE_BASED
        // RANGE_BASED removed - legacy mode that's less robust than decision boundary
    }

    // ========== HARDWARE SETUP (rarely needs changing) ==========
    public Polling polling = new Polling();
    public Hardware hardware = new Hardware();

    // ========== PRESENCE DETECTION (tune these per robot!) ==========
    public static LanePresenceConfig lanePresenceConfig19429 = createLanePresenceConfig19429();
    public static LanePresenceConfig lanePresenceConfig20245 = createLanePresenceConfig20245();
    public Presence presence = new Presence();

    // ========== COLOR CLASSIFICATION (tune if colors are wrong) ==========
    public Classifier classifier = new Classifier();
    public Quality quality = new Quality();
    public Gating gating = new Gating();

    public static class Polling {
        public boolean enablePolling = true;
        public double samplePeriodMs = 200.0;
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

    public static class Gating {
        /** Minimum confidence required to accept a new artifact color classification */
        public double minConfidenceToAccept = 0.4;
        /** Number of consecutive confident samples required before updating lane color */
        public int consecutiveConfirmationsRequired = 2;
    }

    public static class Quality {
        // Quality thresholds (used by all classifiers)
        public double minValue = 0.02;
        public double minSaturation = 0.15;
    }

    public static class Presence {
        /**
         * Whether to use distance for presence gating.
         * If true, uses LanePresenceConfig enter/exit thresholds (robot-specific).
         * If false, relies only on color signal quality.
         */
        public boolean useDistance = true;
    }

    public static class LanePresenceConfig {
        /** Enter/exit distance thresholds per lane (cm) with hysteresis */
        public double leftEnterDistanceCm;
        public double leftExitDistanceCm;
        public double centerEnterDistanceCm;
        public double centerExitDistanceCm;
        public double rightEnterDistanceCm;
        public double rightExitDistanceCm;
    }

    public static class Classifier {
        // Classifier mode selector (DECISION_BOUNDARY recommended)
        public String mode = ClassifierMode.DECISION_BOUNDARY.name();

        // DECISION_BOUNDARY parameters (CURRENTLY ACTIVE - tune these)
        public DecisionBoundary decision = new DecisionBoundary();

        // DISTANCE_BASED parameters (ADVANCED - backup option, usually not needed)
        public DistanceBased distance = new DistanceBased();
    }

    public static class DecisionBoundary {
        // ========== PRIMARY TUNING PARAMETER ==========
        /**
         * Hue decision boundary - classify as GREEN if hue < boundary, PURPLE otherwise.
         * Typical range: 160-190 degrees.
         * Run ArtifactColorCalibration OpMode to find optimal value.
         */
        public double hueDecisionBoundary = 175.0;

        // ========== SECONDARY PARAMETER (rarely changed) ==========
        /**
         * Distance from boundary for low confidence warning (degrees).
         * Smaller = more strict confidence, larger = more lenient.
         */
        public double lowConfidenceMargin = 15.0;
    }

    public static class DistanceBased {
        // ========== ADVANCED - Only use if DECISION_BOUNDARY doesn't work ==========
        // Distance-based classifier computes weighted Euclidean distance in HSV space
        // to target colors. More complex but can handle variable lighting better.

        /** Weight for hue component in distance calculation (higher = hue matters more) */
        public double hueWeight = 2.0;
        /** Weight for saturation component in distance calculation */
        public double saturationWeight = 0.5;
        /** Weight for value component in distance calculation */
        public double valueWeight = 0.3;

        /** Target hue/sat/value for green artifacts (from calibration) */
        public double greenHueTarget = 120.0;
        public double greenSatTarget = 0.45;
        public double greenValTarget = 0.50;

        /** Target hue/sat/value for purple artifacts (from calibration) */
        public double purpleHueTarget = 290.0;
        public double purpleSatTarget = 0.40;
        public double purpleValTarget = 0.45;
    }

    private static LanePresenceConfig createLanePresenceConfig19429() {
        LanePresenceConfig config = new LanePresenceConfig();
        config.leftEnterDistanceCm = 4.5;
        config.leftExitDistanceCm = 5.1;
        config.centerEnterDistanceCm = 8.2;
        config.centerExitDistanceCm = 9.2;
        config.rightEnterDistanceCm = 12.5;
        config.rightExitDistanceCm = 13.3;
        return config;
    }

    private static LanePresenceConfig createLanePresenceConfig20245() {
        LanePresenceConfig config = new LanePresenceConfig();
        // Start with the same defaults; tune via Dashboard per robot
        config.leftEnterDistanceCm = 4.0;
        config.leftExitDistanceCm = 4.1;
        config.centerEnterDistanceCm = 5.0;
        config.centerExitDistanceCm = 5.5;
        config.rightEnterDistanceCm = 5.5;
        config.rightExitDistanceCm = 6.0;
        return config;
    }
}
