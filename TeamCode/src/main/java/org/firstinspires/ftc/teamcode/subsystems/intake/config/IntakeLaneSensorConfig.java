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

    public boolean enablePolling = true;
    public String leftSensor = "lane_left_color";
    public String centerSensor = "lane_center_color";
    public String rightSensor = "lane_right_color";
    public double samplePeriodMs = 200.0;

    // Classifier mode selector
    public String classifierMode = ClassifierMode.DECISION_BOUNDARY.name();

    // Quality thresholds (used by all classifiers)
    public double minValue = 0.02;
    public double minSaturation = 0.15;

    // RANGE_BASED mode parameters (legacy)
    public double greenHueMin = 80.0;
    public double greenHueMax = 160.0;
    public double purpleHueMin = 260.0;
    public double purpleHueMax = 330.0;
    public double purpleHueWrapMax = 40.0;

    // DECISION_BOUNDARY mode parameters (default, recommended)
    /** Typical green hue (set from calibration) */
    public double greenHueTarget = 120.0;
    /** Typical purple hue unwrapped (set from calibration, typically ~290°) */
    public double purpleHueTarget = 290.0;
    /** Hue decision boundary - classify as GREEN if hue < boundary, PURPLE otherwise */
    public double hueDecisionBoundary = 205.0;
    /** Distance from boundary for low confidence warning (degrees) */
    public double lowConfidenceMargin = 15.0;

    // DISTANCE_BASED mode parameters
    /** Weight for hue component in distance calculation */
    public double hueWeight = 2.0;
    /** Weight for saturation component in distance calculation */
    public double saturationWeight = 0.5;
    /** Weight for value component in distance calculation */
    public double valueWeight = 0.3;
    /** Target saturation for green artifacts */
    public double greenSatTarget = 0.45;
    /** Target value for green artifacts */
    public double greenValTarget = 0.50;
    /** Target saturation for purple artifacts */
    public double purpleSatTarget = 0.40;
    /** Target value for purple artifacts */
    public double purpleValTarget = 0.45;

    // Distance gating (used by all classifiers)
    public boolean useDistance = true;
    public double presenceDistanceCm = 5;

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

    // Multi-factor presence detection - improves artifact vs background discrimination
    /** Enable enhanced presence scoring (not just distance) */
    public boolean enablePresenceScoring = true;
    /** Minimum total RGB intensity for artifact presence (0-765 range) */
    public double minTotalIntensity = 50.0;
    /** Weight for distance factor in presence score */
    public double presenceDistanceWeight = 0.4;
    /** Weight for saturation factor in presence score */
    public double presenceSaturationWeight = 0.3;
    /** Weight for value factor in presence score */
    public double presenceValueWeight = 0.2;
    /** Weight for intensity factor in presence score */
    public double presenceIntensityWeight = 0.1;
    /** Minimum presence score (0-1) to consider artifact present */
    public double minPresenceScore = 0.5;
}
