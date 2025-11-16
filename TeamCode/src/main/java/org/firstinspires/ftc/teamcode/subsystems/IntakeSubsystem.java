package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeMode.STOPPED;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Hardware-facing intake wrapper. Handles motor power, roller servo, and lane sensor sampling
 * while higher-level coordinators decide what the subsystem should do.
 */
@Configurable
public class IntakeSubsystem implements Subsystem {

    public enum IntakeMode {
        PASSIVE_REVERSE,
        ACTIVE_FORWARD,
        STOPPED
    }

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

    @Configurable
    public static class LaneSensorConfig {
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
        public double presenceDistanceCm = 3.0;

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

    @Configurable
    public static class MotorConfig {
        public String motorName = "intake";
        public double defaultForwardPower = -.6;
        public double defaultReversePower = .1;
        public boolean brakeOnZero = true;
        public boolean reverseDirection = true;
    }

    @Configurable
    public static class RollerConfig {
        public String servoName = "intake_roller";
        public double activePosition = .5;
        public double inactivePosition = 0.5;
    }

    @Configurable
    public static class PrefeedConfig {
        public String servoName = "prefeed_roller";
        /** Reverse speed (default) - prevents accidental feeding (continuous servo: 0.0 = full reverse) */
        public double reversePosition = 0.0;
        /** Forward speed - helps feed artifacts when firing (continuous servo: 1.0 = full forward) */
        public double forwardPosition = 1.0;
    }

    @Configurable
    public static class ManualModeConfig {
        public boolean enableOverride = false;
        public String overrideMode = STOPPED.name();
    }

    public static LaneSensorConfig laneSensorConfig = new LaneSensorConfig();
    public static MotorConfig motorConfig = new MotorConfig();
    public static RollerConfig rollerConfig = new RollerConfig();
    public static PrefeedConfig prefeedConfig = new PrefeedConfig();
    public static ManualModeConfig manualModeConfig = new ManualModeConfig();

    public static final class LaneSample {
        public final boolean sensorPresent;
        public final boolean distanceAvailable;
        public final double distanceCm;
        public final boolean withinDistance;
        public final int rawRed;
        public final int rawGreen;
        public final int rawBlue;
        public final int scaledRed;
        public final int scaledGreen;
        public final int scaledBlue;
        public final float hue;
        public final float saturation;
        public final float value;
        public final ArtifactColor hsvColor;
        public final ArtifactColor color;
        /** Classification confidence (0.0 = uncertain, 1.0 = very confident) */
        public final double confidence;

        private LaneSample(boolean sensorPresent,
                           boolean distanceAvailable,
                           double distanceCm,
                           boolean withinDistance,
                           int rawRed, int rawGreen, int rawBlue,
                           int scaledRed, int scaledGreen, int scaledBlue,
                           float hue, float saturation, float value,
                           ArtifactColor hsvColor,
                           ArtifactColor color,
                           double confidence) {
            this.sensorPresent = sensorPresent;
            this.distanceAvailable = distanceAvailable;
            this.distanceCm = distanceCm;
            this.withinDistance = withinDistance;
            this.rawRed = rawRed;
            this.rawGreen = rawGreen;
            this.rawBlue = rawBlue;
            this.scaledRed = scaledRed;
            this.scaledGreen = scaledGreen;
            this.scaledBlue = scaledBlue;
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
            this.hsvColor = hsvColor == null ? ArtifactColor.NONE : hsvColor;
            this.color = color == null ? ArtifactColor.NONE : color;
            this.confidence = confidence;
        }

        private static LaneSample absent() {
            return new LaneSample(false, false, Double.NaN, false,
                    0, 0, 0, 0, 0, 0,
                    0.0f, 0.0f, 0.0f,
                    ArtifactColor.NONE, ArtifactColor.NONE, 0.0);
        }

        private static LaneSample present(boolean distanceAvailable,
                                          double distanceCm,
                                          boolean withinDistance,
                                          int rawRed, int rawGreen, int rawBlue,
                                          int scaledRed, int scaledGreen, int scaledBlue,
                                          float hue, float saturation, float value,
                                          ArtifactColor hsvColor,
                                          ArtifactColor color,
                                          double confidence) {
            return new LaneSample(true,
                    distanceAvailable,
                    distanceCm,
                    withinDistance,
                    rawRed, rawGreen, rawBlue,
                    scaledRed, scaledGreen, scaledBlue,
                    hue, saturation, value,
                    hsvColor, color, confidence);
        }
    }

    private static final LaneSample ABSENT_SAMPLE = LaneSample.absent();

    private final ElapsedTime sensorTimer = new ElapsedTime();

    private Alliance alliance = Alliance.UNKNOWN;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, NormalizedColorSensor> laneSensors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, DistanceSensor> laneDistanceSensors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, LaneSample> laneSamples = new EnumMap<>(LauncherLane.class);
    private final float[] hsvBuffer = new float[3];
    private final List<LaneColorListener> laneColorListeners = new ArrayList<>();
    private boolean anyLaneSensorsPresent = false;
    private double lastPeriodicMs = 0.0;
    private double lastModeResolveMs = 0.0;
    private double lastSensorPollMs = 0.0;
    private double lastServoUpdateMs = 0.0;
    private IntakeMode intakeMode = IntakeMode.STOPPED;
    private IntakeMode appliedMode = null;
    private final DcMotorEx intakeMotor;
    private double appliedMotorPower = 0.0;
    private final Servo rollerServo;
    private final Servo prefeedServo;
    private double lastRollerPosition = Double.NaN;
    private double lastPrefeedPosition = Double.NaN;
    private boolean rollerEnabled = false;
    private boolean prefeedEnabled = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = tryGetMotor(hardwareMap, motorConfig.motorName);
        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (motorConfig.brakeOnZero) {
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (motorConfig.reverseDirection) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            intakeMotor.setPower(0.0);
        }
        rollerServo = tryGetServo(hardwareMap, rollerConfig.servoName);
        if (rollerServo != null) {
            lastRollerPosition = rollerConfig.inactivePosition;
            rollerServo.setPosition(rollerConfig.inactivePosition);
        }
        rollerEnabled = false;
        prefeedServo = tryGetServo(hardwareMap, prefeedConfig.servoName);
        if (prefeedServo != null) {
            lastPrefeedPosition = prefeedConfig.reversePosition;
            prefeedServo.setPosition(prefeedConfig.reversePosition);
        }
        prefeedEnabled = false;

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
            laneSamples.put(lane, ABSENT_SAMPLE);
        }
        bindLaneSensors(hardwareMap);
    }

    @Override
    public void initialize() {
        clearLaneColors();
        for (LauncherLane lane : LauncherLane.values()) {
            laneSamples.put(lane, ABSENT_SAMPLE);
        }
        sensorTimer.reset();
        intakeMode = IntakeMode.STOPPED;
        appliedMode = IntakeMode.STOPPED;
        setManualPower(0.0);
        if (rollerServo != null) {
            rollerServo.setPosition(rollerConfig.inactivePosition);
            lastRollerPosition = rollerConfig.inactivePosition;
        }
        rollerEnabled = false;
        if (prefeedServo != null) {
            prefeedServo.setPosition(prefeedConfig.reversePosition);
            lastPrefeedPosition = prefeedConfig.reversePosition;
        }
        prefeedEnabled = false;
    }

    @Override
    public void periodic() {

        long start = System.nanoTime();

        long modeResolveStart = System.nanoTime();
        IntakeMode targetMode = resolveMode();
        if (appliedMode != targetMode) {
            applyModePower(targetMode);
            appliedMode = targetMode;
        }
        lastModeResolveMs = (System.nanoTime() - modeResolveStart) / 1_000_000.0;

        long sensorPollStart = System.nanoTime();
        pollLaneSensorsIfNeeded();
        lastSensorPollMs = (System.nanoTime() - sensorPollStart) / 1_000_000.0;

        long servoStart = System.nanoTime();
        if (rollerServo != null) {
            double target = rollerEnabled ? rollerConfig.activePosition : rollerConfig.inactivePosition;
            rollerServo.setPosition(target);
            lastRollerPosition = target;
        }
        if (prefeedServo != null) {
            double target = prefeedEnabled ? prefeedConfig.forwardPosition : prefeedConfig.reversePosition;
            prefeedServo.setPosition(target);
            lastPrefeedPosition = target;
        }

        lastServoUpdateMs = (System.nanoTime() - servoStart) / 1_000_000.0;
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public void stop() {
        setMode(IntakeMode.STOPPED);
        setManualPower(0.0);
        deactivateRoller();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void activateRoller() {
        rollerEnabled = true;
    }

    public void activatePrefeed() {
        prefeedEnabled = true;
    }

    public void deactivateRoller() {
        rollerEnabled = false;
        if (rollerServo != null) {
            rollerServo.setPosition(rollerConfig.inactivePosition);
            lastRollerPosition = rollerConfig.inactivePosition;
        }
    }

    public void deactivatePrefeed() {
        prefeedEnabled = false;
        if (prefeedServo != null) {
            prefeedServo.setPosition(prefeedConfig.reversePosition);
            lastPrefeedPosition = prefeedConfig.reversePosition;
        }
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void setManualPower(double normalizedPower) {
        double power = Range.clip(normalizedPower, -1.0, 1.0);
        appliedMotorPower = power;
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public double getCurrentPower() {
        return appliedMotorPower;
    }

    public IntakeMode getMode() {
        return intakeMode;
    }

    public void setMode(IntakeMode mode) {
        IntakeMode desired = mode == null ? IntakeMode.STOPPED : mode;
        if (intakeMode != desired) {
            intakeMode = desired;
            appliedMode = null;
        }
    }

    public void startForward() { setMode(IntakeMode.ACTIVE_FORWARD); }
    public void startReverse() { setMode(IntakeMode.PASSIVE_REVERSE); }

    public IntakeMode getResolvedMode() {
        return resolveMode();
    }

    public boolean isRollerPresent() {
        return rollerServo != null;
    }
    public boolean isPrefeedPresent() {
        return prefeedServo != null;
    }


    public double getRollerPosition() {
        return rollerServo == null ? Double.NaN : lastRollerPosition;
    }

    public double getPrefeedPosition() {
        return prefeedServo == null ? Double.NaN : lastPrefeedPosition;
    }

    public boolean isRollerActive() {
        return rollerServo != null
                && Math.abs(lastRollerPosition - rollerConfig.activePosition) < 1e-3;
    }

    public boolean isPrefeedActive() {
        return prefeedServo != null
                && Math.abs(lastPrefeedPosition - prefeedConfig.forwardPosition) < 1e-3;
    }

    private void pollLaneSensorsIfNeeded() {
        if (!laneSensorConfig.enablePolling || !anyLaneSensorsPresent) {
            return;
        }
        if (sensorTimer.milliseconds() < laneSensorConfig.samplePeriodMs) {
            return;
        }
        sensorTimer.reset();
        refreshLaneSensors();
    }

    private IntakeMode resolveMode() {
        if (manualModeConfig.enableOverride) {
            IntakeMode override = parseMode(manualModeConfig.overrideMode);
            if (override != null) {
                return override;
            }
        }
        return intakeMode;
    }

    private void applyModePower(IntakeMode mode) {
        if (mode == null) {
            mode = IntakeMode.STOPPED;
        }
        switch (mode) {
            case ACTIVE_FORWARD:
                setManualPower(motorConfig.defaultForwardPower);
                break;
            case STOPPED:
                setManualPower(0.0);
                break;
            case PASSIVE_REVERSE:
                setManualPower(motorConfig.defaultReversePower);
                break;
            default:
                break;
        }
    }


    private static IntakeMode parseMode(String value) {
        if (value == null || value.isEmpty()) {
            return null;
        }
        try {
            return IntakeMode.valueOf(value.toUpperCase(Locale.US));
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    public void refreshLaneSensors() {
        for (LauncherLane lane : LauncherLane.values()) {
            LaneSample sample = sampleLane(lane);
            laneSamples.put(lane, sample);
            updateLaneColor(lane, sample.color);
        }
    }

    private void bindLaneSensors(HardwareMap hardwareMap) {
        anyLaneSensorsPresent = false;
        laneSensors.put(LauncherLane.LEFT, tryGetColorSensor(hardwareMap, laneSensorConfig.leftSensor));
        laneDistanceSensors.put(LauncherLane.LEFT, tryGetDistanceSensor(hardwareMap, laneSensorConfig.leftSensor));
        laneSensors.put(LauncherLane.CENTER, tryGetColorSensor(hardwareMap, laneSensorConfig.centerSensor));
        laneDistanceSensors.put(LauncherLane.CENTER, tryGetDistanceSensor(hardwareMap, laneSensorConfig.centerSensor));
        laneSensors.put(LauncherLane.RIGHT, tryGetColorSensor(hardwareMap, laneSensorConfig.rightSensor));
        laneDistanceSensors.put(LauncherLane.RIGHT, tryGetDistanceSensor(hardwareMap, laneSensorConfig.rightSensor));
        for (NormalizedColorSensor sensor : laneSensors.values()) {
            if (sensor != null) {
                anyLaneSensorsPresent = true;
                break;
            }
        }
    }

    private static DcMotorEx tryGetMotor(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    private static NormalizedColorSensor tryGetColorSensor(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(NormalizedColorSensor.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    private static DistanceSensor tryGetDistanceSensor(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(DistanceSensor.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    private static Servo tryGetServo(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    private LaneSample sampleLane(LauncherLane lane) {
        NormalizedColorSensor colorSensor = laneSensors.get(lane);
        DistanceSensor distanceSensor = laneDistanceSensors.get(lane);
        if (colorSensor == null) {
            return ABSENT_SAMPLE;
        }

        boolean distanceAvailable = distanceSensor != null;
        double distanceCm = Double.NaN;
        if (distanceAvailable) {
            distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
        }

        boolean distanceValid = distanceAvailable && !Double.isNaN(distanceCm) && !Double.isInfinite(distanceCm);
        boolean withinDistance;
        if (!distanceAvailable || !laneSensorConfig.useDistance) {
            withinDistance = true;
        } else if (!distanceValid) {
            withinDistance = false;
        } else {
            withinDistance = distanceCm <= laneSensorConfig.presenceDistanceCm;
        }

        // SINGLE I2C read for all color channels (red, green, blue, alpha)
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert normalized values (0-1) to 0-255 range
        // NormalizedColorSensor already applies gain internally, so these are our scaled values
        int scaledRed = Math.min(255, Math.round(colors.red * 255.0f));
        int scaledGreen = Math.min(255, Math.round(colors.green * 255.0f));
        int scaledBlue = Math.min(255, Math.round(colors.blue * 255.0f));

        // For raw values, use the same normalized values scaled to 0-255
        int rawRed = scaledRed;
        int rawGreen = scaledGreen;
        int rawBlue = scaledBlue;

        int maxComponent = Math.max(scaledRed, Math.max(scaledGreen, scaledBlue));
        float hue = 0.0f;
        float saturation = 0.0f;
        float value = 0.0f;

        if (maxComponent > 0) {
            Color.RGBToHSV(scaledRed, scaledGreen, scaledBlue, hsvBuffer);
            hue = hsvBuffer[0];
            saturation = hsvBuffer[1];
            value = hsvBuffer[2];
        }

        // Compute total RGB intensity for presence detection
        int totalIntensity = scaledRed + scaledGreen + scaledBlue;

        // Classify color using selected classifier mode with enhanced presence/background detection
        ClassificationResult result = classifyColor(
                hue, saturation, value,
                maxComponent > 0,
                totalIntensity,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance
        );
        ArtifactColor hsvColor = result.color;
        double confidence = result.confidence;

        // Final color already accounts for distance, presence, and background checks
        ArtifactColor finalColor = hsvColor;

        return LaneSample.present(
                distanceAvailable,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance,
                rawRed, rawGreen, rawBlue,
                scaledRed, scaledGreen, scaledBlue,
                hue, saturation, value,
                hsvColor,
                finalColor,
                confidence
        );
    }

    /**
     * Classification result with color and confidence.
     */
    private static class ClassificationResult {
        final ArtifactColor color;
        final double confidence;  // 0.0 = very uncertain, 1.0 = very confident

        ClassificationResult(ArtifactColor color, double confidence) {
            this.color = color;
            this.confidence = confidence;
        }
    }

    /**
     * Classify color using the configured classifier mode with enhanced presence and background detection.
     *
     * @param hue Hue value (0-360°)
     * @param saturation Saturation value (0-1)
     * @param value Value/brightness (0-1)
     * @param hasSignal Whether sensor has a valid signal
     * @param totalIntensity Total RGB intensity (0-765)
     * @param distanceCm Distance reading in cm (or NaN if unavailable)
     * @param withinDistance Whether distance is within presence threshold
     * @return Classification result with color and confidence
     */
    private ClassificationResult classifyColor(float hue, float saturation, float value,
                                               boolean hasSignal, int totalIntensity,
                                               double distanceCm, boolean withinDistance) {
        // Basic quality check - no signal
        if (!hasSignal || value < laneSensorConfig.minValue || saturation < laneSensorConfig.minSaturation) {
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Multi-factor presence detection - is something actually there?
        if (laneSensorConfig.enablePresenceScoring) {
            double presenceScore = computePresenceScore(saturation, value, totalIntensity, distanceCm, withinDistance);
            if (presenceScore < laneSensorConfig.minPresenceScore) {
                // Not enough evidence that an artifact is present
                return new ClassificationResult(ArtifactColor.NONE, 0.0);
            }
        }

        // Background similarity check - does this look like empty space / field mat?
        if (laneSensorConfig.enableBackgroundDetection) {
            double backgroundDistance = computeBackgroundDistance(hue, saturation, value);
            if (backgroundDistance < laneSensorConfig.maxBackgroundDistance) {
                // Looks like background, not an artifact
                double confidence = 1.0 - (backgroundDistance / laneSensorConfig.maxBackgroundDistance);
                return new ClassificationResult(ArtifactColor.BACKGROUND, confidence);
            }
        }

        // Parse classifier mode
        ClassifierMode mode;
        try {
            mode = ClassifierMode.valueOf(laneSensorConfig.classifierMode.toUpperCase(Locale.US));
        } catch (IllegalArgumentException e) {
            mode = ClassifierMode.DECISION_BOUNDARY; // Default
        }

        // Route to appropriate color classifier (GREEN vs PURPLE)
        switch (mode) {
            case RANGE_BASED:
                return classifyColorRangeBased(hue, saturation, value, hasSignal);
            case DECISION_BOUNDARY:
                return classifyColorDecisionBoundary(hue, saturation, value, hasSignal);
            case DISTANCE_BASED:
                return classifyColorDistanceBased(hue, saturation, value, hasSignal);
            default:
                return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }
    }

    /**
     * Compute presence score - how confident are we that an artifact is actually present?
     * Combines distance, saturation, value, and total intensity.
     *
     * @return Presence score 0.0-1.0 (higher = more confident artifact is present)
     */
    private double computePresenceScore(float saturation, float value, int totalIntensity,
                                        double distanceCm, boolean withinDistance) {
        double score = 0.0;

        // Distance factor - is something close enough?
        if (!Double.isNaN(distanceCm) && laneSensorConfig.useDistance) {
            if (withinDistance) {
                // Linear scoring: closer = higher score
                double distanceFactor = 1.0 - (distanceCm / laneSensorConfig.presenceDistanceCm);
                score += laneSensorConfig.presenceDistanceWeight * Math.max(0.0, distanceFactor);
            }
            // else: distance factor contributes 0 (too far)
        } else {
            // No distance sensor or not using it - assume distance is OK
            score += laneSensorConfig.presenceDistanceWeight;
        }

        // Saturation factor - artifacts have more saturated colors than background
        if (saturation > laneSensorConfig.minSaturation) {
            double saturationFactor = Math.min(1.0, saturation / 0.5);  // Scale: 0-0.5 sat → 0-1 factor
            score += laneSensorConfig.presenceSaturationWeight * saturationFactor;
        }

        // Value factor - artifacts reflect more light than empty space
        if (value > laneSensorConfig.minValue) {
            double valueFactor = Math.min(1.0, value / 0.5);  // Scale: 0-0.5 val → 0-1 factor
            score += laneSensorConfig.presenceValueWeight * valueFactor;
        }

        // Intensity factor - total RGB brightness
        if (totalIntensity > laneSensorConfig.minTotalIntensity) {
            double intensityFactor = Math.min(1.0, totalIntensity / 200.0);  // Scale: 0-200 → 0-1
            score += laneSensorConfig.presenceIntensityWeight * intensityFactor;
        }

        return Math.min(1.0, score);  // Clamp to 0-1
    }

    /**
     * Compute weighted distance to background characteristics in HSV space.
     * Lower distance = more similar to background.
     *
     * @return Weighted distance to background (lower = more like background)
     */
    private double computeBackgroundDistance(float hue, float saturation, float value) {
        return hsvDistance(hue, saturation, value,
                laneSensorConfig.backgroundHue,
                laneSensorConfig.backgroundSaturation,
                laneSensorConfig.backgroundValue);
    }

    /**
     * RANGE_BASED classifier: Independent hue ranges for green and purple (legacy).
     * Returns UNKNOWN if hue is outside both ranges.
     */
    private ClassificationResult classifyColorRangeBased(float hue, float saturation, float value, boolean hasSignal) {
        if (!hasSignal || value < laneSensorConfig.minValue || saturation < laneSensorConfig.minSaturation) {
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Check green range
        if (hue >= laneSensorConfig.greenHueMin && hue <= laneSensorConfig.greenHueMax) {
            // Confidence based on how far from range edges
            double distFromMin = hue - laneSensorConfig.greenHueMin;
            double distFromMax = laneSensorConfig.greenHueMax - hue;
            double distFromEdge = Math.min(distFromMin, distFromMax);
            double confidence = Math.min(1.0, distFromEdge / 20.0); // 20° margin
            return new ClassificationResult(ArtifactColor.GREEN, confidence);
        }

        // Check purple range (with wrap-around)
        if ((hue >= laneSensorConfig.purpleHueMin && hue <= laneSensorConfig.purpleHueMax)
                || hue <= laneSensorConfig.purpleHueWrapMax) {
            double confidence = 0.7; // Fixed confidence for purple (wrap makes it harder to compute)
            return new ClassificationResult(ArtifactColor.PURPLE, confidence);
        }

        // Outside both ranges
        return new ClassificationResult(ArtifactColor.UNKNOWN, 0.0);
    }

    /**
     * DECISION_BOUNDARY classifier: Single hue threshold between green and purple (recommended).
     * Always returns GREEN or PURPLE, never UNKNOWN (two-class problem).
     */
    private ClassificationResult classifyColorDecisionBoundary(float hue, float saturation, float value, boolean hasSignal) {
        if (!hasSignal || value < laneSensorConfig.minValue || saturation < laneSensorConfig.minSaturation) {
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Unwrap hue to handle purple wrap-around
        // Purple spans 0° (e.g., 270-30°), so we map to continuous range
        // Green ~120°, Purple ~290° (unwrapped from 0-30° → 360-390°)
        float unwrappedHue = hue;
        if (hue < 90.0f) {  // Likely purple on the wrap side (0-40°)
            unwrappedHue = hue + 360.0f;  // Map 0-40° → 360-400°
        }

        // Classify based on which side of decision boundary
        ArtifactColor color;
        float distanceFromBoundary;

        if (unwrappedHue < laneSensorConfig.hueDecisionBoundary) {
            color = ArtifactColor.GREEN;
            distanceFromBoundary = (float) (laneSensorConfig.hueDecisionBoundary - unwrappedHue);
        } else {
            color = ArtifactColor.PURPLE;
            distanceFromBoundary = (float) (unwrappedHue - laneSensorConfig.hueDecisionBoundary);
        }

        // Confidence based on distance from boundary
        // High confidence if far from boundary, low if close
        double confidence = Math.min(1.0, distanceFromBoundary / laneSensorConfig.lowConfidenceMargin);

        return new ClassificationResult(color, confidence);
    }

    /**
     * DISTANCE_BASED classifier: Euclidean distance in HSV space to color targets.
     * Picks the closer target (green or purple).
     */
    private ClassificationResult classifyColorDistanceBased(float hue, float saturation, float value, boolean hasSignal) {
        if (!hasSignal || value < laneSensorConfig.minValue || saturation < laneSensorConfig.minSaturation) {
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Compute weighted distance to green target
        double distToGreen = hsvDistance(hue, saturation, value,
                laneSensorConfig.greenHueTarget,
                laneSensorConfig.greenSatTarget,
                laneSensorConfig.greenValTarget);

        // Compute weighted distance to purple target
        double distToPurple = hsvDistance(hue, saturation, value,
                laneSensorConfig.purpleHueTarget,
                laneSensorConfig.purpleSatTarget,
                laneSensorConfig.purpleValTarget);

        // Pick closer color
        ArtifactColor color = (distToGreen < distToPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;

        // Confidence based on separation between distances
        // High confidence if one is much closer than the other
        double minDist = Math.min(distToGreen, distToPurple);
        double maxDist = Math.max(distToGreen, distToPurple);
        double separation = maxDist - minDist;
        double confidence = Math.min(1.0, separation / 30.0); // 30° weighted separation for full confidence

        return new ClassificationResult(color, confidence);
    }

    /**
     * Compute weighted distance in HSV space, handling hue wrap-around.
     */
    private double hsvDistance(float hue, float saturation, float value,
                               double targetHue, double targetSat, double targetVal) {
        // Hue distance (circular, 0-360°)
        double hueDiff = Math.abs(hue - targetHue);
        if (hueDiff > 180.0) {
            hueDiff = 360.0 - hueDiff;  // Shortest path on circle
        }

        // Saturation and value differences (linear, 0-1)
        double satDiff = Math.abs(saturation - targetSat);
        double valDiff = Math.abs(value - targetVal);

        // Weighted Euclidean distance
        return laneSensorConfig.hueWeight * hueDiff
                + laneSensorConfig.saturationWeight * satDiff * 100.0  // Scale to ~0-100
                + laneSensorConfig.valueWeight * valDiff * 100.0;      // Scale to ~0-100
    }

    public boolean hasLaneSensors() {
        return anyLaneSensorsPresent;
    }

    public void addLaneColorListener(LaneColorListener listener) {
        if (listener == null || laneColorListeners.contains(listener)) {
            return;
        }
        laneColorListeners.add(listener);
        for (Map.Entry<LauncherLane, ArtifactColor> entry : laneColors.entrySet()) {
            listener.onLaneColorChanged(entry.getKey(), entry.getValue());
        }
    }

    public void removeLaneColorListener(LaneColorListener listener) {
        laneColorListeners.remove(listener);
    }

    public void clearLaneColors() {
        for (LauncherLane lane : LauncherLane.values()) {
            updateLaneColor(lane, ArtifactColor.NONE);
        }
    }

    public void clearLaneColor(LauncherLane lane) {
        updateLaneColor(lane, ArtifactColor.NONE);
    }

    public void updateLaneColor(LauncherLane lane, ArtifactColor color) {
        if (lane == null) {
            return;
        }
        ArtifactColor normalized = color == null ? ArtifactColor.NONE : color;
        ArtifactColor previous = laneColors.put(lane, normalized);
        if (previous == normalized) {
            return;
        }
        for (LaneColorListener listener : laneColorListeners) {
            listener.onLaneColorChanged(lane, normalized);
        }
    }

    public ArtifactColor getLaneColor(LauncherLane lane) {
        return lane == null ? ArtifactColor.NONE : laneColors.getOrDefault(lane, ArtifactColor.NONE);
    }

    public LaneSample getLaneSample(LauncherLane lane) {
        if (lane == null) {
            return ABSENT_SAMPLE;
        }
        return laneSamples.getOrDefault(lane, ABSENT_SAMPLE);
    }

    public EnumMap<LauncherLane, LaneSample> getLaneSampleSnapshot() {
        EnumMap<LauncherLane, LaneSample> copy = new EnumMap<>(LauncherLane.class);
        for (LauncherLane lane : LauncherLane.values()) {
            copy.put(lane, laneSamples.getOrDefault(lane, ABSENT_SAMPLE));
        }
        return copy;
    }

    public EnumMap<LauncherLane, ArtifactColor> getLaneColorSnapshot() {
        return new EnumMap<>(laneColors);
    }
    public LauncherLane[] planSequence(ArtifactColor... desiredPattern) {
        if (desiredPattern == null || desiredPattern.length == 0) {
            return new LauncherLane[0];
        }
        EnumMap<LauncherLane, ArtifactColor> remaining = new EnumMap<>(laneColors);
        List<LauncherLane> order = new ArrayList<>();

        for (ArtifactColor desired : desiredPattern) {
            LauncherLane lane = takeLaneWithColor(remaining, desired);
            if (lane != null) {
                order.add(lane);
                remaining.put(lane, ArtifactColor.NONE);
            }
        }
        return order.toArray(new LauncherLane[0]);
    }

    private static LauncherLane takeLaneWithColor(EnumMap<LauncherLane, ArtifactColor> pool, ArtifactColor desired) {
        ArtifactColor target = desired == null ? ArtifactColor.NONE : desired;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor laneColor = pool.getOrDefault(lane, ArtifactColor.NONE);
            if (laneColor == target) {
                return lane;
            }
        }
        if (target == ArtifactColor.UNKNOWN) {
            for (LauncherLane lane : LauncherLane.values()) {
                ArtifactColor laneColor = pool.getOrDefault(lane, ArtifactColor.NONE);
                if (laneColor == ArtifactColor.NONE) {
                    return lane;
                }
            }
        }
        return null;
    }

    public boolean isFull() {
        int count = 0;
        for (LauncherLane lane : LauncherLane.values()) {
            if (getLaneSample(lane).withinDistance) count++;
        }
        return count >= 3;
    }

    public interface LaneColorListener {
        void onLaneColorChanged(LauncherLane lane, ArtifactColor color);
    }

}
