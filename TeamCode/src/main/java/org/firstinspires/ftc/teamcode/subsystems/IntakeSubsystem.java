package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeMode.STOPPED;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeGateConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeLaneSensorConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeLaneSensorConfig.ClassifierMode;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeManualModeConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeMotorConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeRollerConfig;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CircularMovingAverageFilter;
import org.firstinspires.ftc.teamcode.util.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.util.RobotConfigs;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Hardware-facing intake wrapper. Handles motor power, roller servo, and lane sensor sampling
 * while higher-level coordinators decide what the subsystem should do.
 */
public class IntakeSubsystem implements Subsystem {

    public enum IntakeMode {
        PASSIVE_REVERSE,
        AGGRESSIVE_REVERSE,
        ACTIVE_FORWARD,
        STOPPED
    }

    // Active robot indicator
    public static String ACTIVE_ROBOT = RobotState.getRobotName();

    // Global configuration instances
    public static IntakeLaneSensorConfig laneSensorConfig = new IntakeLaneSensorConfig();
    public static IntakeMotorConfig motorConfig = new IntakeMotorConfig();
    public static IntakeRollerConfig rollerConfig = new IntakeRollerConfig();
    public static IntakeManualModeConfig manualModeConfig = new IntakeManualModeConfig();

    // Robot-specific gate configs
    public static IntakeGateConfig gateConfig_Robot19429 = IntakeGateConfig.gateConfig19429;
    public static IntakeGateConfig gateConfig_Robot20245 = IntakeGateConfig.gateConfig20245;
    public static IntakeGateConfig gateConfig_ACTIVE = RobotConfigs.getGateConfig();

    // Robot-specific lane presence configs (distance thresholds for artifact detection)
    public static IntakeLaneSensorConfig.LanePresenceConfig lanePresenceConfig_Robot19429 = IntakeLaneSensorConfig.lanePresenceConfig19429;
    public static IntakeLaneSensorConfig.LanePresenceConfig lanePresenceConfig_Robot20245 = IntakeLaneSensorConfig.lanePresenceConfig20245;
    public static IntakeLaneSensorConfig.LanePresenceConfig lanePresenceConfig_ACTIVE = RobotConfigs.getLanePresenceConfig();

    /**
     * Gets the robot-specific GateConfig based on RobotState.getRobotName().
     * @return gateConfig19429 or gateConfig20245
     */
    public static IntakeGateConfig gateConfig() {
        return RobotConfigs.getGateConfig();
    }

    public static final class LaneSample {
        public final boolean sensorPresent;
        public final boolean distanceAvailable;
        public final double distanceCm;
        public final boolean withinDistance;
        public final int scaledRed;
        public final int scaledGreen;
        public final int scaledBlue;
        /** Normalized channel values directly from the sensor (0.0-1.0) */
        public final float normalizedRed;
        public final float normalizedGreen;
        public final float normalizedBlue;
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
                           int scaledRed, int scaledGreen, int scaledBlue,
                           float normalizedRed, float normalizedGreen, float normalizedBlue,
                           float hue, float saturation, float value,
                           ArtifactColor hsvColor,
                           ArtifactColor color,
                           double confidence) {
            this.sensorPresent = sensorPresent;
            this.distanceAvailable = distanceAvailable;
            this.distanceCm = distanceCm;
            this.withinDistance = withinDistance;
            this.scaledRed = scaledRed;
            this.scaledGreen = scaledGreen;
            this.scaledBlue = scaledBlue;
            this.normalizedRed = normalizedRed;
            this.normalizedGreen = normalizedGreen;
            this.normalizedBlue = normalizedBlue;
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
            this.hsvColor = hsvColor == null ? ArtifactColor.NONE : hsvColor;
            this.color = color == null ? ArtifactColor.NONE : color;
            this.confidence = confidence;
        }

        private static LaneSample absent() {
            return new LaneSample(false, false, Double.NaN, false,
                    0, 0, 0,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    ArtifactColor.NONE, ArtifactColor.NONE, 0.0);
        }

        private static LaneSample present(boolean distanceAvailable,
                                          double distanceCm,
                                          boolean withinDistance,
                                          int scaledRed, int scaledGreen, int scaledBlue,
                                          float normalizedRed, float normalizedGreen, float normalizedBlue,
                                          float hue, float saturation, float value,
                                          ArtifactColor hsvColor,
                                          ArtifactColor color,
                                          double confidence) {
            return new LaneSample(true,
                    distanceAvailable,
                    distanceCm,
                    withinDistance,
                    scaledRed, scaledGreen, scaledBlue,
                    normalizedRed, normalizedGreen, normalizedBlue,
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
    private final EnumMap<LauncherLane, MovingAverageFilter> laneDistanceFilters = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, CircularMovingAverageFilter> laneHueFilters = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, LaneSample> laneSamples = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Boolean> lanePresenceState = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> laneCandidateColor = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Integer> laneCandidateCount = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneLastGoodDetectionMs = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Integer> laneClearCandidateCount = new EnumMap<>(LauncherLane.class);
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
    private final Servo gateServo;
    private double lastRollerPosition = Double.NaN;
    private double lastGatePosition = Double.NaN;
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

        gateServo = tryGetServo(hardwareMap, gateConfig().servoName);
        if (gateServo != null) {
            gateServo.setPosition(gateConfig().allowArtifacts);
            lastGatePosition = gateConfig().allowArtifacts;
        }

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
            laneSamples.put(lane, ABSENT_SAMPLE);
            lanePresenceState.put(lane, false);
            laneCandidateColor.put(lane, ArtifactColor.NONE);
            laneCandidateCount.put(lane, 0);
            laneLastGoodDetectionMs.put(lane, 0.0);
            laneClearCandidateCount.put(lane, 0);
            // Initialize distance filters with configured window size
            laneDistanceFilters.put(lane, new MovingAverageFilter(laneSensorConfig.distanceFilter.windowSize));
            // Initialize hue filters with configured window size
            laneHueFilters.put(lane, new CircularMovingAverageFilter(laneSensorConfig.hueFilter.windowSize));
        }
        bindLaneSensors(hardwareMap);
    }

    @Override
    public void initialize() {
        clearLaneColors();
        for (LauncherLane lane : LauncherLane.values()) {
            laneSamples.put(lane, ABSENT_SAMPLE);
            lanePresenceState.put(lane, false);
            laneCandidateColor.put(lane, ArtifactColor.NONE);
            laneCandidateCount.put(lane, 0);
            laneLastGoodDetectionMs.put(lane, 0.0);
            laneClearCandidateCount.put(lane, 0);
            // Reset distance filters (recreate with current config in case window size changed)
            laneDistanceFilters.put(lane, new MovingAverageFilter(laneSensorConfig.distanceFilter.windowSize));
            // Reset hue filters (recreate with current config in case window size changed)
            laneHueFilters.put(lane, new CircularMovingAverageFilter(laneSensorConfig.hueFilter.windowSize));
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
        if (gateServo != null) {
            gateServo.setPosition(gateConfig().allowArtifacts);
            lastGatePosition = gateConfig().allowArtifacts;
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

        lastServoUpdateMs = (System.nanoTime() - servoStart) / 1_000_000.0;
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public void stop() {
        setMode(IntakeMode.STOPPED);
        // Wrap hardware operations in try-catch to prevent "expansion hub stopped responding"
        // errors during OpMode shutdown when the hub communication is already terminating
        try {
            setManualPower(0.0);
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        try {
            deactivateRoller();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void forwardRoller() {
        rollerEnabled = true;
        if (rollerServo != null) {
            rollerServo.setPosition(rollerConfig.forward);
            lastRollerPosition = rollerConfig.forward;
        }
    }

    public void reverseRoller() {
        rollerEnabled = true;
        if (rollerServo != null) {
            rollerServo.setPosition(rollerConfig.reverse);
            lastRollerPosition = rollerConfig.reverse;
        }
    }

    public void deactivateRoller() {
        rollerEnabled = false;
        if (rollerServo != null) {
            rollerServo.setPosition(rollerConfig.inactivePosition);
            lastRollerPosition = rollerConfig.inactivePosition;
        }
    }


    public void setGateReverseConfig() {
        if (gateServo != null) {
            gateServo.setPosition(gateConfig().reverseConfig);
            lastGatePosition = gateConfig().reverseConfig;
        }
    }

    public void setGateAllowArtifacts() {
        if (gateServo != null) {
            gateServo.setPosition(gateConfig().allowArtifacts);
            lastGatePosition = gateConfig().allowArtifacts;
        }
    }

    public void setGatePreventArtifact() {
        if (gateServo != null) {
            gateServo.setPosition(gateConfig().preventArtifacts);
            lastGatePosition = gateConfig().preventArtifacts;
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
    public void startReverseIntakeMotor() { setMode(IntakeMode.PASSIVE_REVERSE); }
    public void startEject() { setMode(IntakeMode.AGGRESSIVE_REVERSE); }


    public IntakeMode getResolvedMode() {
        return resolveMode();
    }

    public boolean isRollerPresent() {
        return rollerServo != null;
    }
    public boolean isPrefeedPresent() {
        return gateServo != null;
    }


    public double getRollerPosition() {
        return rollerServo == null ? Double.NaN : lastRollerPosition;
    }

    public double getGatePosition() {
        return gateServo == null ? Double.NaN : lastGatePosition;
    }

    public boolean isRollerActive() {
        return rollerServo != null
                && Math.abs(lastRollerPosition - rollerConfig.forward) < 1e-3;
    }

    private void pollLaneSensorsIfNeeded() {
        if (!laneSensorConfig.polling.enablePolling || !anyLaneSensorsPresent) {
            return;
        }
        if (sensorTimer.milliseconds() < laneSensorConfig.polling.samplePeriodMs) {
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
            case AGGRESSIVE_REVERSE:
                setManualPower(motorConfig.aggressiveReversePower);
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
            applyGatedLaneColor(lane, sample);
        }
    }

    /**
     * Applies debounce + confidence gating before updating lane color to reduce flicker from holes/background.
     * Includes temporal hysteresis (keep-alive) to handle whiffle ball holes.
     */
    private void applyGatedLaneColor(LauncherLane lane, LaneSample sample) {
        if (lane == null || sample == null) {
            return;
        }

        ArtifactColor candidate = sample.color == null ? ArtifactColor.NONE : sample.color;
        ArtifactColor currentLaneColor = laneColors.getOrDefault(lane, ArtifactColor.NONE);
        double currentTimeMs = sensorTimer.milliseconds();

        // --- CHECK IF ARTIFACT IS CLEARLY GONE (distance-based override) ---
        // If distance shows artifact is beyond threshold + margin, it's definitely gone - clear immediately
        // Skip this check when using hue-based presence detection (hue is the authority, not distance)
        if (!laneSensorConfig.presence.useHuePresence
                && currentLaneColor.isArtifact() && sample.distanceAvailable && !Double.isNaN(sample.distanceCm)) {
            IntakeLaneSensorConfig.LanePresenceConfig presenceCfg = RobotConfigs.getLanePresenceConfig();
            double threshold = getThreshold(presenceCfg, lane);
            double clearanceMargin = laneSensorConfig.gating.distanceClearanceMarginCm;

            if (sample.distanceCm > threshold + clearanceMargin) {
                // Artifact is definitely gone - clear immediately regardless of keep-alive
                laneCandidateColor.put(lane, ArtifactColor.NONE);
                laneCandidateCount.put(lane, 0);
                laneClearCandidateCount.put(lane, 0);
                laneLastGoodDetectionMs.put(lane, 0.0);
                updateLaneColor(lane, ArtifactColor.NONE);
                return;
            }
        }

        // --- ARTIFACT DETECTION PATH ---
        if (candidate.isArtifact() && sample.confidence >= laneSensorConfig.gating.minConfidenceToAccept) {
            // Good artifact reading - record timestamp ONLY if not currently in clearing process
            int clearCount = laneClearCandidateCount.getOrDefault(lane, 0);

            // If we're already trying to clear (clearCount > 0), don't reset the timer
            // This prevents a whiffle ball from "reviving" detection after we've started clearing
            if (clearCount == 0) {
                laneLastGoodDetectionMs.put(lane, currentTimeMs);
            }

            // Standard debounce logic for artifact confirmation
            ArtifactColor previousCandidate = laneCandidateColor.getOrDefault(lane, ArtifactColor.NONE);
            int count = laneCandidateCount.getOrDefault(lane, 0);

            if (candidate == previousCandidate) {
                count++;
            } else {
                previousCandidate = candidate;
                count = 1;
            }

            laneCandidateColor.put(lane, previousCandidate);
            laneCandidateCount.put(lane, count);

            if (count >= Math.max(1, laneSensorConfig.gating.consecutiveConfirmationsRequired)) {
                updateLaneColor(lane, previousCandidate);
                // Reset clear counter when we successfully detect/confirm
                laneClearCandidateCount.put(lane, 0);
            }
            return;
        }

        // --- NON-ARTIFACT / CLEARING PATH ---
        // We got a bad reading (NONE, BACKGROUND, UNKNOWN, or low confidence)

        // If we currently have an artifact detected, check keep-alive window
        if (currentLaneColor.isArtifact()) {
            double lastGoodMs = laneLastGoodDetectionMs.getOrDefault(lane, 0.0);
            double timeSinceGoodMs = currentTimeMs - lastGoodMs;

            // Within keep-alive window? Keep current detection alive (likely just hit a hole)
            if (timeSinceGoodMs < laneSensorConfig.gating.keepAliveMs) {
                // Keep alive - don't clear, don't update counters
                return;
            }

            // Outside keep-alive window - start counting consecutive clear confirmations
            int clearCount = laneClearCandidateCount.getOrDefault(lane, 0);
            clearCount++;
            laneClearCandidateCount.put(lane, clearCount);

            // Only clear after N consecutive bad samples (symmetric with detection)
            if (clearCount >= Math.max(1, laneSensorConfig.gating.consecutiveClearConfirmationsRequired)) {
                // Confirmed absent - clear detection
                laneCandidateColor.put(lane, ArtifactColor.NONE);
                laneCandidateCount.put(lane, 0);
                laneClearCandidateCount.put(lane, 0);
                laneLastGoodDetectionMs.put(lane, 0.0);
                updateLaneColor(lane, ArtifactColor.NONE);
            }
        } else {
            // No artifact currently detected - just clear counters
            laneCandidateColor.put(lane, ArtifactColor.NONE);
            laneCandidateCount.put(lane, 0);
            laneClearCandidateCount.put(lane, 0);
        }
    }

    private void bindLaneSensors(HardwareMap hardwareMap) {
        anyLaneSensorsPresent = false;
        laneSensors.put(LauncherLane.LEFT, tryGetColorSensor(hardwareMap, laneSensorConfig.polling.leftSensor));
        laneDistanceSensors.put(LauncherLane.LEFT, tryGetDistanceSensor(hardwareMap, laneSensorConfig.polling.leftSensor));
        applyLaneSensorSettings(laneSensors.get(LauncherLane.LEFT));

        laneSensors.put(LauncherLane.CENTER, tryGetColorSensor(hardwareMap, laneSensorConfig.polling.centerSensor));
        laneDistanceSensors.put(LauncherLane.CENTER, tryGetDistanceSensor(hardwareMap, laneSensorConfig.polling.centerSensor));
        applyLaneSensorSettings(laneSensors.get(LauncherLane.CENTER));

        laneSensors.put(LauncherLane.RIGHT, tryGetColorSensor(hardwareMap, laneSensorConfig.polling.rightSensor));
        laneDistanceSensors.put(LauncherLane.RIGHT, tryGetDistanceSensor(hardwareMap, laneSensorConfig.polling.rightSensor));
        applyLaneSensorSettings(laneSensors.get(LauncherLane.RIGHT));
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

    private void applyLaneSensorSettings(NormalizedColorSensor sensor) {
        if (sensor == null) {
            return;
        }
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(laneSensorConfig.hardware.enableSensorLight);
        }
        if (laneSensorConfig.hardware.overrideSensorGain) {
            try {
                sensor.setGain((float) laneSensorConfig.hardware.sensorGain);
            } catch (IllegalArgumentException | UnsupportedOperationException ignored) {
                // Ignore invalid gain values to avoid crashing during onInit
            }
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

    private static double getThreshold(IntakeLaneSensorConfig.LanePresenceConfig config, LauncherLane lane) {
        switch (lane) {
            case LEFT:
                return config.leftThresholdCm;
            case CENTER:
                return config.centerThresholdCm;
            case RIGHT:
                return config.rightThresholdCm;
            default:
                return config.centerThresholdCm;
        }
    }

    private LaneSample sampleLane(LauncherLane lane) {
        NormalizedColorSensor colorSensor = laneSensors.get(lane);
        DistanceSensor distanceSensor = laneDistanceSensors.get(lane);
        if (colorSensor == null) {
            return ABSENT_SAMPLE;
        }

        boolean distanceAvailable = distanceSensor != null;
        double rawDistanceCm = Double.NaN;
        double distanceCm = Double.NaN;
        if (distanceAvailable) {
            rawDistanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            // Apply moving average filter if enabled
            if (laneSensorConfig.distanceFilter.enableFilter) {
                MovingAverageFilter filter = laneDistanceFilters.get(lane);
                if (filter != null) {
                    distanceCm = filter.calculate(rawDistanceCm);
                } else {
                    distanceCm = rawDistanceCm;
                }
            } else {
                distanceCm = rawDistanceCm;
            }
        }

        boolean distanceValid = distanceAvailable && !Double.isNaN(distanceCm) && !Double.isInfinite(distanceCm);
        boolean withinDistance = false;

        // Simple threshold check - moving average filter handles noise smoothing
        if (!distanceAvailable || !laneSensorConfig.presence.useDistance || !distanceValid) {
            withinDistance = false;
        } else {
            IntakeLaneSensorConfig.LanePresenceConfig presenceCfg = RobotConfigs.getLanePresenceConfig();
            double threshold = getThreshold(presenceCfg, lane);
            withinDistance = distanceCm <= threshold;
        }
        lanePresenceState.put(lane, withinDistance);

        // SINGLE I2C read for all color channels (red, green, blue, alpha)
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normalizedRed = colors.red;
        float normalizedGreen = colors.green;
        float normalizedBlue = colors.blue;

        // Convert normalized values (0-1) to 0-255 range without losing fractional detail
        // NormalizedColorSensor already applies gain internally, so these are our scaled values
        float scaledRedFloat = Math.min(255.0f, normalizedRed * 255.0f);
        float scaledGreenFloat = Math.min(255.0f, normalizedGreen * 255.0f);
        float scaledBlueFloat = Math.min(255.0f, normalizedBlue * 255.0f);
        int scaledRed = Math.round(scaledRedFloat);
        int scaledGreen = Math.round(scaledGreenFloat);
        int scaledBlue = Math.round(scaledBlueFloat);

        float maxComponent = Math.max(scaledRedFloat, Math.max(scaledGreenFloat, scaledBlueFloat));
        float rawHue = 0.0f;
        float saturation = 0.0f;
        float value = 0.0f;

        if (maxComponent > 0) {
            Color.RGBToHSV(scaledRed, scaledGreen, scaledBlue, hsvBuffer);
            rawHue = hsvBuffer[0];
            saturation = hsvBuffer[1];
            value = hsvBuffer[2];
        }

        // Apply circular moving average filter to hue for smoother classification
        float filteredHue = rawHue;
        if (laneSensorConfig.hueFilter.enableFilter && maxComponent > 0) {
            CircularMovingAverageFilter hueFilter = laneHueFilters.get(lane);
            if (hueFilter != null) {
                // Jump detection: if raw hue differs significantly from filtered, snap to new value
                double jumpThreshold = laneSensorConfig.hueFilter.jumpThreshold;
                if (jumpThreshold > 0 && hueFilter.getSampleCount() > 0) {
                    double currentFiltered = hueFilter.get();
                    double hueDiff = Math.abs(rawHue - currentFiltered);
                    // Handle circular wrap-around (e.g., 350° vs 10° = 20° difference, not 340°)
                    if (hueDiff > 180.0) {
                        hueDiff = 360.0 - hueDiff;
                    }
                    if (hueDiff > jumpThreshold) {
                        // Large jump detected - prime filter with new value for instant response
                        hueFilter.prime(rawHue);
                    }
                }
                filteredHue = (float) hueFilter.calculate(rawHue);
            }
        }

        // Override withinDistance based on hue if hue-based presence detection is enabled
        // This ensures isFull() and lanePresenceState work correctly with hue-based detection
        if (laneSensorConfig.presence.useHuePresence) {
            withinDistance = filteredHue >= laneSensorConfig.presence.huePresenceThreshold;
            lanePresenceState.put(lane, withinDistance);
        }

        // Compute total RGB intensity for presence detection (preserve fractional contribution)
        int totalIntensity = Math.round(scaledRedFloat + scaledGreenFloat + scaledBlueFloat);

        String lanePrefix = lane == null ? "unknown" : lane.name().toLowerCase(Locale.US);

        // Classify color using selected classifier mode with enhanced presence/background detection
        // Use filtered hue for stable classification
        ClassificationResult result = classifyColor(
                filteredHue, saturation, value,
                maxComponent > 0,
                totalIntensity,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance,
                lanePrefix
        );
        ArtifactColor hsvColor = result.color;
        double confidence = result.confidence;

        // Final color already accounts for distance, presence, and background checks
        ArtifactColor finalColor = hsvColor;

        // Publish raw sensor metrics every poll so we can see empty vs artifact behavior even when classification fails
        RobotState.packet.put("intake/sample/" + lanePrefix + "/sensor_present", true);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/distance_raw_cm", distanceAvailable ? rawDistanceCm : Double.NaN);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/distance_cm", distanceValid ? distanceCm : Double.NaN);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/within_distance", withinDistance);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/scaled_r", scaledRed);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/scaled_g", scaledGreen);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/scaled_b", scaledBlue);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/norm_r", normalizedRed);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/norm_g", normalizedGreen);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/norm_b", normalizedBlue);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/hue_raw", rawHue);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/hue", filteredHue);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/sat", saturation);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/val", value);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/total_intensity", totalIntensity);

        return LaneSample.present(
                distanceAvailable,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance,
                scaledRed, scaledGreen, scaledBlue,
                normalizedRed, normalizedGreen, normalizedBlue,
                filteredHue, saturation, value,
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
                                               double distanceCm, boolean withinDistance,
                                               String lanePrefix) {
        String reason = "classified";

        // Hue-based presence detection: use filtered hue to determine if artifact is present
        // Hue < threshold = definitely empty (background), Hue >= threshold = artifact present
        if (laneSensorConfig.presence.useHuePresence) {
            boolean hueIndicatesPresent = hue >= laneSensorConfig.presence.huePresenceThreshold;
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/hue_presence_enabled", true);
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/hue_presence_threshold", laneSensorConfig.presence.huePresenceThreshold);
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/hue_indicates_present", hueIndicatesPresent);

            if (!hueIndicatesPresent) {
                RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_score", 0.0);
                reason = "hue_below_threshold";
                RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);
                return new ClassificationResult(ArtifactColor.NONE, 0.0);
            }
            // Hue indicates artifact is present - skip distance check and proceed to classification
        } else if (laneSensorConfig.presence.useDistance && !withinDistance) {
            // Distance-only presence gate: if we're outside hysteresis window, treat as empty.
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_score", 0.0);
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_distance_valid", !Double.isNaN(distanceCm));
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_within_distance", false);
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_total_intensity", totalIntensity);
            reason = "distance_out";
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Basic quality check - no signal
        // Skip this check when using hue-based presence (hue is the authority)
        if (!laneSensorConfig.presence.useHuePresence
                && (!hasSignal || value < laneSensorConfig.quality.minValue || saturation < laneSensorConfig.quality.minSaturation)) {
            reason = "no_signal";
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Parse classifier mode
        ClassifierMode mode;
        try {
            mode = ClassifierMode.valueOf(laneSensorConfig.classifier.mode.toUpperCase(Locale.US));
        } catch (IllegalArgumentException e) {
            mode = ClassifierMode.DECISION_BOUNDARY; // Default
        }
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/mode", mode.name());

        // Route to appropriate color classifier (GREEN vs PURPLE)
        ClassificationResult result;
        switch (mode) {
            case DECISION_BOUNDARY:
                result = classifyColorDecisionBoundary(hue, saturation, value, hasSignal, lanePrefix);
                break;
            case DISTANCE_BASED:
                result = classifyColorDistanceBased(hue, saturation, value, hasSignal, lanePrefix);
                break;
            default:
                // Fallback to decision boundary if mode is invalid
                result = classifyColorDecisionBoundary(hue, saturation, value, hasSignal, lanePrefix);
                break;
        }

        // Tag the reason as the chosen color to make empty vs artifact easier to spot on the dashboard
        reason = result.color == null ? "none" : result.color.name();
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);

        return result;
    }

    /**
     * DECISION_BOUNDARY classifier: Single hue threshold between green and purple (recommended).
     * Always returns GREEN or PURPLE, never UNKNOWN (two-class problem).
     */
    private ClassificationResult classifyColorDecisionBoundary(float hue, float saturation, float value, boolean hasSignal, String lanePrefix) {
        if (!hasSignal || value < laneSensorConfig.quality.minValue || saturation < laneSensorConfig.quality.minSaturation) {
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Unwrap hue to handle purple wrap-around
        // Purple spans across 0°; we map low-end hues onto the high side relative to the boundary.
        float unwrappedHue = hue;
        // Dynamic wrap window: anything more than 180° below the boundary is considered wrap-side purple
        float wrapThreshold = (float) (laneSensorConfig.classifier.decision.hueDecisionBoundary - 180.0);
        if (hue < wrapThreshold) {
            unwrappedHue = hue + 360.0f;
        }

        // Classify based on which side of decision boundary
        ArtifactColor color;
        float distanceFromBoundary;

        if (unwrappedHue < laneSensorConfig.classifier.decision.hueDecisionBoundary) {
            color = ArtifactColor.GREEN;
            distanceFromBoundary = (float) (laneSensorConfig.classifier.decision.hueDecisionBoundary - unwrappedHue);
        } else {
            color = ArtifactColor.PURPLE;
            distanceFromBoundary = (float) (unwrappedHue - laneSensorConfig.classifier.decision.hueDecisionBoundary);
        }

        // Confidence based on distance from boundary
        // High confidence if far from boundary, low if close
        double confidence = Math.min(1.0, distanceFromBoundary / laneSensorConfig.classifier.decision.lowConfidenceMargin);

        RobotState.packet.put("intake/classifier/" + lanePrefix + "/decision_unwrapped_hue", unwrappedHue);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/decision_boundary", laneSensorConfig.classifier.decision.hueDecisionBoundary);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/decision_distance_from_boundary", distanceFromBoundary);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/decision_result", color.name());
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/decision_confidence", confidence);
        return new ClassificationResult(color, confidence);
    }

    /**
     * DISTANCE_BASED classifier: Euclidean distance in HSV space to color targets.
     * Picks the closer target (green or purple).
     */
    private ClassificationResult classifyColorDistanceBased(float hue, float saturation, float value, boolean hasSignal, String lanePrefix) {
        if (!hasSignal || value < laneSensorConfig.quality.minValue || saturation < laneSensorConfig.quality.minSaturation) {
            return new ClassificationResult(ArtifactColor.NONE, 0.0);
        }

        // Compute weighted distance to green target
        double distToGreen = hsvDistance(hue, saturation, value,
                laneSensorConfig.classifier.distance.greenHueTarget,
                laneSensorConfig.classifier.distance.greenSatTarget,
                laneSensorConfig.classifier.distance.greenValTarget);

        // Compute weighted distance to purple target
        double distToPurple = hsvDistance(hue, saturation, value,
                laneSensorConfig.classifier.distance.purpleHueTarget,
                laneSensorConfig.classifier.distance.purpleSatTarget,
                laneSensorConfig.classifier.distance.purpleValTarget);

        // Pick closer color
        ArtifactColor color = (distToGreen < distToPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;

        // Confidence based on separation between distances
        // High confidence if one is much closer than the other
        double minDist = Math.min(distToGreen, distToPurple);
        double maxDist = Math.max(distToGreen, distToPurple);
        double separation = maxDist - minDist;
        double confidence = Math.min(1.0, separation / 30.0); // 30° weighted separation for full confidence

        RobotState.packet.put("intake/classifier/" + lanePrefix + "/distance_green", distToGreen);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/distance_purple", distToPurple);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/distance_separation", separation);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/distance_result", color.name());
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/distance_confidence", confidence);
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
        return laneSensorConfig.classifier.distance.hueWeight * hueDiff
                + laneSensorConfig.classifier.distance.saturationWeight * satDiff * 100.0  // Scale to ~0-100
                + laneSensorConfig.classifier.distance.valueWeight * valDiff * 100.0;      // Scale to ~0-100
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
            laneCandidateColor.put(lane, ArtifactColor.NONE);
            laneCandidateCount.put(lane, 0);
            laneLastGoodDetectionMs.put(lane, 0.0);
            laneClearCandidateCount.put(lane, 0);
        }
    }

    public void clearLaneColor(LauncherLane lane) {
        updateLaneColor(lane, ArtifactColor.NONE);
        laneCandidateColor.put(lane, ArtifactColor.NONE);
        laneCandidateCount.put(lane, 0);
        laneLastGoodDetectionMs.put(lane, 0.0);
        laneClearCandidateCount.put(lane, 0);
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

    /**
     * Get count of artifacts currently detected in lanes.
     * Counts lanes with valid artifact colors (not NONE, not UNKNOWN).
     *
     * @return Number of artifacts detected (0-3)
     */
    public int getArtifactCount() {
        int count = 0;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = laneColors.getOrDefault(lane, ArtifactColor.NONE);
            if (color != ArtifactColor.NONE && color != ArtifactColor.UNKNOWN) {
                count++;
            }
        }
        return count;
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
        return count >= laneSensorConfig.fullCount;
    }

    public interface LaneColorListener {
        void onLaneColorChanged(LauncherLane lane, ArtifactColor color);
    }

}
