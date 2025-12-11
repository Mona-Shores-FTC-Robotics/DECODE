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

        private LaneSample(boolean sensorPresent,
                           boolean distanceAvailable,
                           double distanceCm,
                           boolean withinDistance,
                           int scaledRed, int scaledGreen, int scaledBlue,
                           float normalizedRed, float normalizedGreen, float normalizedBlue,
                           float hue, float saturation, float value,
                           ArtifactColor hsvColor,
                           ArtifactColor color) {
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
        }

        private static LaneSample absent() {
            return new LaneSample(false, false, Double.NaN, false,
                    0, 0, 0,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    ArtifactColor.NONE, ArtifactColor.NONE);
        }

        private static LaneSample present(boolean distanceAvailable,
                                          double distanceCm,
                                          boolean withinDistance,
                                          int scaledRed, int scaledGreen, int scaledBlue,
                                          float normalizedRed, float normalizedGreen, float normalizedBlue,
                                          float hue, float saturation, float value,
                                          ArtifactColor hsvColor,
                                          ArtifactColor color) {
            return new LaneSample(true,
                    distanceAvailable,
                    distanceCm,
                    withinDistance,
                    scaledRed, scaledGreen, scaledBlue,
                    normalizedRed, normalizedGreen, normalizedBlue,
                    hue, saturation, value,
                    hsvColor, color);
        }
    }

    private static final LaneSample ABSENT_SAMPLE = LaneSample.absent();

    private final ElapsedTime sensorTimer = new ElapsedTime();

    private Alliance alliance = Alliance.UNKNOWN;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, NormalizedColorSensor> laneSensors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, DistanceSensor> laneDistanceSensors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, MovingAverageFilter> laneDistanceFilters = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, MovingAverageFilter> laneSaturationFilters = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, MovingAverageFilter> laneValueFilters = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, CircularMovingAverageFilter> laneHueFilters = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, LaneSample> laneSamples = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Boolean> lanePresenceState = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> laneCandidateColor = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Integer> laneCandidateCount = new EnumMap<>(LauncherLane.class);
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
            laneClearCandidateCount.put(lane, 0);
            // Initialize filters with configured window sizes
            laneDistanceFilters.put(lane, new MovingAverageFilter(laneSensorConfig.distanceFilter.windowSize));
            laneSaturationFilters.put(lane, new MovingAverageFilter(laneSensorConfig.saturationFilter.windowSize));
            laneValueFilters.put(lane, new MovingAverageFilter(laneSensorConfig.valueFilter.windowSize));
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
            laneClearCandidateCount.put(lane, 0);
            // Reset filters (recreate with current config in case window size changed)
            laneDistanceFilters.put(lane, new MovingAverageFilter(laneSensorConfig.distanceFilter.windowSize));
            laneSaturationFilters.put(lane, new MovingAverageFilter(laneSensorConfig.saturationFilter.windowSize));
            laneValueFilters.put(lane, new MovingAverageFilter(laneSensorConfig.valueFilter.windowSize));
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
     * Applies debounce before updating lane color to reduce flicker.
     * Uses consecutive sample confirmations for both detection and clearing.
     * Saturation filtering handles whiffle ball hole flicker before this is called.
     */
    private void applyGatedLaneColor(LauncherLane lane, LaneSample sample) {
        if (lane == null || sample == null) {
            return;
        }

        ArtifactColor candidate = sample.color == null ? ArtifactColor.NONE : sample.color;
        ArtifactColor currentLaneColor = laneColors.getOrDefault(lane, ArtifactColor.NONE);

        // --- ARTIFACT DETECTION PATH ---
        // If presence detected an artifact, trust the color classification
        // (misclassification is better than false negative)
        if (candidate.isArtifact()) {
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
        // Presence detection said nothing there (NONE)

        if (currentLaneColor.isArtifact()) {
            // Count consecutive clear confirmations before removing artifact
            int clearCount = laneClearCandidateCount.getOrDefault(lane, 0);
            clearCount++;
            laneClearCandidateCount.put(lane, clearCount);

            // Only clear after N consecutive bad samples (symmetric with detection)
            if (clearCount >= Math.max(1, laneSensorConfig.gating.consecutiveClearConfirmationsRequired)) {
                // Confirmed absent - clear detection
                laneCandidateColor.put(lane, ArtifactColor.NONE);
                laneCandidateCount.put(lane, 0);
                laneClearCandidateCount.put(lane, 0);
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

        // Get per-robot presence config
        IntakeLaneSensorConfig.LanePresenceConfig presenceCfg = RobotConfigs.getLanePresenceConfig();

        // Distance-based presence check
        if (!distanceAvailable || !presenceCfg.useDistance || !distanceValid) {
            withinDistance = false;
        } else {
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

        // Apply saturation filter for presence detection smoothing (handles whiffle ball holes)
        float filteredSaturation = saturation;
        if (laneSensorConfig.saturationFilter.enableFilter && maxComponent > 0) {
            MovingAverageFilter satFilter = laneSaturationFilters.get(lane);
            if (satFilter != null) {
                filteredSaturation = (float) satFilter.calculate(saturation);
            }
        }

        // Apply value filter for presence detection smoothing (handles whiffle ball holes)
        float filteredValue = value;
        if (laneSensorConfig.valueFilter.enableFilter && maxComponent > 0) {
            MovingAverageFilter valFilter = laneValueFilters.get(lane);
            if (valFilter != null) {
                filteredValue = (float) valFilter.calculate(value);
            }
        }

        // Apply circular moving average filter to hue for smoother GREEN vs PURPLE classification
        float filteredHue = rawHue;
        if (laneSensorConfig.hueFilter.enableFilter && maxComponent > 0) {
            CircularMovingAverageFilter hueFilter = laneHueFilters.get(lane);
            if (hueFilter != null) {
                filteredHue = (float) hueFilter.calculate(rawHue);
            }
        }

        // Saturation-based presence detection: colorful artifact vs dull background
        // Can be used alone or combined with distance (AND logic)
        if (presenceCfg.useSaturation) {
            boolean satPresent = filteredSaturation >= presenceCfg.saturationThreshold;
            // If distance is also enabled, require BOTH to pass (AND logic)
            if (presenceCfg.useDistance) {
                withinDistance = withinDistance && satPresent;
            } else {
                withinDistance = satPresent;
            }
            lanePresenceState.put(lane, withinDistance);
        }

        // Value-based presence detection: bright artifact vs dark background
        // Can be used alone or combined with other methods (AND logic)
        if (presenceCfg.useValue) {
            boolean valuePresent = filteredValue >= presenceCfg.valueThreshold;
            // If other methods are enabled, require ALL to pass (AND logic)
            if (presenceCfg.useDistance || presenceCfg.useSaturation) {
                withinDistance = withinDistance && valuePresent;
            } else {
                withinDistance = valuePresent;
            }
            lanePresenceState.put(lane, withinDistance);
        }

        // Hue-based presence detection: artifact hue vs background hue
        // Can be used alone or combined with other methods (AND logic)
        if (presenceCfg.useHue) {
            boolean huePresent = filteredHue >= presenceCfg.hueThreshold;
            // If other methods are enabled, require ALL to pass (AND logic)
            if (presenceCfg.useDistance || presenceCfg.useSaturation || presenceCfg.useValue) {
                withinDistance = withinDistance && huePresent;
            } else {
                withinDistance = huePresent;
            }
            lanePresenceState.put(lane, withinDistance);
        }

        // Compute total RGB intensity for telemetry
        int totalIntensity = Math.round(scaledRedFloat + scaledGreenFloat + scaledBlueFloat);

        String lanePrefix = lane == null ? "unknown" : lane.name().toLowerCase(Locale.US);

        // Classify color using selected classifier mode
        ClassificationResult result = classifyColor(
                filteredHue, filteredSaturation, filteredValue,
                maxComponent > 0,
                totalIntensity,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance,
                lanePrefix
        );
        ArtifactColor hsvColor = result.color;

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
        RobotState.packet.put("intake/sample/" + lanePrefix + "/sat_raw", saturation);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/sat", filteredSaturation);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/val_raw", value);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/val", filteredValue);
        RobotState.packet.put("intake/sample/" + lanePrefix + "/total_intensity", totalIntensity);

        return LaneSample.present(
                distanceAvailable,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance,
                scaledRed, scaledGreen, scaledBlue,
                normalizedRed, normalizedGreen, normalizedBlue,
                filteredHue, filteredSaturation, filteredValue,
                hsvColor,
                finalColor
        );
    }

    /**
     * Classification result with color.
     */
    private static class ClassificationResult {
        final ArtifactColor color;

        ClassificationResult(ArtifactColor color) {
            this.color = color;
        }
    }

    /**
     * Classify color using the configured classifier mode.
     * Presence detection (distance/saturation) is handled in sampleLane() before this is called.
     *
     * @param hue Hue value (0-360°)
     * @param saturation Saturation value (0-1)
     * @param value Value/brightness (0-1)
     * @param hasSignal Whether sensor has a valid signal
     * @param totalIntensity Total RGB intensity (0-765)
     * @param distanceCm Distance reading in cm (or NaN if unavailable)
     * @param withinDistance Whether presence detection indicates artifact present
     * @param lanePrefix Lane name prefix for telemetry
     * @return Classification result with color
     */
    private ClassificationResult classifyColor(float hue, float saturation, float value,
                                               boolean hasSignal, int totalIntensity,
                                               double distanceCm, boolean withinDistance,
                                               String lanePrefix) {
        String reason = "classified";

        // Presence gate: withinDistance is already computed by sampleLane() using distance or saturation
        if (!withinDistance) {
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_distance_valid", !Double.isNaN(distanceCm));
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_within_distance", false);
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/presence_total_intensity", totalIntensity);
            reason = "no_presence";
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);
            return new ClassificationResult(ArtifactColor.NONE);
        }

        // Basic quality check - no signal from sensor
        if (!hasSignal) {
            reason = "no_signal";
            RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);
            return new ClassificationResult(ArtifactColor.NONE);
        }

        // Classify GREEN vs PURPLE using hue decision boundary
        ClassificationResult result = classifyByHue(hue, lanePrefix);

        // Tag the reason as the chosen color
        reason = result.color == null ? "none" : result.color.name();
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/reason", reason);

        return result;
    }

    /**
     * Classify artifact color as GREEN or PURPLE using hue decision boundary.
     * GREEN if hue < boundary, PURPLE otherwise.
     */
    private ClassificationResult classifyByHue(float hue, String lanePrefix) {
        double boundary = laneSensorConfig.colorClassifier.hueDecisionBoundary;

        // Unwrap hue to handle purple wrap-around (purple spans across 0°)
        float unwrappedHue = hue;
        float wrapThreshold = (float) (boundary - 180.0);
        if (hue < wrapThreshold) {
            unwrappedHue = hue + 360.0f;
        }

        // Classify based on which side of decision boundary
        ArtifactColor color;
        float distanceFromBoundary;

        if (unwrappedHue < boundary) {
            color = ArtifactColor.GREEN;
            distanceFromBoundary = (float) (boundary - unwrappedHue);
        } else {
            color = ArtifactColor.PURPLE;
            distanceFromBoundary = (float) (unwrappedHue - boundary);
        }

        RobotState.packet.put("intake/classifier/" + lanePrefix + "/hue_unwrapped", unwrappedHue);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/hue_boundary", boundary);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/hue_distance_from_boundary", distanceFromBoundary);
        RobotState.packet.put("intake/classifier/" + lanePrefix + "/color", color.name());
        return new ClassificationResult(color);
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
            laneClearCandidateCount.put(lane, 0);
        }
    }

    public void clearLaneColor(LauncherLane lane) {
        updateLaneColor(lane, ArtifactColor.NONE);
        laneCandidateColor.put(lane, ArtifactColor.NONE);
        laneCandidateCount.put(lane, 0);
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
