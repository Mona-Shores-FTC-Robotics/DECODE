package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Optional intake stub used by the autonomous routine. It now exposes a minimal
 * state machine so OpModes can request work and poll when it is complete without
 * touching timers or hardware directly.
 */
@Configurable
public class IntakeSubsystem implements Subsystem {

    public enum IntakeState {
        IDLE,
        INTAKING,
        FULL
    }

    @Configurable
    public static class LaneSensorConfig {
        public static boolean enablePolling = true;
        public static String leftSensor = "lane_left_color";
        public static String centerSensor = "lane_center_color";
        public static String rightSensor = "lane_right_color";
        public static double samplePeriodMs = 200.0;

        public static double minValue = 0.02;
        public static double minSaturation = 0.15;

        public static double greenHueMin = 80.0;
        public static double greenHueMax = 160.0;

        public static double purpleHueMin = 260.0;
        public static double purpleHueMax = 330.0;
        public static double purpleHueWrapMax = 40.0;

        public static boolean useDistance = true;
        public static double presenceDistanceCm = 3.0;
    }

    public static double defaultRunTimeMs = 0.0;
    public static int maxIndexed = 3;

    @Configurable
    public static class MotorConfig {
        public static String motorName = "intake";
        public static double defaultForwardPower = .6;
        public static double defaultReversePower = -.3;
        public static boolean brakeOnZero = true;
        public static boolean reverseDirection = true;
    }

    @Configurable
    public static class RollerConfig {
        public static String servoName = "intake_roller";
        public static double activePosition = -1.0;
        public static double inactivePosition = 0.0;
    }

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

        private LaneSample(boolean sensorPresent,
                           boolean distanceAvailable,
                           double distanceCm,
                           boolean withinDistance,
                           int rawRed, int rawGreen, int rawBlue,
                           int scaledRed, int scaledGreen, int scaledBlue,
                           float hue, float saturation, float value,
                           ArtifactColor hsvColor,
                           ArtifactColor color) {
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
        }

        private static LaneSample absent() {
            return new LaneSample(false, false, Double.NaN, false,
                    0, 0, 0, 0, 0, 0,
                    0.0f, 0.0f, 0.0f,
                    ArtifactColor.NONE, ArtifactColor.NONE);
        }

        private static LaneSample present(boolean distanceAvailable,
                                          double distanceCm,
                                          boolean withinDistance,
                                          int rawRed, int rawGreen, int rawBlue,
                                          int scaledRed, int scaledGreen, int scaledBlue,
                                          float hue, float saturation, float value,
                                          ArtifactColor hsvColor,
                                          ArtifactColor color) {
            return new LaneSample(true,
                    distanceAvailable,
                    distanceCm,
                    withinDistance,
                    rawRed, rawGreen, rawBlue,
                    scaledRed, scaledGreen, scaledBlue,
                    hue, saturation, value,
                    hsvColor, color);
        }
    }

    private static final LaneSample ABSENT_SAMPLE = LaneSample.absent();

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sensorTimer = new ElapsedTime();

    private IntakeState state = IntakeState.IDLE;
    private int indexedCount = 0;
    private Alliance alliance = Alliance.UNKNOWN;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ColorSensor> laneSensors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, DistanceSensor> laneDistanceSensors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, LaneSample> laneSamples = new EnumMap<>(LauncherLane.class);
    private final float[] hsvBuffer = new float[3];
    private final List<LaneColorListener> laneColorListeners = new ArrayList<>();
    private boolean anyLaneSensorsPresent = false;
    private double lastPeriodicMs = 0.0;
    private final DcMotorEx intakeMotor;
    private double appliedMotorPower = 0.0;
    private final Servo rollerServo;
    private double lastRollerPosition = Double.NaN;
    private boolean rollerEnabled = false;
    private RobotMode robotMode = RobotMode.DEBUG;

    public static final class Inputs {
        public IntakeState state = IntakeState.IDLE;
        public int indexedCount;
        public int maxIndexed;
        public String alliance = Alliance.UNKNOWN.name();
        public boolean anyLaneSensorsPresent;
        public boolean sensorPollingEnabled;
        public double sensorSamplePeriodMs;
        public double timerMs;
        public String leftColor = ArtifactColor.NONE.name();
        public boolean leftSensorPresent;
        public boolean leftDistanceAvailable;
        public double leftDistanceCm;
        public boolean leftWithinDistance;
        public float leftHue;
        public float leftSaturation;
        public float leftValue;
        public String leftDetectedColor = ArtifactColor.NONE.name();
        public String leftHsvColor = ArtifactColor.NONE.name();
        public int leftRawRed;
        public int leftRawGreen;
        public int leftRawBlue;
        public String centerColor = ArtifactColor.NONE.name();
        public boolean centerSensorPresent;
        public boolean centerDistanceAvailable;
        public double centerDistanceCm;
        public boolean centerWithinDistance;
        public float centerHue;
        public float centerSaturation;
        public float centerValue;
        public String centerDetectedColor = ArtifactColor.NONE.name();
        public String centerHsvColor = ArtifactColor.NONE.name();
        public int centerRawRed;
        public int centerRawGreen;
        public int centerRawBlue;
        public String rightColor = ArtifactColor.NONE.name();
        public boolean rightSensorPresent;
        public boolean rightDistanceAvailable;
        public double rightDistanceCm;
        public boolean rightWithinDistance;
        public float rightHue;
        public float rightSaturation;
        public float rightValue;
        public String rightDetectedColor = ArtifactColor.NONE.name();
        public String rightHsvColor = ArtifactColor.NONE.name();
        public int rightRawRed;
        public int rightRawGreen;
        public int rightRawBlue;
        public double motorPower;
        public boolean rollerPresent;
        public double rollerPosition;
        public boolean rollerActive;
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = tryGetMotor(hardwareMap, MotorConfig.motorName);
        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (MotorConfig.brakeOnZero) {
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (MotorConfig.reverseDirection) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            intakeMotor.setPower(0.0);
        }
        rollerServo = tryGetServo(hardwareMap, RollerConfig.servoName);
        if (rollerServo != null) {
            lastRollerPosition = RollerConfig.inactivePosition;
            rollerServo.setPosition(RollerConfig.inactivePosition);
        }
        rollerEnabled = false;
        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
            laneSamples.put(lane, ABSENT_SAMPLE);
        }
        bindLaneSensors(hardwareMap);
    }

    private void bindLaneSensors(HardwareMap hardwareMap) {
        anyLaneSensorsPresent = false;
        laneSensors.put(LauncherLane.LEFT, tryGetColorSensor(hardwareMap, LaneSensorConfig.leftSensor));
        laneDistanceSensors.put(LauncherLane.LEFT, tryGetDistanceSensor(hardwareMap, LaneSensorConfig.leftSensor));
        laneSensors.put(LauncherLane.CENTER, tryGetColorSensor(hardwareMap, LaneSensorConfig.centerSensor));
        laneDistanceSensors.put(LauncherLane.CENTER, tryGetDistanceSensor(hardwareMap, LaneSensorConfig.centerSensor));
        laneSensors.put(LauncherLane.RIGHT, tryGetColorSensor(hardwareMap, LaneSensorConfig.rightSensor));
        laneDistanceSensors.put(LauncherLane.RIGHT, tryGetDistanceSensor(hardwareMap, LaneSensorConfig.rightSensor));
        for (ColorSensor sensor : laneSensors.values()) {
            if (sensor != null) {
                anyLaneSensorsPresent = true;
                break;
            }
        }
    }

    private static ColorSensor tryGetColorSensor(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(ColorSensor.class, name);
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

    @Override
    public void initialize() {
        state = IntakeState.IDLE;
        indexedCount = 0;
        clearLaneColors();
        for (LauncherLane lane : LauncherLane.values()) {
            laneSamples.put(lane, ABSENT_SAMPLE);
        }
        sensorTimer.reset();
        setManualPower(0.0);
        if (rollerServo != null) {
            rollerServo.setPosition(RollerConfig.inactivePosition);
            lastRollerPosition = RollerConfig.inactivePosition;
        }
        rollerEnabled = false;
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

    public void requestIntake() {
        if (state == IntakeState.INTAKING) {
            return;
        }
        state = IntakeState.INTAKING;
        timer.reset();
        setManualPower(MotorConfig.defaultForwardPower);
        if (defaultRunTimeMs <= 0.0) {
            indexedCount = maxIndexed;
            state = IntakeState.FULL;
            setManualPower(0.0);
        }
    }

    /** Legacy name retained for existing OpModes. */
    public void runIn() {
        requestIntake();
    }

    /** Legacy name retained for existing OpModes. */
    public void start() {
        requestIntake();
    }

    public void stop() {
        if (state != IntakeState.FULL) {
            state = IntakeState.IDLE;
        }
        setManualPower(0.0);
        deactivateRoller();
    }

    /** Legacy loop hook retained for compatibility. */
    public void update() {
        periodic();
    }

    public boolean isBusy() {
        return state == IntakeState.INTAKING;
    }

    public boolean isFull() {
        return indexedCount >= maxIndexed;
    }

    public void seedIndexedCount(int count) {
        indexedCount = clamp(count, 0, maxIndexed);
        if (indexedCount >= maxIndexed) {
            state = IntakeState.FULL;
        }
    }

    public void clearIndexed() {
        indexedCount = 0;
        state = IntakeState.IDLE;
    }

    public int getIndexedCount() {
        return indexedCount;
    }

    public IntakeState getState() {
        return state;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setRobotMode(RobotMode mode) {
        robotMode = RobotMode.orDefault(mode);
    }

    public void activateRoller() {
        rollerEnabled = true;
    }

    public void deactivateRoller() {
        rollerEnabled = false;
        if (rollerServo != null) {
            rollerServo.setPosition(RollerConfig.inactivePosition);
            lastRollerPosition = RollerConfig.inactivePosition;
        }
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        if (robotMode == RobotMode.MATCH) {
            pollLaneSensorsIfNeeded();
        }
        if (rollerServo != null) {
            double target = rollerEnabled ? RollerConfig.activePosition : RollerConfig.inactivePosition;
            rollerServo.setPosition(target);
            lastRollerPosition = target;
        }
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
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
        if (Math.abs(power) > 1e-3) {
            state = IntakeState.INTAKING;
        } else if (state != IntakeState.FULL) {
            state = IntakeState.IDLE;
        }
    }

    public double getCurrentPower() {
        return appliedMotorPower;
    }

    public boolean isRollerPresent() {
        return rollerServo != null;
    }

    public double getRollerPosition() {
        return rollerServo == null ? Double.NaN : lastRollerPosition;
    }

    public boolean isRollerActive() {
        return rollerServo != null
                && Math.abs(lastRollerPosition - RollerConfig.activePosition) < 1e-3;
    }

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        inputs.state = state;
        inputs.indexedCount = indexedCount;
        inputs.maxIndexed = maxIndexed;
        inputs.alliance = alliance.name();
        inputs.anyLaneSensorsPresent = anyLaneSensorsPresent;
        inputs.sensorPollingEnabled = LaneSensorConfig.enablePolling;
        inputs.sensorSamplePeriodMs = LaneSensorConfig.samplePeriodMs;
        inputs.timerMs = timer.milliseconds();
        inputs.motorPower = appliedMotorPower;
        inputs.rollerPresent = rollerServo != null;
        inputs.rollerPosition = rollerServo != null ? lastRollerPosition : Double.NaN;
        inputs.rollerActive = rollerServo != null
                && Math.abs(lastRollerPosition - RollerConfig.activePosition) < 1e-3;

        populateLaneInputs(inputs, LauncherLane.LEFT);
        populateLaneInputs(inputs, LauncherLane.CENTER);
        populateLaneInputs(inputs, LauncherLane.RIGHT);
    }

    private void pollLaneSensorsIfNeeded() {
        if (!LaneSensorConfig.enablePolling || !anyLaneSensorsPresent) {
            return;
        }
        if (sensorTimer.milliseconds() < LaneSensorConfig.samplePeriodMs) {
            return;
        }
        sensorTimer.reset();
        refreshLaneSensors();
    }

    public void refreshLaneSensors() {
        for (LauncherLane lane : LauncherLane.values()) {
            LaneSample sample = sampleLane(lane);
            laneSamples.put(lane, sample);
            updateLaneColor(lane, sample.color);
        }
    }

    private LaneSample sampleLane(LauncherLane lane) {
        ColorSensor colorSensor = laneSensors.get(lane);
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
        if (!distanceAvailable || !LaneSensorConfig.useDistance) {
            withinDistance = true;
        } else if (!distanceValid) {
            withinDistance = false;
        } else {
            withinDistance = distanceCm <= LaneSensorConfig.presenceDistanceCm;
        }

        int rawRed = colorSensor.red();
        int rawGreen = colorSensor.green();
        int rawBlue = colorSensor.blue();
        int maxComponent = Math.max(rawRed, Math.max(rawGreen, rawBlue));
        int scaledRed = 0;
        int scaledGreen = 0;
        int scaledBlue = 0;
        float hue = 0.0f;
        float saturation = 0.0f;
        float value = 0.0f;

        if (maxComponent > 0) {
            float scale = maxComponent > 0 ? 255.0f / maxComponent : 0.0f;
            scaledRed = Math.min(255, Math.round(rawRed * scale));
            scaledGreen = Math.min(255, Math.round(rawGreen * scale));
            scaledBlue = Math.min(255, Math.round(rawBlue * scale));

            Color.RGBToHSV(scaledRed, scaledGreen, scaledBlue, hsvBuffer);
            hue = hsvBuffer[0];
            saturation = hsvBuffer[1];
            value = hsvBuffer[2];
        }

        ArtifactColor hsvColor = classifyColorFromHsv(hue, saturation, value, maxComponent > 0);
        ArtifactColor finalColor = hsvColor;
        if (LaneSensorConfig.useDistance && distanceAvailable && !withinDistance) {
            finalColor = ArtifactColor.NONE;
        }

        return LaneSample.present(
                distanceAvailable,
                distanceValid ? distanceCm : Double.NaN,
                withinDistance,
                rawRed, rawGreen, rawBlue,
                scaledRed, scaledGreen, scaledBlue,
                hue, saturation, value,
                hsvColor,
                finalColor
        );
    }

    private ArtifactColor classifyColorFromHsv(float hue, float saturation, float value, boolean hasSignal) {
        if (!hasSignal || value < LaneSensorConfig.minValue || saturation < LaneSensorConfig.minSaturation) {
            return ArtifactColor.NONE;
        }

        if (hue >= LaneSensorConfig.greenHueMin && hue <= LaneSensorConfig.greenHueMax) {
            return ArtifactColor.GREEN;
        }

        if ((hue >= LaneSensorConfig.purpleHueMin && hue <= LaneSensorConfig.purpleHueMax)
                || hue <= LaneSensorConfig.purpleHueWrapMax) {
            return ArtifactColor.PURPLE;
        }

        return ArtifactColor.UNKNOWN;
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

    public interface LaneColorListener {
        void onLaneColorChanged(LauncherLane lane, ArtifactColor color);
    }


    private static int clamp(int value, int min, int max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    private void populateLaneInputs(Inputs inputs, LauncherLane lane) {
        LaneSample sample = getLaneSample(lane);
        ArtifactColor laneColor = getLaneColor(lane);
        switch (lane) {
            case LEFT:
                inputs.leftColor = laneColor.name();
                inputs.leftSensorPresent = sample.sensorPresent;
                inputs.leftDistanceAvailable = sample.distanceAvailable;
                inputs.leftDistanceCm = sample.distanceCm;
                inputs.leftWithinDistance = sample.withinDistance;
                inputs.leftHue = sample.hue;
                inputs.leftSaturation = sample.saturation;
                inputs.leftValue = sample.value;
                inputs.leftDetectedColor = sample.color.name();
                inputs.leftHsvColor = sample.hsvColor.name();
                inputs.leftRawRed = sample.rawRed;
                inputs.leftRawGreen = sample.rawGreen;
                inputs.leftRawBlue = sample.rawBlue;
                break;
            case CENTER:
                inputs.centerColor = laneColor.name();
                inputs.centerSensorPresent = sample.sensorPresent;
                inputs.centerDistanceAvailable = sample.distanceAvailable;
                inputs.centerDistanceCm = sample.distanceCm;
                inputs.centerWithinDistance = sample.withinDistance;
                inputs.centerHue = sample.hue;
                inputs.centerSaturation = sample.saturation;
                inputs.centerValue = sample.value;
                inputs.centerDetectedColor = sample.color.name();
                inputs.centerHsvColor = sample.hsvColor.name();
                inputs.centerRawRed = sample.rawRed;
                inputs.centerRawGreen = sample.rawGreen;
                inputs.centerRawBlue = sample.rawBlue;
                break;
            case RIGHT:
            default:
                inputs.rightColor = laneColor.name();
                inputs.rightSensorPresent = sample.sensorPresent;
                inputs.rightDistanceAvailable = sample.distanceAvailable;
                inputs.rightDistanceCm = sample.distanceCm;
                inputs.rightWithinDistance = sample.withinDistance;
                inputs.rightHue = sample.hue;
                inputs.rightSaturation = sample.saturation;
                inputs.rightValue = sample.value;
                inputs.rightDetectedColor = sample.color.name();
                inputs.rightHsvColor = sample.hsvColor.name();
                inputs.rightRawRed = sample.rawRed;
                inputs.rightRawGreen = sample.rawGreen;
                inputs.rightRawBlue = sample.rawBlue;
                break;
        }
    }

}
