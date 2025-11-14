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

    @Configurable
    public static class LaneSensorConfig {
        public boolean enablePolling = true;
        public String leftSensor = "lane_left_color";
        public String centerSensor = "lane_center_color";
        public String rightSensor = "lane_right_color";
        public double samplePeriodMs = 200.0;

        public double minValue = 0.02;
        public double minSaturation = 0.15;

        public double greenHueMin = 80.0;
        public double greenHueMax = 160.0;

        public double purpleHueMin = 260.0;
        public double purpleHueMax = 330.0;
        public double purpleHueWrapMax = 40.0;

        public boolean useDistance = true;
        public double presenceDistanceCm = 3.0;
    }

    @Configurable
    public static class MotorConfig {
        public String motorName = "intake";
        public double defaultForwardPower = -.6;
        public double defaultReversePower = .3;
        public boolean brakeOnZero = true;
        public boolean reverseDirection = true;
    }

    @Configurable
    public static class RollerConfig {
        public String servoName = "intake_roller";
        public double activePosition = 1.0;
        public double inactivePosition = 0.5;
    }

    @Configurable
    public static class ManualModeConfig {
        public boolean enableOverride = false;
        public String overrideMode = STOPPED.name();
    }

    public static LaneSensorConfig laneSensorConfig = new LaneSensorConfig();
    public static MotorConfig motorConfig = new MotorConfig();
    public static RollerConfig rollerConfig = new RollerConfig();
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
    private double lastRollerPosition = Double.NaN;
    private boolean rollerEnabled = false;

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

    public void deactivateRoller() {
        rollerEnabled = false;
        if (rollerServo != null) {
            rollerServo.setPosition(rollerConfig.inactivePosition);
            lastRollerPosition = rollerConfig.inactivePosition;
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

    public double getRollerPosition() {
        return rollerServo == null ? Double.NaN : lastRollerPosition;
    }

    public boolean isRollerActive() {
        return rollerServo != null
                && Math.abs(lastRollerPosition - rollerConfig.activePosition) < 1e-3;
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

        ArtifactColor hsvColor = classifyColorFromHsv(hue, saturation, value, maxComponent > 0);
        ArtifactColor finalColor = hsvColor;
        if (laneSensorConfig.useDistance && distanceAvailable && !withinDistance) {
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
        if (!hasSignal || value < laneSensorConfig.minValue || saturation < laneSensorConfig.minSaturation) {
            return ArtifactColor.NONE;
        }

        if (hue >= laneSensorConfig.greenHueMin && hue <= laneSensorConfig.greenHueMax) {
            return ArtifactColor.GREEN;
        }

        if ((hue >= laneSensorConfig.purpleHueMin && hue <= laneSensorConfig.purpleHueMax)
                || hue <= laneSensorConfig.purpleHueWrapMax) {
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
