package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Optional intake stub used by the autonomous routine. It now exposes a minimal
 * state machine so OpModes can request work and poll when it is complete without
 * touching timers or hardware directly.
 */
@Config
public class IntakeSubsystem implements Subsystem {

    public enum IntakeState {
        IDLE,
        INTAKING,
        FULL
    }

    @Config
    public static class LaneSensorConfig {
        public static boolean enablePolling = true;
        public static String leftSensor = "lane_left_color";
        public static String centerSensor = "lane_center_color";
        public static String rightSensor = "lane_right_color";
        public static double samplePeriodMs = 75.0;

        public static double minValue = 0.02;
        public static double minSaturation = 0.15;

        public static double greenHueMin = 80.0;
        public static double greenHueMax = 160.0;

        public static double purpleHueMin = 260.0;
        public static double purpleHueMax = 330.0;
        public static double purpleHueWrapMax = 40.0;
    }

    public static double defaultRunTimeMs = 0.0;
    public static int maxIndexed = 3;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sensorTimer = new ElapsedTime();

    private IntakeState state = IntakeState.IDLE;
    private int indexedCount = 0;
    private Alliance alliance = Alliance.UNKNOWN;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ColorSensor> laneSensors = new EnumMap<>(LauncherLane.class);
    private final List<LaneColorListener> laneColorListeners = new ArrayList<>();
    private boolean anyLaneSensorsPresent = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        // Real implementation would bind motors here.
        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
        }
        bindLaneSensors(hardwareMap);
    }

    private void bindLaneSensors(HardwareMap hardwareMap) {
        anyLaneSensorsPresent = false;
        laneSensors.put(LauncherLane.LEFT, tryGetColorSensor(hardwareMap, LaneSensorConfig.leftSensor));
        laneSensors.put(LauncherLane.CENTER, tryGetColorSensor(hardwareMap, LaneSensorConfig.centerSensor));
        laneSensors.put(LauncherLane.RIGHT, tryGetColorSensor(hardwareMap, LaneSensorConfig.rightSensor));
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

    @Override
    public void initialize() {
        state = IntakeState.IDLE;
        indexedCount = 0;
        clearLaneColors();
        sensorTimer.reset();
    }

    public void requestIntake() {
        if (state == IntakeState.INTAKING) {
            return;
        }
        state = IntakeState.INTAKING;
        timer.reset();
        if (defaultRunTimeMs <= 0.0) {
            indexedCount = maxIndexed;
            state = IntakeState.FULL;
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

    @Override
    public void periodic() {
        if (state == IntakeState.INTAKING && timer.milliseconds() >= defaultRunTimeMs) {
            indexedCount = maxIndexed;
            state = IntakeState.FULL;
        }
        pollLaneSensorsIfNeeded();
    }

    private void pollLaneSensorsIfNeeded() {
        if (!LaneSensorConfig.enablePolling || !anyLaneSensorsPresent) {
            return;
        }
        if (sensorTimer.milliseconds() < LaneSensorConfig.samplePeriodMs) {
            return;
        }
        sensorTimer.reset();
        for (LauncherLane lane : LauncherLane.values()) {
            ColorSensor sensor = laneSensors.get(lane);
            if (sensor == null) {
                continue;
            }
            ArtifactColor detected = classifyColor(sensor);
            updateLaneColor(lane, detected);
        }
    }

    private ArtifactColor classifyColor(ColorSensor sensor) {
        int rawRed = sensor.red();
        int rawGreen = sensor.green();
        int rawBlue = sensor.blue();
        int maxComponent = Math.max(rawRed, Math.max(rawGreen, rawBlue));
        if (maxComponent <= 0) {
            return ArtifactColor.NONE;
        }
        float scale = 255.0f / maxComponent;
        int scaledRed = Math.min(255, Math.round(rawRed * scale));
        int scaledGreen = Math.min(255, Math.round(rawGreen * scale));
        int scaledBlue = Math.min(255, Math.round(rawBlue * scale));

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledRed, scaledGreen, scaledBlue, hsv);
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        if (value < LaneSensorConfig.minValue || saturation < LaneSensorConfig.minSaturation) {
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
}
