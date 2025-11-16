package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumMap;
import java.util.Map;

/**
 * goBILDA RGB Indicator Light wrapper. Manages three lane-linked indicator servos so
 * OpModes can coordinate colours without touching hardware directly. Missing servos are
 * ignored gracefully, preserving compatibility with partial benches.
 */
@Configurable
public class LightingSubsystem implements Subsystem, IntakeSubsystem.LaneColorListener {

    public enum LightingState {
        OFF,
        ALLIANCE,
        BUSY
    }

    @Configurable
    public static class IndicatorConfig {
        public LeftIndicatorConfig left = new LeftIndicatorConfig();
        public CenterIndicatorConfig center = new CenterIndicatorConfig();
        public RightIndicatorConfig right = new RightIndicatorConfig();

        @Configurable
        public static class LeftIndicatorConfig {
            /** Hardware name for the left lane indicator servo */
            public String servoName = "indicator_left";
        }

        @Configurable
        public static class CenterIndicatorConfig {
            /** Hardware name for the center lane indicator servo */
            public String servoName = "indicator_center";
        }

        @Configurable
        public static class RightIndicatorConfig {
            /** Hardware name for the right lane indicator servo */
            public String servoName = "indicator_right";
        }
    }

    @Configurable
    public static class ColorPositionConfig {
        /** Servo position for green color */
        public double greenPosition = 0.500;
        /** Servo position for purple color */
        public double purplePosition = 0.722;
        /** Servo position for red color (alliance) */
        public double redPosition = 0.281;
        /** Servo position for blue color (alliance) */
        public double bluePosition = 0.611;
        /** Position that best represents an "off" state for the installed lights */
        public double offPosition = 0.000;
        /** Fallback position used when marking the robot busy */
        public double busyPosition = 0.722; // PURPLE by default
    }

    public static IndicatorConfig indicatorConfig = new IndicatorConfig();
    public static ColorPositionConfig colorPositionConfig = new ColorPositionConfig();

    private final EnumMap<LauncherLane, LaneIndicator> laneIndicators = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneOutputs = new EnumMap<>(LauncherLane.class);

    private LightingState state = LightingState.OFF;
    private Alliance alliance = Alliance.UNKNOWN;
    private double lastPeriodicMs = 0.0;

    public LightingSubsystem(HardwareMap hardwareMap) {
        laneIndicators.put(LauncherLane.LEFT, new LaneIndicator(
                tryGetServo(hardwareMap, indicatorConfig.left.servoName)));
        laneIndicators.put(LauncherLane.CENTER, new LaneIndicator(
                tryGetServo(hardwareMap, indicatorConfig.center.servoName)));
        laneIndicators.put(LauncherLane.RIGHT, new LaneIndicator(
                tryGetServo(hardwareMap, indicatorConfig.right.servoName)));

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
            laneOutputs.put(lane, colorPositionConfig.offPosition);
        }
    }

    @Override
    public void initialize() {
        indicateAllianceInit();
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        // No periodic work required – kept for interface completeness.
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
        if (state == LightingState.ALLIANCE || state == LightingState.OFF) {
            updateLaneOutputs();
        }
    }

    public void indicateBusy() {
        state = LightingState.BUSY;
        updateLaneOutputs();
    }

    public void indicateIdle() {
        state = LightingState.ALLIANCE;
        updateLaneOutputs();
    }

    public void disable() {
        resetLaneColors();
        state = LightingState.OFF;
        updateLaneOutputs();
    }

    public void setLaneColor(LauncherLane lane, ArtifactColor color) {
        if (lane == null) {
            return;
        }
        ArtifactColor normalized = color == null ? ArtifactColor.NONE : color;
        ArtifactColor previous = laneColors.put(lane, normalized);
        if (previous == normalized) {
            return;
        }
        applyLaneOutput(lane);
    }

    @Override
    public void onLaneColorChanged(LauncherLane lane, ArtifactColor color) {
        setLaneColor(lane, color);
    }

    public void clearLaneColors() {
        if (resetLaneColors()) {
            updateLaneOutputs();
        }
    }

    public void indicateAllianceInit() {
        resetLaneColors();
        state = LightingState.ALLIANCE;
        updateLaneOutputs();
    }

    public void showDecodePattern(ArtifactColor[] pattern) {
        if (pattern == null || pattern.length == 0) {
            if (resetLaneColors()) {
                updateLaneOutputs();
            } else {
                updateLaneOutputs();
            }
            return;
        }
        resetLaneColors();
        LauncherLane[] lanes = LauncherLane.values();
        int limit = Math.min(pattern.length, lanes.length);
        for (int i = 0; i < limit; i++) {
            ArtifactColor normalized = pattern[i] == null ? ArtifactColor.NONE : pattern[i];
            laneColors.put(lanes[i], normalized);
        }
        updateLaneOutputs();
    }

    private void updateLaneOutputs() {
        for (LauncherLane lane : LauncherLane.values()) {
            applyLaneOutput(lane);
        }
    }

    private void applyLaneOutput(LauncherLane lane) {
        LaneIndicator indicator = laneIndicators.get(lane);
        if (indicator == null || !indicator.isPresent()) {
            return;
        }
        ArtifactColor laneColor = laneColors.getOrDefault(lane, ArtifactColor.NONE);
        double position = resolvePosition(laneColor);
        laneOutputs.put(lane, position);
        indicator.apply(position);
    }

    protected double resolvePosition(ArtifactColor laneColor) {
        switch (laneColor) {
            case GREEN:
                return clamp01(colorPositionConfig.greenPosition);
            case PURPLE:
                return clamp01(colorPositionConfig.purplePosition);
            case NONE:
            case UNKNOWN:
            default:
                return fallbackPosition();
        }
    }

    protected double fallbackPosition() {
        switch (state) {
            case BUSY:
                return clamp01(colorPositionConfig.busyPosition);
            case ALLIANCE:
                return alliancePosition();
            case OFF:
            default:
                return clamp01(colorPositionConfig.offPosition);
        }
    }

    protected double alliancePosition() {
        if (alliance == Alliance.RED) {
            return clamp01(colorPositionConfig.redPosition);
        }
        if (alliance == Alliance.BLUE) {
            return clamp01(colorPositionConfig.bluePosition);
        }
        return clamp01(colorPositionConfig.greenPosition);
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    protected boolean resetLaneColors() {
        boolean touched = false;
        for (Map.Entry<LauncherLane, ArtifactColor> entry : laneColors.entrySet()) {
            if (entry.getValue() != ArtifactColor.NONE) {
                entry.setValue(ArtifactColor.NONE);
                touched = true;
            }
        }
        return touched;
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

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    public String getLightingStateString() {
        return state.name();
    }

    public String getAllianceString() {
        return alliance.name();
    }

    public String getLeftColorString() {
        ArtifactColor color = laneColors.getOrDefault(LauncherLane.LEFT, ArtifactColor.NONE);
        return color.name();
    }

    public double getLeftOutput() {
        return laneOutputs.getOrDefault(LauncherLane.LEFT, colorPositionConfig.offPosition);
    }

    public boolean isLeftIndicatorPresent() {
        LaneIndicator indicator = laneIndicators.get(LauncherLane.LEFT);
        return indicator != null && indicator.isPresent();
    }

    public String getCenterColorString() {
        ArtifactColor color = laneColors.getOrDefault(LauncherLane.CENTER, ArtifactColor.NONE);
        return color.name();
    }

    public double getCenterOutput() {
        return laneOutputs.getOrDefault(LauncherLane.CENTER, colorPositionConfig.offPosition);
    }

    public boolean isCenterIndicatorPresent() {
        LaneIndicator indicator = laneIndicators.get(LauncherLane.CENTER);
        return indicator != null && indicator.isPresent();
    }

    public String getRightColorString() {
        ArtifactColor color = laneColors.getOrDefault(LauncherLane.RIGHT, ArtifactColor.NONE);
        return color.name();
    }

    public double getRightOutput() {
        return laneOutputs.getOrDefault(LauncherLane.RIGHT, colorPositionConfig.offPosition);
    }

    public boolean isRightIndicatorPresent() {
        LaneIndicator indicator = laneIndicators.get(LauncherLane.RIGHT);
        return indicator != null && indicator.isPresent();
    }

    private static final class LaneIndicator {
        private final Servo servo;

        LaneIndicator(Servo servo) {
            this.servo = servo;
        }

        boolean isPresent() {
            return servo != null;
        }

        void apply(double position) {
            if (servo == null) {
                return;
            }
            servo.setPosition(clamp01(position));
        }
    }
}
