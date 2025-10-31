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

    /** Hardware names for the lane indicator servos. */
    public static String leftServoName = "indicator_left";
    public static String centerServoName = "indicator_center";
    public static String rightServoName = "indicator_right";

    public static double GREEN_POS = 0.500;
    public static double PURPLE_POS = 0.722;
    public static double RED_POS = 0.281;
    public static double BLUE_POS = 0.611;
    /** Position that best represents an "off" state for the installed lights. */
    public static double OFF_POS = 0.000;
    /** Fallback colour used when marking the robot busy. */
    public static double BUSY_POS = PURPLE_POS;

    private final EnumMap<LauncherLane, LaneIndicator> laneIndicators = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneOutputs = new EnumMap<>(LauncherLane.class);

    private LightingState state = LightingState.OFF;
    private Alliance alliance = Alliance.UNKNOWN;

    public static final class Inputs {
        public LightingState state = LightingState.OFF;
        public String alliance = Alliance.UNKNOWN.name();
        public String leftColor = ArtifactColor.NONE.name();
        public double leftOutput = OFF_POS;
        public boolean leftIndicatorPresent;
        public String centerColor = ArtifactColor.NONE.name();
        public double centerOutput = OFF_POS;
        public boolean centerIndicatorPresent;
        public String rightColor = ArtifactColor.NONE.name();
        public double rightOutput = OFF_POS;
        public boolean rightIndicatorPresent;
    }

    public LightingSubsystem(HardwareMap hardwareMap) {
        laneIndicators.put(LauncherLane.LEFT, new LaneIndicator(
                tryGetServo(hardwareMap, leftServoName)));
        laneIndicators.put(LauncherLane.CENTER, new LaneIndicator(
                tryGetServo(hardwareMap, centerServoName)));
        laneIndicators.put(LauncherLane.RIGHT, new LaneIndicator(
                tryGetServo(hardwareMap, rightServoName)));

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
            laneOutputs.put(lane, OFF_POS);
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
        indicateAllianceInit();
    }

    @Override
    public void periodic() {
        // No periodic work required – kept for interface completeness.
    }

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        inputs.state = state;
        inputs.alliance = alliance.name();
        populateLaneInputs(inputs, LauncherLane.LEFT);
        populateLaneInputs(inputs, LauncherLane.CENTER);
        populateLaneInputs(inputs, LauncherLane.RIGHT);
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

    private double resolvePosition(ArtifactColor laneColor) {
        switch (laneColor) {
            case GREEN:
                return clamp01(GREEN_POS);
            case PURPLE:
                return clamp01(PURPLE_POS);
            case NONE:
            case UNKNOWN:
            default:
                return fallbackPosition();
        }
    }

    private double fallbackPosition() {
        switch (state) {
            case BUSY:
                return clamp01(BUSY_POS);
            case ALLIANCE:
                return alliancePosition();
            case OFF:
            default:
                return clamp01(OFF_POS);
        }
    }

    private double alliancePosition() {
        if (alliance == Alliance.RED) {
            return clamp01(RED_POS);
        }
        if (alliance == Alliance.BLUE) {
            return clamp01(BLUE_POS);
        }
        return clamp01(GREEN_POS);
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private boolean resetLaneColors() {
        boolean touched = false;
        for (Map.Entry<LauncherLane, ArtifactColor> entry : laneColors.entrySet()) {
            if (entry.getValue() != ArtifactColor.NONE) {
                entry.setValue(ArtifactColor.NONE);
                touched = true;
            }
        }
        return touched;
    }

    private void populateLaneInputs(Inputs inputs, LauncherLane lane) {
        ArtifactColor color = laneColors.getOrDefault(lane, ArtifactColor.NONE);
        double output = laneOutputs.getOrDefault(lane, OFF_POS);
        LaneIndicator indicator = laneIndicators.get(lane);
        boolean present = indicator != null && indicator.isPresent();
        switch (lane) {
            case LEFT:
                inputs.leftColor = color.name();
                inputs.leftOutput = output;
                inputs.leftIndicatorPresent = present;
                break;
            case CENTER:
                inputs.centerColor = color.name();
                inputs.centerOutput = output;
                inputs.centerIndicatorPresent = present;
                break;
            case RIGHT:
            default:
                inputs.rightColor = color.name();
                inputs.rightOutput = output;
                inputs.rightIndicatorPresent = present;
                break;
        }
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
