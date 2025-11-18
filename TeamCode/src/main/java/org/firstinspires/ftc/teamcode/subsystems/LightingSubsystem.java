package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;

import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

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
        /** Position used when colour is unknown but presence is detected */
        public double whitePosition = 0.944;
        /** Position used when flashing to indicate aiming alignment */
        public double yellowPosition = 0.167;
        /** Fallback position used when marking the robot busy */
        public double busyPosition = 0.722; // PURPLE by default
    }

    public static IndicatorConfig indicatorConfig = new IndicatorConfig();
    public static ColorPositionConfig colorPositionConfig = new ColorPositionConfig();

    private final EnumMap<LauncherLane, LaneIndicator> laneIndicators = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> sensorLaneColors = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, Double> laneOutputs = new EnumMap<>(LauncherLane.class);

    private LightingState state = LightingState.OFF;
    private Alliance alliance = Alliance.UNKNOWN;
    private double lastPeriodicMs = 0.0;
    private boolean followSensorColors = true;
    private long aimFlashUntilMs = 0L;
    private boolean aimFlashRestoreSensorFollow = true;

    public LightingSubsystem(HardwareMap hardwareMap) {
        laneIndicators.put(LauncherLane.LEFT, new LaneIndicator(
                tryGetServo(hardwareMap, indicatorConfig.left.servoName)));
        laneIndicators.put(LauncherLane.CENTER, new LaneIndicator(
                tryGetServo(hardwareMap, indicatorConfig.center.servoName)));
        laneIndicators.put(LauncherLane.RIGHT, new LaneIndicator(
                tryGetServo(hardwareMap, indicatorConfig.right.servoName)));

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
            sensorLaneColors.put(lane, ArtifactColor.NONE);
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
        long nowMs = System.currentTimeMillis();
        if (aimFlashUntilMs > 0 && nowMs >= aimFlashUntilMs) {
            aimFlashUntilMs = 0L;
            if (aimFlashRestoreSensorFollow) {
                setFollowSensorColors(true);
            }
        }
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
        updateSensorLaneColor(lane, color);
    }

    public void clearLaneColors() {
        if (resetLaneColors()) {
            updateLaneOutputs();
        }
    }

    public void updateSensorLaneColor(LauncherLane lane, ArtifactColor color) {
        if (lane == null) {
            return;
        }
        ArtifactColor normalized = color == null ? ArtifactColor.NONE : color;
        sensorLaneColors.put(lane, normalized);
        if (followSensorColors) {
            setLaneColor(lane, normalized);
        }
    }

    public EnumMap<LauncherLane, ArtifactColor> getSensorLaneColorSnapshot() {
        return new EnumMap<>(sensorLaneColors);
    }

    public void setFollowSensorColors(boolean enabled) {
        if (followSensorColors == enabled) {
            return;
        }
        followSensorColors = enabled;
        if (enabled) {
            for (Map.Entry<LauncherLane, ArtifactColor> entry : sensorLaneColors.entrySet()) {
                setLaneColor(entry.getKey(), entry.getValue());
            }
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

    public void showMotifPattern(ArtifactColor[] pattern) {
        setFollowSensorColors(false);
        showDecodePattern(pattern);
    }

    public void showRainbowAlert(long nowMs) {
        setFollowSensorColors(false);
        int phase = (int) ((nowMs / 300L) % 3);
        switch (phase) {
            case 0:
                applyPattern(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.UNKNOWN);
                break;
            case 1:
                applyPattern(ArtifactColor.GREEN, ArtifactColor.UNKNOWN, ArtifactColor.PURPLE);
                break;
            default:
                applyPattern(ArtifactColor.UNKNOWN, ArtifactColor.PURPLE, ArtifactColor.GREEN);
                break;
        }
    }

    public void showAlliancePulse(Alliance alliance, boolean brightPhase) {
        setAlliance(alliance);
        setFollowSensorColors(false);
        if (brightPhase) {
            state = LightingState.ALLIANCE;
            resetLaneColors();
            updateLaneOutputs();
        } else {
            state = LightingState.OFF;
            updateLaneOutputs();
        }
    }

    public void showWhiteBlink(boolean on) {
        setFollowSensorColors(false);
        double position = on ? clamp01(colorPositionConfig.whitePosition) : clamp01(colorPositionConfig.offPosition);
        applySolidPosition(position);
    }

    public void showSolidAlliance(Alliance alliance) {
        setAlliance(alliance);
        state = LightingState.ALLIANCE;
        resetLaneColors();
        updateLaneOutputs();
    }

    public void showAllianceReminder(Alliance alliance) {
        showSolidAlliance(alliance);
    }

    public void resumeLaneTracking() {
        setFollowSensorColors(true);
    }

    public void flashAimAligned() {
        aimFlashRestoreSensorFollow = followSensorColors;
        setFollowSensorColors(false);
        applySolidPosition(clamp01(colorPositionConfig.yellowPosition));
        aimFlashUntilMs = System.currentTimeMillis() + 2000L;
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
            case UNKNOWN:
                return clamp01(colorPositionConfig.whitePosition);
            case NONE:
            case BACKGROUND:
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

    private void applyPattern(ArtifactColor left, ArtifactColor center, ArtifactColor right) {
        laneColors.put(LauncherLane.LEFT, left);
        laneColors.put(LauncherLane.CENTER, center);
        laneColors.put(LauncherLane.RIGHT, right);
        updateLaneOutputs();
    }

    private void applySolidPosition(double position) {
        for (LauncherLane lane : LauncherLane.values()) {
            laneOutputs.put(lane, clamp01(position));
            LaneIndicator indicator = laneIndicators.get(lane);
            if (indicator != null) {
                indicator.apply(position);
            }
        }
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

    /**
     * Integrated controller that orchestrates staged lighting during init and match play.
     */
    public class InitController {

        public enum Stage {
            PRELOAD_CHECK,
            ALLIANCE_SELECTION,
            MOTIF_TRACKING,
            MATCH
        }

        private static final long WHITE_BLINK_INTERVAL_MS = 450L;
        private static final long ALLIANCE_PULSE_INTERVAL_MS = 650L;
        private static final long ALLIANCE_REMINDER_PERIOD_MS = 10_000L;
        private static final long ALLIANCE_REMINDER_DURATION_MS = 1_000L;

        private final Robot robot;
        private final AllianceSelector allianceSelector;
        private final Random random = new Random();

        private Stage stage = Stage.PRELOAD_CHECK;
        private MotifPattern observedMotif = MotifPattern.UNKNOWN;
        private boolean motifPersisted = false;
        private long lastWhiteBlinkMs = 0L;
        private boolean whiteBlinkState = false;
        private long lastAlliancePulseMs = 0L;
        private boolean alliancePulsePhase = false;
        private long nextAllianceReminderMs = 0L;
        private long allianceReminderUntilMs = 0L;

        public InitController(Robot robot, AllianceSelector allianceSelector) {
            this.robot = robot;
            this.allianceSelector = allianceSelector;
        }

        public void initialize() {
            stage = Stage.PRELOAD_CHECK;
            observedMotif = RobotState.getMotif();
            motifPersisted = observedMotif != null && observedMotif != MotifPattern.UNKNOWN;
            setFollowSensorColors(false);
            showRainbowAlert(System.currentTimeMillis());
            nextAllianceReminderMs = System.currentTimeMillis() + ALLIANCE_REMINDER_PERIOD_MS;
        }

        public void updateDuringInit(boolean confirmAlliance) {
            long now = System.currentTimeMillis();
            if (stage == Stage.PRELOAD_CHECK) {
                handlePreloadCheck(now);
            } else if (stage == Stage.ALLIANCE_SELECTION) {
                handleAllianceSelection(now, confirmAlliance);
            } else if (stage == Stage.MOTIF_TRACKING) {
                handleMotifTracking(now);
            }
        }

        public void onStart() {
            if (observedMotif == null) {
                observedMotif = MotifPattern.UNKNOWN;
            }
            RobotState.setMotif(observedMotif);
            motifPersisted = observedMotif != MotifPattern.UNKNOWN;
            stage = Stage.MATCH;
            resumeLaneTracking();
        }

        public void updateDuringMatch() {
            if (stage != Stage.MATCH || motifPersisted) {
                return;
            }
            Optional<Integer> motifTag = findMotifTagId();
            if (motifTag.isPresent()) {
                observedMotif = MotifPattern.fromTagId(motifTag.get());
                RobotState.setMotif(observedMotif);
                motifPersisted = true;
            }
        }

        private void handlePreloadCheck(long now) {
            if (!hasRequiredPreloads()) {
                showRainbowAlert(now);
                return;
            }
            stage = Stage.ALLIANCE_SELECTION;
            lastAlliancePulseMs = now;
        }

        private void handleAllianceSelection(long now, boolean confirmAlliance) {
            Alliance selectedAlliance = allianceSelector == null ? Alliance.UNKNOWN : allianceSelector.getSelectedAlliance();
            Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = Optional.empty();
            if (allianceSelector != null && robot != null) {
                snapshotOpt = allianceSelector.updateFromVision(robot.vision);
                allianceSelector.applySelection(robot, LightingSubsystem.this);
                selectedAlliance = allianceSelector.getSelectedAlliance();
            }

            if (confirmAlliance && allianceSelector != null) {
                allianceSelector.lockSelection();
            }

            if (allianceSelector != null && snapshotOpt.isPresent() && !allianceSelector.isSelectionLocked()) {
                allianceSelector.lockSelection();
            }

            if (selectedAlliance != Alliance.UNKNOWN && allianceSelector != null && allianceSelector.isSelectionLocked()) {
                showSolidAlliance(selectedAlliance);
                stage = Stage.MOTIF_TRACKING;
                nextAllianceReminderMs = now + ALLIANCE_REMINDER_PERIOD_MS;
                return;
            }

            Alliance pulseAlliance = random.nextBoolean() ? Alliance.BLUE : Alliance.RED;
            if (now - lastAlliancePulseMs >= ALLIANCE_PULSE_INTERVAL_MS) {
                alliancePulsePhase = !alliancePulsePhase;
                lastAlliancePulseMs = now;
            }
            showAlliancePulse(pulseAlliance, alliancePulsePhase);
        }

        private void handleMotifTracking(long now) {
            Optional<Integer> motifTag = findMotifTagId();
            if (motifTag.isPresent()) {
                observedMotif = MotifPattern.fromTagId(motifTag.get());
            }

            boolean showingAlliance = allianceReminderUntilMs > now;
            if (observedMotif == MotifPattern.UNKNOWN && !showingAlliance) {
                if (now - lastWhiteBlinkMs >= WHITE_BLINK_INTERVAL_MS) {
                    lastWhiteBlinkMs = now;
                    whiteBlinkState = !whiteBlinkState;
                }
                showWhiteBlink(whiteBlinkState);
            } else if (!showingAlliance) {
                showMotifPattern(observedMotif.getLaneColors());
            }

            Alliance selectedAlliance = allianceSelector == null ? RobotState.getAlliance() : allianceSelector.getSelectedAlliance();
            if (selectedAlliance == null) {
                selectedAlliance = Alliance.UNKNOWN;
            }
            if (selectedAlliance != Alliance.UNKNOWN && now >= nextAllianceReminderMs) {
                allianceReminderUntilMs = now + ALLIANCE_REMINDER_DURATION_MS;
                nextAllianceReminderMs = now + ALLIANCE_REMINDER_PERIOD_MS;
            }

            if (selectedAlliance != Alliance.UNKNOWN && allianceReminderUntilMs > now) {
                showAllianceReminder(selectedAlliance);
            }
        }

        private Optional<Integer> findMotifTagId() {
            if (robot == null || robot.vision == null) {
                return Optional.empty();
            }
            return robot.vision.findMotifTagId();
        }

        private boolean hasRequiredPreloads() {
            EnumMap<LauncherLane, ArtifactColor> snapshot = getSensorLaneColorSnapshot();
            int greens = 0;
            int purples = 0;
            int detected = 0;
            for (ArtifactColor color : snapshot.values()) {
                if (color == ArtifactColor.GREEN) {
                    greens++;
                    detected++;
                } else if (color == ArtifactColor.PURPLE) {
                    purples++;
                    detected++;
                }
            }
            return detected == 3 && greens == 1 && purples == 2;
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
