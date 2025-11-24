package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.lighting.config.LightingColorPositionConfig;
import org.firstinspires.ftc.teamcode.subsystems.lighting.config.LightingIndicatorConfig;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

/**
 * goBILDA RGB Indicator Light wrapper.
 * Manages three lane-linked indicator servos.
 */
@Configurable
public class LightingSubsystem implements Subsystem, IntakeSubsystem.LaneColorListener {

    public enum LightingState {
        OFF,
        ALLIANCE,
        BUSY
    }

    // Global configuration instances
    public static LightingIndicatorConfig indicatorConfig = new LightingIndicatorConfig();
    public static LightingColorPositionConfig colorPositionConfig = new LightingColorPositionConfig();

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
    private long motifTailFlashUntilMs = 0L;
    private boolean motifTailRestoreSensorFollow = true;
    private long decodeModeSwitchFlashUntilMs = 0L;

    public LightingSubsystem(HardwareMap hardwareMap) {
        laneIndicators.put(LauncherLane.LEFT, new LaneIndicator(tryGetServo(hardwareMap, indicatorConfig.left.servoName)));
        laneIndicators.put(LauncherLane.CENTER, new LaneIndicator(tryGetServo(hardwareMap, indicatorConfig.center.servoName)));
        laneIndicators.put(LauncherLane.RIGHT, new LaneIndicator(tryGetServo(hardwareMap, indicatorConfig.right.servoName)));

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

        // Show rainbow pattern during DECODE mode switch notification
        if (decodeModeSwitchFlashUntilMs > 0) {
            if (nowMs >= decodeModeSwitchFlashUntilMs) {
                decodeModeSwitchFlashUntilMs = 0L;
                setFollowSensorColors(true); // Restore normal tracking
            } else {
                // Continue showing rainbow pattern
                showRainbowAlert(nowMs);
            }
        }

        // Handle aim flash timeout

        if (aimFlashUntilMs > 0 && nowMs >= aimFlashUntilMs) {
            aimFlashUntilMs = 0L;
            if (aimFlashRestoreSensorFollow) {
                setFollowSensorColors(true);
            }
        }


        // Handle motif tail feedback timeout
        if (motifTailFlashUntilMs > 0 && nowMs >= motifTailFlashUntilMs) {
            motifTailFlashUntilMs = 0L;
            if (motifTailRestoreSensorFollow) {
                setFollowSensorColors(true);
            }
        }

        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public double getLastPeriodicMs() { return lastPeriodicMs; }

    public void setAlliance(Alliance alliance) {
        this.alliance = (alliance == null ? Alliance.UNKNOWN : alliance);
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
        if (followSensorColors == enabled) return;

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
            resetLaneColors();
            updateLaneOutputs();
            return;
        }
        resetLaneColors();
        LauncherLane[] lanes = LauncherLane.values();
        int limit = Math.min(pattern.length, lanes.length);
        for (int i = 0; i < limit; i++) {
            ArtifactColor normalized = (pattern[i] == null ? ArtifactColor.NONE : pattern[i]);
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
            case 0:  applyPattern(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.UNKNOWN); break;
            case 1:  applyPattern(ArtifactColor.GREEN, ArtifactColor.UNKNOWN, ArtifactColor.PURPLE); break;
            default: applyPattern(ArtifactColor.UNKNOWN, ArtifactColor.PURPLE, ArtifactColor.GREEN); break;
        }
    }

    public void showAlliancePulse(Alliance alliance, boolean brightPhase) {
        setAlliance(alliance);
        setFollowSensorColors(false);

        if (brightPhase) {
            state = LightingState.ALLIANCE;
            resetLaneColors();
        } else {
            state = LightingState.OFF;
        }

        updateLaneOutputs();
    }

    public void showWhiteBlink(boolean on) {
        setFollowSensorColors(false);
        double position = on ? clamp01(colorPositionConfig.whitePosition)
                : clamp01(colorPositionConfig.offPosition);
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
        aimFlashUntilMs = System.currentTimeMillis() + 8000L;
    }

    /**
     * Shows visual feedback for motif tail selection using orange/yellow blink pattern.
     *
     * Motif tail 0: All 3 lanes blink orange
     * Motif tail 1: Left lane blinks orange (center/right off)
     * Motif tail 2: Left and center blink orange (right off)
     *
     * Pattern displays for 2 seconds, then resumes normal lane tracking.
     *
     * @param value The motif tail value (0, 1, or 2)
     */
    public void showMotifTailFeedback(int value) {
        motifTailRestoreSensorFollow = followSensorColors;
        setFollowSensorColors(false);

        // Use yellow/orange position for visual feedback
        double orangePosition = clamp01(colorPositionConfig.yellowPosition);
        double offPosition = clamp01(colorPositionConfig.offPosition);

        // Clear all lanes first
        resetLaneColors();

        // Set pattern based on motif tail value
        switch (value) {
            case 0:
                // All 3 lanes blink orange
                laneOutputs.put(LauncherLane.LEFT, orangePosition);
                laneOutputs.put(LauncherLane.CENTER, orangePosition);
                laneOutputs.put(LauncherLane.RIGHT, orangePosition);
                break;
            case 1:
                // Left lane only
                laneOutputs.put(LauncherLane.LEFT, orangePosition);
                laneOutputs.put(LauncherLane.CENTER, offPosition);
                laneOutputs.put(LauncherLane.RIGHT, offPosition);
                break;
            case 2:
                // Left and center
                laneOutputs.put(LauncherLane.LEFT, orangePosition);
                laneOutputs.put(LauncherLane.CENTER, orangePosition);
                laneOutputs.put(LauncherLane.RIGHT, offPosition);
                break;
            default:
                // Invalid value - show all off
                laneOutputs.put(LauncherLane.LEFT, offPosition);
                laneOutputs.put(LauncherLane.CENTER, offPosition);
                laneOutputs.put(LauncherLane.RIGHT, offPosition);
                break;
        }

        // Apply the pattern to servos
        for (LauncherLane lane : LauncherLane.values()) {
            LaneIndicator indicator = laneIndicators.get(lane);
            if (indicator != null && indicator.isPresent()) {
                indicator.apply(laneOutputs.get(lane));
            }
        }

        // Set timeout to restore normal tracking after 2 seconds
        motifTailFlashUntilMs = System.currentTimeMillis() + 2000L;
    }

    /**
     * Shows visual notification that robot has switched to DECODE mode.
     * Displays rainbow pattern for 2 seconds.
     */
    public void showDecodeModeSwitchNotification() {
        setFollowSensorColors(false);
        // Start rainbow alert that will cycle during periodic()
        decodeModeSwitchFlashUntilMs = System.currentTimeMillis() + 2000L;
    }

    private void updateLaneOutputs() {
        for (LauncherLane lane : LauncherLane.values()) {
            applyLaneOutput(lane);
        }
    }

    private void applyLaneOutput(LauncherLane lane) {
        LaneIndicator indicator = laneIndicators.get(lane);
        if (indicator == null || !indicator.isPresent()) return;

        ArtifactColor laneColor = laneColors.getOrDefault(lane, ArtifactColor.NONE);
        double position = resolvePosition(laneColor);

        laneOutputs.put(lane, position);
        indicator.apply(position);
    }

    protected double resolvePosition(ArtifactColor laneColor) {
        switch (laneColor) {
            case GREEN:   return clamp01(colorPositionConfig.greenPosition);
            case PURPLE:  return clamp01(colorPositionConfig.purplePosition);
            case UNKNOWN: return clamp01(colorPositionConfig.whitePosition);
            case NONE:
            case BACKGROUND:
            default:      return fallbackPosition();
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
        if (alliance == Alliance.RED)  return clamp01(colorPositionConfig.redPosition);
        if (alliance == Alliance.BLUE) return clamp01(colorPositionConfig.bluePosition);
        return clamp01(colorPositionConfig.greenPosition);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
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
            if (indicator != null) indicator.apply(position);
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
        if (name == null || name.isEmpty()) return null;

        try {
            return hardwareMap.get(Servo.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    /**
     * Init Controller (static nested class), now using `lights` reference
     */
    public static class InitController {

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
        private final LightingSubsystem lightingSubsystem;
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

        public InitController(Robot robot, AllianceSelector allianceSelector, LightingSubsystem lightingSubsystem) {
            this.robot = robot;
            this.allianceSelector = allianceSelector;
            this.lightingSubsystem = lightingSubsystem;
        }

        public void initialize() {
            stage = Stage.PRELOAD_CHECK;
            observedMotif = RobotState.getMotif();
            motifPersisted = (observedMotif != null && observedMotif != MotifPattern.UNKNOWN);

            lightingSubsystem.setFollowSensorColors(false);
//            lightingSubsystem.showRainbowAlert(System.currentTimeMillis());

            nextAllianceReminderMs = System.currentTimeMillis() + ALLIANCE_REMINDER_PERIOD_MS;
        }

        public void updateDuringInit(boolean confirmAlliance) {
            long now = System.currentTimeMillis();

            switch (stage) {
                case PRELOAD_CHECK:
                    handlePreloadCheck(now);
                    break;

                case ALLIANCE_SELECTION:
                    handleAllianceSelection(now, confirmAlliance);
                    break;

                case MOTIF_TRACKING:
                    handleMotifTracking(now);
                    break;

                default:
                    break;
            }
        }

        public void onStart() {
            if (observedMotif == null) observedMotif = MotifPattern.UNKNOWN;

            RobotState.setMotif(observedMotif);
            motifPersisted = (observedMotif != MotifPattern.UNKNOWN);

            stage = Stage.MATCH;
            lightingSubsystem.resumeLaneTracking();
        }

        public void updateDuringMatch() {
            if (stage != Stage.MATCH || motifPersisted) return;

            Optional<Integer> motifTag = findMotifTagId();
            if (motifTag.isPresent()) {
                observedMotif = MotifPattern.fromTagId(motifTag.get());
                RobotState.setMotif(observedMotif);
                motifPersisted = true;
            }
        }

        private void handlePreloadCheck(long now) {
            if (!hasRequiredPreloads()) {
                lightingSubsystem.showRainbowAlert(now);
                return;
            }

            stage = Stage.ALLIANCE_SELECTION;
            lastAlliancePulseMs = now;
        }

        private void handleAllianceSelection(long now, boolean confirmAlliance) {
            Alliance selectedAlliance = (allianceSelector == null)
                    ? Alliance.UNKNOWN
                    : allianceSelector.getSelectedAlliance();

            Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = Optional.empty();

            if (allianceSelector != null && robot != null) {
                snapshotOpt = allianceSelector.updateFromVision(robot.vision);
                allianceSelector.applySelection(robot, lightingSubsystem);

                selectedAlliance = allianceSelector.getSelectedAlliance();
            }

            if (confirmAlliance && allianceSelector != null) {
                allianceSelector.lockSelection();
            }

            if (allianceSelector != null
                    && snapshotOpt.isPresent()
                    && !allianceSelector.isSelectionLocked()) {
                allianceSelector.lockSelection();
            }

            if (selectedAlliance != Alliance.UNKNOWN
                    && allianceSelector != null
                    && allianceSelector.isSelectionLocked()) {

                lightingSubsystem.showSolidAlliance(selectedAlliance);

                stage = Stage.MOTIF_TRACKING;
                nextAllianceReminderMs = now + ALLIANCE_REMINDER_PERIOD_MS;
                return;
            }

            // Alliance pulsing animation
            Alliance pulseAlliance = random.nextBoolean() ? Alliance.BLUE : Alliance.RED;

            if (now - lastAlliancePulseMs >= ALLIANCE_PULSE_INTERVAL_MS) {
                alliancePulsePhase = !alliancePulsePhase;
                lastAlliancePulseMs = now;
            }

            lightingSubsystem.showAlliancePulse(pulseAlliance, alliancePulsePhase);
        }

        private void handleMotifTracking(long now) {
            Optional<Integer> motifTag = findMotifTagId();
            if (motifTag.isPresent()) {
                observedMotif = MotifPattern.fromTagId(motifTag.get());
            }

            boolean showingAlliance = (allianceReminderUntilMs > now);

            if (observedMotif == MotifPattern.UNKNOWN && !showingAlliance) {

                if (now - lastWhiteBlinkMs >= WHITE_BLINK_INTERVAL_MS) {
                    lastWhiteBlinkMs = now;
                    whiteBlinkState = !whiteBlinkState;
                }

                lightingSubsystem.showWhiteBlink(whiteBlinkState);

            } else if (!showingAlliance) {

                lightingSubsystem.showMotifPattern(observedMotif.getLaneColors());
            }

            Alliance selectedAlliance =
                    (allianceSelector == null)
                            ? RobotState.getAlliance()
                            : allianceSelector.getSelectedAlliance();

            if (selectedAlliance == null) selectedAlliance = Alliance.UNKNOWN;

            if (selectedAlliance != Alliance.UNKNOWN && now >= nextAllianceReminderMs) {
                allianceReminderUntilMs = now + ALLIANCE_REMINDER_DURATION_MS;
                nextAllianceReminderMs = now + ALLIANCE_REMINDER_PERIOD_MS;
            }

            if (selectedAlliance != Alliance.UNKNOWN && allianceReminderUntilMs > now) {
                lightingSubsystem.showAllianceReminder(selectedAlliance);
            }
        }

        private Optional<Integer> findMotifTagId() {
            if (robot == null || robot.vision == null) return Optional.empty();
            return robot.vision.findMotifTagId();
        }

        private boolean hasRequiredPreloads() {
            EnumMap<LauncherLane, ArtifactColor> snapshot = lightingSubsystem.getSensorLaneColorSnapshot();

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

    /** Servo wrapper */
    private static final class LaneIndicator {
        private final Servo servo;

        LaneIndicator(Servo servo) { this.servo = servo; }

        boolean isPresent() { return servo != null; }

        void apply(double position) {
            if (servo == null) return;
            servo.setPosition(clamp01(position));
        }
    }
}
