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
 * Manages three lane-linked indicator servos with priority-based pattern control.
 *
 * Architecture follows FRC-style priority pattern:
 * - Goal pattern: What the lights SHOULD show based on robot state
 * - Current pattern: What's actively being displayed (may differ temporarily)
 * - Priority resolution: Higher priority patterns override lower ones
 */
@Configurable
public class LightingSubsystem implements Subsystem, IntakeSubsystem.LaneColorListener {

    /**
     * Lighting patterns with built-in priority ordering.
     * Higher priority values take precedence over lower ones.
     */
    public enum LightingPattern {
        OFF(0),                      // Disabled/off state
        BUSY(1),                     // Subsystem busy indication
        ALLIANCE(2),                 // Alliance color fallback
        LANE_TRACKING(3),            // Normal operation - sensor colors
        MOTIF_PATTERN(4),            // Showing scanned motif
        WHITE_BLINK(5),              // Waiting for motif scan
        ALLIANCE_PULSE(6),           // Init animation
        ALLIANCE_REMINDER(7),        // Periodic reminder during init
        MOTIF_TAIL_FEEDBACK(8),      // Operator feedback
        AIM_ALIGNED(9),              // Driver feedback - important for scoring
        RAINBOW_ALERT(10);           // DECODE mode switch - highest visual priority

        public final int priority;
        LightingPattern(int priority) {
            this.priority = priority;
        }

        public boolean hasHigherPriorityThan(LightingPattern other) {
            return this.priority > other.priority;
        }
    }

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

    // Priority-based pattern management
    private LightingPattern currentPattern = LightingPattern.OFF;
    private LightingPattern goalPattern = LightingPattern.OFF;
    private long patternExpirationMs = 0L;  // When current pattern expires (0 = permanent)

    // Legacy state tracking (kept for backward compatibility)
    private LightingState state = LightingState.OFF;
    private Alliance alliance = Alliance.UNKNOWN;
    private double lastPeriodicMs = 0.0;
    private boolean followSensorColors = true;

    // Pattern-specific state
    private int motifTailValue = 0;
    private ArtifactColor[] motifPattern = null;
    private boolean whiteBlinkState = false;
    private long lastWhiteBlinkMs = 0L;
    private Alliance pulseAlliance = Alliance.UNKNOWN;
    private boolean pulseBrightPhase = false;
    private Alliance reminderAlliance = Alliance.UNKNOWN;

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

        // Check if current pattern has expired
        if (patternExpirationMs > 0 && nowMs >= patternExpirationMs) {
            patternExpirationMs = 0L;
            currentPattern = LightingPattern.LANE_TRACKING; // Revert to normal operation
            followSensorColors = true;
        }

        // Update goal pattern based on robot state
        updateGoalPattern();

        // Apply priority resolution: only allow higher-priority patterns to interrupt
        if (goalPattern.hasHigherPriorityThan(currentPattern) || patternExpirationMs == 0L) {
            currentPattern = goalPattern;
        }

        // Render the current pattern
        renderCurrentPattern(nowMs);

        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    /**
     * Updates the goal pattern based on current robot state and legacy state flags.
     * Does NOT directly change what's displayed - priority resolution handles that.
     */
    private void updateGoalPattern() {
        // Default to lane tracking or alliance based on legacy state
        if (followSensorColors) {
            goalPattern = LightingPattern.LANE_TRACKING;
        } else if (state == LightingState.BUSY) {
            goalPattern = LightingPattern.BUSY;
        } else if (state == LightingState.ALLIANCE) {
            goalPattern = LightingPattern.ALLIANCE;
        } else {
            goalPattern = LightingPattern.OFF;
        }
    }

    /**
     * Renders the current pattern to the physical LEDs.
     * Called every periodic() cycle.
     */
    private void renderCurrentPattern(long nowMs) {
        switch (currentPattern) {
            case RAINBOW_ALERT:
                renderRainbowAlert(nowMs);
                break;

            case AIM_ALIGNED:
                renderAimAligned();
                break;

            case MOTIF_TAIL_FEEDBACK:
                renderMotifTailFeedback();
                break;

            case ALLIANCE_REMINDER:
                renderAllianceReminder();
                break;

            case ALLIANCE_PULSE:
                renderAlliancePulse();
                break;

            case WHITE_BLINK:
                renderWhiteBlink(nowMs);
                break;

            case MOTIF_PATTERN:
                renderMotifPattern();
                break;

            case LANE_TRACKING:
                renderLaneTracking();
                break;

            case ALLIANCE:
                renderAlliance();
                break;

            case BUSY:
                renderBusy();
                break;

            case OFF:
            default:
                renderOff();
                break;
        }
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
        motifPattern = pattern;
        requestPattern(LightingPattern.MOTIF_PATTERN, 0L);  // Permanent until overridden
    }

    public void showRainbowAlert(long nowMs) {
        // Legacy method - redirect to new pattern system
        requestPattern(LightingPattern.RAINBOW_ALERT, 2000L);
    }

    public void showAlliancePulse(Alliance alliance, boolean brightPhase) {
        setAlliance(alliance);
        pulseAlliance = alliance;
        pulseBrightPhase = brightPhase;
        requestPattern(LightingPattern.ALLIANCE_PULSE, 0L);  // Managed by InitController
    }

    public void showWhiteBlink(boolean on) {
        whiteBlinkState = on;
        requestPattern(LightingPattern.WHITE_BLINK, 0L);  // Managed by InitController
    }

    public void showSolidAlliance(Alliance alliance) {
        setAlliance(alliance);
        reminderAlliance = alliance;
        state = LightingState.ALLIANCE;
        requestPattern(LightingPattern.ALLIANCE, 0L);
    }

    public void showAllianceReminder(Alliance alliance) {
        reminderAlliance = alliance;
        setAlliance(alliance);
        requestPattern(LightingPattern.ALLIANCE_REMINDER, 1000L);
    }

    public void resumeLaneTracking() {
        setFollowSensorColors(true);
    }

    /**
     * Requests aim-aligned visual feedback (yellow flash).
     * Priority: HIGH - will override most patterns except rainbow alert.
     * Duration: 8 seconds
     */
    public void flashAimAligned() {
        requestPattern(LightingPattern.AIM_ALIGNED, 8000L);
    }

    /**
     * Shows visual feedback for motif tail selection using orange/yellow pattern.
     *
     * Motif tail 0: All 3 lanes blink orange
     * Motif tail 1: Left lane blinks orange (center/right off)
     * Motif tail 2: Left and center blink orange (right off)
     *
     * Priority: MEDIUM - can be interrupted by aim feedback or rainbow alert.
     * Duration: 2 seconds
     *
     * @param value The motif tail value (0, 1, or 2)
     */
    public void showMotifTailFeedback(int value) {
        motifTailValue = value;
        requestPattern(LightingPattern.MOTIF_TAIL_FEEDBACK, 2000L);
    }

    /**
     * Shows visual notification that robot has switched to DECODE mode.
     * Displays rainbow pattern.
     * Priority: HIGHEST - overrides all other patterns.
     * Duration: 2 seconds
     */
    public void showDecodeModeSwitchNotification() {
        requestPattern(LightingPattern.RAINBOW_ALERT, 2000L);
    }

    /**
     * Core method to request a temporary pattern with expiration.
     * Honors priority system - only activates if higher priority than current.
     *
     * @param pattern The pattern to display
     * @param durationMs How long to show it (0 = permanent)
     */
    private void requestPattern(LightingPattern pattern, long durationMs) {
        if (pattern.hasHigherPriorityThan(currentPattern) || patternExpirationMs == 0L) {
            currentPattern = pattern;
            goalPattern = pattern;
            patternExpirationMs = (durationMs > 0) ? System.currentTimeMillis() + durationMs : 0L;
            followSensorColors = false;
        }
    }

    // ========== RENDER METHODS ==========
    // Each method renders a specific pattern to the physical LEDs

    private void renderRainbowAlert(long nowMs) {
        int phase = (int) ((nowMs / 300L) % 3);
        switch (phase) {
            case 0:  applyPattern(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.UNKNOWN); break;
            case 1:  applyPattern(ArtifactColor.GREEN, ArtifactColor.UNKNOWN, ArtifactColor.PURPLE); break;
            default: applyPattern(ArtifactColor.UNKNOWN, ArtifactColor.PURPLE, ArtifactColor.GREEN); break;
        }
    }

    private void renderAimAligned() {
        applySolidPosition(clamp01(colorPositionConfig.yellowPosition));
    }

    private void renderMotifTailFeedback() {
        double orangePosition = clamp01(colorPositionConfig.yellowPosition);
        double offPosition = clamp01(colorPositionConfig.offPosition);

        switch (motifTailValue) {
            case 0:  // All 3 lanes
                applySolidPosition(orangePosition);
                break;
            case 1:  // Left lane only
                laneOutputs.put(LauncherLane.LEFT, orangePosition);
                laneOutputs.put(LauncherLane.CENTER, offPosition);
                laneOutputs.put(LauncherLane.RIGHT, offPosition);
                applyLaneOutputs();
                break;
            case 2:  // Left and center
                laneOutputs.put(LauncherLane.LEFT, orangePosition);
                laneOutputs.put(LauncherLane.CENTER, orangePosition);
                laneOutputs.put(LauncherLane.RIGHT, offPosition);
                applyLaneOutputs();
                break;
            default:  // Invalid - all off
                applySolidPosition(offPosition);
                break;
        }
    }

    private void renderAllianceReminder() {
        // Show solid alliance color
        resetLaneColors();
        updateLaneOutputs();
    }

    private void renderAlliancePulse() {
        if (pulseBrightPhase) {
            resetLaneColors();
            updateLaneOutputs();
        } else {
            applySolidPosition(clamp01(colorPositionConfig.offPosition));
        }
    }

    private void renderWhiteBlink(long nowMs) {
        if (nowMs - lastWhiteBlinkMs >= 450L) {
            whiteBlinkState = !whiteBlinkState;
            lastWhiteBlinkMs = nowMs;
        }
        double position = whiteBlinkState
                ? clamp01(colorPositionConfig.whitePosition)
                : clamp01(colorPositionConfig.offPosition);
        applySolidPosition(position);
    }

    private void renderMotifPattern() {
        if (motifPattern != null && motifPattern.length > 0) {
            resetLaneColors();
            LauncherLane[] lanes = LauncherLane.values();
            int limit = Math.min(motifPattern.length, lanes.length);
            for (int i = 0; i < limit; i++) {
                ArtifactColor normalized = (motifPattern[i] == null ? ArtifactColor.NONE : motifPattern[i]);
                laneColors.put(lanes[i], normalized);
            }
            updateLaneOutputs();
        }
    }

    private void renderLaneTracking() {
        // Update lane colors from sensors
        for (Map.Entry<LauncherLane, ArtifactColor> entry : sensorLaneColors.entrySet()) {
            laneColors.put(entry.getKey(), entry.getValue());
        }
        updateLaneOutputs();
    }

    private void renderAlliance() {
        resetLaneColors();
        updateLaneOutputs();
    }

    private void renderBusy() {
        applySolidPosition(clamp01(colorPositionConfig.busyPosition));
    }

    private void renderOff() {
        applySolidPosition(clamp01(colorPositionConfig.offPosition));
    }

    // ========== END RENDER METHODS ==========

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

    private void applyLaneOutputs() {
        for (LauncherLane lane : LauncherLane.values()) {
            LaneIndicator indicator = laneIndicators.get(lane);
            if (indicator != null && indicator.isPresent()) {
                indicator.apply(laneOutputs.get(lane));
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
