package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.EnumMap;
import java.util.Optional;
import java.util.Random;

/**
 * Manages staged lighting behavior during init and throughout the match.
 */
public class LightingInitController {

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

    public LightingInitController(Robot robot, AllianceSelector allianceSelector) {
        this.robot = robot;
        this.allianceSelector = allianceSelector;
    }

    public void initialize() {
        stage = Stage.PRELOAD_CHECK;
        observedMotif = RobotState.getMotif();
        motifPersisted = observedMotif != null && observedMotif != MotifPattern.UNKNOWN;
        if (robot != null && robot.lighting != null) {
            robot.lighting.setFollowSensorColors(false);
            robot.lighting.showRainbowAlert(System.currentTimeMillis());
        }
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
        if (robot != null && robot.lighting != null) {
            robot.lighting.resumeLaneTracking();
        }
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
            if (robot != null && robot.lighting != null) {
                robot.lighting.showRainbowAlert(now);
            }
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
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }

        if (confirmAlliance && allianceSelector != null) {
            allianceSelector.lockSelection();
        }

        if (allianceSelector != null && snapshotOpt.isPresent() && !allianceSelector.isSelectionLocked()) {
            allianceSelector.lockSelection();
        }

        if (selectedAlliance != Alliance.UNKNOWN && allianceSelector != null && allianceSelector.isSelectionLocked()) {
            if (robot != null && robot.lighting != null) {
                robot.lighting.showSolidAlliance(selectedAlliance);
            }
            stage = Stage.MOTIF_TRACKING;
            nextAllianceReminderMs = now + ALLIANCE_REMINDER_PERIOD_MS;
            return;
        }

        Alliance pulseAlliance = random.nextBoolean() ? Alliance.BLUE : Alliance.RED;
        if (now - lastAlliancePulseMs >= ALLIANCE_PULSE_INTERVAL_MS) {
            alliancePulsePhase = !alliancePulsePhase;
            lastAlliancePulseMs = now;
        }
        if (robot != null && robot.lighting != null) {
            robot.lighting.showAlliancePulse(pulseAlliance, alliancePulsePhase);
        }
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
            if (robot != null && robot.lighting != null) {
                robot.lighting.showWhiteBlink(whiteBlinkState);
            }
        } else if (!showingAlliance) {
            if (robot != null && robot.lighting != null) {
                robot.lighting.showMotifPattern(observedMotif.getLaneColors());
            }
        }

        Alliance selectedAlliance = allianceSelector == null ? RobotState.getAlliance() : allianceSelector.getSelectedAlliance();
        if (selectedAlliance == null) {
            selectedAlliance = Alliance.UNKNOWN;
        }
        if (selectedAlliance != Alliance.UNKNOWN && now >= nextAllianceReminderMs) {
            allianceReminderUntilMs = now + ALLIANCE_REMINDER_DURATION_MS;
            nextAllianceReminderMs = now + ALLIANCE_REMINDER_PERIOD_MS;
        }

        if (selectedAlliance != Alliance.UNKNOWN && allianceReminderUntilMs > now && robot != null && robot.lighting != null) {
            robot.lighting.showAllianceReminder(selectedAlliance);
        }
    }

    private Optional<Integer> findMotifTagId() {
        if (robot == null || robot.vision == null) {
            return Optional.empty();
        }
        return robot.vision.findMotifTagId();
    }

    private boolean hasRequiredPreloads() {
        if (robot == null || robot.lighting == null) {
            return false;
        }
        EnumMap<LauncherLane, ArtifactColor> snapshot = robot.lighting.getSensorLaneColorSnapshot();
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
