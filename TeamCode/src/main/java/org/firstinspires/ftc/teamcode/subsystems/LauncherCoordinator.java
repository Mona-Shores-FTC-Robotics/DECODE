package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogOutput;

import java.util.EnumMap;
import java.util.Map;

/**
 * Coordinates the launcher subsystems: intake lane sensing, lighting feedback, and launcher spin/feed.
 * Shared by bench diagnostics, teleop, and autonomous so behaviour remains consistent.
 */
@Configurable
@AutoLog
public class LauncherCoordinator implements Subsystem, IntakeSubsystem.LaneColorListener {

    public enum ArtifactState {
        EMPTY(0),
        ONE(1),
        TWO(2),
        THREE(3);

        private final int count;

        ArtifactState(int count) {
            this.count = count;
        }

        public int count() {
            return count;
        }

        public static ArtifactState fromCount(int count) {
            if (count <= 0) {
                return EMPTY;
            }
            if (count == 1) {
                return ONE;
            }
            if (count == 2) {
                return TWO;
            }
            return THREE;
        }
    }

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final LightingSubsystem lighting;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private boolean lightingRegistered = false;
    private double lastPeriodicMs = 0.0;
    private boolean intakeAutomationEnabled = true;
    private ArtifactState artifactState = ArtifactState.EMPTY;
    private boolean manualSpinOverride = false;
    private IntakeSubsystem.IntakeMode manualIntakeOverride = IntakeSubsystem.IntakeMode.STOPPED;
    private IntakeSubsystem.IntakeMode appliedIntakeMode = IntakeSubsystem.IntakeMode.STOPPED;
    private boolean intakeLocked = false;

    public LauncherCoordinator(LauncherSubsystem launcher ,
                               IntakeSubsystem intake,
                               LightingSubsystem lighting) {
        this.launcher = launcher;
        this.intake = intake;
        this.lighting = lighting;

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
        }
    }

    @Override
    public void initialize() {
        intake.addLaneColorListener(this);
        refreshLightingRegistration();
        manualIntakeOverride = null;
        appliedIntakeMode = null;
        updateArtifactState();
        applyIntakeMode();
        manualSpinOverride = false;
        recomputeSpinMode();
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        updateArtifactState();
        applyIntakeMode();
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public void stop() {
        intake.removeLaneColorListener(this);
        removeLightingListener();
        clearIntakeOverride();
        appliedIntakeMode = null;
    }

    @Override
    public void onLaneColorChanged(LauncherLane lane, ArtifactColor color) {
        if (lane == null) {
            return;
        }
        ArtifactColor normalized = color == null ? ArtifactColor.NONE : color;
        laneColors.put(lane, normalized);
        if (lighting != null) {
            lighting.setLaneColor(lane, normalized);
        }
        updateArtifactState();
        applyIntakeMode();
        recomputeSpinMode();
    }

    public ArtifactColor getLaneColor(LauncherLane lane) {
        return lane == null ? ArtifactColor.NONE : laneColors.getOrDefault(lane, ArtifactColor.NONE);
    }

    public void requestKick(LauncherLane lane) {
        ArtifactColor color = getLaneColor(lane);
        if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
            return;
        }
        launcher.queueShot(lane);
    }

    public void requestBurst(double spacingMs) {
        double offset = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = getLaneColor(lane);
            if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
                continue;
            }
            launcher.queueShot(lane, offset);
            offset += Math.max(0.0, spacingMs);
        }
    }

    public ArtifactState getArtifactState() {
        return artifactState;
    }

    public int getArtifactCount() {
        return artifactState.count();
    }

    public void setIntakeAutomationEnabled(boolean enabled) {
        if (intakeAutomationEnabled != enabled) {
            intakeAutomationEnabled = enabled;
            applyIntakeMode();
        }
    }

    public boolean isIntakeAutomationEnabled() {
        return intakeAutomationEnabled;
    }

    public void overrideIntakeMode(IntakeSubsystem.IntakeMode mode) {
        if (mode == null) {
            clearIntakeOverride();
            return;
        }
        manualIntakeOverride = mode;
        applyIntakeMode();
    }

    public void clearIntakeOverride() {
        if (manualIntakeOverride != null) {
            manualIntakeOverride = null;
            applyIntakeMode();
        }
    }

    public boolean isManualIntakeOverrideActive() {
        return manualIntakeOverride != null;
    }

    public IntakeSubsystem.IntakeMode getRequestedIntakeMode() {
        if (intakeLocked) {
            return IntakeSubsystem.IntakeMode.STOPPED;
        }
        IntakeSubsystem.IntakeMode override = manualIntakeOverride;
        return override != null ? override : computeAutomationIntakeMode();
    }

    public void lockIntake() {
        if (!intakeLocked) {
            intakeLocked = true;
            applyIntakeMode();
        }
    }

    public void unlockIntake() {
        if (intakeLocked) {
            intakeLocked = false;
            applyIntakeMode();
        }
    }

    public IntakeSubsystem.IntakeMode getAppliedIntakeMode() {
        if (intake != null) {
            return intake.getResolvedMode();
        }
        return appliedIntakeMode != null ? appliedIntakeMode : IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
    }

    private void recomputeSpinMode() {
        if (manualSpinOverride) {
            return;
        }
        boolean anyActive = false;
        for (Map.Entry<LauncherLane, ArtifactColor> entry : laneColors.entrySet()) {
            ArtifactColor color = entry.getValue();
            if (color != ArtifactColor.NONE && color != ArtifactColor.UNKNOWN) {
                anyActive = true;
                break;
            }
        }
        if (anyActive) {
            launcher.requestSpinUp();
        } else {
            if (launcher.getRequestedSpinMode() != LauncherSubsystem.SpinMode.OFF) {
                launcher.requestStandbySpin();
            }
        }
    }

    private void updateArtifactState() {
        int count = 0;
        for (LauncherLane lane : LauncherLane.values()) {
            if (isArtifactPresent(lane)) {
                count++;
            }
        }
        artifactState = ArtifactState.fromCount(count);
    }

    protected boolean isArtifactPresent(LauncherLane lane) {
        ArtifactColor color = laneColors.getOrDefault(lane, ArtifactColor.NONE);
        if (color != ArtifactColor.NONE && color != ArtifactColor.UNKNOWN) {
            return true;
        }
        if (intake == null) {
            return false;
        }
        IntakeSubsystem.LaneSample sample = intake.getLaneSample(lane);
        if (sample == null || !sample.sensorPresent) {
            return false;
        }
        if (sample.withinDistance) {
            return true;
        }
        ArtifactColor hsvColor = sample.hsvColor;
        return hsvColor != ArtifactColor.NONE && hsvColor != ArtifactColor.UNKNOWN;
    }

    private IntakeSubsystem.IntakeMode computeAutomationIntakeMode() {
        if (!intakeAutomationEnabled) {
            return intake != null ? intake.getMode() : IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
        }
        if (artifactState == ArtifactState.THREE) {
            return IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
        }
        return IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
    }

    private void applyIntakeMode() {
        if (intake == null) {
            return;
        }
        IntakeSubsystem.IntakeMode desired = getRequestedIntakeMode();
        if (desired == null) {
            desired = IntakeSubsystem.IntakeMode.STOPPED;
        }
        if (appliedIntakeMode != desired) {
            intake.setMode(desired);
            appliedIntakeMode = desired;
        }
    }


    public void publishLaneTelemetry(Telemetry dsTelemetry,
                                     TelemetryManager panelsTelemetry) {
        if (dsTelemetry == null && panelsTelemetry == null) {
            return;
        }

        for (LauncherLane lane : LauncherLane.values()) {
            IntakeSubsystem.LaneSample sample = intake.getLaneSample(lane);
            ArtifactColor artifact = getLaneColor(lane);

            double distanceValue = sample.distanceAvailable && !Double.isNaN(sample.distanceCm)
                    ? Math.round(sample.distanceCm * 10.0) / 10.0
                    : Double.NaN;
            String distanceText;
            if (!sample.sensorPresent) {
                distanceText = "no sensor";
            } else if (!sample.distanceAvailable) {
                distanceText = "--";
            } else if (Double.isNaN(distanceValue)) {
                distanceText = "n/a";
            } else {
                distanceText = String.format("%.1fcm", distanceValue);
            }

            double hueValue = sample.sensorPresent ? Math.round(sample.hue) : Double.NaN;
            String hueText = sample.sensorPresent ? String.format("%.0f°", hueValue) : "--";

            String laneLabel = lane.name().substring(0, 1) + lane.name().substring(1).toLowerCase();
            if (dsTelemetry != null) {
                dsTelemetry.addData(
                        laneLabel,
                        "dist=%s | hue=%s %s | artifact=%s",
                        distanceText,
                        hueText,
                        sample.hsvColor.name(),
                        artifact.name()
                );
            }

            if (panelsTelemetry != null) {
                String prefix = "teleop/" + lane.name().toLowerCase();
                panelsTelemetry.debug(prefix + "/distance_cm", Double.isNaN(distanceValue) ? 0.0 : distanceValue);
                panelsTelemetry.debug(prefix + "/hue_deg", Double.isNaN(hueValue) ? 0.0 : hueValue);
                panelsTelemetry.debug(prefix + "/hue_color", sample.hsvColor.name());
                panelsTelemetry.debug(prefix + "/artifact", artifact.name());
            }
        }
    }

    public void setManualSpinOverride(boolean enabled) {
        if (manualSpinOverride == enabled) {
            return;
        }
        manualSpinOverride = enabled;
        if (!manualSpinOverride) {
            recomputeSpinMode();
        }
    }

    /**
     * Creates a ManualSpinController that manages manual spin override for this coordinator.
     * Multiple sources can hold the spin active; it only releases when all sources exit.
     *
     * @return A ManualSpinController instance that controls this coordinator's manual spin override
     */
    public org.firstinspires.ftc.teamcode.commands.LauncherCommands.ManualSpinController createManualSpinController() {
        return new org.firstinspires.ftc.teamcode.commands.LauncherCommands.ManualSpinController() {
            private int activeSources = 0;

            @Override
            public void enterManualSpin() {
                if (activeSources++ == 0) {
                    setManualSpinOverride(true);
                }
            }

            @Override
            public void exitManualSpin() {
                if (activeSources <= 0) {
                    return;
                }
                if (--activeSources == 0) {
                    setManualSpinOverride(false);
                }
            }
        };
    }


    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    private void refreshLightingRegistration() {
        if (lighting == null) {
            return;
        }
        if (!lightingRegistered) {
            intake.addLaneColorListener(lighting);
            lightingRegistered = true;
        }
    }

    private void removeLightingListener() {
        if (lighting != null && lightingRegistered) {
            intake.removeLaneColorListener(lighting);
            lightingRegistered = false;
        }
    }

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    @AutoLogOutput
    public boolean getLightingRegistered() {
        return lightingRegistered;
    }

    @AutoLogOutput
    public String getArtifactStateString() {
        return artifactState.name();
    }

    @AutoLogOutput
    public int getArtifactCountLogged() {
        return artifactState.count();
    }

    @AutoLogOutput
    public boolean getIntakeAutomationEnabled() {
        return intakeAutomationEnabled;
    }

    @AutoLogOutput
    public boolean getIntakeOverrideActive() {
        return manualIntakeOverride != null;
    }

    @AutoLogOutput
    public String getIntakeRequestedMode() {
        IntakeSubsystem.IntakeMode requestedMode = getRequestedIntakeMode();
        return requestedMode == null
                ? IntakeSubsystem.IntakeMode.PASSIVE_REVERSE.name()
                : requestedMode.name();
    }

    @AutoLogOutput
    public String getIntakeAppliedModeString() {
        IntakeSubsystem.IntakeMode appliedMode = getAppliedIntakeMode();
        return appliedMode == null
                ? IntakeSubsystem.IntakeMode.PASSIVE_REVERSE.name()
                : appliedMode.name();
    }

    @AutoLogOutput
    public boolean getAnyActiveLanes() {
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = laneColors.getOrDefault(lane, ArtifactColor.NONE);
            if (color != ArtifactColor.NONE && color != ArtifactColor.UNKNOWN) {
                return true;
            }
        }
        return false;
    }

    @AutoLogOutput
    public String getLeftColor() {
        return laneColors.getOrDefault(LauncherLane.LEFT, ArtifactColor.NONE).name();
    }

    @AutoLogOutput
    public String getCenterColor() {
        return laneColors.getOrDefault(LauncherLane.CENTER, ArtifactColor.NONE).name();
    }

    @AutoLogOutput
    public String getRightColor() {
        return laneColors.getOrDefault(LauncherLane.RIGHT, ArtifactColor.NONE).name();
    }

    @AutoLogOutput
    public String getLauncherState() {
        return launcher.getState().name();
    }

    @AutoLogOutput
    public String getLauncherSpinMode() {
        return launcher.getEffectiveSpinMode().name();
    }

    @AutoLogOutput
    public int getLauncherQueuedShots() {
        return launcher.getQueuedShots();
    }

}
