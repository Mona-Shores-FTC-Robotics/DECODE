package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.telemetry.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotMode;

import java.util.EnumMap;
import java.util.Map;

/**
 * Coordinates the launcher subsystems: intake lane sensing, lighting feedback, and shooter spin/feed.
 * Shared by bench diagnostics, teleop, and autonomous so behaviour remains consistent.
 */
@Configurable
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

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final LightingSubsystem lighting;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private boolean lightingRegistered = false;
    private final Inputs inputs = new Inputs();
    private RobotLogger logger;
    private RobotLogger.Source loggerSource;
    private boolean lastShooterReady = false;
    private double lastPeriodicMs = 0.0;
    private boolean autoSpinEnabled = false;
    private boolean intakeAutomationEnabled = true;
    private ArtifactState artifactState = ArtifactState.EMPTY;
    private IntakeSubsystem.IntakeMode manualIntakeOverride = null;
    private IntakeSubsystem.IntakeMode appliedIntakeMode = null;
    private RobotMode robotMode = RobotMode.DEBUG;

    public LauncherCoordinator(ShooterSubsystem shooter,
                               IntakeSubsystem intake,
                               LightingSubsystem lighting) {
        this.shooter = shooter;
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
        detachLogger();
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

    public void enableAutoSpin(boolean enabled) {
        autoSpinEnabled = enabled;
        recomputeSpinMode();
    }

    public boolean isAutoSpinEnabled() {
        return autoSpinEnabled;
    }

    public void requestKick(LauncherLane lane) {
        ArtifactColor color = getLaneColor(lane);
        if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
            return;
        }
        shooter.queueShot(lane);
    }

    public void requestBurst(double spacingMs) {
        double offset = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = getLaneColor(lane);
            if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
                continue;
            }
            shooter.queueShot(lane, offset);
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
        IntakeSubsystem.IntakeMode override = manualIntakeOverride;
        return override != null ? override : computeAutomationIntakeMode();
    }

    public IntakeSubsystem.IntakeMode getAppliedIntakeMode() {
        if (intake != null) {
            return intake.getResolvedMode();
        }
        return appliedIntakeMode != null ? appliedIntakeMode : IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
    }

    private void recomputeSpinMode() {
        if (!autoSpinEnabled || robotMode != RobotMode.MATCH) {
            shooter.requestStandbySpin();
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
            shooter.requestSpinUp();
        } else {
            shooter.requestStandbySpin();
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

    private boolean isArtifactPresent(LauncherLane lane) {
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
            desired = IntakeSubsystem.IntakeMode.PASSIVE_REVERSE;
        }
        if (appliedIntakeMode != desired) {
            intake.setMode(desired);
            appliedIntakeMode = desired;
        }
    }

    /**
     * Publishes launcher telemetry to the driver station and Panels while also recording
     * shooter-ready events. Keeps lane diagnostics close to the subsystem so OpModes don't have to
     * duplicate formatting logic.
     */
    public boolean logShooterReadyEvent(RobotLogger robotLogger) {
        boolean shooterReady = shooter.atTarget();
        if (robotLogger != null && shooterReady && !lastShooterReady) {
            robotLogger.logEvent("Shooter", "Ready");
        }
        lastShooterReady = shooterReady;
        return shooterReady;
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

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        inputs.autoSpinEnabled = autoSpinEnabled;
        inputs.lightingRegistered = lightingRegistered;
        inputs.artifactState = artifactState.name();
        inputs.artifactCount = artifactState.count();
        inputs.intakeAutomationEnabled = intakeAutomationEnabled;
        inputs.intakeOverrideActive = manualIntakeOverride != null;
        IntakeSubsystem.IntakeMode requestedMode = getRequestedIntakeMode();
        IntakeSubsystem.IntakeMode appliedMode = getAppliedIntakeMode();
        inputs.intakeRequestedMode = requestedMode == null
                ? IntakeSubsystem.IntakeMode.PASSIVE_REVERSE.name()
                : requestedMode.name();
        inputs.intakeAppliedMode = appliedMode == null
                ? IntakeSubsystem.IntakeMode.PASSIVE_REVERSE.name()
                : appliedMode.name();
        boolean anyActive = false;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = laneColors.getOrDefault(lane, ArtifactColor.NONE);
            switch (lane) {
                case LEFT:
                    inputs.leftColor = color.name();
                    break;
                case CENTER:
                    inputs.centerColor = color.name();
                    break;
                case RIGHT:
                default:
                    inputs.rightColor = color.name();
                    break;
            }
            if (color != ArtifactColor.NONE && color != ArtifactColor.UNKNOWN) {
                anyActive = true;
            }
        }
        inputs.anyActiveLanes = anyActive;
        inputs.shooterState = shooter.getState().name();
        inputs.shooterSpinMode = shooter.getEffectiveSpinMode().name();
        inputs.shooterQueuedShots = shooter.getQueuedShots();
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void attachLogger(RobotLogger robotLogger) {
        if (robotLogger == null || loggerSource != null) {
            return;
        }
        logger = robotLogger;
        loggerSource = new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "LauncherCoordinator";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                populateInputs(inputs);
                logger.logInputs("LauncherCoordinator", inputs);
            }
        };
        logger.registerSource(loggerSource);
    }

    public void detachLogger() {
        if (logger != null && loggerSource != null) {
            logger.unregisterSource(loggerSource);
            loggerSource = null;
        }
    }

    public void setRobotMode(RobotMode mode) {
        robotMode = RobotMode.orDefault(mode);
        if (robotMode == RobotMode.DEBUG) {
            enableAutoSpin(false);
            setIntakeAutomationEnabled(false);
        } else {
            setIntakeAutomationEnabled(true);
        }
        refreshLightingRegistration();
    }

    private void refreshLightingRegistration() {
        if (lighting == null) {
            return;
        }
        if (robotMode == RobotMode.MATCH) {
            if (!lightingRegistered) {
                intake.addLaneColorListener(lighting);
                lightingRegistered = true;
            }
        } else {
            removeLightingListener();
        }
    }

    private void removeLightingListener() {
        if (lighting != null && lightingRegistered) {
            intake.removeLaneColorListener(lighting);
            lightingRegistered = false;
        }
    }

    public static final class Inputs {
        public boolean autoSpinEnabled;
        public boolean lightingRegistered;
        public String artifactState = ArtifactState.EMPTY.name();
        public int artifactCount;
        public boolean intakeAutomationEnabled;
        public boolean intakeOverrideActive;
        public String intakeRequestedMode = IntakeSubsystem.IntakeMode.PASSIVE_REVERSE.name();
        public String intakeAppliedMode = IntakeSubsystem.IntakeMode.PASSIVE_REVERSE.name();
        public boolean anyActiveLanes;
        public String leftColor = ArtifactColor.NONE.name();
        public String centerColor = ArtifactColor.NONE.name();
        public String rightColor = ArtifactColor.NONE.name();
        public String shooterState;
        public String shooterSpinMode;
        public int shooterQueuedShots;
    }
}
