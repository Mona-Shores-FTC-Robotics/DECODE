package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.EnumMap;
import java.util.Map;

/**
 * Coordinates the launcher subsystems: intake lane sensing, lighting feedback, and shooter spin/feed.
 * Shared by bench diagnostics, teleop, and autonomous so behaviour remains consistent.
 */
@Configurable
public class LauncherCoordinator implements Subsystem, IntakeSubsystem.LaneColorListener {

    public static boolean autoSpinEnabled = true;

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
        if (lighting != null) {
            intake.addLaneColorListener(lighting);
            lightingRegistered = true;
        }
        recomputeSpinMode();
    }

    @Override
    public void periodic() {
        long start = System.nanoTime();
        // Nothing required each cycle beyond spin maintenance handled on colour change.
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public void stop() {
        intake.removeLaneColorListener(this);
        if (lighting != null && lightingRegistered) {
            intake.removeLaneColorListener(lighting);
            lightingRegistered = false;
        }
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
        recomputeSpinMode();
    }

    public ArtifactColor getLaneColor(LauncherLane lane) {
        return lane == null ? ArtifactColor.NONE : laneColors.getOrDefault(lane, ArtifactColor.NONE);
    }

    public void enableAutoSpin(boolean enabled) {
        autoSpinEnabled = enabled;
        recomputeSpinMode();
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

    private void recomputeSpinMode() {
        if (!autoSpinEnabled) {
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

    public static final class Inputs {
        public boolean autoSpinEnabled;
        public boolean lightingRegistered;
        public boolean anyActiveLanes;
        public String leftColor = ArtifactColor.NONE.name();
        public String centerColor = ArtifactColor.NONE.name();
        public String rightColor = ArtifactColor.NONE.name();
        public String shooterState;
        public String shooterSpinMode;
        public int shooterQueuedShots;
    }
}
