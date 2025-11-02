package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Sensor-driven bench diagnostic that mirrors the intended match behaviour without driving.
 * Colour sensors light the indicators, the shooter spins when artefacts are present, and
 * X/A/B/Y trigger per-lane or burst feeds through the shooter subsystem.
 */
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
        name = "Bench Launcher Sensor Diagnostic",
        group = "Diagnostics"
)
public class BenchLauncherSensorDiagnosticTeleOp extends NextFTCOpMode {

    @Configurable
    public static class BenchConfig {
        public static double burstSpacingMs = 250.0;
    }

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LightingSubsystem lighting;
    private LauncherCoordinator launcherCoordinator;
    private VisionSubsystemLimelight vision;
    private GamepadEx driverPad;
    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.BLUE;
    private TelemetryManager panelsTelemetry;

    @Override
    public void onInit() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        launcherCoordinator = new LauncherCoordinator(shooter, intake, lighting);
        vision = new VisionSubsystemLimelight(hardwareMap);

        driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, Alliance.BLUE);

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(shooter),
                new SubsystemComponent(intake),
                new SubsystemComponent(lighting),
                new SubsystemComponent(vision)
        );

        shooter.initialize();
        shooter.homeAllFeeders();
        intake.initialize();
        lighting.initialize();
        vision.initialize();
        launcherCoordinator.initialize();
        launcherCoordinator.enableAutoSpin(true);
        panelsTelemetry = PanelsBridge.preparePanels();
        allianceSelector.applySelection(null, lighting);
        selectedAlliance = allianceSelector.getSelectedAlliance();
    }

    @Override
    public void onStartButtonPressed() {
        allianceSelector.lockSelection();
        registerBindings();
        shooter.homeAllFeeders();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        intake.refreshLaneSensors();
        pushTelemetry();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        shooter.abort();
        shooter.homeAllFeeders();
        lighting.disable();
        launcherCoordinator.stop();
        if (vision != null) {
            vision.stop();
        }
        panelsTelemetry = null;
    }

    @Override
    public void onWaitForStart() {
        BindingManager.update();
        allianceSelector.updateFromVision(vision);
        allianceSelector.applySelection(null, lighting);
        selectedAlliance = allianceSelector.getSelectedAlliance();

        telemetry.clear();
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addLine("D-pad Left/Right override, Down returns to auto");
        telemetry.addLine("Press START to begin diagnostic");
        telemetry.update();
    }

    private void registerBindings() {
        driverPad.x().whenBecomesTrue(() -> launcherCoordinator.requestKick(LauncherLane.LEFT));
        driverPad.a().whenBecomesTrue(() -> launcherCoordinator.requestKick(LauncherLane.CENTER));
        driverPad.b().whenBecomesTrue(() -> launcherCoordinator.requestKick(LauncherLane.RIGHT));
        driverPad.y().whenBecomesTrue(() -> launcherCoordinator.requestBurst(BenchConfig.burstSpacingMs));
    }

    private void pushTelemetry() {
        telemetry.clear();
        telemetry.addData("Info", "X/A/B kick lanes | Y burst all");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Shooter", "%s | spin=%s | queued=%d",
                shooter.getState(),
                shooter.getEffectiveSpinMode(),
                shooter.getQueuedShots());

        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor artifact = launcherCoordinator.getLaneColor(lane);
            IntakeSubsystem.LaneSample sample = intake.getLaneSample(lane);

            String laneLabel = toTitleCase(lane.name());
            String distanceText = "--";
            double distanceValue = 0.0;
            if (sample.sensorPresent && sample.distanceAvailable && !Double.isNaN(sample.distanceCm)) {
                distanceValue = Math.round(sample.distanceCm * 10.0) / 10.0;
                distanceText = String.format(Locale.US, "%.1fcm", distanceValue);
            } else if (sample.sensorPresent && sample.distanceAvailable) {
                distanceText = "n/a";
            } else if (!sample.sensorPresent) {
                distanceText = "no sensor";
            }

            double hueValue = sample.sensorPresent ? Math.round(sample.hue) : 0.0;
            String hueText = sample.sensorPresent ? String.format(Locale.US, "%.0f°", hueValue) : "--";
            String hueColor = sample.sensorPresent ? sample.hsvColor.name() : "NONE";
            String artifactColor = artifact.name();

            telemetry.addData(laneLabel,
                    "dist=%s | hue=%s %s | artifact=%s",
                    distanceText,
                    hueText,
                    hueColor,
                    artifactColor);

            if (panelsTelemetry != null) {
                String prefix = "benchLauncher/" + lane.name().toLowerCase(Locale.US);
                panelsTelemetry.debug(prefix + "/distance_cm", distanceValue);
                panelsTelemetry.debug(prefix + "/hue_deg", hueValue);
                panelsTelemetry.debug(prefix + "/hue_color", hueColor);
                panelsTelemetry.debug(prefix + "/artifact", artifactColor);
            }
        }

        if (panelsTelemetry != null) {
            panelsTelemetry.update(telemetry);
        } else {
            telemetry.update();
        }
    }

    private static String toTitleCase(String value) {
        if (value == null || value.isEmpty()) {
            return value;
        }
        String lower = value.toLowerCase(Locale.US);
        return Character.toUpperCase(lower.charAt(0)) + lower.substring(1);
    }
}
