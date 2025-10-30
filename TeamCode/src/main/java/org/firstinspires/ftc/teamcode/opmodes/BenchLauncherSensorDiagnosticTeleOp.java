package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
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
    private VisionSubsystem vision;
    private GamepadEx driverPad;
    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.BLUE;

    @Override
    public void onInit() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        launcherCoordinator = new LauncherCoordinator(shooter, intake, lighting);
        vision = new VisionSubsystem(hardwareMap);

        driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, Alliance.BLUE);

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(shooter),
                new SubsystemComponent(intake),
                new SubsystemComponent(lighting),
                new SubsystemComponent(launcherCoordinator),
                new SubsystemComponent(vision)
        );

        shooter.initialize();
        shooter.homeAllFeeders();
        intake.initialize();
        lighting.initialize();
        vision.initialize();
        launcherCoordinator.enableAutoSpin(true);
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
        if (vision != null) {
            vision.stop();
        }
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
        telemetry.addLine("X/A/B: kick left/centre/right. Y: burst detected lanes.");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Shooter state", shooter.getState());
        telemetry.addData("Spin mode", shooter.getEffectiveSpinMode());
        telemetry.addData("Queued shots", shooter.getQueuedShots());
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = launcherCoordinator.getLaneColor(lane);
            double feederPos = shooter.getFeederPosition(lane);
            double rpm = shooter.getCurrentRpm(lane);
            boolean ready = shooter.isLaneReady(lane);
            telemetry.addData(lane.name(),
                    "artifact=%s feeder=%.2f rpm=%.0f ready=%s",
                    color.name(),
                    feederPos,
                    rpm,
                    ready);
        }
        telemetry.update();
    }
}
