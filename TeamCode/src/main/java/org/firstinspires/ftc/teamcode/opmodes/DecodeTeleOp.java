package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

@TeleOp(name = "Decode TeleOp", group = "TeleOp")
public class DecodeTeleOp extends NextFTCOpMode {

    private static final long TELEMETRY_INTERVAL_NS = 50_000_000L; // 50 ms cadence (~20 Hz)
    private static final RobotMode ACTIVE_MODE = RobotMode.MATCH;

    private Robot robot;
    private GamepadEx driverPad;
    private GamepadEx operatorPad;
    private DriverBindings driverBindings;
    private OperatorBindings operatorBindings;
    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.UNKNOWN;

    private long lastTelemetryNs = 0L;
    private String visionRelocalizeStatus = "Press A to re-localize";
    private long visionRelocalizeStatusMs = 0L;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        robot.setRobotMode(ACTIVE_MODE);

        driverPad = new GamepadEx(() -> gamepad1);
        driverBindings = new DriverBindings(driverPad);
        driverBindings.onRelocalizeRequested(this::handleVisionRelocalizeRequest);

        operatorPad = new GamepadEx(() -> gamepad2);
        operatorBindings = new OperatorBindings(operatorPad, robot, robot.launcherCoordinator);

        allianceSelector = new AllianceSelector(driverPad, RobotState.getAlliance());

        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);

        robot.initialize();

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.shooter),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision),
                new SubsystemComponent(robot.launcherCoordinator)
        );
    }

    @Override
    public void onWaitForStart() {
        BindingManager.update();
        syncVisionDuringInit();
        pushInitTelemetry();
        telemetry.clear();
        telemetry.addData("Alliance", selectedAlliance.displayName());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
        robot.intake.activateRoller();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        // Main loop: gather inputs, run drive, then publish telemetry/diagnostics.
        long mainLoopStartNs = System.nanoTime();

        // DriveSubsystem handles auto heading lock when the rotation stick is centered.
        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();
        long driveCallStartNs = System.nanoTime();
        robot.drive.driveTeleOp(request.fieldX, request.fieldY, request.rotation, request.slowMode, request.rampMode);
        double driveCallMs = nanosToMs(System.nanoTime() - driveCallStartNs);

        if (operatorBindings != null) {
            operatorBindings.update(gamepad2.left_trigger, gamepad2.right_trigger);
        }

        boolean telemetrySent = false;
        double telemetryMsThisLoop = 0.0;
        long nowNs = System.nanoTime();
        long nowMs = System.currentTimeMillis();
        if (nowNs - lastTelemetryNs >= TELEMETRY_INTERVAL_NS) {
            long telemetryCallStartNs = nowNs;
            robot.telemetry.publishLoopTelemetry(
                    robot.drive,
                    robot.shooter,
                    robot.vision,
                    request,
                    robot.launcherCoordinator,
                    selectedAlliance,
                    getRuntime(),
                    null,
                    robot.logger,
                    "TeleOp"
            );
            telemetryMsThisLoop = nanosToMs(System.nanoTime() - telemetryCallStartNs);
            lastTelemetryNs = System.nanoTime();
            telemetrySent = true;
        }

        Pose2D pose = robot.drive.getPose();

        long mainloopEndNs = System.nanoTime();
        double loopMs = nanosToMs(mainloopEndNs - mainLoopStartNs);
        LoopTiming loopTiming = buildLoopTiming(loopMs, driveCallMs, telemetryMsThisLoop, telemetrySent);
        boolean showDiagnostics = diagnosticsRequested();
        publishTelemetryDisplay(pose, nowMs, loopTiming, showDiagnostics);
    }

    @Override
    public void onStop() {
        if (operatorBindings != null) {
            operatorBindings.reset();
        }
        BindingManager.reset();
        robot.drive.stop();
        robot.shooter.abort();
        robot.intake.stop();
        robot.intake.deactivateRoller();
        robot.lighting.indicateIdle();
        if (allianceSelector != null) {
            allianceSelector.unlockSelection();
        }
    }

    private void publishTelemetryDisplay(Pose2D pose, long nowMs, LoopTiming timing, boolean showDiagnostics) {
        // Keep the default dashboard concise; only show deep diagnostics when requested.
        telemetry.addData("Alliance", selectedAlliance.displayName());
        addPoseTelemetry(pose);
        addVisionTelemetry(nowMs);
        telemetry.addData("Total Loop", "%.1f ms", timing.totalMs());
        if (showDiagnostics) {
            addLoopDiagnostics(timing);
            addIntakeTelemetry();
        } else {
            telemetry.addData("Main Loop", "%.1f ms", timing.loopMs);
            telemetry.addData("Detail", "Hold RT for subsystem timings");
        }
        telemetry.update();
    }

    private void addPoseTelemetry(Pose2D pose) {
        if (pose != null) {
            telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Pose", "(unavailable)");
        }
    }

    private void addVisionTelemetry(long nowMs) {
        if (nowMs - visionRelocalizeStatusMs <= 2000) {
            telemetry.addData("Vision re-localize", visionRelocalizeStatus);
        } else {
            telemetry.addData("Vision re-localize", "Press A to sync with Limelight");
        }
    }

    private void addLoopDiagnostics(LoopTiming timing) {
        telemetry.addData("Main Loop", "%.1f ms", timing.loopMs);
        telemetry.addData("Control Call", "%.1f ms", timing.driveCallMs);
        telemetry.addData("Drive periodic", "%.1f ms", timing.driveMs);
        telemetry.addData("Intake periodic", "%.1f ms", timing.intakeMs);
        telemetry.addData("Shooter periodic", "%.1f ms", timing.shooterMs);
        telemetry.addData("Lighting periodic", "%.1f ms", timing.lightingMs);
        telemetry.addData("Launcher periodic", "%.1f ms", timing.launcherMs);
        telemetry.addData("Vision periodic", "%.1f ms", timing.visionMs);
        telemetry.addData("Telemetry publish", timing.telemetrySent
                ? String.format(Locale.US, "%.1f ms", timing.telemetryMs)
                : "skipped");
    }

    private void addIntakeTelemetry() {
        telemetry.addData("Intake power", "%.2f", robot.intake.getCurrentPower());
        telemetry.addData("Intake roller",
                robot.intake.isRollerPresent()
                        ? String.format(Locale.US, "%.2f (%s)",
                        robot.intake.getRollerPosition(),
                        robot.intake.isRollerActive() ? "active" : "idle")
                        : "missing");
    }

    private LoopTiming buildLoopTiming(double loopMs, double driveCallMs, double telemetryMsThisLoop, boolean telemetrySent) {
        double drivePeriodicMs = robot.drive.getLastPeriodicMs();
        double intakePeriodicMs = robot.intake.getLastPeriodicMs();
        double shooterPeriodicMs = robot.shooter.getLastPeriodicMs();
        double lightingPeriodicMs = robot.lighting.getLastPeriodicMs();
        double launcherPeriodicMs = robot.launcherCoordinator.getLastPeriodicMs();
        double visionPeriodicMs = robot.vision.getLastPeriodicMs();
        return new LoopTiming(loopMs, driveCallMs, telemetryMsThisLoop, telemetrySent,
                drivePeriodicMs, intakePeriodicMs, shooterPeriodicMs, lightingPeriodicMs, launcherPeriodicMs, visionPeriodicMs);
    }

    private boolean diagnosticsRequested() {
        return gamepad1.right_trigger > 0.5;
    }

    private static double nanosToMs(long nanos) {
        return nanos / 1_000_000.0;
    }

    private void handleVisionRelocalizeRequest() {
        if (robot == null || robot.drive == null || robot.vision == null) {
            return;
        }
        boolean tagVisible = robot.vision.hasValidTag();
        boolean success = robot.drive.forceRelocalizeFromVision();
        if (success) {
            visionRelocalizeStatus = "Pose updated from Limelight";
        } else if (!tagVisible) {
            visionRelocalizeStatus = "No AprilTag visible";
        } else {
            visionRelocalizeStatus = "Vision pose unavailable";
        }
        visionRelocalizeStatusMs = System.currentTimeMillis();
    }

    private void syncVisionDuringInit() {
        if (robot == null || robot.drive == null || robot.vision == null) {
            selectedAlliance = RobotState.getAlliance();
            return;
        }

        if (allianceSelector != null) {
            allianceSelector.updateFromVision(robot.vision);
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        } else {
            selectedAlliance = RobotState.getAlliance();
        }

        if (robot.vision.shouldUpdateOdometry() && robot.drive.forceRelocalizeFromVision()) {
            visionRelocalizeStatus = "Pose synced from Limelight";
            visionRelocalizeStatusMs = System.currentTimeMillis();
        }
    }

    private static final class LoopTiming {
        final double loopMs;
        final double driveCallMs;
        final double telemetryMs;
        final boolean telemetrySent;
        final double driveMs;
        final double intakeMs;
        final double shooterMs;
        final double lightingMs;
        final double launcherMs;
        final double visionMs;

        LoopTiming(double loopMs,
                   double driveCallMs,
                   double telemetryMs,
                   boolean telemetrySent,
                   double driveMs,
                   double intakeMs,
                   double shooterMs,
                   double lightingMs,
                   double launcherMs,
                   double visionMs) {
            this.loopMs = loopMs;
            this.driveCallMs = driveCallMs;
            this.telemetryMs = telemetryMs;
            this.telemetrySent = telemetrySent;
            this.driveMs = driveMs;
            this.intakeMs = intakeMs;
            this.shooterMs = shooterMs;
            this.lightingMs = lightingMs;
            this.launcherMs = launcherMs;
            this.visionMs = visionMs;
        }

        double totalMs() {
            return loopMs + driveMs + intakeMs + shooterMs + lightingMs + launcherMs + visionMs + telemetryMs;
        }
    }



    private void pushInitTelemetry() {
        if (robot == null) {
            return;
        }
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.shooter,
                robot.vision,
                null,
                robot.launcherCoordinator,
                selectedAlliance,
                getRuntime(),
                null,
                null,
                "TeleOpInit"
        );
    }

}
