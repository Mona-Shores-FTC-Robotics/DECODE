package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

import koalog.AutoLogManager;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

@TeleOp(name = "Decode TeleOp", group = "TeleOp")
public class DecodeTeleOp extends NextFTCOpMode {

    private static final long TELEMETRY_INTERVAL_NS = 50_000_000L; // 50 ms cadence (~20 Hz)
    private static final RobotMode ACTIVE_MODE = RobotMode.MATCH;

    private Robot robot;
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
        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.launcherCoordinator.lockIntake();

        robot.initializeForTeleOp();

        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        driverBindings = new DriverBindings(driverPad, robot);
        driverBindings.onRelocalizeRequested(this::handleVisionRelocalizeRequest);

        GamepadEx operatorPad = new GamepadEx(() -> gamepad2);
        operatorBindings = new OperatorBindings(operatorPad, robot);

        allianceSelector = new AllianceSelector(driverPad, RobotState.getAlliance());

        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
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

        // Attempt to correct initial heading from vision if available
        if (robot.vision.hasValidTag()) {
            boolean headingCorrected = robot.drive.correctInitialHeadingFromVision();
            if (headingCorrected) {
                telemetry.clear();
                telemetry.addData("Heading Correction", "Applied from AprilTag");
            }
        }

        telemetry.clear();
        telemetry.addData("Alliance", selectedAlliance.displayName());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        addHeadingDiagnostics();
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        robot.launcherCoordinator.unlockIntake();
        robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
        robot.intake.activateRoller();
    }

    @Override
    public void onUpdate() {
        long mainLoopStartNs = System.nanoTime();

        // Update bindings and command scheduler
        // Commands (DefaultDriveCommand, AimAndDriveCommand, CaptureAndAimCommand) handle drive control
        BindingManager.update();

        // Periodic logging for KoalaLog (WPILOG files)
        AutoLogManager.periodic();

        // Sample driver inputs for telemetry/logging only (not for control)
        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();

        TelemetryTiming telemetryTiming = publishTelemetryIfNeeded(request);
        Pose2D pose = robot.drive.getPose();

        long mainloopEndNs = System.nanoTime();
        double loopMs = nanosToMs(mainloopEndNs - mainLoopStartNs);
        LoopTiming loopTiming = buildLoopTiming(loopMs, telemetryTiming.telemetryMs, telemetryTiming.telemetrySent);
        publishTelemetryDisplay(pose, telemetryTiming.wallClockMs, loopTiming, diagnosticsRequested());
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        robot.drive.stop();
        robot.launcher.abort();
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
        telemetry.addData("Drive periodic", "%.1f ms", timing.driveMs);
        telemetry.addData("Intake periodic", "%.1f ms", timing.intakeMs);
        telemetry.addData("Launcher periodic", "%.1f ms", timing.launcherMs);
        telemetry.addData("Lighting periodic", "%.1f ms", timing.lightingMs);
        telemetry.addData("Launcher periodic", "%.1f ms", timing.launchCoordMs);
        telemetry.addData("Vision periodic", "%.1f ms", timing.visionMs);
        telemetry.addData("Telemetry publish", timing.telemetrySent
                ? String.format(Locale.US, "%.1f ms", timing.telemetryMs)
                : "skipped");
        addHeadingDiagnostics();
    }

    private void addIntakeTelemetry() {
        telemetry.addData("Intake mode",
                "%s (applied %s)%s",
                robot.intake.getMode(),
                robot.intake.getResolvedMode(),
                IntakeSubsystem.ManualModeConfig.enableOverride
                        ? " [override]"
                        : "");
        telemetry.addData("Intake power", "%.2f", robot.intake.getCurrentPower());
        telemetry.addData("Intake roller",
                robot.intake.isRollerPresent()
                        ? String.format(Locale.US, "%.2f (%s)",
                        robot.intake.getRollerPosition(),
                        robot.intake.isRollerActive() ? "active" : "idle")
                        : "missing");
        LauncherCoordinator coordinator = robot.launcherCoordinator;
        if (coordinator != null) {
            telemetry.addData("Artifacts",
                    "%s (%d)",
                    coordinator.getArtifactState(),
                    coordinator.getArtifactCount());
            telemetry.addData("Intake control",
                    coordinator.isManualIntakeOverrideActive()
                            ? String.format(Locale.US, "override -> %s", coordinator.getRequestedIntakeMode())
                            : "automation");
        }
    }

    private void addHeadingDiagnostics() {
        telemetry.addLine("--- HEADING DIAGNOSTICS ---");

        // Current odometry heading
        Pose2D pose = robot.drive.getPose();
        if (pose != null) {
            double odomHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Odom Heading", "%.1f°", odomHeadingDeg);
        }

        // Vision heading
        if (robot.vision.hasValidTag()) {
            telemetry.addData("Vision Tag ID", robot.vision.getCurrentTagId());
            java.util.Optional<com.pedropathing.geometry.Pose> visionPose = robot.vision.getRobotPoseFromTag();
            if (visionPose.isPresent()) {
                double visionHeadingDeg = Math.toDegrees(visionPose.get().getHeading());
                telemetry.addData("Vision Heading", "%.1f°", visionHeadingDeg);

                // Calculate heading error
                if (pose != null) {
                    double odomHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
                    double headingErrorDeg = normalizeAngleDeg(visionHeadingDeg - odomHeadingDeg);
                    telemetry.addData("Heading Error", "%.1f° (Vision - Odom)", headingErrorDeg);
                }
            }
        } else {
            telemetry.addData("Vision", "No AprilTag detected");
        }

        // Fusion diagnostics
        DriveSubsystem.Inputs inputs = new DriveSubsystem.Inputs();
        robot.drive.populateInputs(inputs);

        if (inputs.fusionHasPose) {
            telemetry.addData("Fusion Heading", "%.1f°", inputs.fusionPoseHeadingDeg);
            if (Double.isFinite(inputs.fusionVisionHeadingErrorDeg)) {
                telemetry.addData("Fusion Vision Err", "%.1f°", inputs.fusionVisionHeadingErrorDeg);
            }
            if (Double.isFinite(inputs.fusionDeltaHeadingDeg)) {
                telemetry.addData("Fusion Delta", "%.1f° (Fusion - Odom)", inputs.fusionDeltaHeadingDeg);
            }
            telemetry.addData("Vision Weight", "%.2f", inputs.fusionVisionWeight);
            telemetry.addData("Vision Accepted", inputs.fusionVisionAccepted ? "YES" : "NO");
        }

        // Raw Pinpoint heading if accessible
        double rawPinpointHeading = robot.drive.getRawPinpointHeadingDeg();
        if (Double.isFinite(rawPinpointHeading)) {
            telemetry.addData("Raw Pinpoint IMU", "%.1f°", rawPinpointHeading);
        }
    }

    private static double normalizeAngleDeg(double angleDeg) {
        while (angleDeg > 180.0) {
            angleDeg -= 360.0;
        }
        while (angleDeg < -180.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    private LoopTiming buildLoopTiming(double loopMs, double telemetryMsThisLoop, boolean telemetrySent) {
        double drivePeriodicMs = robot.drive.getLastPeriodicMs();
        double intakePeriodicMs = robot.intake.getLastPeriodicMs();
        double launcherPeriodicMs = robot.launcher.getLastPeriodicMs();
        double lightingPeriodicMs = robot.lighting.getLastPeriodicMs();
        double launcherCoordPeriodicMs = robot.launcherCoordinator.getLastPeriodicMs();
        double visionPeriodicMs = robot.vision.getLastPeriodicMs();
        return new LoopTiming(loopMs, telemetryMsThisLoop, telemetrySent,
                drivePeriodicMs, intakePeriodicMs, launcherPeriodicMs, lightingPeriodicMs, launcherCoordPeriodicMs, visionPeriodicMs);
    }

    private boolean diagnosticsRequested() {
        return gamepad1.right_trigger > 0.5;
    }

    private TelemetryTiming publishTelemetryIfNeeded(DriverBindings.DriveRequest request) {
        long nowNs = System.nanoTime();
        long nowMs = System.currentTimeMillis();
        double telemetryMsThisLoop = 0.0;
        boolean telemetrySent = false;
        if (nowNs - lastTelemetryNs >= TELEMETRY_INTERVAL_NS) {
            long telemetryCallStartNs = nowNs;
            robot.telemetry.publishLoopTelemetry(
                    robot.drive,
                    robot.launcher ,
                    robot.vision,
                    request,
                    robot.launcherCoordinator,
                    selectedAlliance,
                    getRuntime(),
                    null,
                    robot.logger,
                    "TeleOp",
                    false,
                    null
            );
            telemetryMsThisLoop = nanosToMs(System.nanoTime() - telemetryCallStartNs);
            lastTelemetryNs = System.nanoTime();
            telemetrySent = true;
        }
        return new TelemetryTiming(telemetrySent, telemetryMsThisLoop, nowMs);
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
        final double telemetryMs;
        final boolean telemetrySent;
        final double driveMs;
        final double intakeMs;
        final double launcherMs;
        final double lightingMs;
        final double launchCoordMs;
        final double visionMs;

        LoopTiming(double loopMs,
                   double telemetryMs,
                   boolean telemetrySent,
                   double driveMs,
                   double intakeMs,
                   double launcherMs ,
                   double lightingMs,
                   double launchCoordMs ,
                   double visionMs) {
            this.loopMs = loopMs;
            this.telemetryMs = telemetryMs;
            this.telemetrySent = telemetrySent;
            this.driveMs = driveMs;
            this.intakeMs = intakeMs;
            this.launcherMs = launcherMs;
            this.lightingMs = lightingMs;
            this.launchCoordMs = launchCoordMs;
            this.visionMs = visionMs;
        }

        double totalMs() {
            return loopMs + driveMs + intakeMs + launcherMs + lightingMs + launchCoordMs + visionMs + telemetryMs;
        }
    }

    private static final class TelemetryTiming {
        final boolean telemetrySent;
        final double telemetryMs;
        final long wallClockMs;

        TelemetryTiming(boolean telemetrySent,
                        double telemetryMs,
                        long wallClockMs) {
            this.telemetrySent = telemetrySent;
            this.telemetryMs = telemetryMs;
            this.wallClockMs = wallClockMs;
        }
    }



    private void pushInitTelemetry() {
        if (robot == null) {
            return;
        }
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher ,
                robot.vision,
                null,
                robot.launcherCoordinator,
                selectedAlliance,
                getRuntime(),
                null,
                null,
                "TeleOpInit",
                true,
                null
        );
    }

}
