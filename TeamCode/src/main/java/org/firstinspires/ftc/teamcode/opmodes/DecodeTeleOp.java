package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Decode TeleOp", group = "TeleOp")
@Configurable
public class DecodeTeleOp extends NextFTCOpMode {

    private Robot robot;
    private DriverBindings driverBindings;
    private OperatorBindings operatorBindings;
    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.UNKNOWN;

    private String visionRelocalizeStatus = "Press A to re-localize";
    private long visionRelocalizeStatusMs = 0L;

    // Previous loop timing (for telemetry reporting)
    private double prevMainLoopMs = 0.0;
    private long telemetryStartNs = 0;

    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);

        robot.attachPedroFollower();

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

//        // Attempt to correct initial heading from vision if available
//        if (robot.vision.hasValidTag()) {
//            boolean headingCorrected = robot.drive.correctInitialHeadingFromVision();
//            if (headingCorrected) {
//                telemetry.clear();
//                telemetry.addData("Heading Correction", "Applied from AprilTag");
//            }
//        }

        telemetry.clear();
        telemetry.addData("Alliance", selectedAlliance.displayName());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Enable drive motors now that match is starting
        robot.drive.startTeleopDrive();

        robot.launcherCoordinator.unlockIntake();
        robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
        robot.intake.activateRoller();
        robot.intake.activatePrefeed();
    }

    @Override
    public void onUpdate() {
        long mainLoopStartMs = System.nanoTime();
        BindingManager.update();
        relocalizeWithVision(mainLoopStartMs);




        telemetryStartNs = System.nanoTime();
        prevMainLoopMs =  TimeUnit.NANOSECONDS.toMillis(mainLoopStartMs - telemetryStartNs);
        publishTelemetry();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        robot.drive.stop();
        robot.launcher.abort();
        robot.intake.stop();
        robot.intake.deactivateRoller();
        robot.intake.deactivatePrefeed();
        robot.lighting.indicateIdle();
        if (allianceSelector != null) {
            allianceSelector.unlockSelection();
        }
    }

    private void publishTelemetry() {
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher,
                robot.intake,
                robot.vision,
                robot.lighting,
                robot.launcherCoordinator,
                driverBindings.sampleDriveRequest(),
                gamepad1,
                gamepad2,
                selectedAlliance,
                getRuntime(),
                Math.max(0.0, 150.0 - getRuntime()),
                telemetry,
                "TeleOp",
                false,
                null,
                prevMainLoopMs,
                telemetryStartNs
        );

    }


    private void relocalizeWithVision(long nowMs) {
        if (nowMs - visionRelocalizeStatusMs <= 2000) {
            telemetry.addData("Vision re-localize", visionRelocalizeStatus);
        } else {
            telemetry.addData("Vision re-localize", "Press A to sync with Limelight");
        }
        telemetry.addLine();
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
                   double visionMs
                   ) {
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
                robot.launcher,
                robot.intake,
                robot.vision,
                robot.lighting,
                robot.launcherCoordinator,
                null,  // driveRequest (not needed in init)
                null,  // gamepad1 (not needed in init)
                null,  // gamepad2 (not needed in init)
                selectedAlliance,
                getRuntime(),
                150.0,  // Full TeleOp duration (match hasn't started yet)
                telemetry,
                "TeleOpInit",
                false,
                null,
                0,      // prevMainLoopMs (no previous loop in init)
                0      // prevTelemetryMs (no previous loop in init)
        );
    }

}
