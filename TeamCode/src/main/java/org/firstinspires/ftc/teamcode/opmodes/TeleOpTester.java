package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;
import org.firstinspires.ftc.teamcode.util.RobotState;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleopTester", group = "Field Centric TeleOp")
public class TeleOpTester extends NextFTCOpMode {

    private Robot robot;
    private DriverBindings driverBindings;
    private OperatorBindings operatorBindings;
    private TelemetryPublisher telemetryPublisher;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        telemetryPublisher = robot.telemetry.publisher();

        robot.initialize();

        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        GamepadEx operatorPad = new GamepadEx(() -> gamepad2);
        driverBindings = new DriverBindings(driverPad);
        operatorBindings = new OperatorBindings(operatorPad, robot);
        robot.drive.setRobotCentric(false);

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.shooter),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting)
        );

        robot.lighting.clearLaneColors();
        robot.lighting.setAlliance(Alliance.BLUE);
    }

    @Override
    public void onStartButtonPressed() {
        robot.logger.startSession(hardwareMap.appContext, getClass().getSimpleName(), RobotState.getAlliance(), "TeleOpTester");
        robot.lighting.indicateIdle();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();

        robot.drive.driveScaled(request.fieldX, request.fieldY, request.rotation, request.slowMode);

        PanelsBridge.drawFollowerDebug(robot.drive.getFollower());
        if (robot.telemetry.panelsTelemetry() != null) {
            robot.telemetry.panelsTelemetry().debug("Mode", "TeleOpTester");
            robot.telemetry.panelsTelemetry().debug("DriveMode", robot.drive.getDriveMode());
        }

        double currentRpm = robot.shooter.getCurrentRpm();
        telemetryPublisher.publishDrive(robot.drive, request.fieldX, request.fieldY, request.rotation, request.slowMode);
        telemetryPublisher.publishShooter(
                robot.shooter.getTargetRpm(),
                currentRpm,
                robot.shooter.getLastPower(),
                robot.shooter.getTargetRpm() - currentRpm
        );

        robot.logger.logNumber("Drive", "LX", request.fieldX);
        robot.logger.logNumber("Drive", "LY", request.fieldY);
        robot.logger.logNumber("Drive", "RX", request.rotation);
        robot.logger.logNumber("Shooter", "RPM", currentRpm);
        robot.logger.sampleSources();

        Pose2D pose = robot.drive.getPose();
        telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Shooter RPM", "%.0f / %.0f  %s",
                currentRpm,
                robot.shooter.getTargetRpm(),
                robot.shooter.atTarget() ? "ready" : "spooling");
        telemetry.addData("Drive Mode", robot.drive.getDriveMode());
        telemetry.update();

        robot.telemetry.updateDriverStation(telemetry);
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
        robot.lighting.indicateIdle();
        robot.logger.stopSession();
    }
}
