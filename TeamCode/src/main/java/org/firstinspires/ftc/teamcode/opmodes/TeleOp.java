package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.TelemetryManager;

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
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FieldCentricTeleOp", group = "Field Centric TeleOp")
public class TeleOp extends NextFTCOpMode {

    private Robot robot;
    private DriverBindings driverBindings;
    private OperatorBindings operatorBindings;
    private TelemetryPublisher telemetryPublisher;
    private TelemetryManager panelsTelemetry;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        telemetryPublisher = new TelemetryPublisher();

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
    }

    public void onStart() {
        panelsTelemetry = PanelsBridge.preparePanels();
        robot.lighting.indicateIdle();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();

        robot.drive.driveScaled(request.fieldX, request.fieldY, request.rotation, request.slowMode);

        PanelsBridge.drawFollowerDebug(robot.drive.getFollower());
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Mode", "TeleOp");
            panelsTelemetry.debug("DriveMode", robot.drive.getDriveMode());
            panelsTelemetry.update(telemetry);
        }

        double currentRpm = robot.shooter.getCurrentRpm();
        telemetryPublisher.publishDrive(robot.drive, request.fieldX, request.fieldY, request.rotation, request.slowMode);
        telemetryPublisher.publishFlywheel(
                robot.shooter.getTargetRpm(),
                currentRpm,
                robot.shooter.getLastPower(),
                robot.shooter.getTargetRpm() - currentRpm
        );

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
        panelsTelemetry = null;
    }
}
