package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.TeleopBindings;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;

/**
 * Robot-centric variant of the minimal velocity TeleOp.
 * Useful for isolating drivetrain behaviour without heading transforms.
 */
@TeleOp(name = "RobotCentricVelocitySimple", group = "Robot Centric TeleOp")
public class RobotCentricVelocitySimpleTeleOp extends NextFTCOpMode {

    private Robot robot;
    private TeleopBindings bindings;
    private TelemetryPublisher telemetryPublisher;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        telemetryPublisher = new TelemetryPublisher();

        GamepadEx driver = new GamepadEx(() -> gamepad1);
        bindings = new TeleopBindings(driver, robot.flywheel);

        robot.drive.setRobotCentric(true);
        robot.drive.setAutoHeadingEnabled(false);

        addComponents(BindingsComponent.INSTANCE, new SubsystemComponent(robot.drive, robot.flywheel));
        robot.drive.initialize();
        robot.flywheel.initialize();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        TeleopBindings.DriveRequest request = bindings.sampleDriveRequest();

        robot.drive.driveFieldCentricVelocitySimple(request.fieldX, request.fieldY, request.rotation, request.precisionMode);

        double currentRpm = robot.flywheel.getCurrentRpm();
        telemetryPublisher.publishDrive(robot.drive, request.fieldX, request.fieldY, request.rotation, request.precisionMode);
        telemetryPublisher.publishFlywheel(
                robot.flywheel.getTargetRpm(),
                currentRpm,
                robot.flywheel.getLastPower(),
                robot.flywheel.getTargetRpm() - currentRpm
        );

        Pose2D pose = robot.drive.getPose();
        telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Flywheel RPM", "%.0f / %.0f  %s",
                currentRpm,
                robot.flywheel.getTargetRpm(),
                robot.flywheel.atTarget() ? "ready" : "spooling");
        telemetry.addData("Drive Mode", robot.drive.getDriveMode());
        telemetry.update();
    }

    @Override
    public void onStop() {
        robot.drive.stop();
        robot.flywheel.stop();
        bindings.reset();
    }
}
