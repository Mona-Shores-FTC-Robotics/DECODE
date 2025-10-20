package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.TeleopBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PsiKitAdapter;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;

@TeleOp(name = "Robot Centric TeleOp")
public class RobotCentricTeleop extends NextFTCOpMode {

    private Robot robot;
    private DcMotorEx lf;
    private DcMotorEx lr;
    private DcMotorEx rf;
    private DcMotorEx rr;
    private TeleopBindings driveBindings;
    private RobotTeleopHelper helper;
    private TelemetryPublisher pub;
    private PsiKitAdapter logger;

    @Override
    public void onInit() {
        // Hardware + app objects
        robot = new Robot(hardwareMap);
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rb");


        GamepadEx driver   = new GamepadEx(() -> gamepad1);
        GamepadEx operator = new GamepadEx(() -> gamepad2);
        driveBindings = new TeleopBindings(driver, robot.flywheel);
        helper = new RobotTeleopHelper(driver, operator);

        logger = new PsiKitAdapter();
        logger.startSession();
        pub = new TelemetryPublisher(logger);

        // Configure drive BEFORE components so initialize() sees these settings.
        robot.drive.setDefaultMode(DriveSubsystem.DriveMode.NORMAL); // default mode
        robot.drive.setRobotCentric(true);
        robot.drive.setAutoHeadingEnabled(false); // disable auto-heading for manual rotation control
        robot.drive.setPose(0.0, 0.0, 0.0);

        // Register components so NextFTC handles initialize()/periodic()/stop()
        addComponents(
                new SubsystemComponent(robot.flywheel, robot.drive));
        // NOTE: Do NOT call robot.drive.initialize() here; SubsystemComponent will handle it.
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        TeleopBindings.DriveRequest request = driveBindings.sampleDriveRequest();

        double lx = -request.fieldX;           // +left
        double ly = request.fieldY;            // +forward
        double rx = -request.rotation;         // +CCW

        robot.drive.driveWithModeHolds(lx, ly, rx, request.precisionMode, request.normalModeHold);

        // Telemetry
        double currentRPM = robot.flywheel.getRpm();

        pub.publishDrive(
                robot.drive,
                request.fieldX, request.fieldY, request.rotation,
                request.precisionMode
        );

        pub.publishFlywheel(
                robot.flywheel.getTargetRpm(),
                currentRPM,
                robot.flywheel.getLastPower(),
                robot.flywheel.getTargetRpm() - currentRPM
        );

        helper.publishHelp(telemetry);

        Pose2D pose = robot.drive.getPose();

        double x    = pose.getX(DistanceUnit.INCH);                          // in pose.getUnit()
        double y    = pose.getY(DistanceUnit.INCH);                          // in pose.getUnit()
        double hDeg = pose.getHeading(AngleUnit.DEGREES);   // convert from radians if needed

        telemetry.addData("pose", "(%.1f, %.1f) %s  h=%.1f°",
                x, y, "DEGREES", hDeg);

        telemetry.addData("flywheel", "%.0f / %.0f rpm",
                currentRPM, robot.flywheel.getTargetRpm());

        telemetry.addData("motor power", "(%.1f, %.1f, %.1f, %.1f)", lr.getPower(), rr.getPower(), lf.getPower(), rf.getPower());
        telemetry.update();
    }

    @Override
    public void onStop() {
        // SubsystemComponent will call stop; still fine to be explicit for hardware safety
        robot.flywheel.stop();
        robot.drive.shutdown();
        driveBindings.reset();
        helper.reset();
        logger.stopSession();
    }
}
