package org.firstinspires.ftc.teamcode.opmodes.teleop;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PsiKitAdapter;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple Mecanum Drive", group = "TeleOp")
public class SimpleMecanumDrive extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;

    @Override
    public void runOpMode() {
        // Map hardware
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // Reverse left side motors
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Get joystick inputs
            double y = -gamepad1.left_stick_y;  // Forward/Backward
            double x = gamepad1.left_stick_x;   // Left/Right (Strafe)
            double rx = gamepad1.right_stick_x; // Rotation




            // Calculate motor powers
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            // Apply powers to motors
            lf.setPower(flPower);
            lb.setPower(blPower);
            rf.setPower(frPower);
            rb.setPower(brPower);

            telemetry.addData("FL", flPower);
            telemetry.addData("FR", frPower);
            telemetry.addData("BL", blPower);
            telemetry.addData("BR", brPower);
            telemetry.update();
        }
    }
}

