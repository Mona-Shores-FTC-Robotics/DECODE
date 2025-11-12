package org.firstinspires.ftc.teamcode.opmodes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Test bench OpMode for tuning flywheel bang-bang control.
 *
 * Controls:
 * - Gamepad1 A: Spin up to launch RPM
 * - Gamepad1 B: Spin to idle RPM
 * - Gamepad1 X: Stop
 * - Gamepad1 Y: Fire (simulate shot)
 * - Gamepad1 DPad Up/Down: Adjust target RPM by 100
 *
 * Hardware needed:
 * - One motor configured as "launcher_left"
 *
 * Dashboard:
 * - Connect to http://192.168.49.1:8080/dash
 * - View Config tab to tune parameters live
 * - View Telemetry tab to see RPM graphs
 */
@TeleOp(name = "Flywheel Test Bench", group = "Examples")
public class FlywheelTestBench extends LinearOpMode {

    private LauncherSubsystem launcher;
    private double customTargetRpm = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing flywheel test bench...");
        telemetry.update();

        launcher = new LauncherSubsystem(hardwareMap);
        launcher.initialize();

        telemetry.addLine("Ready!");
        telemetry.addLine();
        telemetry.addLine("A: Spin to launch RPM");
        telemetry.addLine("B: Spin to idle RPM");
        telemetry.addLine("X: Stop");
        telemetry.addLine("Y: Queue shot (test recovery)");
        telemetry.addLine("DPad Up/Down: Adjust target RPM");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle controls
            if (gamepad1.a) {
                launcher.requestSpinUp();
                customTargetRpm = 0.0;
            } else if (gamepad1.b) {
                launcher.requestStandbySpin();
                customTargetRpm = 0.0;
            } else if (gamepad1.x) {
                launcher.setSpinMode(LauncherSubsystem.SpinMode.OFF);
                customTargetRpm = 0.0;
            }

            if (gamepad1.y) {
                launcher.queueShot(LauncherLane.LEFT);
            }

            // Adjust target RPM with DPad
            if (gamepad1.dpad_up) {
                customTargetRpm = Math.max(0, launcher.getLaunchRpm(LauncherLane.LEFT)) + 100;
                launcher.setLaunchRpm(LauncherLane.LEFT, customTargetRpm);
                launcher.requestSpinUp();
                sleep(200);
            } else if (gamepad1.dpad_down) {
                customTargetRpm = Math.max(0, launcher.getLaunchRpm(LauncherLane.LEFT)) - 100;
                launcher.setLaunchRpm(LauncherLane.LEFT, customTargetRpm);
                launcher.requestSpinUp();
                sleep(200);
            }

            // Update subsystem
            launcher.periodic();

            // Populate and display telemetry
            LauncherSubsystem.Inputs inputs = new LauncherSubsystem.Inputs();
            launcher.populateInputs(inputs);

            telemetry.addData("State", inputs.state);
            telemetry.addData("Control Mode", inputs.controlMode);
            telemetry.addData("Phase", launcher.getPhaseName(LauncherLane.LEFT));
            telemetry.addLine();

            telemetry.addData("Target RPM", "%.0f", inputs.leftTargetRpm);
            telemetry.addData("Current RPM", "%.0f", inputs.leftCurrentRpm);
            telemetry.addData("Error", "%.0f RPM", inputs.leftTargetRpm - inputs.leftCurrentRpm);
            telemetry.addData("Power", "%.3f", inputs.leftPower);
            telemetry.addLine();

            telemetry.addData("At Target?", inputs.leftReady);
            telemetry.addData("Busy?", inputs.busy);
            telemetry.addData("Queued Shots", inputs.queuedShots);
            telemetry.addLine();

            // Show bang-bang config
            telemetry.addData("High Power", "%.2f", LauncherSubsystem.BangBangConfig.highPower);
            telemetry.addData("Low Power", "%.2f", LauncherSubsystem.BangBangConfig.lowPower);
            telemetry.addData("Exit Bang Threshold", "%.0f RPM", LauncherSubsystem.BangBangConfig.exitBangThresholdRpm);
            telemetry.addData("Bang Deadband", "%.0f RPM", LauncherSubsystem.BangBangConfig.bangDeadbandRpm);
            telemetry.addLine();

            // Phase transition counter
            telemetry.addData("Bang→Hold Counter", launcher.getBangToHoldCount(LauncherLane.LEFT));
            telemetry.addLine();

            telemetry.addData("Loop Time", "%.1f ms", inputs.stateElapsedSec * 1000);

            telemetry.update();
        }

        launcher.abort();
    }
}
