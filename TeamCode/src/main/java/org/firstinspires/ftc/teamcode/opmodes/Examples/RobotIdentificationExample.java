package org.firstinspires.ftc.teamcode.opmodes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotIdentifier;

/**
 * Example OpMode demonstrating how to identify which physical robot is running.
 *
 * This is useful when you have multiple robots (e.g., competition robot and practice robot)
 * and need to distinguish between them.
 *
 * Three main methods:
 * 1. Configuration name - easiest way (just use different configs on each robot)
 * 2. Control Hub serial number - unique identifier that never changes
 * 3. Quick check - simple boolean check for a specific robot
 */
@TeleOp(name = "Example: Robot Identification", group = "Examples")
@Disabled
public class RobotIdentificationExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Create a RobotIdentifier from the hardware map
        RobotIdentifier identifier = new RobotIdentifier(hardwareMap);

        // Display all robot identification info
        telemetry.addLine("=== Robot Identity ===");
        telemetry.addData("Config Name", identifier.getConfigName());
        telemetry.addData("Control Hub Serial", identifier.getControlHubSerial());
        telemetry.addData("Expansion Hub Serial", identifier.getExpansionHubSerial());
        telemetry.addLine();

        // Example 1: Check if this is a specific robot by config name
        if (identifier.isRobot("19429")) {
            telemetry.addLine("This is robot 19429!");
        } else {
            telemetry.addLine("This is the other robot");
        }
        telemetry.addLine();

        // Example 2: Use config name to adjust behavior
        String configName = identifier.getConfigName();
        double maxSpeed;
        if (configName.contains("Practice")) {
            maxSpeed = 1.0; // Full speed on practice robot
            telemetry.addLine("Practice robot - full speed enabled");
        } else {
            maxSpeed = 0.8; // Conservative on competition robot
            telemetry.addLine("Competition robot - conservative speed");
        }
        telemetry.addData("Max Speed", maxSpeed);
        telemetry.addLine();

        // Example 3: Use serial number for precise identification
        String controlHubSerial = identifier.getControlHubSerial();
        if (controlHubSerial.equals("DQ3YBV6L")) {
            telemetry.addLine("This is the competition robot (by serial)");
        } else {
            telemetry.addLine("Serial: " + controlHubSerial);
        }
        telemetry.addLine();

        // Example 4: Get a summary string
        telemetry.addLine("=== Summary ===");
        telemetry.addLine(identifier.getSummary());

        telemetry.addLine();
        telemetry.addLine("Press START to continue");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Robot", identifier.toString());
            telemetry.update();
        }
    }
}
