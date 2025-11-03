package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

@TeleOp(name = "Shooter Debug (Right Lane + RPM Adjust)", group = "Debug")
public class ShooterDebugTeleOp extends OpMode {

    private ShooterSubsystem shooter;
    private boolean lastB = false;
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean rightActive = false;

    // Start around a normal test RPM
    private double targetRpm = 4200.0;
    private static final double RPM_STEP = 100.0; // amount to bump up/down each press

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.initialize();
        shooter.setDebugOverrideEnabled(true);

        telemetry.addLine("Shooter debug ready. Press B to toggle, bumpers to adjust RPM.");
    }

    @Override
    public void loop() {
        shooter.periodic(); // always update control logic

        boolean bNow = gamepad1.b;
        boolean rbNow = gamepad1.right_bumper;
        boolean lbNow = gamepad1.left_bumper;

        // --- B: Toggle right flywheel on/off ---
        if (bNow && !lastB) {
            rightActive = !rightActive;
            if (rightActive) {
                shooter.debugSetLaneTargetRpm(LauncherLane.RIGHT, targetRpm);
            } else {
                shooter.debugSetLaneTargetRpm(LauncherLane.RIGHT, 0.0);
            }
        }
        lastB = bNow;

        // --- Right bumper: increase target RPM ---
        if (rbNow && !lastRB) {
            targetRpm += RPM_STEP;
            if (rightActive) {
                shooter.debugSetLaneTargetRpm(LauncherLane.RIGHT, targetRpm);
            }
        }
        lastRB = rbNow;

        // --- Left bumper: decrease target RPM ---
        if (lbNow && !lastLB) {
            targetRpm -= RPM_STEP;
            if (targetRpm < 0) targetRpm = 0;
            if (rightActive) {
                shooter.debugSetLaneTargetRpm(LauncherLane.RIGHT, targetRpm);
            }
        }
        lastLB = lbNow;

        // --- Telemetry ---
        ShooterSubsystem.Inputs inputs = new ShooterSubsystem.Inputs();
        shooter.populateInputs(inputs);

        telemetry.addData("Right active", rightActive);
        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Right current RPM", inputs.rightCurrentRpm);
        telemetry.addData("Right applied power", inputs.rightPower);
        telemetry.addData("Ready?", inputs.rightReady);
        telemetry.update();
    }
}
