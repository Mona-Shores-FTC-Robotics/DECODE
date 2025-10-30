package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AdvLogger;

@TeleOp(name = "AdvantageScopeLite Test", group = "Diagnostics")
public class AdvScopeLiteTest extends OpMode {

    private long lastEventTime;

    @Override
    public void init() {
        AdvLogger.init(hardwareMap.appContext);
        telemetry.addLine("Initialized AdvantageScope Lite");
        telemetry.update();
    }

    @Override
    public void start() {
        lastEventTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        double runtimeSec = getRuntime();
        double simulatedRPM = 5000 + 300 * Math.sin(runtimeSec);

        AdvLogger.record("Flywheel/RPM", simulatedRPM);
        AdvLogger.record("Robot/RuntimeSec", runtimeSec);
        AdvLogger.recordBoolean("Robot/Enabled", true);

        if (System.currentTimeMillis() - lastEventTime > 5000) {
            AdvLogger.recordEvent("Heartbeat");
            lastEventTime = System.currentTimeMillis();
        }

        telemetry.addData("Runtime (s)", runtimeSec);
        telemetry.addData("Sim RPM", simulatedRPM);
        telemetry.update();
    }

    @Override
    public void stop() {
        AdvLogger.recordEvent("TestOpMode/Stop");
        AdvLogger.close();
    }
}
