package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogOutput;
import Ori.Coval.Logging.AutoLogManager;
import Ori.Coval.Logging.Logger.KoalaLog;

/**
 * Test OpMode to verify @AutoLog annotation works correctly.
 * This validates that Koala-Log can automatically log fields to FTC Dashboard
 * and AdvantageScope Lite before we convert the actual subsystems.
 *
 * Expected behavior:
 * - Fields appear hierarchically in FTC Dashboard
 * - Data streams to AdvantageScope Lite
 * - @Configurable parameters editable in Dashboard Config tab
 * - No conflicts between @AutoLog and @Configurable
 */
@Configurable
@TeleOp(name="Test AutoLog", group="Test")
public class TestAutoLogOpMode extends LinearOpMode {

    /**
     * Tunable configuration parameters for test subsystem.
     * These should remain separate from logged state.
     */
    @Configurable
    public static class TestConfig {
        /** Maximum motor power */
        public double maxPower = 1.0;

        /** P controller gain */
        public double kP = 0.5;
    }

    // Public static instance for FTC Dashboard Config tab
    public static TestConfig config = new TestConfig();

    /**
     * Test subsystem with @AutoLog annotation.
     * Use @AutoLogOutput on getter methods to expose values for logging.
     */
    @AutoLog
    public static class TestSubsystem {
        // Private state fields
        private double motorPower = 0.0;
        private double encoderPosition = 0.0;
        private boolean isActive = false;
        private String state = "IDLE";

        /**
         * Updates the subsystem state based on power input.
         * @param power Motor power command (-1.0 to 1.0)
         */
        public void update(double power) {
            // Clip power to configured maximum
            power = Math.max(-config.maxPower, Math.min(config.maxPower, power));

            this.motorPower = power;
            this.encoderPosition += power * 10; // Simulate encoder
            this.isActive = Math.abs(power) > 0.01;
            this.state = isActive ? "RUNNING" : "IDLE";
        }

        // ========================================================================
        // AutoLog Output Methods
        // These methods are automatically logged by KoalaLog to WPILOG files
        // and published to FTC Dashboard for AdvantageScope Lite
        // ========================================================================

        @AutoLogOutput
        public double getMotorPower() {
            return motorPower;
        }

        @AutoLogOutput
        public double getEncoderPosition() {
            return encoderPosition;
        }

        @AutoLogOutput
        public boolean isActive() {
            return isActive;
        }

        @AutoLogOutput
        public String getState() {
            return state;
        }
    }

    @Override
    public void runOpMode() {
        // Initialize KoalaLog for WPILOG file logging
        KoalaLog.setup(hardwareMap);

        // Create the auto-logged test subsystem
        TestSubsystemAutoLogged testSubsystem = new TestSubsystemAutoLogged();

        telemetry.addLine("Test AutoLog OpMode");
        telemetry.addLine("====================");
        telemetry.addLine();
        telemetry.addLine("Verification checklist:");
        telemetry.addLine("1. Check FTC Dashboard for logged values");
        telemetry.addLine("2. Check AdvantageScope Lite connection");
        telemetry.addLine("3. Verify Config tab shows TestConfig params");
        telemetry.addLine("4. Try editing kP/maxPower in Config");
        telemetry.addLine();
        telemetry.addLine("Press START to begin sine wave test");
        telemetry.update();

        waitForStart();

        double time = 0.0;
        while (opModeIsActive()) {
            // Generate sine wave for testing
            time += 0.02;
            double power = Math.sin(time);
            testSubsystem.update(power);

            // Display on driver station
            telemetry.addData("Time", "%.2f s", time);
            telemetry.addData("Power", "%.3f", testSubsystem.getMotorPower());
            telemetry.addData("Position", "%.1f ticks", testSubsystem.getEncoderPosition());
            telemetry.addData("State", testSubsystem.getState());
            telemetry.addData("Active", testSubsystem.isActive());
            telemetry.addLine();
            telemetry.addData("Config kP", config.kP);
            telemetry.addData("Config maxPower", config.maxPower);
            telemetry.update();

            // Sample all @AutoLog annotated values for logging
            AutoLogManager.periodic();

            sleep(20);
        }
    }
}
