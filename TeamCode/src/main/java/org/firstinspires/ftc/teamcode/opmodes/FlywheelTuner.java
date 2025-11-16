package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

/**
 * Flywheel Tuner OpMode
 *
 * This OpMode provides easy controls for tuning the launcher flywheels.
 * All flywheel parameters can be adjusted live via FTC Dashboard.
 *
 * CONTROLS:
 *
 * Gamepad1:
 *   DPAD UP      - Spin all flywheels to FULL (launch RPM)
 *   DPAD DOWN    - Spin all flywheels to IDLE
 *   DPAD LEFT    - Stop all flywheels (OFF)
 *   DPAD RIGHT   - Toggle debug override mode
 *
 *   LEFT BUMPER  - Fire LEFT lane
 *   RIGHT BUMPER - Fire RIGHT lane
 *   A            - Fire CENTER lane
 *   B            - Fire all lanes (burst)
 *
 *   X            - Cycle hood position (SHORT -> MID -> LONG)
 *   Y            - Home all feeders
 *
 *   LEFT TRIGGER - Manual feeder control (hold to move to fire position)
 *   RT (hold)    - Show detailed per-lane diagnostics
 *
 * FTC Dashboard Configuration:
 *   Connect to: http://192.168.49.1:8080/dash
 *   Navigate to Config tab
 *   Find LauncherSubsystem to tune all parameters live:
 *     - FlywheelParameters: RPM tolerance, gear ratio, ticks per rev
 *     - BangBangConfig: Power levels and thresholds
 *     - HybridPidConfig: kP, kF, max power
 *     - HoldConfig: Hold power settings
 *     - Timing: Spin-up, recovery, burst timing
 *     - Per-lane configs: Launch RPM, idle RPM, feeder positions, hood positions
 *
 * @author Claude Code
 * @version 1.0
 */
@Configurable
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends OpMode {

    private LauncherSubsystem launcher;
    private TelemetryManager telemetryM;

    private LauncherRange currentHoodPosition = LauncherRange.MID;
    private boolean debugOverrideMode = false;

    @Configurable
    public static class TunerConfig {
        /** Enable verbose telemetry output */
        public static boolean verboseTelemetry = true;

        /** Show control phase information */
        public static boolean showPhaseInfo = true;

        /** Update telemetry every N loops (lower = more frequent, higher load) */
        public static int telemetryUpdateInterval = 2;
    }

    private int loopCount = 0;

    @Override
    public void init() {
        // Initialize telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize launcher subsystem
        launcher = new LauncherSubsystem(hardwareMap);
        launcher.initialize();

        // Register with configurables
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetryM.info("Flywheel Tuner Initialized");
        telemetryM.info("Use FTC Dashboard to adjust parameters live");
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        telemetryM.info("=== FLYWHEEL TUNER ===");
        telemetryM.info("");
        telemetryM.info("Ready to start!");
        telemetryM.info("Configure parameters via FTC Dashboard:");
        telemetryM.info("  http://192.168.49.1:8080/dash");
        telemetryM.info("");
        telemetryM.info("Press START to begin tuning");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        launcher.homeAllFeeders();
        launcher.setAllHoodsForRange(currentHoodPosition);
        telemetryM.info("Tuner started - use gamepad controls");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        loopCount++;

        // Update launcher subsystem
        launcher.periodic();

        // Handle gamepad inputs
        handleGamepadInputs();

        // Update telemetry (throttled)
        if (loopCount % TunerConfig.telemetryUpdateInterval == 0) {
            updateTelemetry();
        }
    }

    private void handleGamepadInputs() {
        // Spin mode controls
        if (gamepad1.dpad_up) {
            launcher.requestSpinUp();
        }

        if (gamepad1.dpad_down) {
            launcher.requestStandbySpin();
        }

        if (gamepad1.dpad_left) {
            launcher.setSpinMode(LauncherSubsystem.SpinMode.OFF);
        }

        // Firing controls (these should be repeatable, so no wasPressed)
        if (gamepad1.left_bumper && gamepad1.left_trigger < 0.5) {
            launcher.queueShot(LauncherLane.LEFT);
        }

        if (gamepad1.right_bumper && gamepad1.left_trigger < 0.5) {
            launcher.queueShot(LauncherLane.RIGHT);
        }

        if (gamepad1.a && gamepad1.left_trigger < 0.5) {
            launcher.queueShot(LauncherLane.CENTER);
        }

        if (gamepad1.b) {
            launcher.queueBurstAll();
        }

        // One-shot controls
        if (gamepad1.dpad_right) {
            debugOverrideMode = !debugOverrideMode;
            launcher.setDebugOverrideEnabled(debugOverrideMode);
        }

        if (gamepad1.x) {
            cycleHoodPosition();
        }

        if (gamepad1.y) {
            launcher.homeAllFeeders();
        }

        // Manual feeder control (for testing feed timing)
        if (gamepad1.left_trigger > 0.5) {
            if (gamepad1.left_bumper) {
                launcher.moveFeederToFire(LauncherLane.LEFT);
            } else if (gamepad1.right_bumper) {
                launcher.moveFeederToFire(LauncherLane.RIGHT);
            } else if (gamepad1.a) {
                launcher.moveFeederToFire(LauncherLane.CENTER);
            }
        }
    }

    private void cycleHoodPosition() {
        switch (currentHoodPosition) {
            case SHORT:
                currentHoodPosition = LauncherRange.MID;
                break;
            case MID:
                currentHoodPosition = LauncherRange.LONG;
                break;
            case LONG:
                currentHoodPosition = LauncherRange.SHORT;
                break;
        }
        launcher.setAllHoodsForRange(currentHoodPosition);
        telemetryM.info("Hood Position: " + currentHoodPosition);
    }

    private void updateTelemetry() {
        telemetryM.clear();

        // Header
        telemetryM.info("=== FLYWHEEL TUNER ===");
        telemetryM.info("Control Mode: " + LauncherSubsystem.getFlywheelControlMode());
        telemetryM.info("State: " + launcher.getState() + " (" + String.format("%.1f", launcher.getStateElapsedSeconds()) + "s)");
        telemetryM.info("");

        // Quick status
        telemetryM.info("--- Quick Status ---");
        telemetryM.info("Spin Mode: " + launcher.getRequestedSpinMode() + " -> " + launcher.getEffectiveSpinMode());
        telemetryM.info("Queue: " + launcher.getQueuedShots() + " shots | Busy: " + launcher.isBusy());
        telemetryM.info("Hood: " + currentHoodPosition);
        telemetryM.info("Debug Override: " + (debugOverrideMode ? "ON" : "OFF"));
        telemetryM.info("");

        // Average metrics
        telemetryM.info("--- Average Metrics ---");
        telemetryM.info(String.format("RPM:   %.0f / %.0f", launcher.getCurrentRpm(), launcher.getTargetRpm()));
        telemetryM.info(String.format("Power: %.3f", launcher.getLastPower()));
        telemetryM.info(String.format("At Target: %s", launcher.atTarget() ? "YES" : "NO"));
        telemetryM.info("");

        // Detailed per-lane info (when RT held)
        if (gamepad1.right_trigger > 0.5 || TunerConfig.verboseTelemetry) {
            displayLaneDetails();
        } else {
            telemetryM.info("Hold RT for detailed per-lane info");
            telemetryM.info("");
        }

        // Controls reminder
        telemetryM.info("--- Controls ---");
        telemetryM.info("DPAD: UP=Full, DOWN=Idle, LEFT=Off, RIGHT=Debug");
        telemetryM.info("Fire: LB=Left, RB=Right, A=Center, B=Burst");
        telemetryM.info("X=Cycle Hood | Y=Home | LT=Manual Feed");
        telemetryM.info("");

        // Dashboard reminder
        telemetryM.info("Tune parameters at: http://192.168.49.1:8080/dash");

        telemetryM.update(telemetry);
    }

    private void displayLaneDetails() {
        telemetryM.info("--- LEFT LANE ---");
        displayLaneInfo(LauncherLane.LEFT);
        telemetryM.info("");

        telemetryM.info("--- CENTER LANE ---");
        displayLaneInfo(LauncherLane.CENTER);
        telemetryM.info("");

        telemetryM.info("--- RIGHT LANE ---");
        displayLaneInfo(LauncherLane.RIGHT);
        telemetryM.info("");
    }

    private void displayLaneInfo(LauncherLane lane) {
        double currentRpm = launcher.getCurrentRpm(lane);
        double targetRpm = launcher.getTargetRpm(lane);
        double power = launcher.getLastPower(lane);
        boolean ready = launcher.isLaneReady(lane);
        double feederPos = launcher.getFeederPosition(lane);

        telemetryM.info(String.format("  Current: %.0f RPM", currentRpm));
        telemetryM.info(String.format("  Target:  %.0f RPM (Launch: %.0f, Idle: %.0f)",
            targetRpm, launcher.getLaunchRpm(lane), launcher.getIdleRpm(lane)));
        telemetryM.info(String.format("  Power:   %.3f", power));
        telemetryM.info(String.format("  Error:   %.0f RPM", targetRpm - currentRpm));
        telemetryM.info(String.format("  Ready:   %s", ready ? "YES" : "NO"));

        if (TunerConfig.showPhaseInfo) {
            telemetryM.info(String.format("  Phase:   %s", launcher.getPhaseName(lane)));
        }

        telemetryM.info(String.format("  Feeder:  %.2f", feederPos));
    }

    @Override
    public void stop() {
        // Clean shutdown
        launcher.abort();
        telemetryM.info("Tuner stopped");
        telemetryM.update(telemetry);
    }
}
