package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.ControlHubIdentifierUtil;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.List;


/**
 * The main TeleOp OpMode — this is where driver control comes to life.
 *
 * FTC OpModes have a fixed lifecycle called by the Driver Station app:
 *   init()      — runs once when INIT is pressed; builds the robot and schedules subsystems
 *   init_loop() — loops while waiting for START; used here for alliance selection
 *   start()     — runs once when START is pressed; activates drive and wires gamepad buttons
 *   loop()      — runs every ~20 ms during the match; the main game loop
 *   stop()      — runs once when STOP is pressed; shuts everything down cleanly
 *
 * The Ivy Scheduler (see init()) runs all subsystem periodic() methods automatically
 * every loop, so you don't need to call them manually in loop().
 */
@TeleOp(name = "Decode TeleOp", group = "TeleOp")
public class DecodeTeleOp extends OpMode {

    public static class EndgameConfig {
        /** Seconds remaining when auto-switch to DECODE mode triggers */
        public double decodeModeSwitchSecondsRemaining = 50.0;
    }

    /** TeleOp duration in seconds (2 minutes) */
    private static final double TELEOP_DURATION_SEC = 120.0;

    public static EndgameConfig endgameConfig = new EndgameConfig();

    private Robot robot;

    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.UNKNOWN;
    private LightingSubsystem.InitController lightingInitController;

    // Previous loop timing (for telemetry reporting)
    private double prevMainLoopMs = 0.0;
    private long telemetryStartNs = 0;
    private long lastLoopStartNs = 0;

    // Endgame mode switch tracking
    private boolean autoSwitchedToDecodeMode = false;

    OperatorBindings operatorBindings = new OperatorBindings(() -> gamepad2);
    DriverBindings driverBindings = new DriverBindings(() -> gamepad1);

    private List<LynxModule> hubs;

    @Override
    public void init() {
        // Cache control + expansion hub bulk reads (was BulkReadComponent's job).
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        robot = new Robot(hardwareMap);
        ControlHubIdentifierUtil.setRobotName(hardwareMap, telemetry);
        robot.attachPedroFollower();
        robot.telemetry.startSession();

        // Apply alliance from previous OpMode (auto) before initializing
        // so lighting subsystem shows correct color from the start.
        // Default to BLUE if no valid alliance from auto (never leave as UNKNOWN)
        Alliance persistedAlliance = RobotState.getAlliance();
        if (persistedAlliance != null && persistedAlliance != Alliance.UNKNOWN) {
            robot.setAlliance(persistedAlliance);
            selectedAlliance = persistedAlliance;
        } else {
            // No valid handoff from auto - default to BLUE so we can always shoot
            robot.setAlliance(Alliance.BLUE);
            selectedAlliance = Alliance.BLUE;
        }
        updateAllianceLighting();
        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.initializeForTeleOp();
        // Wire up drive subsystem for relocalization warning checks
        robot.lighting.setDriveSubsystem(robot.drive);
        // Use persisted alliance if valid, otherwise default to BLUE (not UNKNOWN)
        Alliance selectorDefault = (persistedAlliance != null && persistedAlliance != Alliance.UNKNOWN)
                ? persistedAlliance
                : Alliance.BLUE;
        allianceSelector = new AllianceSelector(gamepad1, selectorDefault);

        // Subsystem periodics are scheduled as Ivy infinite Commands; no NextFTC SubsystemComponent registration.
        // Reset Ivy's process-global Scheduler to clear any leftover state from a prior OpMode.
        com.pedropathing.ivy.Scheduler.reset();
        com.pedropathing.ivy.Scheduler.schedule(
                robot.drive.periodic(),
                robot.launcher.periodic(),
                robot.intake.periodic(),
                robot.lighting.periodic(),
                robot.vision.periodic()
        );
    }

    @Override
    public void init_loop() {
        if (allianceSelector != null) {
            selectedAlliance = allianceSelector.updateDuringInit(robot.vision, robot, robot.lighting);
        }
        updateAllianceLighting();
        telemetry.addData("Alliance", selectedAlliance.displayName());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.addLine();
        if (lightingInitController != null) {
            lightingInitController.updateDuringInit(gamepad1.dpad_up);
        }
        pushInitTelemetry();

    }

    @Override
    public void start() {
        // Initialize launcher mode to THROUGHPUT for TeleOp start
        RobotState.setLauncherMode(LauncherMode.THROUGHPUT);
        RobotState.resetMotifTail(); // Start with fresh motif tail
        autoSwitchedToDecodeMode = false;

        // Enable drive motors and command control now that match is starting
        robot.drive.startTeleopDrive();
        driverBindings.configureTeleopBindings(robot);
        operatorBindings.configureTeleopBindings(robot, gamepad2);

        robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
        robot.lighting.resumeLaneTracking();
        if (lightingInitController != null) {
            lightingInitController.onStart();
        }
        robot.intake.forwardRoller();
        robot.intake.setGatePreventArtifact();
    }

    @Override
    public void loop() {
        // Manual bulk cache clear (was BulkReadComponent's job).
        for (LynxModule hub : hubs) hub.clearBulkCache();
        long loopStartNs = System.nanoTime();
        if (lastLoopStartNs != 0) {
            prevMainLoopMs = (loopStartNs - lastLoopStartNs) / 1_000_000.0;
        } else {
            prevMainLoopMs = 0.0;
        }
        lastLoopStartNs = loopStartNs;
        driverBindings.update();      // tick IvyBindings shim for driver
        operatorBindings.update();    // tick IvyBindings shim for operator
        com.pedropathing.ivy.Scheduler.execute();

        // Auto-switch to DECODE mode when endgame threshold is reached
        checkEndgameModeSwitch();

        long nowMs = System.currentTimeMillis();
        relocalizeTelemetry(nowMs);
        if (lightingInitController != null) {
            lightingInitController.updateDuringMatch();
        }

        telemetryStartNs = loopStartNs;
        publishTelemetry();
    }

    /**
     * Automatically switches to DECODE mode when configured time threshold is reached.
     * Default: 50 seconds remaining in match (70 seconds elapsed).
     */
    private void checkEndgameModeSwitch() {
        if (autoSwitchedToDecodeMode) {
            return; // Already switched, don't check again
        }

        double timeRemaining = Math.max(0.0, TELEOP_DURATION_SEC - getRuntime());
        if (timeRemaining <= endgameConfig.decodeModeSwitchSecondsRemaining) {
            RobotState.setLauncherMode(LauncherMode.DECODE);
            autoSwitchedToDecodeMode = true;

            // Show visual notification on lights (rainbow flash for 2 seconds)
            if (robot != null && robot.lighting != null) {
                robot.lighting.showDecodeModeSwitchNotification();
            }

            // Add telemetry notification
            telemetry.addData("MODE SWITCH", "Endgame - DECODE mode active");
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        // Cancel all scheduled Ivy commands first to prevent them from running during cleanup
        com.pedropathing.ivy.Scheduler.reset();

        // Wrap all subsystem stop calls in try-catch to prevent "expansion hub stopped responding"
        // errors during OpMode shutdown. The individual subsystem stop methods also have
        // their own try-catch blocks, but this provides an extra layer of protection.
        try {
            robot.drive.stop();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        try {
            robot.launcher.abort();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        try {
            robot.intake.stop();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        try {
            robot.intake.deactivateRoller();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        try {
            robot.vision.stop();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        try {
            robot.lighting.stop();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }
        if (allianceSelector != null) {
            allianceSelector.unlockSelection();
        }
    }

    private void publishTelemetry() {
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher,
                robot.intake,
                robot.vision,
                robot.lighting,
                driverBindings.sampleDriveRequest(),
                gamepad1,
                gamepad2,
                selectedAlliance,
                getRuntime(),
                Math.max(0.0, TELEOP_DURATION_SEC - getRuntime()),
                telemetry,
                "TeleOp",
                false,
                null,
                prevMainLoopMs,
                telemetryStartNs
        );
    }

    private void relocalizeTelemetry(long nowMs) {
        if (nowMs - robot.drive.visionRelocalizeStatusMs <= 2000) {
            telemetry.addData("Vision re-localize", robot.drive.visionRelocalizeStatus);
        } else {
            telemetry.addData("Vision re-localize", "Press A to sync with Limelight");
        }
        telemetry.addLine();
    }

    private void updateAllianceLighting() {
        if (robot == null || robot.lighting == null) {
            return;
        }
        robot.lighting.showSolidAlliance(selectedAlliance);
    }

    private void pushInitTelemetry() {
        if (robot == null) {
            return;
        }
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher,
                robot.intake,
                robot.vision,
                robot.lighting,
                null,  // driveRequest (not needed in init)
                null,  // gamepad1 (not needed in init)
                null,  // gamepad2 (not needed in init)
                selectedAlliance,
                getRuntime(),
                TELEOP_DURATION_SEC,  // Full TeleOp duration (match hasn't started yet)
                telemetry,
                "TeleOpInit",
                false,
                null,
                0,      // prevMainLoopMs (no previous loop in init)
                0      // prevTelemetryMs (no previous loop in init)
        );
    }

}
