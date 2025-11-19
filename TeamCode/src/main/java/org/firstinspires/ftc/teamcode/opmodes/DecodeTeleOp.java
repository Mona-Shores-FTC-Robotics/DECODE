package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Decode TeleOp", group = "TeleOp")
@Configurable
public class DecodeTeleOp extends NextFTCOpMode {

    @Configurable
    public static class EndgameConfig {
        /** Seconds remaining when auto-switch to DECODE mode triggers */
        public double decodeModeSwitchSecondsRemaining = 30.0;
    }

    public static EndgameConfig endgameConfig = new EndgameConfig();

    private Robot robot;

    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.UNKNOWN;
    private LightingSubsystem.InitController lightingInitController;

    // Previous loop timing (for telemetry reporting)
    private double prevMainLoopMs = 0.0;
    private long telemetryStartNs = 0;

    // Endgame mode switch tracking
    private boolean autoSwitchedToDecodeMode = false;

    GamepadEx driverPad = new GamepadEx(() -> gamepad1);
    GamepadEx operatorPad = new GamepadEx(() -> gamepad2);

    OperatorBindings operatorBindings = new OperatorBindings(operatorPad);
    DriverBindings driverBindings = new DriverBindings(driverPad);

    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        robot.attachPedroFollower();

        // Apply alliance from previous OpMode (auto) before initializing
        // so lighting subsystem shows correct color from the start
        Alliance persistedAlliance = RobotState.getAlliance();
        if (persistedAlliance != null && persistedAlliance != Alliance.UNKNOWN) {
            robot.setAlliance(persistedAlliance);
            selectedAlliance = persistedAlliance;
        }
        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.initializeForTeleOp();
        allianceSelector = new AllianceSelector(driverPad, RobotState.getAlliance());

        //should we really do this in teleop?
//        lightingInitController = new LightingSubsystem.InitController(robot, allianceSelector);
//        lightingInitController.initialize();

        addComponents(
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision)
        );
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance", selectedAlliance.displayName());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.addLine();
        BindingManager.update();
        if (lightingInitController != null) {
            lightingInitController.updateDuringInit(gamepad1.dpad_up);
        }
        syncVisionDuringInit();
        pushInitTelemetry();

    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();

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
        if (lightingInitController != null) {
            lightingInitController.onStart();
        }
        robot.intake.activateRoller();
        robot.intake.setPrefeedReverse();
    }

    @Override
    public void onUpdate() {
        long mainLoopStartMs = System.currentTimeMillis();
        BindingManager.update();

        // Auto-switch to DECODE mode when endgame threshold is reached
        checkEndgameModeSwitch();

        relocalizeTelemetry(mainLoopStartMs);
        if (lightingInitController != null) {
            lightingInitController.updateDuringMatch();
        }




        telemetryStartNs = System.nanoTime();
        prevMainLoopMs =  TimeUnit.NANOSECONDS.toMillis(mainLoopStartMs - telemetryStartNs);
        publishTelemetry();
    }

    /**
     * Automatically switches to DECODE mode when configured time threshold is reached.
     * Default: 30 seconds remaining in match (120 seconds elapsed).
     */
    private void checkEndgameModeSwitch() {
        if (autoSwitchedToDecodeMode) {
            return; // Already switched, don't check again
        }

        double timeRemaining = Math.max(0.0, 150.0 - getRuntime());
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
    public void onStop() {
        BindingManager.reset();
        robot.drive.stop();
        robot.launcher.abort();
        robot.intake.stop();
        robot.intake.deactivateRoller();
        robot.intake.deactivatePrefeed();
        robot.lighting.indicateIdle();
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
                Math.max(0.0, 150.0 - getRuntime()),
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

    private void syncVisionDuringInit() {
        if (robot == null || robot.drive == null || robot.vision == null) {
            selectedAlliance = RobotState.getAlliance();
            return;
        }
        selectedAlliance = RobotState.getAlliance();

        if (selectedAlliance==null && allianceSelector != null) {
            allianceSelector.updateFromVision(robot.vision);
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
        // DISABLED: Skip initial relocalization to allow Limelight MT2 to stabilize with new heading
        // User can manually relocalize by pressing A after START if needed
        // if (robot.vision.shouldUpdateOdometry() && robot.drive.forceRelocalizeFromVision()) {
        //     robot.drive.visionRelocalizeStatus = "Pose synced from Limelight";
        //     robot.drive.visionRelocalizeStatusMs = System.currentTimeMillis();
        // }
    }

    private static final class LoopTiming {
        final double loopMs;
        final double telemetryMs;
        final boolean telemetrySent;
        final double driveMs;
        final double intakeMs;
        final double launcherMs;
        final double lightingMs;
        final double launchCoordMs;
        final double visionMs;

        LoopTiming(double loopMs,
                   double telemetryMs,
                   boolean telemetrySent,
                   double driveMs,
                   double intakeMs,
                   double launcherMs ,
                   double lightingMs,
                   double launchCoordMs ,
                   double visionMs
                   ) {
            this.loopMs = loopMs;
            this.telemetryMs = telemetryMs;
            this.telemetrySent = telemetrySent;
            this.driveMs = driveMs;
            this.intakeMs = intakeMs;
            this.launcherMs = launcherMs;
            this.lightingMs = lightingMs;
            this.launchCoordMs = launchCoordMs;
            this.visionMs = visionMs;
        }

    }

    private static final class TelemetryTiming {
        final boolean telemetrySent;
        final double telemetryMs;
        final long wallClockMs;

        TelemetryTiming(boolean telemetrySent,
                        double telemetryMs,
                        long wallClockMs) {
            this.telemetrySent = telemetrySent;
            this.telemetryMs = telemetryMs;
            this.wallClockMs = wallClockMs;
        }
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
                150.0,  // Full TeleOp duration (match hasn't started yet)
                telemetry,
                "TeleOpInit",
                false,
                null,
                0,      // prevMainLoopMs (no previous loop in init)
                0      // prevTelemetryMs (no previous loop in init)
        );
    }

}
