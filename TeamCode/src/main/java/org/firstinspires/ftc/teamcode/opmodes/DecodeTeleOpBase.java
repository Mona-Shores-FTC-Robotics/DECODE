package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorControls;
import org.firstinspires.ftc.teamcode.bindings.OperatorControls.LaneDebugState;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

abstract class DecodeTeleOpBase extends NextFTCOpMode {

    private static final boolean ENABLE_LOGGING = false;
    private static final long TELEMETRY_INTERVAL_NS = 50_000_000L; // 50 ms cadence (~20 Hz)

    protected Robot robot;
    protected GamepadEx driverPad;
    protected GamepadEx operatorPad;
    protected DriverBindings driverBindings;
    protected OperatorControls operatorControls;
    protected LauncherCoordinator launcherCoordinator;
    protected AllianceSelector allianceSelector;
    protected Alliance selectedAlliance = Alliance.UNKNOWN;

    private RobotMode robotMode = RobotMode.DEBUG;
    private long lastTelemetryNs = 0L;

    protected abstract RobotMode getDesiredRobotMode();

    protected abstract OperatorControls buildOperatorControls(GamepadEx operatorPad,
                                                               Robot robot,
                                                               LauncherCoordinator launcherCoordinator,
                                                               RobotMode mode);

    protected void configureForMode(Robot robot, LauncherCoordinator coordinator, RobotMode mode) {
        // optional override
    }

    @Override
    public void onInit() {
        robotMode = RobotMode.orDefault(getDesiredRobotMode());

        robot = new Robot(hardwareMap);
        robot.setRobotMode(robotMode);

        driverPad = new GamepadEx(() -> gamepad1);
        operatorPad = new GamepadEx(() -> gamepad2);
        driverBindings = new DriverBindings(driverPad);
        launcherCoordinator = new LauncherCoordinator(robot.shooter, robot.intake, robot.lighting);
        launcherCoordinator.setRobotMode(robotMode);
        operatorControls = buildOperatorControls(operatorPad, robot, launcherCoordinator, robotMode);
        allianceSelector = new AllianceSelector(driverPad, RobotState.getAlliance());
        robot.drive.setRobotCentric(false);

        robot.initialize();
        launcherCoordinator.initialize();
        if (ENABLE_LOGGING) {
            launcherCoordinator.attachLogger(robot.logger);
        }
        configureForMode(robot, launcherCoordinator, robotMode);
        allianceSelector.applySelection(robot, robot.lighting);
        selectedAlliance = allianceSelector.getSelectedAlliance();

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.shooter),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision)
        );
    }

    @Override
    public void onWaitForStart() {
        BindingManager.update();
        if (allianceSelector != null) {
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        } else {
            selectedAlliance = RobotState.getAlliance();
        }
        telemetry.clear();
        telemetry.addData("Alliance", selectedAlliance.displayName());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        if (ENABLE_LOGGING) {
            robot.logger.startSession(hardwareMap.appContext, getClass().getSimpleName(), RobotState.getAlliance(), "TeleOp");
        }
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        long mainLoopStartNs = System.nanoTime();

        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();
        long driveCallStartNs = System.nanoTime();
        robot.drive.driveScaled(request.fieldX, request.fieldY, request.rotation, request.slowMode);
        double driveCallMs = nanosToMs(System.nanoTime() - driveCallStartNs);

        if (operatorControls != null) {
            operatorControls.update(gamepad2.left_trigger, gamepad2.right_trigger);
        }

        boolean telemetrySent = false;
        double telemetryMsThisLoop = 0.0;
        long nowNs = System.nanoTime();
        if (nowNs - lastTelemetryNs >= TELEMETRY_INTERVAL_NS) {
            long telemetryCallStartNs = nowNs;
            robot.telemetry.publishLoopTelemetry(
                    robot.drive,
                    robot.shooter,
                    robot.vision,
                    request,
                    launcherCoordinator,
                    selectedAlliance,
                    getRuntime(),
                    null,
                    ENABLE_LOGGING ? robot.logger : null,
                    "TeleOp"
            );
            telemetryMsThisLoop = nanosToMs(System.nanoTime() - telemetryCallStartNs);
            lastTelemetryNs = System.nanoTime();
            telemetrySent = true;
        }

        Pose2D pose = robot.drive.getPose();

        telemetry.addData("Alliance", selectedAlliance.displayName());
        if (pose != null) {
            telemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Pose", "(unavailable)");
        }

        long mainloopEndNs = System.nanoTime();
        double loopMs = nanosToMs(mainloopEndNs - mainLoopStartNs);
        boolean showDiag = gamepad1.right_trigger > 0.5;
        double drivePeriodicMs = robot.drive.getLastPeriodicMs();
        double intakePeriodicMs = robot.intake.getLastPeriodicMs();
        double shooterPeriodicMs = robot.shooter.getLastPeriodicMs();
        double lightingPeriodicMs = robot.lighting.getLastPeriodicMs();
        double launcherPeriodicMs = launcherCoordinator != null ? launcherCoordinator.getLastPeriodicMs() : 0.0;
        double visionPeriodicMs = robot.vision.getLastPeriodicMs();
        double totalLoopMs = loopMs
                + drivePeriodicMs
                + intakePeriodicMs
                + shooterPeriodicMs
                + lightingPeriodicMs
                + launcherPeriodicMs
                + visionPeriodicMs
                + telemetryMsThisLoop;

        telemetry.addData("Total Loop", "%.1f ms", totalLoopMs);
        if (showDiag) {
            telemetry.addData("Main Loop", "%.1f ms", loopMs);
            telemetry.addData("Control Call", "%.1f ms", driveCallMs);
            telemetry.addData("Drive periodic", "%.1f ms", drivePeriodicMs);
            telemetry.addData("Intake periodic", "%.1f ms", intakePeriodicMs);
            telemetry.addData("Shooter periodic", "%.1f ms", shooterPeriodicMs);
            telemetry.addData("Lighting periodic", "%.1f ms", lightingPeriodicMs);
            telemetry.addData("Launcher periodic", "%.1f ms", launcherPeriodicMs);
            telemetry.addData("Vision periodic", "%.1f ms", visionPeriodicMs);
            telemetry.addData("Telemetry publish", telemetrySent
                    ? String.format(Locale.US, "%.1f ms", telemetryMsThisLoop)
                    : "skipped");
            telemetry.addData("Intake power", "%.2f", robot.intake.getCurrentPower());
            telemetry.addData("Intake roller",
                    robot.intake.isRollerPresent()
                            ? String.format(Locale.US, "%.2f (%s)",
                            robot.intake.getRollerPosition(),
                            robot.intake.isRollerActive() ? "active" : "idle")
                            : "missing");
            if (operatorControls != null && operatorControls.isShooterDebugMode()) {
                for (LauncherLane lane : LauncherLane.values()) {
                    LaneDebugState state = operatorControls.getLaneDebugState(lane);
                    if (state != null) {
                        String label = lane.name().toLowerCase(Locale.US);
                        String status = state.enabled ? "ON" : "off";
                        telemetry.addData("Shooter " + label, String.format(Locale.US, "%s %.0f rpm", status, state.targetRpm));
                    }
                }
            }
        } else {
            telemetry.addData("Main Loop", "%.1f ms", loopMs);
            telemetry.addData("Detail", "Hold RT for subsystem timings");
        }

        telemetry.update();
        if (ENABLE_LOGGING) {
            robot.logger.sampleSources();
        }
    }

    @Override
    public void onStop() {
        if (operatorControls != null) {
            operatorControls.reset();
        }
        BindingManager.reset();
        robot.drive.stop();
        robot.shooter.abort();
        robot.intake.stop();
        robot.lighting.indicateIdle();
        if (launcherCoordinator != null) {
            launcherCoordinator.stop();
        }
        if (allianceSelector != null) {
            allianceSelector.unlockSelection();
        }
        if (ENABLE_LOGGING) {
            robot.logger.stopSession();
        }
    }

    private static double nanosToMs(long nanos) {
        return nanos / 1_000_000.0;
    }
}
