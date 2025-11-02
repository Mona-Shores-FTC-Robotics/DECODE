package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.bindings.OperatorBindings;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.RobotState;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Decode Teleop", group = "TeleOp")
public class TeleOp extends NextFTCOpMode {

    private Robot robot;
    private GamepadEx driverPad;
    private GamepadEx operatorPad;
    private DriverBindings driverBindings;
    private OperatorBindings operatorBindings;
    private LauncherCoordinator launcherCoordinator;
    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.UNKNOWN;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);

        driverPad = new GamepadEx(() -> gamepad1);
        operatorPad = new GamepadEx(() -> gamepad2);
        driverBindings = new DriverBindings(driverPad);
        launcherCoordinator = new LauncherCoordinator(robot.shooter, robot.intake, robot.lighting);
        operatorBindings = new OperatorBindings(operatorPad, robot, launcherCoordinator);
        allianceSelector = new AllianceSelector(driverPad, RobotState.getAlliance());
        robot.drive.setRobotCentric(false);

        robot.initialize();
        launcherCoordinator.initialize();
        launcherCoordinator.enableAutoSpin(true);
        launcherCoordinator.attachLogger(robot.logger);
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
        robot.logger.startSession(hardwareMap.appContext, getClass().getSimpleName(), RobotState.getAlliance(), "TeleOp");
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
            allianceSelector.applySelection(robot, robot.lighting);
            selectedAlliance = allianceSelector.getSelectedAlliance();
        }
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();

        robot.drive.driveScaled(request.fieldX, request.fieldY, request.rotation, request.slowMode);
        robot.intake.refreshLaneSensors();

        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.shooter,
                robot.vision,
                request,
                launcherCoordinator,
                selectedAlliance,
                getRuntime(),
                telemetry,
                robot.logger,
                "TeleOp"
        );
        robot.logger.sampleSources();
    }

    @Override
    public void onStop() {
        if (operatorBindings != null) {
            operatorBindings.reset();
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
        robot.logger.stopSession();
    }

}
