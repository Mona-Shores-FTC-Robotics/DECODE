package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.commands.generated.TrajectoryAutoCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.RobotState;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * EXAMPLE: OpMode using generated autonomous command from trajectory.pp
 *
 * This demonstrates how simple your OpMode becomes when using the generator:
 * 1. Standard robot setup (same as always)
 * 2. Instantiate the generated command
 * 3. Done!
 */
@Autonomous(name = "Trajectory Auto (Generated)", group = "Generated")
public class TrajectoryAutoOpMode extends NextFTCOpMode {

    private Robot robot;
    private IntakeCommands intakeCommands;
    private LauncherCommands launcherCommands;
    private AllianceSelector allianceSelector;
    private Alliance activeAlliance = Alliance.BLUE;

    {
        addComponents(
            BulkReadComponent.INSTANCE,
            new PedroComponent(Constants::createFollower),
            BindingsComponent.INSTANCE,
            CommandManager.INSTANCE
        );
    }

    @Override
    public void onInit() {
        BindingManager.reset();
        robot = new Robot(hardwareMap);

        robot.attachPedroFollower();
        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();
        robot.initializeForAuto();

        // Initialize command factories
        intakeCommands = new IntakeCommands(robot.intake);
        launcherCommands = new LauncherCommands(robot.launcher, robot.intake);

        // Alliance selection
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, Alliance.UNKNOWN);
        activeAlliance = allianceSelector.getSelectedAlliance();
        robot.setAlliance(activeAlliance);
        allianceSelector.applySelection(robot, robot.lighting);

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
        BindingManager.update();
        robot.intake.periodic();
        robot.vision.periodic();

        allianceSelector.updateFromVision(robot.vision);
        allianceSelector.applySelection(robot, robot.lighting);

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        if (selectedAlliance != activeAlliance) {
            activeAlliance = selectedAlliance;
            robot.setAlliance(activeAlliance);
        }

        telemetry.clear();
        telemetry.addData("Alliance", activeAlliance.displayName());
        telemetry.addLine("Generated from trajectory.pp");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();

        robot.intake.forwardRoller();
        robot.intake.setGateAllowArtifacts();

        // ===== THIS IS ALL YOU NEED! =====
        // Just instantiate the generated command
        Command auto = new TrajectoryAutoCommand(robot, intakeCommands, launcherCommands);
        CommandManager.INSTANCE.scheduleCommand(auto);
    }

    @Override
    public void onUpdate() {
        publishTelemetry();
    }

    @Override
    public void onStop() {
        allianceSelector.unlockSelection();
        BindingManager.reset();
        robot.launcher.abort();
        robot.drive.stop();
        robot.vision.stop();
        RobotState.setHandoffPose(robot.drive.getFollower().getPose());
    }

    private void publishTelemetry() {
        robot.telemetry.publishLoopTelemetry(
            robot.drive,
            robot.launcher,
            robot.intake,
            robot.vision,
            robot.lighting,
            null,
            gamepad1,
            gamepad2,
            RobotState.getAlliance(),
            getRuntime(),
            Math.max(0.0, 150.0 - getRuntime()),
            telemetry,
            "Generated Auto",
            true,
            null,
            0,
            0
        );
    }
}
