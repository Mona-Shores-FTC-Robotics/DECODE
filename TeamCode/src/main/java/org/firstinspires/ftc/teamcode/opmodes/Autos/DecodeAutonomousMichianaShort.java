package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.MichianaShortCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.ControlHubIdentifierUtil;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherModeSelector;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.AutoPrestartHelper;

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
 * Michiana Short: fires all 3 lanes mid-return-path at T=fireAtT (default 0.85) instead of
 * stopping at the launch position to shoot. Saves ~0.5–1 s per cycle by overlapping the
 * last stretch of the return path with the shot sequence.
 *
 * Waypoints mirror CloseThreeAtOnce — tune MichianaShortCommand.Waypoints as needed.
 */
@Autonomous(name = "Michiana Short", group = "Auto")
public class DecodeAutonomousMichianaShort extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    private Robot robot;
    private AllianceSelector allianceSelector;
    private LauncherModeSelector modeSelector;
    private Alliance activeAlliance = Alliance.BLUE;
    private FieldLayout currentLayout;
    private AutoPrestartHelper prestartHelper;

    private Pose lastAppliedStartPosePedro;
    private Pose lastDetectedStartPosePedro;

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
        ControlHubIdentifierUtil.setRobotName(hardwareMap, telemetry);
        robot.attachPedroFollower();

        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();

        robot.initializeForAuto();
        robot.lighting.setDriveSubsystem(robot.drive);

        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        GamepadEx operatorPad = new GamepadEx(() -> gamepad2);

        allianceSelector = new AllianceSelector(driverPad, Alliance.UNKNOWN);
        modeSelector = new LauncherModeSelector(operatorPad, LauncherMode.THROUGHPUT);

        activeAlliance = allianceSelector.getSelectedAlliance();

        applyAlliance(activeAlliance, MichianaShortCommand.getDefaultStartPose());

        allianceSelector.applySelection(robot, robot.lighting);
        modeSelector.applySelection(robot.lighting);
        prestartHelper = new AutoPrestartHelper(robot, allianceSelector);

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
        AutoPrestartHelper.InitStatus initStatus = prestartHelper.update(activeAlliance);
        applyInitSelections(initStatus);

        modeSelector.updateDuringInit(robot.lighting);

        updateInitTelemetry(initStatus);
        updateDriverStationTelemetry(initStatus);
        robot.telemetry.publishLoopTelemetry(
                robot.drive, robot.launcher, robot.intake, robot.vision, robot.lighting,
                null, gamepad1, gamepad2,
                RobotState.getAlliance(), getRuntime(),
                Math.max(0.0, 150.0 - getRuntime()),
                telemetry, "AutoInit", true, null, 0, 0
        );
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();
        modeSelector.lockSelection();

        Pose startPoseOverride = lastAppliedStartPosePedro != null
                ? lastAppliedStartPosePedro
                : null;

        Command autoRoutine = MichianaShortCommand.create(robot, activeAlliance, startPoseOverride);
        CommandManager.INSTANCE.scheduleCommand(autoRoutine);

        robot.lighting.resumeLaneTracking();
        robot.intake.forwardRoller();
        robot.intake.setGateAllowArtifacts();
    }

    @Override
    public void onUpdate() {
        publishTelemetry();
    }

    @Override
    public void onStop() {
        try {
            RobotState.setHandoffPose(robot.drive.getFollower().getPose());
        } catch (Exception ignored) {}

        CommandManager.INSTANCE.cancelAll();
        allianceSelector.unlockSelection();
        modeSelector.unlockSelection();
        BindingManager.reset();

        try { robot.launcher.abort(); } catch (Exception ignored) {}
        try { robot.drive.stop(); } catch (Exception ignored) {}
        try { robot.intake.stop(); } catch (Exception ignored) {}
        try { robot.vision.stop(); } catch (Exception ignored) {}
        try { robot.lighting.stop(); } catch (Exception ignored) {}
    }

    // ---- Init helpers ----

    private void applyInitSelections(AutoPrestartHelper.InitStatus status) {
        if (status == null) return;

        if (status.alliance != activeAlliance) {
            activeAlliance = status.alliance;
            Pose defaultPose = lastDetectedStartPosePedro != null
                    ? lastDetectedStartPosePedro
                    : MichianaShortCommand.getDefaultStartPose();
            applyAlliance(activeAlliance, defaultPose);
        }

        lastDetectedStartPosePedro = status.startPoseFromVision;
        if (shouldUpdateStartPose(lastDetectedStartPosePedro)) {
            applyAlliance(activeAlliance, lastDetectedStartPosePedro);
        }
    }

    private void applyAlliance(Alliance alliance, Pose startPose) {
        Alliance safeAlliance = alliance != null && alliance != Alliance.UNKNOWN
                ? alliance : DEFAULT_ALLIANCE;
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);

        currentLayout = AutoField.layoutForAlliance(activeAlliance);
        currentLayout.overrideStart(startPose);
        lastAppliedStartPosePedro = copyPose(startPose);

        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
    }

    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) return false;
        double fieldWidthIn = FieldConstants.FIELD_WIDTH_INCHES;
        return candidate.getX() >= 0 && candidate.getX() <= fieldWidthIn
                && candidate.getY() >= 0 && candidate.getY() <= fieldWidthIn;
    }

    private Pose copyPose(Pose pose) {
        if (pose == null) return null;
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private void updateInitTelemetry(AutoPrestartHelper.InitStatus status) {
        RobotState.packet.put("init/alliance", activeAlliance.name());
        if (status != null && status.relocalizedPose != null) {
            RobotState.packet.put("init/relocalize/x", status.relocalizedPose.getX());
            RobotState.packet.put("init/relocalize/y", status.relocalizedPose.getY());
        }
        RobotState.packet.put("init/fireAtT", MichianaShortCommand.Config.fireAtT);
    }

    private void updateDriverStationTelemetry(AutoPrestartHelper.InitStatus status) {
        telemetry.clear();

        Pose followerPose = robot.drive.getFollower().getPose();
        Pose targetPose = AutoField.poseForAlliance(
                MichianaShortCommand.Waypoints.startX,
                MichianaShortCommand.Waypoints.startY,
                MichianaShortCommand.Waypoints.startHeading,
                activeAlliance
        );

        String visionLine = (status != null && status.startPoseFromVision != null)
                ? "VISION LOCKED" : "No tag";

        telemetry.addData(">> RELOCALIZE", visionLine);
        telemetry.addLine();
        telemetry.addData("Target", "X=%.1f Y=%.1f θ=%.0f°",
                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
        telemetry.addData("Follower", "X=%.1f Y=%.1f θ=%.0f°",
                followerPose.getX(), followerPose.getY(), Math.toDegrees(followerPose.getHeading()));
        telemetry.addLine();
        telemetry.addData("Alliance", activeAlliance.displayName());
        telemetry.addData("Launcher Mode", modeSelector.getDisplayText());
        telemetry.addData("Fire at T", "%.2f  (heading done at T=%.2f)",
                MichianaShortCommand.Config.fireAtT,
                MichianaShortCommand.Config.headingInterpEnd);
        telemetry.addLine("Driver D-pad: Alliance (L=BLUE R=RED)");
        telemetry.addLine("Operator D-pad: Mode (L=THRU R=SEQ)");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    private boolean isOppositeGoalTag(int tagId) {
        if (activeAlliance == Alliance.UNKNOWN) return false;
        return (activeAlliance == Alliance.BLUE && tagId == FieldConstants.RED_GOAL_TAG_ID)
                || (activeAlliance == Alliance.RED && tagId == FieldConstants.BLUE_GOAL_TAG_ID);
    }

    private void publishTelemetry() {
        robot.telemetry.publishLoopTelemetry(
                robot.drive, robot.launcher, robot.intake, robot.vision, robot.lighting,
                null, gamepad1, gamepad2,
                RobotState.getAlliance(), getRuntime(),
                Math.max(0.0, 150.0 - getRuntime()),
                telemetry, "Auto", true, null, 0, 0
        );
    }
}
