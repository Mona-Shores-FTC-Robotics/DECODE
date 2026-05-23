package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.CloseThreeAtOnceCommand;

/**
 * Autonomous: drive from close start, collect and score three samples at LAUNCH_CLOSE.
 */
@Autonomous(name = "Close Three At Once", group = "Auto")
public class DecodeAutonomousCloseThreeAtOnce extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return CloseThreeAtOnceCommand.waypoints.startX; }

    @Override
    protected double getStartY() { return CloseThreeAtOnceCommand.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return CloseThreeAtOnceCommand.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return CloseThreeAtOnceCommand.create(robot, activeAlliance, startPoseOverride);
    }
}
