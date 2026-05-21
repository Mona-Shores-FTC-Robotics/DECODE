package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.CloseTogetherCommand;

/**
 * Autonomous: drive from close start, collect samples and score together at LAUNCH_CLOSE.
 */
@Autonomous(name = "Close Together", group = "Auto")
public class DecodeAutonomousCloseTogether extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return CloseTogetherCommand.waypoints.startX; }

    @Override
    protected double getStartY() { return CloseTogetherCommand.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return CloseTogetherCommand.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return CloseTogetherCommand.create(robot, activeAlliance, startPoseOverride);
    }
}
