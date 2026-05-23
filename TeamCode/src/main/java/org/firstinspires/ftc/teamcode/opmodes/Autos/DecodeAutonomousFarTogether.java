package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.FarTogetherCommand;

/**
 * Autonomous: drive from far start, collect samples and score together at LAUNCH_FAR.
 */
@Autonomous(name = "Far Together", group = "Auto")
public class DecodeAutonomousFarTogether extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return FarTogetherCommand.waypoints.startX; }

    @Override
    protected double getStartY() { return FarTogetherCommand.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return FarTogetherCommand.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return FarTogetherCommand.create(robot, activeAlliance, startPoseOverride);
    }
}
