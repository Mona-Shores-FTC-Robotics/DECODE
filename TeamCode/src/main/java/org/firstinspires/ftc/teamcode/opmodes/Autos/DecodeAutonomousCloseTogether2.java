package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.CloseTogether2Command;

/**
 * Autonomous: close-side "together" variant 2 — two gate pushes, two pickup-and-shoot
 * cycles, then park. See {@link CloseTogether2Command} for the full sequence.
 */
@Autonomous(name = "Close Together 2", group = "Auto")
public class DecodeAutonomousCloseTogether2 extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return CloseTogether2Command.waypoints.startX; }

    @Override
    protected double getStartY() { return CloseTogether2Command.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return CloseTogether2Command.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return CloseTogether2Command.create(robot, activeAlliance, startPoseOverride);
    }
}
