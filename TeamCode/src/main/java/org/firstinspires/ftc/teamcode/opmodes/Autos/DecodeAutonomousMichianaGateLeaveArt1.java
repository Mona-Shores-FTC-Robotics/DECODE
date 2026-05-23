package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.MichianaGateLeaveArt1Command;

/**
 * Michiana Gate (Art1 Last): skips Art1 on the way in, opens gate first,
 * runs 3 gate slant cycles, then picks up Art1 at the end to fill the chute.
 */
@Autonomous(name = "Michiana Gate (Art1 Last)", group = "Auto")
public class DecodeAutonomousMichianaGateLeaveArt1 extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return MichianaGateLeaveArt1Command.Waypoints.startX; }

    @Override
    protected double getStartY() { return MichianaGateLeaveArt1Command.Waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return MichianaGateLeaveArt1Command.Waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return MichianaGateLeaveArt1Command.create(robot, activeAlliance, startPoseOverride);
    }
}
