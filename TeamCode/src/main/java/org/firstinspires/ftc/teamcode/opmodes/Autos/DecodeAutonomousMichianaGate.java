package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.MichianaGateCommand;

/**
 * Michiana Gate: 5-artifact close-side auto.
 * Fires preloads on fly, opens gate during Art1/Art2 cycles, then
 * does repeated slant-gate collection cycles until time runs out.
 */
@Autonomous(name = "Michiana Gate", group = "Auto")
public class DecodeAutonomousMichianaGate extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return MichianaGateCommand.Waypoints.startX; }

    @Override
    protected double getStartY() { return MichianaGateCommand.Waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return MichianaGateCommand.Waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return MichianaGateCommand.create(robot, activeAlliance, startPoseOverride);
    }
}
