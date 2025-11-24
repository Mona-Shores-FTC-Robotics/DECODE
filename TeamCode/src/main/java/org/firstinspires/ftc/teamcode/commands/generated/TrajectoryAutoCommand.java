package org.firstinspires.ftc.teamcode.commands.generated;

import static org.firstinspires.ftc.teamcode.commands.generated.TrajectoryAutoConfig.*;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

/**
 * AUTO-GENERATED autonomous command from .pp file
 *
 * Add robot actions where indicated.
 * Waypoints are in TrajectoryAutoConfig.java
 */
public class TrajectoryAutoCommand extends Command {

    private final SequentialGroup sequence;

    public TrajectoryAutoCommand(
            Robot robot,
            IntakeCommands intakeCommands,
            LauncherCommands launcherCommands) {

        sequence = new SequentialGroup(
            // ======================================
            // SEGMENT 1: path1
            // start → path1 (linear heading)
            // ======================================
            new ParallelGroup(
                new FollowPath(
                    robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(
                            Waypoints.start.toPose(),
                            Waypoints.path1.toPose()
                        ))
                        .setLinearHeadingInterpolation(
                            Waypoints.start.toPose().getHeading(),
                            Waypoints.path1.toPose().getHeading()
                        )
                        .build(),
                    false,  // reverse
                    pathPower.path1
                )
                // TODO: Add robot actions during path1
            ),
            // TODO: Add actions after reaching path1

            // ======================================
            // SEGMENT 2: path2
            // path1 → path2 (linear heading)
            // ======================================
            new ParallelGroup(
                new FollowPath(
                    robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(
                            Waypoints.path1.toPose(),
                            Waypoints.path2.toPose()
                        ))
                        .setLinearHeadingInterpolation(
                            Waypoints.path1.toPose().getHeading(),
                            Waypoints.path2.toPose().getHeading()
                        )
                        .build(),
                    false,  // reverse
                    pathPower.path2
                )
                // TODO: Add robot actions during path2
            ),
            // TODO: Add actions after reaching path2

            // ======================================
            // SEGMENT 3: path3
            // path2 → path3 (tangential heading)
            // ======================================
            new ParallelGroup(
                new FollowPath(
                    robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(
                            Waypoints.path2.toPose(),
                            Waypoints.path3.toPose()
                        ))
                        .setTangentialHeadingInterpolation()
                        .build(),
                    false,  // reverse
                    pathPower.path3
                )
                // TODO: Add robot actions during path3
            ),
            // TODO: Add actions after reaching path3

            // ======================================
            // SEGMENT 4: path4
            // path3 → path4 (linear heading)
            // ======================================
            new ParallelGroup(
                new FollowPath(
                    robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(
                            Waypoints.path3.toPose(),
                            Waypoints.path4.toPose()
                        ))
                        .setLinearHeadingInterpolation(
                            Waypoints.path3.toPose().getHeading(),
                            Waypoints.path4.toPose().getHeading()
                        )
                        .build(),
                    false,  // reverse
                    pathPower.path4
                )
                // TODO: Add robot actions during path4
            )
            // TODO: Add actions after reaching path4
        );
    }

    @Override
    public void start() {
        sequence.start();
    }

    @Override
    public void update() {
        sequence.update();
    }

    @Override
    public boolean isDone() {
        return sequence.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
    }
}
