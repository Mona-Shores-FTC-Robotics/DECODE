package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.command.CommandScheduler;
import dev.nextftc.ftc.command.SequentialCommandGroup;
import dev.nextftc.pedro.PedroOpMode;
import dev.nextftc.pedro.command.FollowPathCommand;
import dev.nextftc.pedro.command.WaitCommand;

/**
 * Example autonomous using Sequential Command Groups
 *
 * This autonomous demonstrates:
 * - Sequential execution of commands (one after another)
 * - Following paths using FollowPathCommand
 * - Adding delays with WaitCommand
 * - Turning to specific headings
 */
@Autonomous(name = "Sequential Command Auto", group = "Examples")
public class SequentialCommandAuto extends PedroOpMode {

    @Override
    public void onInit() {
        // Set starting pose
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        // Create paths for the autonomous routine

        // Path 1: Drive forward to position (24, 24)
        Path pathForward = new Path(new BezierLine(
            startPose,
            new Pose(24, 24, 0)
        ));
        pathForward.setHeadingInterpolation(HeadingInterpolator.constantFrom(0));

        // Path 2: Turn to 90 degrees while staying at current position
        Path turnPath = new Path(new BezierLine(
            new Pose(24, 24, 0),
            new Pose(24, 24, Math.toRadians(90))
        ));
        turnPath.setHeadingInterpolation(HeadingInterpolator.constantFrom(Math.toRadians(90)));

        // Path 3: Strafe to the right
        Path strafeRight = new Path(new BezierLine(
            new Pose(24, 24, Math.toRadians(90)),
            new Pose(24, 0, Math.toRadians(90))
        ));
        strafeRight.setHeadingInterpolation(HeadingInterpolator.constantFrom(Math.toRadians(90)));

        // Path 4: Return to start
        Path returnToStart = new Path(new BezierLine(
            new Pose(24, 0, Math.toRadians(90)),
            new Pose(0, 0, 0)
        ));
        returnToStart.setHeadingInterpolation(HeadingInterpolator.linearFrom(Math.toRadians(90), 0));

        // Create a sequential command group
        // Commands run one after another in order
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
            // Step 1: Drive forward
            new FollowPathCommand(follower, follower.pathBuilder()
                .addPath(pathForward)
                .build()),

            // Step 2: Wait 1 second
            new WaitCommand(1.0),

            // Step 3: Turn to 90 degrees
            new FollowPathCommand(follower, follower.pathBuilder()
                .addPath(turnPath)
                .build()),

            // Step 4: Wait 0.5 seconds
            new WaitCommand(0.5),

            // Step 5: Strafe right
            new FollowPathCommand(follower, follower.pathBuilder()
                .addPath(strafeRight)
                .build()),

            // Step 6: Wait 1 second
            new WaitCommand(1.0),

            // Step 7: Return to start
            new FollowPathCommand(follower, follower.pathBuilder()
                .addPath(returnToStart)
                .build())
        );

        // Schedule the autonomous sequence to run when start is pressed
        onStartPressed(() -> CommandScheduler.scheduleForState(autoSequence, OpModeState.RUN));
    }
}
