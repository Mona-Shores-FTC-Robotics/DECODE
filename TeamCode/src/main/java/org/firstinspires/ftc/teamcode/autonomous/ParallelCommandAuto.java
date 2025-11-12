package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.command.CommandScheduler;
import dev.nextftc.ftc.command.InstantCommand;
import dev.nextftc.ftc.command.ParallelCommandGroup;
import dev.nextftc.ftc.command.SequentialCommandGroup;
import dev.nextftc.pedro.PedroOpMode;
import dev.nextftc.pedro.command.FollowPathCommand;
import dev.nextftc.pedro.command.WaitCommand;

/**
 * Example autonomous using Parallel Command Groups
 *
 * This autonomous demonstrates:
 * - Parallel execution of commands (multiple commands running at the same time)
 * - Running actions while the robot is driving
 * - Combining parallel and sequential groups
 *
 * In a real robot, you would typically:
 * - Drive to a position while raising an arm
 * - Drive while activating an intake
 * - Drive while extending a slide
 * etc.
 */
@Autonomous(name = "Parallel Command Auto", group = "Examples")
public class ParallelCommandAuto extends PedroOpMode {

    @Override
    public void onInit() {
        // Set starting pose
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        // Create paths
        Path driveToSample = new Path(new BezierLine(
            startPose,
            new Pose(30, 30, Math.toRadians(45))
        ));
        driveToSample.setHeadingInterpolation(HeadingInterpolator.linearFrom(0, Math.toRadians(45)));

        Path driveToBasket = new Path(new BezierLine(
            new Pose(30, 30, Math.toRadians(45)),
            new Pose(12, 48, Math.toRadians(90))
        ));
        driveToBasket.setHeadingInterpolation(HeadingInterpolator.linearFrom(Math.toRadians(45), Math.toRadians(90)));

        // Create command sequence with parallel actions
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
            // Action 1: Drive to sample while "lowering intake" (simulated)
            new ParallelCommandGroup(
                // Drive to sample position
                new FollowPathCommand(follower, follower.pathBuilder()
                    .addPath(driveToSample)
                    .build()),

                // Simulate lowering an intake mechanism
                // In a real robot, this would be a command that controls your intake subsystem
                new SequentialCommandGroup(
                    new WaitCommand(0.2), // Wait briefly before starting
                    new InstantCommand(() ->
                        telemetry.addData("Action", "Intake lowering...")),
                    new WaitCommand(0.5), // Simulate time to lower
                    new InstantCommand(() ->
                        telemetry.addData("Action", "Intake at position"))
                )
            ),

            // Action 2: "Grab sample" while stationary
            new InstantCommand(() ->
                telemetry.addData("Action", "Grabbing sample...")),
            new WaitCommand(0.5), // Time to grab
            new InstantCommand(() ->
                telemetry.addData("Action", "Sample secured")),

            // Action 3: Drive to basket while "raising arm" (simulated)
            new ParallelCommandGroup(
                // Drive to basket
                new FollowPathCommand(follower, follower.pathBuilder()
                    .addPath(driveToBasket)
                    .build()),

                // Simulate raising arm with sample
                // In a real robot, this would be a command that controls your arm/slide subsystem
                new SequentialCommandGroup(
                    new InstantCommand(() ->
                        telemetry.addData("Action", "Arm raising...")),
                    new WaitCommand(1.0), // Simulate time to raise arm
                    new InstantCommand(() ->
                        telemetry.addData("Action", "Arm at high position"))
                )
            ),

            // Action 4: "Score sample"
            new InstantCommand(() ->
                telemetry.addData("Action", "Scoring sample...")),
            new WaitCommand(0.3),
            new InstantCommand(() ->
                telemetry.addData("Action", "Sample scored!"))
        );

        // Schedule the autonomous sequence
        onStartPressed(() -> CommandScheduler.scheduleForState(autoSequence, OpModeState.RUN));
    }
}
