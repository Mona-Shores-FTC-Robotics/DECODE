package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMode;

/**
 * Minimal two-leg autonomous used to sanity-check Pedro pathing.
 * Leg 1: drive forward 10 inches.
 * Leg 2: strafe left 10 inches (in Pedro coordinates).
 */
@Autonomous(name = "Simple Two Leg Auto", group = "Debug")
public class SimpleTwoLegAuto extends OpMode {

    private enum Step {
        FORWARD,
        WAIT_FORWARD,
        STRAFE,
        WAIT_STRAFE,
        DONE
    }

    private Robot robot;
    private Follower follower;
    private PathChain forwardPath;
    private PathChain strafePath;
    private Step step = Step.FORWARD;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.setRobotMode(RobotMode.MATCH);
        robot.telemetry.startSession();
        robot.initializeForAuto();
        follower = robot.drive.getFollower();

        Pose start = new Pose(56, 8, Math.toRadians(90));
        Pose forwardEnd = new Pose(56.279,19.817, Math.toRadians(109));
        Pose strafeEnd = new Pose( 23.780,  23.780, Math.toRadians(90));

        follower.setStartingPose(start);
        follower.setPose(start);

        forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(start, forwardEnd))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        strafePath = follower.pathBuilder()
                .addPath(new BezierLine(forwardEnd, strafeEnd))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Simple path debug");
        telemetry.update();
    }

    @Override
    public void start() {
        step = Step.FORWARD;
    }

    @Override
    public void loop() {
        robot.drive.updateFollower();

        switch (step) {
            case FORWARD:
                robot.drive.followPath(forwardPath, 0.1, false);
                step = Step.WAIT_FORWARD;
                break;
            case WAIT_FORWARD:
                if (!robot.drive.isFollowerBusy()) {
                    step = Step.STRAFE;
                }
                break;
            case STRAFE:
                robot.drive.followPath(strafePath, 0.1, false);
                step = Step.WAIT_STRAFE;
                break;
            case WAIT_STRAFE:
                if (!robot.drive.isFollowerBusy()) {
                    step = Step.DONE;
                }
                break;
            case DONE:
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.drive.stop();
        robot.telemetry.stopSession();
    }
}
