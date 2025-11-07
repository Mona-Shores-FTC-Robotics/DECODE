package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Minimal autonomous used for quick sanity checks. Drives forward a short distance, then turns in place.
 */
@Autonomous(name = "Simple Forward + Turn", group = "Autonomous")
public class SimpleForwardTurnAuto extends LinearOpMode {

    private static final double FORWARD_DISTANCE_IN = 24.0;
    private static final double TURN_DEGREES = 90.0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.setRobotMode(RobotMode.MATCH);
        robot.initializeForAuto();

        DriveSubsystem drive = robot.drive;
        Follower follower = drive.getFollower();

        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        PathChain forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        new Pose(startPose.getX(), startPose.getY() + FORWARD_DISTANCE_IN, startPose.getHeading())
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();

        telemetry.addLine("Simple auto ready: forward then turn");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        drive.followPath(forwardPath, true);
        runFollowerUntilIdle(robot);

        follower.turnDegrees(TURN_DEGREES, true);
        runFollowerUntilIdle(robot);

        Pose finalPose = follower.getPose();
        if (finalPose != null) {
            RobotState.setHandoffPose(new Pose(finalPose.getX(), finalPose.getY(), finalPose.getHeading()));
        }

        robot.drive.stop();
    }

    private void runFollowerUntilIdle(Robot robot) {
        DriveSubsystem drive = robot.drive;
        Follower follower = drive.getFollower();
        while (opModeIsActive() && (drive.isFollowerBusy() || follower.isTurning())) {
            drive.updateFollower();
            Pose pose = follower.getPose();
            if (pose != null) {
                telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                        pose.getX(),
                        pose.getY(),
                        Math.toDegrees(pose.getHeading()));
            }
            telemetry.addData("Busy", drive.isFollowerBusy());
            telemetry.addData("Turning", follower.isTurning());
            telemetry.update();
            idle();
        }
    }
}
