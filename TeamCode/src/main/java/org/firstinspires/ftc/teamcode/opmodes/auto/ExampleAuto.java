package org.firstinspires.ftc.teamcode.opmodes.auto; // keep package aligned with path

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    private int pathState;
    private Alliance activeAlliance = Alliance.BLUE;

    private Path scorePreload;
    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain grabPickup2;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;

    public void buildPaths(FieldLayout layout) {
        scorePreload = new Path(new BezierLine(layout.startPose, layout.scorePose));
        scorePreload.setLinearHeadingInterpolation(
                layout.startPose.getHeading(),
                layout.scorePose.getHeading()
        );

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(layout.scorePose, layout.pickup1Pose))
                .setLinearHeadingInterpolation(layout.scorePose.getHeading(), layout.pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(layout.pickup1Pose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.pickup1Pose.getHeading(), layout.scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(layout.scorePose, layout.pickup2Pose))
                .setLinearHeadingInterpolation(layout.scorePose.getHeading(), layout.pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(layout.pickup2Pose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.pickup2Pose.getHeading(), layout.scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(layout.scorePose, layout.pickup3Pose))
                .setLinearHeadingInterpolation(layout.scorePose.getHeading(), layout.pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(layout.pickup3Pose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.pickup3Pose.getHeading(), layout.scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            default:
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("alliance", activeAlliance.displayName());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        applyAlliance(activeAlliance);
    }

    @Override
    public void init_loop() {
        Alliance desiredAlliance = activeAlliance;
        if (gamepad1.x) {
            desiredAlliance = Alliance.BLUE;
        } else if (gamepad1.b) {
            desiredAlliance = Alliance.RED;
        }

        if (desiredAlliance != activeAlliance) {
            applyAlliance(desiredAlliance);
        }

        telemetry.addLine("Press X for Blue alliance or B for Red");
        telemetry.addData("Selected alliance", activeAlliance.displayName());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void stop() {
        // Nothing to clean up; follower stops itself when idle.
    }

    private void applyAlliance(Alliance alliance) {
        activeAlliance = alliance;
        FieldLayout layout = FieldLayout.forAlliance(alliance);
        buildPaths(layout);
        follower.setStartingPose(layout.startPose);
    }

    private static class FieldLayout {
        final Pose startPose;
        final Pose scorePose;
        final Pose pickup1Pose;
        final Pose pickup2Pose;
        final Pose pickup3Pose;

        FieldLayout(Pose startPose,
                    Pose scorePose,
                    Pose pickup1Pose,
                    Pose pickup2Pose,
                    Pose pickup3Pose) {
            this.startPose = startPose;
            this.scorePose = scorePose;
            this.pickup1Pose = pickup1Pose;
            this.pickup2Pose = pickup2Pose;
            this.pickup3Pose = pickup3Pose;
        }

        static FieldLayout forAlliance(Alliance alliance) {
            Pose blueStart = new Pose(57, 9, Math.toRadians(90));
            Pose blueScore = new Pose(57, 18, Math.toRadians(115));
            Pose bluePickup1 = new Pose(12, 12, Math.toRadians(180));
            Pose bluePickup2 = new Pose(24, 36, Math.toRadians(90));
            Pose bluePickup3 = new Pose(24, 60, Math.toRadians(90));

            if (alliance == Alliance.RED) {
                return new FieldLayout(
                        mirrorPoseAcrossField(blueStart),
                        mirrorPoseAcrossField(blueScore),
                        mirrorPoseAcrossField(bluePickup1),
                        mirrorPoseAcrossField(bluePickup2),
                        mirrorPoseAcrossField(bluePickup3)
                );
            }

            return new FieldLayout(blueStart, blueScore, bluePickup1, bluePickup2, bluePickup3);
        }

        private static Pose mirrorPoseAcrossField(Pose pose) {
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - pose.getHeading());
            return new Pose(-pose.getX(), pose.getY(), mirroredHeading);
        }
    }
}
