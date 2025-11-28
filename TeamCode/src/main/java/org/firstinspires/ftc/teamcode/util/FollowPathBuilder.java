package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;
import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.FollowPath;

import org.firstinspires.ftc.teamcode.Robot;

public class FollowPathBuilder {

    private final Robot robot;
    private final Alliance alliance;

    private Pose start;
    private Pose end;
    private Pose control;

    private boolean useControlPoint = false;

    private boolean constantHeading = false;
    private double constantHeadingDeg = 0;

    private double linearInterpWeight = 0.7;

    public FollowPathBuilder(Robot robot, Alliance alliance) {
        this.robot = robot;
        this.alliance = alliance;
    }

    // ---------------------------
    // Fluent API
    // ---------------------------

    /**
     * Sets start pose from waypoint coordinates (will be mirrored for red alliance)
     */
    public FollowPathBuilder from(Pose startPose) {
        this.start = mirror(startPose);
        return this;
    }

    /**
     * Sets start pose from world coordinates (already accounts for alliance, no mirroring)
     * Use this for vision-detected poses or follower's current pose
     */
    public FollowPathBuilder fromWorldCoordinates(Pose worldStartPose) {
        this.start = worldStartPose;
        return this;
    }

    public FollowPathBuilder to(Pose endPose) {
        this.end = mirror(endPose);
        return this;
    }

    public FollowPathBuilder withControl(Pose controlPose) {
        this.control = mirror(controlPose);
        this.useControlPoint = true;
        return this;
    }

    public FollowPathBuilder withConstantHeading(double headingDeg) {
        this.constantHeading = true;
        this.constantHeadingDeg = headingDeg;
        return this;
    }

    public FollowPathBuilder withLinearHeadingCompletion(double w) {
        this.linearInterpWeight = w;
        return this;
    }

    // ---------------------------
    // Build
    // ---------------------------

    public Command build(double maxPower) {
        PathBuilder builder = robot.drive.getFollower().pathBuilder();

        if (useControlPoint) {
            builder.addPath(  new BezierCurve(
                    start,
                    control,
                    end
                    )
            );
        } else {
            builder.addPath(new BezierLine(start, end));
        }

        if (constantHeading) {
            builder.setConstantHeadingInterpolation(constantHeadingDeg);
        } else {
            builder.setLinearHeadingInterpolation(
                    start.getHeading(),
                    end.getHeading(),
                    linearInterpWeight
            );
        }

        PathChain chain = builder.build();
        double clippedPower = Range.clip(maxPower, 0.0, 1.0);

        return new FollowPath(chain, false, clippedPower);
    }

    // ---------------------------
    // Internal: Mirror utility
    // ---------------------------

    private Pose mirror(Pose p) {
        return poseForAlliance(
                p.getX(),
                p.getY(),
                Math.toDegrees(p.getHeading()),
                alliance
        );
    }

    // This forwards to your existing method
    private static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
        return AutoField.poseForAlliance(x, y, headingDeg, alliance);
    }
}
