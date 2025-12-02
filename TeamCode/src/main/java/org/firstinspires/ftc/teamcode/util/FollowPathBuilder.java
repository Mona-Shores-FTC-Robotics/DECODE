package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;
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

    private Double timeoutMilliSec = 0.0;


    private final List<Pose> controlPoints = new ArrayList<>();

    private boolean constantHeading = false;
    private double constantHeadingDeg = 0;
    private double translationalConstraint = 3;
    private double headingConstraint = 2;


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
        this.controlPoints.add(mirror(controlPose));
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

    public FollowPathBuilder withTranslationalConstraint(double t) {
        this.translationalConstraint = t;
        return this;
    }

    public FollowPathBuilder withHeadingConstraint(double radians) {
        this.headingConstraint = radians;
        return this;
    }



    // ---------------------------
    // Build
    // ---------------------------

    public Command build(double maxPower) {
        PathBuilder builder = robot.drive.getFollower().pathBuilder();
        List<Pose> pts = new ArrayList<>();
        pts.add(start);
        pts.addAll(controlPoints);
        pts.add(end);

        if (pts.size() == 2) {
            builder.addPath(new BezierLine(start, end));
        } else {
            // Keep continuity when multiple control points are supplied: each curve starts where the
            // previous one ended, instead of jumping back to earlier controls.
            Pose segmentStart = start;
            for (int i = 0; i < pts.size() - 1; i++) {
                Pose control = pts.get(i + 1);
                Pose segmentEnd = (i + 2 < pts.size()) ? pts.get(i + 2) : end;
                // Avoid degenerate curves when the last control equals the end point
                if (control.equals(segmentEnd)) {
                    builder.addPath(new BezierLine(segmentStart, segmentEnd));
                } else {
                    builder.addPath(new BezierCurve(segmentStart, control, segmentEnd));
                }
                segmentStart = segmentEnd;
                if (segmentEnd == end) {
                    break;
                }
            }
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

        if (timeoutMilliSec != null) {
            // Assuming your chain only has one path
            Path onlyPath = chain.getPath(0);
            onlyPath.setTimeoutConstraint(timeoutMilliSec);
        }

        if (headingConstraint != 0) {
            // Assuming your chain only has one path
            Path onlyPath = chain.getPath(0);
            onlyPath.setHeadingConstraint(headingConstraint);
        }

        if (translationalConstraint != 0) {
            // Assuming your chain only has one path
            Path onlyPath = chain.getPath(0);
            onlyPath.setTranslationalConstraint(translationalConstraint);
        }

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

    public FollowPathBuilder withTimeout(double milliseconds) {
        this.timeoutMilliSec = milliseconds;
        return this;
    }
}
