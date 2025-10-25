package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Dashboard-backed model for the ExampleGateAuto path plan. Exposes tunable poses along the primary
 * cycle and offers lightweight data classes for both pickup legs and shooting volleys.
 */
@Config
public final class TimelineModel {

    public static double startPoseX = 12.0;
    public static double startPoseY = 60.0;
    public static double startPoseHeadingDeg = 180.0;

    public static double scorePoseX = 50.0;
    public static double scorePoseY = 36.0;
    public static double scorePoseHeadingDeg = 0.0;

    public static double midShootPoseX = 40.0;
    public static double midShootPoseY = 36.0;
    public static double midShootPoseHeadingDeg = 0.0;

    public static double pickup1PoseX = 24.0;
    public static double pickup1PoseY = 50.0;
    public static double pickup1PoseHeadingDeg = 180.0;

    public static double pickup2PoseX = 24.0;
    public static double pickup2PoseY = 36.0;
    public static double pickup2PoseHeadingDeg = 180.0;

    public static double pickup3PoseX = 24.0;
    public static double pickup3PoseY = 22.0;
    public static double pickup3PoseHeadingDeg = 180.0;

    public static double lineupControlX = 31.871;
    public static double lineupControlY = 16.225;
    public static double lineupControlHeadingDeg = 0.0;

    private static final String TAG = "TimelineModel";
    private static final double FIELD_WIDTH_IN = 144.0;

    private TimelineModel() { }

    public static Pose getStartPose(boolean mirrorForBlue) {
        return pose(startPoseX, startPoseY, startPoseHeadingDeg, mirrorForBlue);
    }

    public static Pose getScorePose(boolean mirrorForBlue) {
        return pose(scorePoseX, scorePoseY, scorePoseHeadingDeg, mirrorForBlue);
    }

    public static Pose getMidShootPose(boolean mirrorForBlue) {
        return pose(midShootPoseX, midShootPoseY, midShootPoseHeadingDeg, mirrorForBlue);
    }

    public static Pose getLineupControlPose(boolean mirrorForBlue) {
        if (Double.isNaN(lineupControlX) || Double.isNaN(lineupControlY)) {
            return null;
        }
        return pose(lineupControlX, lineupControlY, lineupControlHeadingDeg, mirrorForBlue);
    }

    public static List<PickupPlan> buildDefaultPickups(boolean mirrorForBlue) {
        List<PickupPlan> plans = new ArrayList<>(3);
        plans.add(new PickupPlan("pickup-1", pose(pickup1PoseX, pickup1PoseY, pickup1PoseHeadingDeg, mirrorForBlue)));
        plans.add(new PickupPlan("pickup-2", pose(pickup2PoseX, pickup2PoseY, pickup2PoseHeadingDeg, mirrorForBlue)));
        plans.add(new PickupPlan("pickup-3", pose(pickup3PoseX, pickup3PoseY, pickup3PoseHeadingDeg, mirrorForBlue)));
        return Collections.unmodifiableList(plans);
    }

    public static List<ShotPlan> buildShotPlan(int preloadCount, int cycles, boolean mirrorForBlue) {
        List<ShotPlan> shots = new ArrayList<>();
        Pose scorePose = getScorePose(mirrorForBlue);

        if (preloadCount > 0) {
            shots.add(new ShotPlan("preloads", preloadCount, scorePose));
        }

        for (int cycle = 0; cycle < cycles; cycle++) {
            shots.add(new ShotPlan("cycle-" + (cycle + 1), 3, scorePose));
        }

        return Collections.unmodifiableList(shots);
    }

    public static double degToRad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double radToDeg(double radians) {
        return Math.toDegrees(radians);
    }

    private static Pose pose(double x, double y, double headingDeg, boolean mirrorForBlue) {
        double headingRad = Math.toRadians(headingDeg);
        if (!mirrorForBlue) {
            return new Pose(x, y, headingRad);
        }

        double mirroredX = FIELD_WIDTH_IN - x;
        double mirroredHeading = normalize(Math.PI - headingRad);
        return new Pose(mirroredX, y, mirroredHeading);
    }

    private static double normalize(double angle) {
        double wrapped = angle;
        while (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        while (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        return wrapped;
    }

    public static class PickupPlan {
        public final String label;
        public final Pose targetPose;

        public PickupPlan(String label, Pose targetPose) {
            this.label = label;
            this.targetPose = targetPose;
        }

        @Override
        public String toString() {
            return label + "@" + format(targetPose);
        }
    }

    public static class ShotPlan {
        public final String label;
        public final int count;
        public final Pose targetPose;

        public ShotPlan(String label, int count, Pose targetPose) {
            if (count < 0) {
                RobotLog.ww(TAG, "Shot plan %s had negative count, clamping to zero", label);
                count = 0;
            }
            this.label = label;
            this.count = count;
            this.targetPose = targetPose;
        }

        @Override
        public String toString() {
            return label + " x" + count + "@" + format(targetPose);
        }
    }

    private static String format(Pose pose) {
        return String.format("(%.1f, %.1f, %.1f°)", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
