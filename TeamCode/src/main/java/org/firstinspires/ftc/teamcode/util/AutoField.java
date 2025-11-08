package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.EnumMap;

/**
 * Shared field geometry used by autonomous routines. Provides Pedro-aligned poses for key waypoints
 * and mirrors them automatically for the red alliance.
 */
public final class AutoField {

    private AutoField() {
        // Utility class
    }

    public enum FieldPoint {
        START,
        LAUNCH_FAR,
        SETUP_PARKING_ARTIFACTS,
        PARKING_ARTIFACTS
    }

    @com.bylazar.configurables.annotations.Configurable
    public static class Waypoints {
        public static double fieldWidthIn = 144.0;

        public static double startX = 56.000;
        public static double startY = 8.0;
        public static double startHeadingDeg = 90.0;

        public static double launchFarX = 56.279;
        public static double launchFarY = 19.817;
        public static double launchFarHeadingDeg = 90;

        public static double setupParkingX = 23.780;
        public static double setupParkingY = 23.780;
        public static double setupParkingHeadingDeg = 90.0;

        public static double parkingArtifactsX = 23.516;
        public static double parkingArtifactsY = 39.633;
        public static double parkingArtifactsHeadingDeg = 90.0;
    }

    public static FieldLayout layoutForAlliance(Alliance alliance) {
        EnumMap<FieldPoint, Pose> layout = new EnumMap<>(FieldPoint.class);
        layout.put(FieldPoint.START, poseForAlliance(
                Waypoints.startX,
                Waypoints.startY,
                Waypoints.startHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.LAUNCH_FAR, poseForAlliance(
                Waypoints.launchFarX,
                Waypoints.launchFarY,
                Waypoints.launchFarHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.SETUP_PARKING_ARTIFACTS, poseForAlliance(
                Waypoints.setupParkingX,
                Waypoints.setupParkingY,
                Waypoints.setupParkingHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.PARKING_ARTIFACTS, poseForAlliance(
                Waypoints.parkingArtifactsX,
                Waypoints.parkingArtifactsY,
                Waypoints.parkingArtifactsHeadingDeg,
                alliance
        ));
        return new FieldLayout(layout);
    }

    private static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
        Pose base = new Pose(x, y, Math.toRadians(headingDeg));
        if (alliance == Alliance.RED) {
            double mirroredX = Waypoints.fieldWidthIn - base.getX();
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - base.getHeading());
            return new Pose(mirroredX, base.getY(), mirroredHeading);
        }
        return base;
    }

    public static final class FieldLayout {
        private final EnumMap<FieldPoint, Pose> poses;

        private FieldLayout(EnumMap<FieldPoint, Pose> poses) {
            this.poses = poses;
        }

        public Pose pose(FieldPoint point) {
            Pose stored = poses.get(point);
            return stored == null ? null : new Pose(stored.getX(), stored.getY(), stored.getHeading());
        }

        public void overrideStart(Pose startPose) {
            if (startPose == null) {
                return;
            }
            poses.put(FieldPoint.START, new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
        }
    }
}
