package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
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
        START_FAR,
        LAUNCH_FAR,
        ALLIANCE_WALL_ARTIFACTS_PICKUP,
        PARKING_ARTIFACTS_PICKUP_90_DEG,
        GATE_FAR_ARTIFACTS_PICKUP_90_DEG,
        LAUNCH_CLOSE,
        GATE_CLOSE_ARTIFACTS_PICKUP,
        GATE_FAR_ARTIFACTS_PICKUP_270_DEG,
        PARKING_ARTIFACTS_PICKUP_270_DEG
    }

    @com.bylazar.configurables.annotations.Configurable
    public static class Waypoints {
        public static double fieldWidthIn = 144.0;

        public static double startX = 56;
        public static double startY = 8;
        public static double startHeadingDeg = 90.0;

        public static double launchFarX = 55;
        public static double launchFarY = 17.3;
        public static double launchFarHeadingDeg = 109.0;

        public static double allianceWallArtifactsX = 8;
        public static double allianceWallArtifactsY = 8;
        public static double allianceWallArtifactsHeadingDeg = 180;

        public static double parkingArtifactsX = 23;
        public static double parkingArtifactsY = 32;
        public static double parkingArtifactsHeadingDeg90 = 90.0;
        public static double parkingArtifactsControlPointX = 28;
        public static double parkingArtifactsControlPointY = 2;

        public static double gateFarX = 24;
        public static double gateFarY = 56;
        public static double gateFarHeadingDeg90 = 90.0;
        public static double gateFarHeadingDeg270 = 270;

        public static double gateFarControlPointX = 22;
        public static double gateFarControlPointY = 29;

        public static double launchCloseX = 21.5;
        public static double launchCloseY = 124;
        public static double launchCloseHeading = 324;

        public static double gateCloseX = 24;
        public static double gateCloseY = 87;

        public static double gateCloseHeading = 270;
        public static double parkingArtifactsHeading270 = 270;


        ArrayList<Pose> parkingControlPoints = new ArrayList<>();
        Pose gateFarControlPoint = new Pose(gateFarControlPointX, gateFarControlPointY, 0);


    }

    public static FieldLayout layoutForAlliance(Alliance alliance) {
        EnumMap<FieldPoint, Pose> layout = new EnumMap<>(FieldPoint.class);
        layout.put(FieldPoint.START_FAR, poseForAlliance(
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
        layout.put(FieldPoint.ALLIANCE_WALL_ARTIFACTS_PICKUP, poseForAlliance(
                Waypoints.allianceWallArtifactsX,
                Waypoints.allianceWallArtifactsY,
                Waypoints.allianceWallArtifactsHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.PARKING_ARTIFACTS_PICKUP_90_DEG, poseForAlliance(
                Waypoints.parkingArtifactsX,
                Waypoints.parkingArtifactsY,
                Waypoints.parkingArtifactsHeadingDeg90,
                alliance
        ));
        layout.put(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_90_DEG, poseForAlliance(
                Waypoints.gateFarX,
                Waypoints.gateFarY,
                Waypoints.gateFarHeadingDeg90,
                alliance
        ));
        layout.put(FieldPoint.LAUNCH_CLOSE, poseForAlliance(
                Waypoints.launchCloseX,
                Waypoints.launchCloseY,
                Waypoints.launchCloseHeading,
                alliance

        ));
        layout.put(FieldPoint.GATE_CLOSE_ARTIFACTS_PICKUP, poseForAlliance(
                Waypoints.gateCloseX,
                Waypoints.gateCloseY,
                Waypoints.gateCloseHeading,
                alliance
        ));
        layout.put(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(
                Waypoints.gateFarX,
                Waypoints.gateFarY,
                Waypoints.gateFarHeadingDeg270,
                alliance

        ));
        layout.put(FieldPoint.PARKING_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(
                Waypoints.parkingArtifactsX,
                Waypoints.parkingArtifactsY,
                Waypoints.parkingArtifactsHeading270,
                alliance

        ));
        return new FieldLayout(layout);
    }

    public static Pose parkingArtifactsControlPoint(Alliance alliance) {
        return poseForAlliance(
                Waypoints.parkingArtifactsControlPointX,
                Waypoints.parkingArtifactsControlPointY,
                Waypoints.parkingArtifactsHeadingDeg90,

                alliance
        );
    }
    public static Pose gateFarArtifactsControlPoint(Alliance alliance) {
        return poseForAlliance(
                Waypoints.gateFarControlPointX,
                Waypoints.gateFarControlPointY,
                Waypoints.gateFarHeadingDeg90,
                alliance
        );
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
            poses.put(FieldPoint.START_FAR, new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
        }
    }
}
