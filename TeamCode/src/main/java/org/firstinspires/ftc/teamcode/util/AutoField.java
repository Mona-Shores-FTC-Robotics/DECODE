package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.EnumMap;

/**
 * Shared field geometry used by autonomous routines. Provides Pedro-aligned poses for key waypoints
 * and mirrors them automatically for the red alliance.
 */
@Configurable
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

@Configurable
public static class Waypoints {

        public double startX = 56;
        public double startY = 9;
        public double startHeadingDeg = 90.0;

        public double launchFarX = 55;
        public double launchFarY = 17.3;
        public double launchFarHeadingDeg = 112;

        public double allianceWallArtifactsX = 9;
        public double allianceWallArtifactsY = 8.2;
        public double allianceWallArtifactsHeadingDeg = 180;

        public double parkingArtifactsX = 23;
        public double parkingArtifactsY = 33;
        public double parkingArtifactsHeadingDeg90 = 90.0;
        public double parkingArtifactsControlPointX = 28;
        public double parkingArtifactsControlPointY = 2;

        public double gateFarX = 24;
        public double gateFarY = 56;
        public double gateFarHeadingDeg90 = 90.0;
        public double gateFarHeadingDeg270 = 270;

        public double launchCloseX = 21.5;
        public double launchCloseY = 124;
        public double launchCloseHeading = 324;

        public double gateCloseX = 24;
        public double gateCloseY = 87;

        public double gateCloseHeading = 270;
        public double parkingArtifactsHeading270 = 270;

        ArrayList<Pose> parkingControlPoints = new ArrayList<>();
        Pose gateFarControlPoint = new Pose(22, 29, 0);
    }

    public static AutoField.Waypoints waypoints = new AutoField.Waypoints();


    public static FieldLayout layoutForAlliance(Alliance alliance) {
        EnumMap<FieldPoint, Pose> layout = new EnumMap<>(FieldPoint.class);
        layout.put(FieldPoint.START_FAR, poseForAlliance(
                waypoints.startX,
                waypoints.startY,
                waypoints.startHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.LAUNCH_FAR, poseForAlliance(
                waypoints.launchFarX,
                waypoints.launchFarY,
                waypoints.launchFarHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.ALLIANCE_WALL_ARTIFACTS_PICKUP, poseForAlliance(
                waypoints.allianceWallArtifactsX,
                waypoints.allianceWallArtifactsY,
                waypoints.allianceWallArtifactsHeadingDeg,
                alliance
        ));
        layout.put(FieldPoint.PARKING_ARTIFACTS_PICKUP_90_DEG, poseForAlliance(
                waypoints.parkingArtifactsX,
                waypoints.parkingArtifactsY,
                waypoints.parkingArtifactsHeadingDeg90,
                alliance
        ));
        layout.put(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_90_DEG, poseForAlliance(
                waypoints.gateFarX,
                waypoints.gateFarY,
                waypoints.gateFarHeadingDeg90,
                alliance
        ));
        layout.put(FieldPoint.LAUNCH_CLOSE, poseForAlliance(
                waypoints.launchCloseX,
                waypoints.launchCloseY,
                waypoints.launchCloseHeading,
                alliance

        ));
        layout.put(FieldPoint.GATE_CLOSE_ARTIFACTS_PICKUP, poseForAlliance(
                waypoints.gateCloseX,
                waypoints.gateCloseY,
                waypoints.gateCloseHeading,
                alliance
        ));
        layout.put(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(
                waypoints.gateFarX,
                waypoints.gateFarY,
                waypoints.gateFarHeadingDeg270,
                alliance

        ));
        layout.put(FieldPoint.PARKING_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(
                waypoints.parkingArtifactsX,
                waypoints.parkingArtifactsY,
                waypoints.parkingArtifactsHeading270,
                alliance

        ));
        return new FieldLayout(layout);
    }

    public static Pose parkingArtifactsControlPoint(Alliance alliance) {
        return poseForAlliance(
                waypoints.parkingArtifactsControlPointX,
                waypoints.parkingArtifactsControlPointY,
                waypoints.parkingArtifactsHeadingDeg90,

                alliance
        );
    }
    public static Pose gateFarArtifactsControlPoint(Alliance alliance) {
        return poseForAlliance(
                waypoints.gateFarControlPoint.getX(),
                waypoints.gateFarControlPoint.getY(),
                waypoints.gateFarHeadingDeg90,
                alliance
        );
    }


    private static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
        Pose base = new Pose(x, y, Math.toRadians(headingDeg));
        if (alliance == Alliance.RED) {
            double mirroredX = FieldConstants.FIELD_WIDTH_INCHES  - base.getX();
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
