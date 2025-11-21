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
        //Far Auto Points
        START_FAR,
        LAUNCH_FAR,
        ALLIANCE_WALL_ARTIFACTS_PICKUP,
        PARKING_ARTIFACTS_PICKUP_90_DEG,
        GATE_FAR_ARTIFACTS_PICKUP_90_DEG,

        //Close Auto Points
        START_CLOSE,
        LAUNCH_CLOSE,
        PRE_GATE_ARTIFACTS_PICKUP_270_DEG,
        ARTIFACTS_SET_1_270,
        ARTIFACTS_SET_2_270,
        ARTIFACTS_SET_3_270,
        MOVE_TO_GATE
        }


@Configurable
public static class Waypoints {

        public double startFarX = 56;
        public double startFarY = 8;
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


        //Auto Close
         public double startCloseX = 26.445;
        public double startCloseY = 131.374;
        public double startCloseHeading = 144;

        public double launchCloseX = 30.19905213270142;
        public double launchCloseY = 112.9478672985782;
        public double launchCloseHeading = 136;

        public double preGateArtifactsX = 23.886255924170616;
        public double preGateArtifactsY = 99.4691943127962;
        public double preGateArtifactsHeading270 = 270;

        public double artifactSet1X = 23.886255924170616;
        public double artifactSet1Y = 80.3601895734597;
        public double artifactSet1Heading = 270;

        public double artifactSet2X = 23.886255924170616;
        public double artifactSet2Y = 80.3601895734597;
        public double artifactSet2Heading = 270;
        public double artifactSet2ControlX = 23.203791469194314;
        public double artifactSet2ControlY = 94.69194312796208;

        public double artifactSet3X = 23.886255924170616;
        public double artifactSet3Y = 41;
        public double artifactSet3Heading = 270;
        public double artifactSet3ControlX = 24.398104265402843;
        public double artifactSet3ControlY = 56.4739336492891;

        public double moveToGateX = 23.886255924170616;
        public double moveToGateY = 70.63507109004739;
        public double moveToGateHeading  = 180;

        ArrayList<Pose> parkingControlPoints = new ArrayList<>();
        Pose gateFarControlPoint = new Pose(22, 29, 0);

    }

    public static AutoField.Waypoints waypoints = new AutoField.Waypoints();


    public static FieldLayout layoutForAlliance(Alliance alliance) {
        EnumMap<FieldPoint, Pose> layout = new EnumMap<>(FieldPoint.class);
        layout.put(FieldPoint.START_FAR, poseForAlliance(
                waypoints.startFarX,
                waypoints.startFarY,
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

        layout.put(FieldPoint.START_CLOSE, poseForAlliance(
                waypoints.startCloseX,
                waypoints.startCloseY,
                waypoints.startCloseHeading,
                alliance
        ));

        layout.put(FieldPoint.LAUNCH_CLOSE, poseForAlliance(
                waypoints.launchCloseX,
                waypoints.launchCloseY,
                waypoints.launchCloseHeading,
                alliance
        ));

        layout.put(FieldPoint.PRE_GATE_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(
                waypoints.preGateArtifactsX,
                waypoints.preGateArtifactsY,
                waypoints.preGateArtifactsHeading270,
                alliance
        ));

        layout.put(FieldPoint.ARTIFACTS_SET_1_270, poseForAlliance(
                waypoints.artifactSet1X,
                waypoints.artifactSet1Y,
                waypoints.artifactSet1Heading,
                alliance
        ));

        layout.put(FieldPoint.ARTIFACTS_SET_2_270, poseForAlliance(
                waypoints.artifactSet2X,
                waypoints.artifactSet2Y,
                waypoints.artifactSet2Heading,
                alliance
        ));

        layout.put(FieldPoint.ARTIFACTS_SET_3_270, poseForAlliance(
                waypoints.artifactSet3X,
                waypoints.artifactSet3Y,
                waypoints.artifactSet3Heading,
                alliance
        ));

        layout.put(FieldPoint.MOVE_TO_GATE, poseForAlliance(
                waypoints.moveToGateX,
                waypoints.moveToGateY,
                waypoints.moveToGateHeading,
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


    public static Pose artifactsSet2ControlPoint(Alliance alliance) {
        return poseForAlliance(
                waypoints.artifactSet2ControlX,
                waypoints.artifactSet2ControlY,
                waypoints.artifactSet2Heading,
                alliance
        );
    }

    public static Pose artifactSet3ControlPoint(Alliance alliance) {
        return poseForAlliance(
                waypoints.artifactSet3ControlX,
                waypoints.artifactSet3ControlY,
                waypoints.artifactSet3Heading,
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
