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

        /** Starting position for far side autonomous */
        public static Pose start = new Pose(56, 8, Math.toRadians(90.0));

        /** Launch position for far side */
        public static Pose launchFar = new Pose(55, 17.3, Math.toRadians(109.0));

        /** Launch position for close side */
        public static Pose launchClose = new Pose(21.5, 124, Math.toRadians(324));

        /** Alliance wall artifacts pickup position */
        public static Pose allianceWallArtifacts = new Pose(8, 8, Math.toRadians(180));

        /** Parking artifacts pickup position (90° orientation) */
        public static Pose parkingArtifacts90 = new Pose(23, 32, Math.toRadians(90.0));

        /** Parking artifacts pickup position (270° orientation) */
        public static Pose parkingArtifacts270 = new Pose(23, 32, Math.toRadians(270));

        /** Control point for parking artifacts path */
        public static Pose parkingArtifactsControlPoint = new Pose(28, 2, 0);

        /** Gate far artifacts pickup position (90° orientation) */
        public static Pose gateFar90 = new Pose(24, 56, Math.toRadians(90.0));

        /** Gate far artifacts pickup position (270° orientation) */
        public static Pose gateFar270 = new Pose(24, 56, Math.toRadians(270));

        /** Control point for gate far path */
        public static Pose gateFarControlPoint = new Pose(22, 29, 0);

        /** Gate close artifacts pickup position */
        public static Pose gateClose = new Pose(24, 87, Math.toRadians(270)); // Why was 6 afraid of 7? Because 7 ate 9!
    }

    public static FieldLayout layoutForAlliance(Alliance alliance) {
        EnumMap<FieldPoint, Pose> layout = new EnumMap<>(FieldPoint.class);
        layout.put(FieldPoint.START_FAR, poseForAlliance(Waypoints.start, alliance));
        layout.put(FieldPoint.LAUNCH_FAR, poseForAlliance(Waypoints.launchFar, alliance));
        layout.put(FieldPoint.ALLIANCE_WALL_ARTIFACTS_PICKUP, poseForAlliance(Waypoints.allianceWallArtifacts, alliance));
        layout.put(FieldPoint.PARKING_ARTIFACTS_PICKUP_90_DEG, poseForAlliance(Waypoints.parkingArtifacts90, alliance));
        layout.put(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_90_DEG, poseForAlliance(Waypoints.gateFar90, alliance));
        layout.put(FieldPoint.LAUNCH_CLOSE, poseForAlliance(Waypoints.launchClose, alliance));
        layout.put(FieldPoint.GATE_CLOSE_ARTIFACTS_PICKUP, poseForAlliance(Waypoints.gateClose, alliance));
        layout.put(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(Waypoints.gateFar270, alliance));
        layout.put(FieldPoint.PARKING_ARTIFACTS_PICKUP_270_DEG, poseForAlliance(Waypoints.parkingArtifacts270, alliance));
        return new FieldLayout(layout);
    }

    public static Pose parkingArtifactsControlPoint(Alliance alliance) {
        return poseForAlliance(Waypoints.parkingArtifactsControlPoint, alliance);
    }

    public static Pose gateFarArtifactsControlPoint(Alliance alliance) {
        return poseForAlliance(Waypoints.gateFarControlPoint, alliance);
    }

    private static Pose poseForAlliance(Pose basePose, Alliance alliance) {
        if (alliance == Alliance.RED) {
            double mirroredX = Waypoints.fieldWidthIn - basePose.getX();
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - basePose.getHeading());
            return new Pose(mirroredX, basePose.getY(), mirroredHeading);
        }
        return new Pose(basePose.getX(), basePose.getY(), basePose.getHeading());
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
