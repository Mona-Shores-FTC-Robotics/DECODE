package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Vision subsystem telemetry data.
 * Includes AprilTag detection and pose estimation data.
 */
public class VisionTelemetryData {
    // Tag detection
    public final boolean hasTag;
    public final int tagId;
    public final Alliance alliance;
    public final boolean odometryPending;

    // Tag geometry
    public final double rangeIn;
    public final double bearingDeg;
    public final double yawDeg;

    // Raw Limelight values
    public final double txDeg;
    public final double tyDeg;
    public final double targetAreaPercent;

    // Pose from tag (if available)
    public final double poseXIn;
    public final double poseYIn;
    public final double headingRad;

    public VisionTelemetryData(
            boolean hasTag,
            int tagId,
            Alliance alliance,
            boolean odometryPending,
            double rangeIn,
            double bearingDeg,
            double yawDeg,
            double txDeg,
            double tyDeg,
            double targetAreaPercent,
            double poseXIn,
            double poseYIn,
            double headingRad
    ) {
        this.hasTag = hasTag;
        this.tagId = tagId;
        this.alliance = alliance != null ? alliance : Alliance.UNKNOWN;
        this.odometryPending = odometryPending;
        this.rangeIn = rangeIn;
        this.bearingDeg = bearingDeg;
        this.yawDeg = yawDeg;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.targetAreaPercent = targetAreaPercent;
        this.poseXIn = poseXIn;
        this.poseYIn = poseYIn;
        this.headingRad = headingRad;
    }

    public static VisionTelemetryData capture(VisionSubsystemLimelight vision, Alliance allianceFallback) {
        if (vision == null) {
            return new VisionTelemetryData(
                    false, -1, allianceFallback, false,
                    Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN
            );
        }

        boolean hasTag = vision.hasValidTag();
        int tagId = hasTag ? vision.getCurrentTagId() : -1;
        Alliance alliance = vision.getAlliance();
        if (alliance == Alliance.UNKNOWN) {
            alliance = allianceFallback;
        }
        boolean odometryPending = vision.shouldUpdateOdometry();

        VisionSubsystemLimelight.TagSnapshot snapshot = vision.getLastSnapshot().orElse(null);
        if (snapshot != null) {
            double yawDeg = snapshot.getFtcYaw();
            double headingRad = Double.isNaN(yawDeg) ? Double.NaN : Math.toRadians(yawDeg);

            return new VisionTelemetryData(
                    hasTag,
                    tagId,
                    alliance,
                    odometryPending,
                    snapshot.getFtcRange(),
                    snapshot.getFtcBearing(),
                    yawDeg,
                    snapshot.getTxDegrees(),
                    snapshot.getTyDegrees(),
                    snapshot.getTargetAreaPercent(),
                    snapshot.getFtcPose().get().getX(),
                    snapshot.getFtcPose().get().getY(),
                    headingRad
            );
        } else {
            return new VisionTelemetryData(
                    hasTag, tagId, alliance, odometryPending,
                    Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN
            );
        }
    }
}
