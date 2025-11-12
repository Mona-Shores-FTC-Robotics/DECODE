package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

/**
 * Lightweight sensor fusion layer that blends Pinpoint odometry with AprilTag vision snapshots.
 * The fused pose is not yet used for control, but the estimates (and supporting diagnostics)
 * are exposed through {@link RobotLogger} so AdvantageScope can chart the combined solution.
 */
public class PoseFusion {

    @Configurable
    public static class Config {
        /** Base trust assigned to any AprilTag measurement before scaling. */
        public static double visionBaseTrust = 0.35;
        /** Larger values increase how quickly vision confidence decays with distance. */
        public static double visionDistanceFalloff = 0.015;
        /**
         * Blend factor applied to the Limelight decision margin (0 = ignore, 1 = fully scale).
         * Useful when balancing pose updates from close but ambiguous tags.
         */
        public static double visionDecisionScale = 0.6;
        /** Minimum weight applied to any accepted vision measurement. */
        public static double minVisionTrust = 0.05;
        /** Maximum weight applied to any accepted vision measurement. */
        public static double maxVisionTrust = 0.85;
        /** Additional scaling applied to heading corrections relative to translation updates. */
        public static double headingTrustScale = 0.25;
        /** Guard-rail to reject obviously invalid translation corrections. */
        public static double outlierTranslationThresholdIn = 24.0;
        /** Guard-rail to reject obviously invalid heading corrections. */
        public static double outlierHeadingThresholdDeg = 25.0;
    }

    /** Snapshot returned to loggers so that diagnostics stay immutable for a given loop. */
    public static final class State {
        public double fusedXInches = Double.NaN;
        public double fusedYInches = Double.NaN;
        public double fusedHeadingRad = Double.NaN;

        public double odometryXInches = Double.NaN;
        public double odometryYInches = Double.NaN;
        public double odometryHeadingRad = Double.NaN;

        public double lastVisionXInches = Double.NaN;
        public double lastVisionYInches = Double.NaN;
        public double lastVisionHeadingRad = Double.NaN;
        public double lastVisionRangeInches = Double.NaN;
        public double lastVisionDecisionMargin = Double.NaN;

        public double lastVisionWeight = 0.0;
        public double lastVisionTranslationErrorInches = Double.NaN;
        public double lastVisionHeadingErrorDeg = Double.NaN;
        public double lastOdometryDtMs = Double.NaN;
        public double ageOfLastVisionMs = Double.POSITIVE_INFINITY;

        public boolean lastVisionAccepted = false;
        public boolean hasFusedPose = false;

        public long fusedTimestampMs = 0L;
        public long odometryTimestampMs = 0L;
        public long visionTimestampMs = 0L;
    }

    private final State state = new State();
    private Pose fusedPose;
    private Pose lastOdometryPose;

    public synchronized void reset(Pose pose, long timestampMs) {
        if (pose == null) {
            fusedPose = null;
            lastOdometryPose = null;
            resetState(timestampMs);
            return;
        }
        fusedPose = copyPose(pose);
        lastOdometryPose = copyPose(pose);
        state.hasFusedPose = true;
        state.fusedXInches = pose.getX();
        state.fusedYInches = pose.getY();
        state.fusedHeadingRad = pose.getHeading();
        state.fusedTimestampMs = timestampMs;
        state.odometryXInches = pose.getX();
        state.odometryYInches = pose.getY();
        state.odometryHeadingRad = pose.getHeading();
        state.odometryTimestampMs = timestampMs;
        state.lastOdometryDtMs = 0.0;
        state.ageOfLastVisionMs = state.visionTimestampMs == 0L
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0, timestampMs - state.visionTimestampMs);
    }

    public synchronized Pose updateWithOdometry(Pose odometryPose, long timestampMs) {
        if (odometryPose == null) {
            return fusedPose;
        }

        double x = odometryPose.getX();
        double y = odometryPose.getY();
        double heading = odometryPose.getHeading();
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            return fusedPose;
        }

        if (lastOdometryPose == null || fusedPose == null) {
            fusedPose = copyPose(odometryPose);
            lastOdometryPose = copyPose(odometryPose);
            state.hasFusedPose = true;
        } else {
            double dx = x - lastOdometryPose.getX();
            double dy = y - lastOdometryPose.getY();
            double dHeading = wrapRadians(heading - lastOdometryPose.getHeading());
            fusedPose = new Pose(
                    fusedPose.getX() + dx,
                    fusedPose.getY() + dy,
                    wrapRadians(fusedPose.getHeading() + dHeading)
            );
            lastOdometryPose = copyPose(odometryPose);
        }

        double dtMs = state.odometryTimestampMs == 0L ? 0.0 : (timestampMs - state.odometryTimestampMs);
        state.lastOdometryDtMs = dtMs;
        state.odometryTimestampMs = timestampMs;
        state.odometryXInches = x;
        state.odometryYInches = y;
        state.odometryHeadingRad = heading;

        if (fusedPose != null) {
            state.hasFusedPose = true;
            state.fusedXInches = fusedPose.getX();
            state.fusedYInches = fusedPose.getY();
            state.fusedHeadingRad = fusedPose.getHeading();
            state.fusedTimestampMs = timestampMs;
        }

        state.ageOfLastVisionMs = state.visionTimestampMs == 0L
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0, timestampMs - state.visionTimestampMs);
        return fusedPose;
    }

    public synchronized Pose addVisionMeasurement(Pose visionPose,
                                                   long timestampMs,
                                                   double rangeInches,
                                                   double decisionMargin) {
        state.lastVisionAccepted = false;
        if (visionPose == null || fusedPose == null) {
            cacheVisionMeta(visionPose, timestampMs, rangeInches, decisionMargin);
            state.lastVisionWeight = 0.0;
            return fusedPose;
        }

        double vx = visionPose.getX();
        double vy = visionPose.getY();
        double vh = visionPose.getHeading();
        if (!Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(vh)) {
            cacheVisionMeta(visionPose, timestampMs, rangeInches, decisionMargin);
            state.lastVisionWeight = 0.0;
            return fusedPose;
        }

        cacheVisionMeta(visionPose, timestampMs, rangeInches, decisionMargin);

        double translationError = Math.hypot(vx - fusedPose.getX(), vy - fusedPose.getY());
        double headingErrorDeg = Math.toDegrees(wrapRadians(vh - fusedPose.getHeading()));
        state.lastVisionTranslationErrorInches = translationError;
        state.lastVisionHeadingErrorDeg = headingErrorDeg;

        if (Double.isFinite(Config.outlierTranslationThresholdIn)
                && translationError > Config.outlierTranslationThresholdIn) {
            state.lastVisionWeight = 0.0;
            return fusedPose;
        }
        if (Double.isFinite(Config.outlierHeadingThresholdDeg)
                && Math.abs(headingErrorDeg) > Config.outlierHeadingThresholdDeg) {
            state.lastVisionWeight = 0.0;
            return fusedPose;
        }

        double weight = computeVisionWeight(rangeInches, decisionMargin);
        double headingWeight = Range.clip(weight * Config.headingTrustScale, 0.0, 1.0);
        state.lastVisionWeight = weight;
        state.lastVisionAccepted = weight > 0.0;

        double fusedX = lerp(fusedPose.getX(), vx, weight);
        double fusedY = lerp(fusedPose.getY(), vy, weight);
        double headingDelta = wrapRadians(vh - fusedPose.getHeading());
        double fusedHeading = wrapRadians(fusedPose.getHeading() + headingWeight * headingDelta);

        fusedPose = new Pose(fusedX, fusedY, fusedHeading);
        state.hasFusedPose = true;
        state.fusedXInches = fusedX;
        state.fusedYInches = fusedY;
        state.fusedHeadingRad = fusedHeading;
        state.fusedTimestampMs = timestampMs;
        return fusedPose;
    }

    public synchronized State getStateSnapshot() {
        State snapshot = new State();
        snapshot.fusedXInches = state.fusedXInches;
        snapshot.fusedYInches = state.fusedYInches;
        snapshot.fusedHeadingRad = state.fusedHeadingRad;
        snapshot.odometryXInches = state.odometryXInches;
        snapshot.odometryYInches = state.odometryYInches;
        snapshot.odometryHeadingRad = state.odometryHeadingRad;
        snapshot.lastVisionXInches = state.lastVisionXInches;
        snapshot.lastVisionYInches = state.lastVisionYInches;
        snapshot.lastVisionHeadingRad = state.lastVisionHeadingRad;
        snapshot.lastVisionRangeInches = state.lastVisionRangeInches;
        snapshot.lastVisionDecisionMargin = state.lastVisionDecisionMargin;
        snapshot.lastVisionWeight = state.lastVisionWeight;
        snapshot.lastVisionTranslationErrorInches = state.lastVisionTranslationErrorInches;
        snapshot.lastVisionHeadingErrorDeg = state.lastVisionHeadingErrorDeg;
        snapshot.lastOdometryDtMs = state.lastOdometryDtMs;
        snapshot.ageOfLastVisionMs = state.ageOfLastVisionMs;
        snapshot.lastVisionAccepted = state.lastVisionAccepted;
        snapshot.hasFusedPose = state.hasFusedPose;
        snapshot.fusedTimestampMs = state.fusedTimestampMs;
        snapshot.odometryTimestampMs = state.odometryTimestampMs;
        snapshot.visionTimestampMs = state.visionTimestampMs;
        return snapshot;
    }

    private void cacheVisionMeta(Pose visionPose,
                                 long timestampMs,
                                 double rangeInches,
                                 double decisionMargin) {
        if (visionPose != null) {
            state.lastVisionXInches = visionPose.getX();
            state.lastVisionYInches = visionPose.getY();
            state.lastVisionHeadingRad = visionPose.getHeading();
        } else {
            state.lastVisionXInches = Double.NaN;
            state.lastVisionYInches = Double.NaN;
            state.lastVisionHeadingRad = Double.NaN;
        }
        state.lastVisionRangeInches = rangeInches;
        state.lastVisionDecisionMargin = decisionMargin;
        state.visionTimestampMs = timestampMs;
        state.ageOfLastVisionMs = 0.0;
    }

    private void resetState(long timestampMs) {
        state.hasFusedPose = false;
        state.fusedXInches = Double.NaN;
        state.fusedYInches = Double.NaN;
        state.fusedHeadingRad = Double.NaN;
        state.fusedTimestampMs = timestampMs;
        state.odometryXInches = Double.NaN;
        state.odometryYInches = Double.NaN;
        state.odometryHeadingRad = Double.NaN;
        state.odometryTimestampMs = timestampMs;
        state.lastVisionXInches = Double.NaN;
        state.lastVisionYInches = Double.NaN;
        state.lastVisionHeadingRad = Double.NaN;
        state.lastVisionRangeInches = Double.NaN;
        state.lastVisionDecisionMargin = Double.NaN;
        state.lastVisionWeight = 0.0;
        state.lastVisionTranslationErrorInches = Double.NaN;
        state.lastVisionHeadingErrorDeg = Double.NaN;
        state.lastVisionAccepted = false;
        state.lastOdometryDtMs = Double.NaN;
        state.ageOfLastVisionMs = Double.POSITIVE_INFINITY;
        state.visionTimestampMs = 0L;
    }

    private static Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private static double wrapRadians(double angle) {
        double wrapped = angle;
        while (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        while (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    private static double lerp(double start, double end, double t) {
        double weight = Range.clip(Double.isNaN(t) ? 0.0 : t, 0.0, 1.0);
        return start + (end - start) * weight;
    }

    private static double computeVisionWeight(double rangeInches, double decisionMargin) {
        double weight = Config.visionBaseTrust;
        if (Double.isFinite(rangeInches) && rangeInches > 0.0) {
            double clampedRange = Range.clip(rangeInches, 0.0, 500.0);
            weight *= Math.exp(-Config.visionDistanceFalloff * clampedRange);
        }
        if (Double.isFinite(decisionMargin)) {
            double margin = Range.clip(decisionMargin, 0.0, 1.0);
            double mix = Range.clip(Config.visionDecisionScale, 0.0, 1.0);
            weight *= mix * margin + (1.0 - mix);
        }
        weight = Range.clip(weight, Config.minVisionTrust, Config.maxVisionTrust);
        if (!Double.isFinite(weight)) {
            return 0.0;
        }
        return Math.max(weight, 0.0);
    }
}

