package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private final Localizer deadReckoning;
    private Pose currentPosition;
    private Pose currentVelocity;
    private Matrix P;
    private final Matrix Q;
    private final Matrix R;
    private long lastUpdateTime = -1;
    private final NavigableMap<Long, Pose> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, Pose> twistHistory = new TreeMap<>();
    private final NavigableMap<Long, Matrix> covarianceHistory = new TreeMap<>();
    private final int bufferSize;

    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();
        this.P = Matrix.diag(initialCovariance.getX(), initialCovariance.getY(), initialCovariance.getHeading());
        this.Q = Matrix.diag(processVariance.getX(), processVariance.getY(), processVariance.getHeading());
        this.R = Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());
        this.bufferSize = bufferSize;
        twistHistory.put(0L, new Pose());
    }

    @Override
    public void update() {
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        Pose twist = deadReckoning.getVelocity();
        twistHistory.put(now, twist.copy());
        currentVelocity = twist.copy();

        currentPosition = integrate(currentPosition, twist, dt);
        updateCovariance(dt);

        poseHistory.put(now, currentPosition.copy());
        covarianceHistory.put(now, P.copy());
        if (poseHistory.size() > bufferSize) poseHistory.pollFirstEntry();
        if (twistHistory.size() > bufferSize) twistHistory.pollFirstEntry();
        if (covarianceHistory.size() > bufferSize) covarianceHistory.pollFirstEntry();
    }

    private void updateCovariance(double dt) {
        Matrix G = Matrix.createRotation(getPose().getHeading()).multiply(dt);
        P = P.plus(G.multiply(Q.multiply(G.transposed())));
    }

    public void addMeasurement(Pose measuredPose, long timestamp) {
        addMeasurement(measuredPose, timestamp, null);
    }

    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        Matrix measurementR = measurementVariance == null
                ? R
                : Matrix.diag(measurementVariance.getX(), measurementVariance.getY(), measurementVariance.getHeading());

        if (poseHistory.isEmpty() || timestamp < poseHistory.firstKey() || timestamp > poseHistory.lastKey())
            return;

        Pose pastPose = interpolate(timestamp, poseHistory);
        if (pastPose == null)
            pastPose = getPose();

        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());
        // Heading comes from Pinpoint (MT2 requires it as input, so correcting it
        // with MT2 output would be circular). getTotalHeading() delegates to
        // deadReckoning, so KF heading corrections would also be silently ignored.
        boolean measH = false;

        Matrix y = new Matrix(new double[][]{
                {measX ? measuredPose.getX() - pastPose.getX() : 0},
                {measY ? measuredPose.getY() - pastPose.getY() : 0},
                {measH ? MathFunctions.normalizeAngleSigned(measuredPose.getHeading() - pastPose.getHeading()) : 0}
        });

        Matrix M = Matrix.diag(
                measX ? 1 : 0,
                measY ? 1 : 0,
                measH ? 1 : 0
        );

        Matrix Pm = covarianceHistory.floorEntry(timestamp).getValue();
        Matrix S = Pm.plus(measurementR);
        Matrix K = Pm.multiply(S.inverse());
        K = M.multiply(K);
        y = M.multiply(y);

        Matrix Ky = K.multiply(y);
        Pose updatedPast = new Pose(
                pastPose.getX() + Ky.get(0, 0),
                pastPose.getY() + Ky.get(1, 0),
                MathFunctions.normalizeAngle(pastPose.getHeading() + Ky.get(2, 0))
        );
        poseHistory.put(timestamp, updatedPast);

        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K);
        Matrix updatedCovariance =
                IK.multiply(Pm).multiply(IK.transposed())
                        .plus(K.multiply(measurementR).multiply(K.transposed()));
        covarianceHistory.put(timestamp, updatedCovariance);

        long prevTime = timestamp;
        Pose prevPose = updatedPast;
        Matrix prevCov = updatedCovariance;

        // Snapshot keys before the loop — modifying poseHistory/covarianceHistory
        // while iterating a live tailMap view causes ConcurrentModificationException.
        Long[] replayKeys = poseHistory.tailMap(timestamp, false).keySet().toArray(new Long[0]);
        for (long t : replayKeys) {
            Pose twist = interpolate(t, twistHistory);
            if (twist == null)
                twist = getVelocity();
            double dt = (t - prevTime) / 1e9;
            Pose nextPose = integrate(prevPose, twist, dt);
            poseHistory.put(t, nextPose);
            Matrix G = Matrix.createRotation(prevPose.getHeading()).multiply(dt);
            prevCov = prevCov.plus(G.multiply(Q.multiply(G.transposed())));
            covarianceHistory.put(t, prevCov);
            prevPose = nextPose;
            prevTime = t;
        }

        currentPosition = poseHistory.lastEntry().getValue().copy();
        P = covarianceHistory.lastEntry().getValue().copy();
    }

    private static Pose interpolate(long timestamp, NavigableMap<Long, Pose> history) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);
        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey).copy();
        Pose lowerPose = history.get(lowerKey);
        Pose upperPose = history.get(upperKey);
        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);
        double x = lowerPose.getX() + ratio * (upperPose.getX() - lowerPose.getX());
        double y = lowerPose.getY() + ratio * (upperPose.getY() - lowerPose.getY());
        double headingDiff = MathFunctions.getSmallestAngleDifference(upperPose.getHeading(), lowerPose.getHeading());
        double heading = MathFunctions.normalizeAngle(lowerPose.getHeading() + ratio * headingDiff);
        return new Pose(x, y, heading);
    }

    private Pose integrate(Pose previousPose, Pose twist, double dt) {
        double dx = twist.getX() * dt;
        double dy = twist.getY() * dt;
        double dTheta = twist.getHeading() * dt;
        return new Pose(
                previousPose.getX() + dx,
                previousPose.getY() + dy,
                MathFunctions.normalizeAngle(previousPose.getHeading() + dTheta)
        );
    }

    @Override
    public Pose getPose() { return currentPosition.withHeading(deadReckoning.getPose().getHeading()); }

    @Override
    public Pose getVelocity() {
        return currentVelocity != null ? currentVelocity : deadReckoning.getVelocity();
    }

    @Override
    public Vector getVelocityVector() { return getVelocity().getAsVector(); }

    @Override
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        poseHistory.put(0L, setStart.copy());
        covarianceHistory.put(0L, P.copy());
        currentPosition = setStart.copy();
    }

    @Override
    public void setPose(Pose setPose) {
        currentPosition = setPose.copy();
        deadReckoning.setPose(setPose);
        if (poseHistory.lastEntry() != null)
            poseHistory.put(poseHistory.lastKey(), setPose.copy());
        else
            setStartPose(setPose);
    }

    /**
     * Driver-asserted relocalization. Snaps position AND resets the Kalman
     * covariance matrix P to a confident estimate, signaling "trust this
     * position going forward." Use from manual relocalize buttons; do NOT
     * call from Pedro lifecycle code — {@link #setPose(Pose)} is the
     * Pedro-conformant path and intentionally leaves covariance alone
     * (Pedro calls it during init when the filter hasn't built up
     * uncertainty yet anyway).
     *
     * Also rewrites the last covariance history entry so a vision
     * measurement arriving immediately after this call doesn't apply
     * a stale pre-press covariance during its time-correction step.
     *
     * @param pose             new position (also propagated to deadReckoning)
     * @param xyCovariance     variance for x and y after the reset
     * @param headingCovariance variance for heading after the reset (radians)
     */
    public void forceRelocalize(Pose pose, double xyCovariance, double headingCovariance) {
        setPose(pose);
        P = Matrix.diag(xyCovariance, xyCovariance, headingCovariance);
        if (!covarianceHistory.isEmpty()) {
            covarianceHistory.put(covarianceHistory.lastKey(), P.copy());
        }
    }

    @Override
    public double getTotalHeading() { return deadReckoning.getTotalHeading(); }

    @Override
    public double getForwardMultiplier() { return deadReckoning.getForwardMultiplier(); }

    @Override
    public double getLateralMultiplier() { return deadReckoning.getLateralMultiplier(); }

    @Override
    public double getTurningMultiplier() { return deadReckoning.getTurningMultiplier(); }

    @Override
    public void resetIMU() throws InterruptedException { deadReckoning.resetIMU(); }

    @Override
    public double getIMUHeading() { return deadReckoning.getIMUHeading(); }

    /** Debug: the filter's raw fused position (before the getPose() heading override). */
    public Pose debugCurrentPosition() { return currentPosition.copy(); }

    /** Debug: the dead-reckoning (Pinpoint) pose this filter wraps. */
    public Pose debugDeadReckoningPose() { return deadReckoning.getPose(); }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPosition.getX()) || Double.isNaN(currentPosition.getY()) || Double.isNaN(currentPosition.getHeading());
    }
}
