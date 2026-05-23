package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class PoseFusionTest {

    private static final double EPS = 1e-9;
    private static final double EPS_LOOSE = 1e-6;

    private PoseFusion fusion;

    @Before
    public void setUp() {
        PoseFusion.Config.visionBaseTrust = 0.35;
        PoseFusion.Config.visionDistanceFalloff = 0.015;
        PoseFusion.Config.visionDecisionScale = 0.6;
        PoseFusion.Config.minVisionTrust = 0.05;
        PoseFusion.Config.maxVisionTrust = 0.85;
        PoseFusion.Config.headingTrustScale = 0.25;
        PoseFusion.Config.outlierTranslationThresholdIn = 24.0;
        PoseFusion.Config.outlierHeadingThresholdDeg = 25.0;
        fusion = new PoseFusion();
    }

    // -------------------------------------------------------------------------
    // reset()
    // -------------------------------------------------------------------------

    @Test
    public void reset_validPose_initializesAllStateFields() {
        fusion.reset(new Pose(5.0, 3.0, 1.0), 1000L);
        PoseFusion.State s = fusion.getStateSnapshot();

        assertTrue(s.hasFusedPose);
        assertEquals(5.0, s.fusedXInches, EPS);
        assertEquals(3.0, s.fusedYInches, EPS);
        assertEquals(1.0, s.fusedHeadingRad, EPS);
        assertEquals(5.0, s.odometryXInches, EPS);
        assertEquals(3.0, s.odometryYInches, EPS);
        assertEquals(1.0, s.odometryHeadingRad, EPS);
        assertEquals(1000L, s.fusedTimestampMs);
        assertEquals(0.0, s.lastOdometryDtMs, EPS);
        assertEquals(Double.POSITIVE_INFINITY, s.ageOfLastVisionMs, 0.0);
    }

    @Test
    public void reset_null_clearsStateAndHasFusedPoseFalse() {
        fusion.reset(new Pose(10, 10, 0), 500L);
        fusion.reset(null, 1000L);
        PoseFusion.State s = fusion.getStateSnapshot();

        assertFalse(s.hasFusedPose);
        assertTrue(Double.isNaN(s.fusedXInches));
        assertTrue(Double.isNaN(s.fusedYInches));
        assertTrue(Double.isNaN(s.fusedHeadingRad));
    }

    @Test
    public void reset_afterVision_preservesVisionAgeRelativeToNewTimestamp() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 100L, 10.0, 1.0);
        // Re-reset at t=500; vision was at t=100, so age should be 500-100=400
        fusion.reset(new Pose(5, 5, 0), 500L);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertEquals(400.0, s.ageOfLastVisionMs, EPS);
    }

    // -------------------------------------------------------------------------
    // updateWithOdometry()
    // -------------------------------------------------------------------------

    @Test
    public void updateWithOdometry_firstCallWithNoPriorState_initializesFromOdometry() {
        Pose result = fusion.updateWithOdometry(new Pose(7.0, 2.0, 0.5), 100L);

        assertNotNull(result);
        assertEquals(7.0, result.getX(), EPS);
        assertEquals(2.0, result.getY(), EPS);
        assertEquals(0.5, result.getHeading(), EPS);
        assertTrue(fusion.getStateSnapshot().hasFusedPose);
    }

    @Test
    public void updateWithOdometry_afterReset_tracksDeltaExactly() {
        fusion.reset(new Pose(0, 0, 0), 0L);

        fusion.updateWithOdometry(new Pose(10, 5, 0.2), 100L);
        Pose result = fusion.updateWithOdometry(new Pose(20, 8, 0.3), 200L);

        assertEquals(20.0, result.getX(), EPS);
        assertEquals(8.0, result.getY(), EPS);
    }

    @Test
    public void updateWithOdometry_null_returnsPreviousFusedPoseUnchanged() {
        fusion.reset(new Pose(5, 5, 0), 0L);
        Pose result = fusion.updateWithOdometry(null, 100L);

        assertEquals(5.0, result.getX(), EPS);
        assertEquals(5.0, result.getY(), EPS);
    }

    @Test
    public void updateWithOdometry_nanComponent_ignored() {
        fusion.reset(new Pose(5, 5, 0), 0L);
        Pose result = fusion.updateWithOdometry(new Pose(Double.NaN, 5, 0), 100L);

        assertEquals(5.0, result.getX(), EPS);
    }

    @Test
    public void updateWithOdometry_updatesTimestampAndComputesDt() {
        fusion.reset(new Pose(0, 0, 0), 1000L);
        fusion.updateWithOdometry(new Pose(1, 0, 0), 1050L);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertEquals(1050L, s.odometryTimestampMs);
        assertEquals(50.0, s.lastOdometryDtMs, EPS);
    }

    @Test
    public void updateWithOdometry_incrementsVisionAge() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 100L, 10.0, 1.0);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 300L);

        assertEquals(200.0, fusion.getStateSnapshot().ageOfLastVisionMs, EPS);
    }

    // -------------------------------------------------------------------------
    // addVisionMeasurement()
    // -------------------------------------------------------------------------

    @Test
    public void addVisionMeasurement_withNoFusedPose_markedRejectedAndWeightZero() {
        fusion.addVisionMeasurement(new Pose(5, 5, 0), 100L, 20.0, 1.0);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertFalse(s.lastVisionAccepted);
        assertEquals(0.0, s.lastVisionWeight, EPS);
    }

    @Test
    public void addVisionMeasurement_withinThresholds_blendedTowardVision() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(10, 10, 0), 100L);
        // Vision says robot is 2 inches off in X
        fusion.addVisionMeasurement(new Pose(12, 10, 0), 150L, 20.0, 1.0);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertTrue(s.lastVisionAccepted);
        assertTrue(s.fusedXInches > 10.0 && s.fusedXInches < 12.0);
        assertEquals(10.0, s.fusedYInches, EPS_LOOSE);
    }

    @Test
    public void addVisionMeasurement_weightMatchesExpectedFormula() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 150L, 20.0, 1.0);

        // weight = clip(0.35 * exp(-0.015*20) * (0.6*1.0 + 0.4), 0.05, 0.85)
        //        = clip(0.35 * 0.74082 * 1.0, 0.05, 0.85) ≈ 0.2593
        double expected = Math.min(0.85, Math.max(0.05,
                0.35 * Math.exp(-0.015 * 20) * (0.6 * 1.0 + 0.4)));
        assertEquals(expected, fusion.getStateSnapshot().lastVisionWeight, 1e-6);
    }

    @Test
    public void addVisionMeasurement_headingCorrectionScaledByTrustFactor() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);

        double visionHeading = Math.toRadians(10.0);
        fusion.addVisionMeasurement(new Pose(0, 0, visionHeading), 150L, 10.0, 1.0);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertTrue(s.lastVisionAccepted);

        double w = s.lastVisionWeight;
        double headingW = Math.min(w * PoseFusion.Config.headingTrustScale, 1.0);
        double expectedHeading = headingW * visionHeading; // start heading was 0
        assertEquals(expectedHeading, s.fusedHeadingRad, 1e-8);
        // Heading correction must be smaller than full weight
        assertTrue(s.fusedHeadingRad < w * visionHeading);
    }

    @Test
    public void addVisionMeasurement_translationOutlier_rejected() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        // 25 inches away; threshold is 24
        fusion.addVisionMeasurement(new Pose(25, 0, 0), 150L, 25.0, 1.0);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertFalse(s.lastVisionAccepted);
        assertEquals(0.0, s.lastVisionWeight, EPS);
        assertEquals(0.0, s.fusedXInches, EPS);
    }

    @Test
    public void addVisionMeasurement_exactlyAtTranslationThreshold_accepted() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        // Exactly 24 inches: condition is >, not >=, so this should pass
        fusion.addVisionMeasurement(new Pose(24, 0, 0), 150L, 24.0, 1.0);

        assertTrue(fusion.getStateSnapshot().lastVisionAccepted);
    }

    @Test
    public void addVisionMeasurement_headingOutlier_rejected() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        // 30° heading error; threshold is 25°
        fusion.addVisionMeasurement(new Pose(0, 0, Math.toRadians(30)), 150L, 10.0, 1.0);

        assertFalse(fusion.getStateSnapshot().lastVisionAccepted);
    }

    @Test
    public void addVisionMeasurement_farRange_lowerWeightThanNearRange() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 150L, 5.0, 1.0);
        double nearWeight = fusion.getStateSnapshot().lastVisionWeight;

        fusion = new PoseFusion();
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 150L, 100.0, 1.0);
        double farWeight = fusion.getStateSnapshot().lastVisionWeight;

        assertTrue("Near range must produce higher weight than far range", nearWeight > farWeight);
    }

    @Test
    public void addVisionMeasurement_lowDecisionMargin_reducesWeight() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 150L, 20.0, 1.0);
        double highMarginWeight = fusion.getStateSnapshot().lastVisionWeight;

        fusion = new PoseFusion();
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 150L, 20.0, 0.1);
        double lowMarginWeight = fusion.getStateSnapshot().lastVisionWeight;

        assertTrue("High decision margin must produce higher weight", highMarginWeight > lowMarginWeight);
    }

    @Test
    public void addVisionMeasurement_weightClampedToMinimum_whenVeryFar() {
        PoseFusion.Config.outlierTranslationThresholdIn = Double.POSITIVE_INFINITY;
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 150L, 400.0, 0.0);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertTrue(s.lastVisionAccepted);
        assertTrue(s.lastVisionWeight >= PoseFusion.Config.minVisionTrust);
        assertTrue(s.lastVisionWeight <= PoseFusion.Config.maxVisionTrust);
    }

    @Test
    public void addVisionMeasurement_null_markedRejected_andFusedPoseUnchanged() {
        fusion.reset(new Pose(5, 5, 0), 0L);
        fusion.updateWithOdometry(new Pose(5, 5, 0), 100L);
        fusion.addVisionMeasurement(null, 150L, 20.0, 1.0);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertFalse(s.lastVisionAccepted);
        assertEquals(0.0, s.lastVisionWeight, EPS);
        assertEquals(5.0, s.fusedXInches, EPS);
    }

    @Test
    public void addVisionMeasurement_storesVisionMetaEvenWhenRejectedByOutlier() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        // Outlier — but meta should still be cached
        fusion.addVisionMeasurement(new Pose(30, 7, 0), 200L, 30.0, 0.8);

        PoseFusion.State s = fusion.getStateSnapshot();
        assertFalse(s.lastVisionAccepted);
        assertEquals(30.0, s.lastVisionXInches, EPS);
        assertEquals(7.0, s.lastVisionYInches, EPS);
        assertEquals(200L, s.visionTimestampMs);
        assertEquals(30.0, s.lastVisionRangeInches, EPS);
        assertEquals(0.8, s.lastVisionDecisionMargin, EPS);
    }

    // -------------------------------------------------------------------------
    // Vision age
    // -------------------------------------------------------------------------

    @Test
    public void visionAge_initiallyInfinity_withNoVisionEver() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        assertEquals(Double.POSITIVE_INFINITY, fusion.getStateSnapshot().ageOfLastVisionMs, 0.0);
    }

    @Test
    public void visionAge_zeroImmediatelyAfterVisionUpdate() {
        fusion.reset(new Pose(0, 0, 0), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, 0), 100L);
        fusion.addVisionMeasurement(new Pose(1, 0, 0), 200L, 10.0, 1.0);

        assertEquals(0.0, fusion.getStateSnapshot().ageOfLastVisionMs, EPS);
    }

    // -------------------------------------------------------------------------
    // getStateSnapshot()
    // -------------------------------------------------------------------------

    @Test
    public void getStateSnapshot_isIndependentCopy_mutatingItDoesNotAffectInternal() {
        fusion.reset(new Pose(5, 5, 0), 0L);
        PoseFusion.State snap = fusion.getStateSnapshot();
        snap.fusedXInches = 999.0;

        fusion.updateWithOdometry(new Pose(6, 6, 0), 100L);
        PoseFusion.State fresh = fusion.getStateSnapshot();

        assertNotEquals(999.0, fresh.fusedXInches, EPS);
        assertEquals(6.0, fresh.fusedXInches, EPS);
    }

    // -------------------------------------------------------------------------
    // Heading wrap
    // -------------------------------------------------------------------------

    @Test
    public void headingDelta_wrapsCorrectlyAcrossPiBoundary() {
        // Start at heading PI-0.1, odometry steps to -PI+0.1 (small rotation crossing PI)
        fusion.reset(new Pose(0, 0, Math.PI - 0.1), 0L);
        fusion.updateWithOdometry(new Pose(1, 0, -Math.PI + 0.1), 100L);

        // Delta should be +0.2 rad (wrap), so fused heading = PI-0.1+0.2 = PI+0.1, wrapped to -(PI-0.1)
        PoseFusion.State s = fusion.getStateSnapshot();
        assertEquals(-Math.PI + 0.1, s.fusedHeadingRad, EPS_LOOSE);
    }

    @Test
    public void headingDelta_negativeWrapWorks() {
        // Start at -(PI-0.1), odometry steps to PI-0.1 (crossing -PI boundary)
        fusion.reset(new Pose(0, 0, -Math.PI + 0.1), 0L);
        fusion.updateWithOdometry(new Pose(0, 0, Math.PI - 0.1), 100L);

        // Delta = wrapRadians((PI-0.1) - (-PI+0.1)) = wrapRadians(2PI-0.2) = -0.2
        // fusedHeading = wrapRadians((-PI+0.1) + (-0.2)) = wrapRadians(-PI-0.1) = PI-0.1
        PoseFusion.State s = fusion.getStateSnapshot();
        assertEquals(Math.PI - 0.1, s.fusedHeadingRad, EPS_LOOSE);
    }
}
