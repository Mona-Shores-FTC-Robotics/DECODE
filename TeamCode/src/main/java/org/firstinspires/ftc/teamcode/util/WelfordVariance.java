package org.firstinspires.ftc.teamcode.util;

/**
 * Welford's online algorithm for numerically stable running mean / variance.
 *
 * <p>Ported from BeepBot99/CodeBloodedDecodeV3, with an added {@link #reset()}
 * so a tuner can clear stats without reallocating.
 *
 * <p>Update is O(1); intermediate sums never lose precision the way
 * naive {@code sumOfSquares - sum*sum/n} does once n gets large or values
 * cluster far from zero.
 */
public final class WelfordVariance {
    private int n = 0;
    private double mean = 0.0;
    private double m2 = 0.0;

    public void update(double x) {
        n++;
        double delta = x - mean;
        mean += delta / n;
        double delta2 = x - mean;
        m2 += delta * delta2;
    }

    public void reset() {
        n = 0;
        mean = 0.0;
        m2 = 0.0;
    }

    /** Sample variance (Bessel-corrected). NaN until at least 2 samples. */
    public double variance() {
        return n < 2 ? Double.NaN : m2 / (n - 1);
    }

    /** Sample standard deviation. NaN until at least 2 samples. */
    public double stdDev() {
        return Math.sqrt(variance());
    }

    public double mean() {
        return mean;
    }

    public int n() {
        return n;
    }
}
