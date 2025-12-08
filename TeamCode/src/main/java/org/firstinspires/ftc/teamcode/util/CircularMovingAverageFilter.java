package org.firstinspires.ftc.teamcode.util;

/**
 * A moving average filter for circular/angular values like hue (0-360°).
 * <p>
 * Standard averaging doesn't work for circular values because of wrap-around:
 * averaging 350° and 10° should give ~0°, not 180°. This filter uses unit
 * vector averaging to correctly handle circular data.
 * <p>
 * Usage:
 * <pre>
 *     CircularMovingAverageFilter hueFilter = new CircularMovingAverageFilter(5);
 *     double smoothedHue = hueFilter.calculate(rawHue);
 * </pre>
 * <p>
 * Each sensor or data stream should have its own filter instance.
 */
public class CircularMovingAverageFilter {

    private final double[] buffer;
    private final int windowSize;
    private int index;
    private int count;

    /**
     * Creates a new circular moving average filter with the specified window size.
     *
     * @param windowSize The number of samples to average. Must be at least 1.
     *                   Larger values provide more smoothing but increase lag.
     *                   Typical values: 3-5 for hue filtering in FTC applications.
     */
    public CircularMovingAverageFilter(int windowSize) {
        if (windowSize < 1) {
            windowSize = 1;
        }
        this.windowSize = windowSize;
        this.buffer = new double[windowSize];
        this.index = 0;
        this.count = 0;
    }

    /**
     * Adds a new angular value to the filter and returns the filtered (averaged) result.
     * Uses unit vector averaging to properly handle circular wrap-around.
     *
     * @param degrees The new angle in degrees (typically 0-360 for hue)
     * @return The circular moving average of the last N readings in degrees (0-360)
     */
    public double calculate(double degrees) {
        // Handle NaN or infinite values - don't add them to the buffer
        if (Double.isNaN(degrees) || Double.isInfinite(degrees)) {
            // Return the current average if we have data, otherwise return NaN
            return count > 0 ? computeCircularMean() : Double.NaN;
        }

        // Add new value to circular buffer
        if (count < windowSize) {
            count++;
        }
        buffer[index] = degrees;
        index = (index + 1) % windowSize;

        return computeCircularMean();
    }

    /**
     * Computes the circular mean of buffered values using unit vector averaging.
     * This is the mathematically correct way to average angles.
     *
     * Algorithm:
     * 1. Convert each angle to a unit vector (cos, sin)
     * 2. Average the vectors
     * 3. Convert back to angle using atan2
     *
     * @return The circular mean in degrees (0-360)
     */
    private double computeCircularMean() {
        if (count == 0) {
            return Double.NaN;
        }

        // Sum of unit vectors
        double sumCos = 0.0;
        double sumSin = 0.0;

        for (int i = 0; i < count; i++) {
            double radians = Math.toRadians(buffer[i]);
            sumCos += Math.cos(radians);
            sumSin += Math.sin(radians);
        }

        // Average the vectors
        double avgCos = sumCos / count;
        double avgSin = sumSin / count;

        // Convert back to angle
        double meanRadians = Math.atan2(avgSin, avgCos);
        double meanDegrees = Math.toDegrees(meanRadians);

        // Normalize to 0-360 range
        if (meanDegrees < 0) {
            meanDegrees += 360.0;
        }

        return meanDegrees;
    }

    /**
     * Returns the current filtered value without adding a new sample.
     *
     * @return The current circular moving average, or NaN if no samples have been added
     */
    public double get() {
        return count > 0 ? computeCircularMean() : Double.NaN;
    }

    /**
     * Resets the filter, clearing all buffered values.
     */
    public void reset() {
        for (int i = 0; i < windowSize; i++) {
            buffer[i] = 0.0;
        }
        index = 0;
        count = 0;
    }

    /**
     * Returns the number of samples currently in the buffer.
     *
     * @return Number of samples (0 to windowSize)
     */
    public int getSampleCount() {
        return count;
    }

    /**
     * Returns true if the buffer is full (has windowSize samples).
     *
     * @return true if filter has reached full averaging capacity
     */
    public boolean isFull() {
        return count >= windowSize;
    }

    /**
     * Returns the configured window size.
     *
     * @return The window size this filter was created with
     */
    public int getWindowSize() {
        return windowSize;
    }

    /**
     * Primes the filter by filling the buffer with an initial value.
     * Useful when you have a known starting value and want immediate full filtering.
     *
     * @param initialDegrees The angle in degrees to fill the buffer with
     */
    public void prime(double initialDegrees) {
        if (Double.isNaN(initialDegrees) || Double.isInfinite(initialDegrees)) {
            return;
        }
        for (int i = 0; i < windowSize; i++) {
            buffer[i] = initialDegrees;
        }
        index = 0;
        count = windowSize;
    }
}
