package org.firstinspires.ftc.teamcode.util;

/**
 * A simple moving average filter for smoothing noisy sensor readings.
 * <p>
 * This filter maintains a circular buffer of the last N readings and returns
 * their average. This is effective for reducing sensor noise while maintaining
 * reasonable response time.
 * <p>
 * Usage:
 * <pre>
 *     MovingAverageFilter filter = new MovingAverageFilter(5);
 *     double smoothedValue = filter.calculate(rawSensorReading);
 * </pre>
 * <p>
 * Each sensor or data stream should have its own filter instance.
 */
public class MovingAverageFilter {

    private final double[] buffer;
    private final int windowSize;
    private int index;
    private int count;
    private double sum;

    /**
     * Creates a new moving average filter with the specified window size.
     *
     * @param windowSize The number of samples to average. Must be at least 1.
     *                   Larger values provide more smoothing but increase lag.
     *                   Typical values: 3-10 for most FTC sensor applications.
     */
    public MovingAverageFilter(int windowSize) {
        if (windowSize < 1) {
            windowSize = 1;
        }
        this.windowSize = windowSize;
        this.buffer = new double[windowSize];
        this.index = 0;
        this.count = 0;
        this.sum = 0.0;
    }

    /**
     * Adds a new value to the filter and returns the filtered (averaged) result.
     *
     * @param value The new raw sensor reading
     * @return The moving average of the last N readings
     */
    public double calculate(double value) {
        // Handle NaN or infinite values - don't add them to the buffer
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            // Return the current average if we have data, otherwise return NaN
            return count > 0 ? sum / count : Double.NaN;
        }

        // If buffer is full, subtract the oldest value from sum
        if (count >= windowSize) {
            sum -= buffer[index];
        } else {
            count++;
        }

        // Add new value
        buffer[index] = value;
        sum += value;

        // Advance circular buffer index
        index = (index + 1) % windowSize;

        return sum / count;
    }

    /**
     * Returns the current filtered value without adding a new sample.
     *
     * @return The current moving average, or NaN if no samples have been added
     */
    public double get() {
        return count > 0 ? sum / count : Double.NaN;
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
        sum = 0.0;
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
     * @param initialValue The value to fill the buffer with
     */
    public void prime(double initialValue) {
        if (Double.isNaN(initialValue) || Double.isInfinite(initialValue)) {
            return;
        }
        for (int i = 0; i < windowSize; i++) {
            buffer[i] = initialValue;
        }
        index = 0;
        count = windowSize;
        sum = initialValue * windowSize;
    }
}
