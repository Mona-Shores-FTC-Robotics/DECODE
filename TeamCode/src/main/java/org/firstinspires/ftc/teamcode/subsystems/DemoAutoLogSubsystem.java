package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogOutput;

/**
 * Simple demonstration subsystem showing @AutoLog usage.
 *
 * This subsystem doesn't require any hardware - it just tracks time and counters
 * to demonstrate how @AutoLog automatically logs all @AutoLogOutput methods
 * to both WPILOG files and FTC Dashboard.
 *
 * Perfect for testing on a testbench without full robot hardware!
 */
@AutoLog
public class DemoAutoLogSubsystem {

    private final ElapsedTime timer = new ElapsedTime();
    private int updateCount = 0;
    private double lastDeltaTimeMs = 0.0;
    private double averageLoopTimeMs = 20.0;
    private boolean isRunning = false;

    // Simulated "sensor" values that change over time
    private double simulatedTemperature = 25.0;
    private double simulatedVoltage = 12.6;

    public DemoAutoLogSubsystem() {
        timer.reset();
    }

    /**
     * Call this periodically to update the subsystem state.
     * This simulates a real subsystem's periodic() method.
     */
    public void periodic() {
        double currentTime = timer.milliseconds();
        lastDeltaTimeMs = currentTime - (updateCount * averageLoopTimeMs);
        updateCount++;

        // Simulate temperature rising slowly
        simulatedTemperature = 25.0 + Math.sin(currentTime / 1000.0) * 5.0;

        // Simulate voltage dropping slightly
        simulatedVoltage = 12.6 - (currentTime / 60000.0) * 0.5;

        // Update average loop time with simple moving average
        if (lastDeltaTimeMs > 0) {
            averageLoopTimeMs = 0.9 * averageLoopTimeMs + 0.1 * lastDeltaTimeMs;
        }
    }

    public void start() {
        isRunning = true;
        timer.reset();
        updateCount = 0;
    }

    public void stop() {
        isRunning = false;
    }

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    @AutoLogOutput
    public double getElapsedTimeSeconds() {
        return timer.seconds();
    }

    @AutoLogOutput
    public int getUpdateCount() {
        return updateCount;
    }

    @AutoLogOutput
    public double getLastDeltaTimeMs() {
        return lastDeltaTimeMs;
    }

    @AutoLogOutput
    public double getAverageLoopTimeMs() {
        return averageLoopTimeMs;
    }

    @AutoLogOutput
    public boolean isRunning() {
        return isRunning;
    }

    @AutoLogOutput
    public double getSimulatedTemperatureCelsius() {
        return simulatedTemperature;
    }

    @AutoLogOutput
    public double getSimulatedVoltage() {
        return simulatedVoltage;
    }

    @AutoLogOutput
    public double getLoopFrequencyHz() {
        if (averageLoopTimeMs <= 0) {
            return 0.0;
        }
        return 1000.0 / averageLoopTimeMs;
    }

    @AutoLogOutput
    public String getStatus() {
        return isRunning ? "RUNNING" : "STOPPED";
    }
}
