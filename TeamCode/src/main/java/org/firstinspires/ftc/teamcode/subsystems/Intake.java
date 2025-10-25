package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Optional intake stub used by the autonomous routine. It simply tracks whether the intake is
 * running so the OpMode can coordinate state transitions without blocking.
 */
@Config
public class Intake {

    public static double defaultRunTimeMs = 0.0;
    public static int maxIndexed = 3;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean runningIn = false;
    private int indexedCount = 0;

    public Intake(HardwareMap hardwareMap) {
        // Real implementation would bind motors here.
    }

    public void runIn() {
        runningIn = true;
        timer.reset();
        if (defaultRunTimeMs <= 0.0) {
            indexedCount = maxIndexed;
            runningIn = false;
        }
    }

    public void start() {
        runIn();
    }

    public void stop() {
        runningIn = false;
    }

    public void update() {
        if (runningIn && timer.milliseconds() >= defaultRunTimeMs) {
            indexedCount = maxIndexed;
            runningIn = false;
        }
    }

    public boolean isRunning() {
        return runningIn;
    }

    public void seedIndexedCount(int count) {
        indexedCount = clamp(count, 0, maxIndexed);
    }

    public void clearIndexed() {
        indexedCount = 0;
    }

    public int getIndexedCount() {
        return indexedCount;
    }

    private static int clamp(int value, int min, int max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }
}
