package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple gate abstraction. Uses a servo when available but degrades gracefully when the hardware
 * configuration does not provide one.
 */
@Config
public class Gate {

    public static String servoName = "gate";
    public static double openPosition = 0.85;
    public static double closedPosition = 0.15;
    public static double transitionTimeMs = 250.0;

    private final Servo servo;
    private final ElapsedTime motionTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();

    private boolean open = false;
    private boolean moving = false;
    private boolean timedExtend = false;
    private double holdDurationMs = 0.0;

    public Gate(HardwareMap hardwareMap) {
        Servo candidate = null;
        if (hardwareMap != null) {
            try {
                candidate = hardwareMap.get(Servo.class, servoName);
            } catch (Exception ignored) {
                candidate = null;
            }
        }
        servo = candidate;
        closeGate();
    }

    public void openGate() {
        timedExtend = false;
        if (servo != null) {
            servo.setPosition(openPosition);
        }
        open = true;
        motionTimer.reset();
        moving = true;
    }

    public void closeGate() {
        if (servo != null) {
            servo.setPosition(closedPosition);
        }
        open = false;
        timedExtend = false;
        motionTimer.reset();
        moving = true;
    }

    public void close() {
        closeGate();
    }

    public void extendForMs(double durationMs) {
        if (durationMs <= 0) {
            openGate();
            return;
        }
        openGate();
        timedExtend = true;
        holdDurationMs = durationMs;
        holdTimer.reset();
    }

    public void update() {
        if (moving && motionTimer.milliseconds() >= transitionTimeMs) {
            moving = false;
        }
        if (timedExtend && holdTimer.milliseconds() >= holdDurationMs) {
            timedExtend = false;
            closeGate();
        }
    }

    public boolean isOpen() {
        update();
        return open;
    }

    public boolean isBusy() {
        update();
        return moving;
    }
}
