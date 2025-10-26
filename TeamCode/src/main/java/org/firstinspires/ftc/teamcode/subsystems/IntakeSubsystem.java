package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.auto.Alliance;

/**
 * Optional intake stub used by the autonomous routine. It now exposes a minimal
 * state machine so OpModes can request work and poll when it is complete without
 * touching timers or hardware directly.
 */
@Config
public class IntakeSubsystem implements Subsystem {

    public enum IntakeState {
        IDLE,
        INTAKING,
        FULL
    }

    public static double defaultRunTimeMs = 0.0;
    public static int maxIndexed = 3;

    private final ElapsedTime timer = new ElapsedTime();

    private IntakeState state = IntakeState.IDLE;
    private int indexedCount = 0;
    private Alliance alliance = Alliance.UNKNOWN;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        // Real implementation would bind motors here.
    }

    @Override
    public void initialize() {
        state = IntakeState.IDLE;
        indexedCount = 0;
    }

    public void requestIntake() {
        if (state == IntakeState.INTAKING) {
            return;
        }
        state = IntakeState.INTAKING;
        timer.reset();
        if (defaultRunTimeMs <= 0.0) {
            indexedCount = maxIndexed;
            state = IntakeState.FULL;
        }
    }

    /** Legacy name retained for existing OpModes. */
    public void runIn() {
        requestIntake();
    }

    /** Legacy name retained for existing OpModes. */
    public void start() {
        requestIntake();
    }

    public void stop() {
        if (state != IntakeState.FULL) {
            state = IntakeState.IDLE;
        }
    }

    /** Legacy loop hook retained for compatibility. */
    public void update() {
        periodic();
    }

    public boolean isBusy() {
        return state == IntakeState.INTAKING;
    }

    public boolean isFull() {
        return indexedCount >= maxIndexed;
    }

    public void seedIndexedCount(int count) {
        indexedCount = clamp(count, 0, maxIndexed);
        if (indexedCount >= maxIndexed) {
            state = IntakeState.FULL;
        }
    }

    public void clearIndexed() {
        indexedCount = 0;
        state = IntakeState.IDLE;
    }

    public int getIndexedCount() {
        return indexedCount;
    }

    public IntakeState getState() {
        return state;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    @Override
    public void periodic() {
        if (state == IntakeState.INTAKING && timer.milliseconds() >= defaultRunTimeMs) {
            indexedCount = maxIndexed;
            state = IntakeState.FULL;
        }
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
