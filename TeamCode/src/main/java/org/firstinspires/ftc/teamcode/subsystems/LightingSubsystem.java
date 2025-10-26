package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceLight;

/**
 * Simple indicator light wrapper that exposes a state machine so OpModes can
 * request colours without touching the underlying servo hardware directly.
 */
public class LightingSubsystem implements Subsystem {

    public enum LightingState {
        OFF,
        ALLIANCE,
        BUSY
    }

    private final AllianceLight light;
    private LightingState state = LightingState.OFF;
    private Alliance alliance = Alliance.UNKNOWN;

    public LightingSubsystem(HardwareMap hardwareMap) {
        AllianceLight detected = null;
        try {
            detected = AllianceLight.onServo(hardwareMap, "indicator");
        } catch (IllegalArgumentException ignored) {
            // Servo not configured – leave light null so calls become no-ops.
        }
        light = detected;
    }

    @Override
    public void initialize() {
        applyAllianceColor();
    }

    @Override
    public void periodic() {
        // No periodic work required – kept for interface completeness.
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
        applyAllianceColor();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void indicateBusy() {
        if (light != null) {
            light.setRaw(AllianceLight.PURPLE_POS);
        }
        state = LightingState.BUSY;
    }

    public void indicateIdle() {
        applyAllianceColor();
    }

    public LightingState getState() {
        return state;
    }

    private void applyAllianceColor() {
        if (light == null) {
            state = LightingState.OFF;
            return;
        }
        if (alliance == Alliance.UNKNOWN) {
            light.setRaw(AllianceLight.GREEN_POS);
            state = LightingState.OFF;
        } else {
            light.applyAlliance(alliance);
            state = LightingState.ALLIANCE;
        }
    }
}
