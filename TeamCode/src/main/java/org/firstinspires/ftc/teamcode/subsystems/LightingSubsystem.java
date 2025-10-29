package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * goBILDA RGB Indicator Light wrapper. Exposes a simple state machine and keeps
 * the servo-specific logic inside the subsystem so OpModes can request colours
 * without touching hardware directly.
 */
@Configurable
public class LightingSubsystem implements Subsystem, IntakeSubsystem.LaneColorListener {

    public void indicateBusy() {
    }

    public void indicateIdle() {
    }

    public enum LightingState {
        OFF,
        ALLIANCE,
        BUSY
    }

    public static double GREEN_POS = 0.500;
    public static double PURPLE_POS = 0.722;
    public static double RED_POS = 0.281;
    public static double BLUE_POS = 0.611;

    private final Servo led;
    private LightingState state = LightingState.OFF;
    private Alliance alliance = Alliance.UNKNOWN;

    public LightingSubsystem(HardwareMap hardwareMap) {
        Servo detected = null;
        try {
            detected = hardwareMap.get(Servo.class, "indicator");
        } catch (IllegalArgumentException ignored) {
            // Servo not configured – leave null so downstream calls no-op safely.
        }
        led = detected;
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

    public void setRaw(double position) {
        if (led == null) {
            state = LightingState.OFF;
            return;
        }
        led.setPosition(clamp01(position));
    }

    private void applyAllianceColor() {
        if (led == null) {
            state = LightingState.OFF;
            return;
        }
        if (alliance == Alliance.UNKNOWN) {
            setRaw(GREEN_POS);
            state = LightingState.OFF;
        } else {
            setRaw(alliance == Alliance.RED ? RED_POS : BLUE_POS);
            state = LightingState.ALLIANCE;
        }
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    @Override
    public void onLaneColorChanged(LauncherLane lane, ArtifactColor color) {
        if (color == null || led == null) {
            return;
        }
        switch (color) {
            case GREEN:
                setRaw(GREEN_POS);
                break;
            case PURPLE:
                setRaw(PURPLE_POS);
                break;
            default:
                applyAllianceColor();
                break;
        }
    }

    public void indicateAllianceInit() {
        applyAllianceColor();
    }

    public void showDecodePattern(ArtifactColor[] pattern) {
        if (pattern == null || pattern.length == 0) {
            applyAllianceColor();
            return;
        }
        ArtifactColor primary = pattern[0];
        switch (primary) {
            case GREEN:
                setRaw(GREEN_POS);
                break;
            case PURPLE:
                setRaw(PURPLE_POS);
                break;
            default:
                applyAllianceColor();
                break;
        }
    }
}
