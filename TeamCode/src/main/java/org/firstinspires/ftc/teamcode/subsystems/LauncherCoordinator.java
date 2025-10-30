package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.EnumMap;
import java.util.Map;

/**
 * Coordinates the launcher subsystems: intake lane sensing, lighting feedback, and shooter spin/feed.
 * Shared by bench diagnostics, teleop, and autonomous so behaviour remains consistent.
 */
@Configurable
public class LauncherCoordinator implements Subsystem, IntakeSubsystem.LaneColorListener {

    public static boolean autoSpinEnabled = true;

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final LightingSubsystem lighting;
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);
    private boolean lightingRegistered = false;

    public LauncherCoordinator(ShooterSubsystem shooter,
                               IntakeSubsystem intake,
                               LightingSubsystem lighting) {
        this.shooter = shooter;
        this.intake = intake;
        this.lighting = lighting;

        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
        }
    }

    @Override
    public void initialize() {
        intake.addLaneColorListener(this);
        if (lighting != null) {
            intake.addLaneColorListener(lighting);
            lightingRegistered = true;
        }
        recomputeSpinMode();
    }

    @Override
    public void periodic() {
        // Nothing required each cycle beyond spin maintenance handled on colour change.
    }

    public void stop() {
        intake.removeLaneColorListener(this);
        if (lighting != null && lightingRegistered) {
            intake.removeLaneColorListener(lighting);
            lightingRegistered = false;
        }
    }

    @Override
    public void onLaneColorChanged(LauncherLane lane, ArtifactColor color) {
        if (lane == null) {
            return;
        }
        ArtifactColor normalized = color == null ? ArtifactColor.NONE : color;
        laneColors.put(lane, normalized);
        if (lighting != null) {
            lighting.setLaneColor(lane, normalized);
        }
        recomputeSpinMode();
    }

    public ArtifactColor getLaneColor(LauncherLane lane) {
        return lane == null ? ArtifactColor.NONE : laneColors.getOrDefault(lane, ArtifactColor.NONE);
    }

    public void enableAutoSpin(boolean enabled) {
        autoSpinEnabled = enabled;
        recomputeSpinMode();
    }

    public void requestKick(LauncherLane lane) {
        ArtifactColor color = getLaneColor(lane);
        if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
            return;
        }
        shooter.queueShot(lane);
    }

    public void requestBurst(double spacingMs) {
        double offset = 0.0;
        for (LauncherLane lane : LauncherLane.values()) {
            ArtifactColor color = getLaneColor(lane);
            if (color == ArtifactColor.NONE || color == ArtifactColor.UNKNOWN) {
                continue;
            }
            shooter.queueShot(lane, offset);
            offset += Math.max(0.0, spacingMs);
        }
    }

    private void recomputeSpinMode() {
        if (!autoSpinEnabled) {
            shooter.requestStandbySpin();
            return;
        }
        boolean anyActive = false;
        for (Map.Entry<LauncherLane, ArtifactColor> entry : laneColors.entrySet()) {
            ArtifactColor color = entry.getValue();
            if (color != ArtifactColor.NONE && color != ArtifactColor.UNKNOWN) {
                anyActive = true;
                break;
            }
        }
        if (anyActive) {
            shooter.requestSpinUp();
        } else {
            shooter.requestStandbySpin();
        }
    }
}
