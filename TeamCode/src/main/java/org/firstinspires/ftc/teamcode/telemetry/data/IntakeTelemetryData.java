package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;

/**
 * Intake subsystem telemetry data.
 * Includes mode, power, roller state, and artifact tracking.
 */
public class IntakeTelemetryData {
    // Intake state
    public final String mode;
    public final String resolvedMode;
    public final double power;

    // Roller
    public final boolean rollerPresent;
    public final boolean rollerActive;
    public final double rollerPosition;
    public final boolean prefeedPresent;
    public final boolean prefeedActive;
    public final double prefeedPosition;

    // Artifact tracking
    public final int artifactCount;
    public final String artifactState;

    public IntakeTelemetryData(
            String mode,
            String resolvedMode,
            double power,
            boolean rollerPresent,
            boolean rollerActive,
            double rollerPosition,
            boolean prefeedPresent,
            boolean prefeedActive,
            double prefeedPosition,
            int artifactCount,
            String artifactState
    ) {
        this.mode = mode;
        this.resolvedMode = resolvedMode;
        this.power = power;
        this.rollerPresent = rollerPresent;
        this.rollerActive = rollerActive;
        this.rollerPosition = rollerPosition;
        this.prefeedPresent = prefeedPresent;
        this.prefeedActive = prefeedActive;
        this.prefeedPosition = prefeedPosition;
        this.artifactCount = artifactCount;
        this.artifactState = artifactState;
    }

    public static IntakeTelemetryData capture(IntakeSubsystem intake, LauncherCoordinator coordinator) {
        int artifactCount = 0;
        String artifactState = "UNKNOWN";

        if (coordinator != null) {
            artifactCount = coordinator.getArtifactCount();
            artifactState = coordinator.getArtifactState().name();
        }

        return new IntakeTelemetryData(
                intake.getMode().name(),
                intake.getResolvedMode().name(),
                intake.getCurrentPower(),
                intake.isRollerPresent(),
                intake.isRollerActive(),
                intake.getRollerPosition(),
                intake.isPrefeedPresent(),
                intake.isPrefeedActive(),
                intake.getPrefeedPosition(),
                artifactCount,
                artifactState
        );
    }
}
