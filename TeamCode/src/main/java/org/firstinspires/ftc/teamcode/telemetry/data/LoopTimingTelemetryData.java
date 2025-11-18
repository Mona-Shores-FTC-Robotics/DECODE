package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;

/**
 * Loop timing telemetry data - captures complete loop execution times.
 * <p>
 * This data is used for performance diagnostics and loop timing analysis.
 * All times are in milliseconds.
 * </p>
 * <p>
 * Timing data is from the PREVIOUS loop iteration (N-1), since timing for the current
 * loop (N) isn't available until after telemetry is published.
 * </p>
 */
public class LoopTimingTelemetryData {
    // Main loop overhead (not including subsystem periodic or telemetry)
    public final double mainLoopMs;

    // Subsystem periodic execution times
    public final double driveMs;
    public final double intakeMs;
    public final double launcherMs;
    public final double lightingMs;
    public final double visionMs;

    // Telemetry and logging overhead
    public final long telemetryStartNs;

    public LoopTimingTelemetryData(
            double mainLoopMs,
            double driveMs,
            double intakeMs,
            double launcherMs,
            double lightingMs,
            double visionMs,
            long telemetryStartNs
    ) {
        this.mainLoopMs = mainLoopMs;
        this.driveMs = driveMs;
        this.intakeMs = intakeMs;
        this.launcherMs = launcherMs;
        this.lightingMs = lightingMs;
        this.visionMs = visionMs;
        this.telemetryStartNs = telemetryStartNs;
    }

    /**
     * Calculate total subsystem periodic time.
     */
    public double totalSubsystemMs() {
        return driveMs + intakeMs + launcherMs + lightingMs + visionMs;
    }

    /**
     * Calculate total loop time including all overhead.
     */
    public double totalLoopMs( double totalTelemetryMs) {
        return mainLoopMs + totalSubsystemMs() + totalTelemetryMs;
    }

    /**
     * Capture loop timing data from subsystems and previous loop overhead.
     *
     * @param mainLoopMs Main loop overhead from previous iteration (0 if not available)
     * @param telemetryStartNs Telemetry overhead from previous iteration (0 if not available)
     * @param drive Drive subsystem
     * @param intake Intake subsystem
     * @param launcher Launcher subsystem
     * @param lighting Lighting subsystem
     * @param vision Vision subsystem
     */
    public static LoopTimingTelemetryData capture(
            double mainLoopMs,
            long telemetryStartNs,
            DriveSubsystem drive,
            IntakeSubsystem intake,
            LauncherSubsystem launcher,
            LightingSubsystem lighting,
            VisionSubsystemLimelight vision
    ) {
        double driveMs = drive != null ? drive.getLastPeriodicMs() : 0.0;
        double intakeMs = intake != null ? intake.getLastPeriodicMs() : 0.0;
        double launcherMs = launcher != null ? launcher.getLastPeriodicMs() : 0.0;
        double lightingMs = lighting != null ? lighting.getLastPeriodicMs() : 0.0;
        double visionMs = vision != null ? vision.getLastPeriodicMs() : 0.0;

        return new LoopTimingTelemetryData(
                mainLoopMs,
                driveMs,
                intakeMs,
                launcherMs,
                lightingMs,
                visionMs,
                telemetryStartNs
        );
    }

    /**
     * Create empty timing data (for first loop iteration or when timing not available).
     */
    public static LoopTimingTelemetryData empty() {
        return new LoopTimingTelemetryData(0, 0, 0, 0, 0, 0, 0);
    }
}
