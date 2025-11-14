package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TelemetrySettings {
    private TelemetrySettings() {}

    /** Enable FTC Dashboard telemetry packets (AdvantageScope Lite connection) */
    public static boolean enableDashboardTelemetry = true;

    /** Enable FullPanels telemetry publishing (disable for competition to save 10-15ms) */
    public static boolean enableFullPanelsTelemetry = false;

    /** Enable verbose driver station telemetry (disable for competition to save 2-3ms) */
    public static boolean enableVerboseDriverStation = true;
}
