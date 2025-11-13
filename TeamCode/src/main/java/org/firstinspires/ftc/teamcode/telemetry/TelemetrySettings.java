package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TelemetrySettings {
    private TelemetrySettings() {}

    public static boolean enablePsiKitLogging = true;  // Changed to true - saves CSV logs to /sdcard/FIRST/PsiKitLogs/
    public static boolean enableDashboardTelemetry = true;
}
