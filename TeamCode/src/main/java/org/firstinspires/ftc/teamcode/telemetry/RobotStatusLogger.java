package org.firstinspires.ftc.teamcode.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import Ori.Coval.Logging.Logger.KoalaLog;

/**
 * Helper utility to log FTC Dashboard _Status fields to WPILOG files.
 *
 * This ensures consistent robot state metadata logging across all OpModes,
 * matching exactly what FTC Dashboard publishes to AdvantageScope Lite.
 *
 * Usage in LinearOpMode:
 * <pre>
 * // In init loop:
 * RobotStatusLogger.logStatus(this, hardwareMap, false);
 *
 * // In main loop:
 * RobotStatusLogger.logStatus(this, hardwareMap, opModeIsActive());
 * </pre>
 */
public class RobotStatusLogger {

    // Cache for slow-changing status fields
    private static long lastStatusLogMs = 0L;
    private static final long STATUS_LOG_INTERVAL_MS = 500L; // 500ms - status rarely changes
    private static String cachedErrorMsg = "";
    private static String cachedWarningMsg = "";
    private static double cachedBatteryVoltage = 0.0;

    /**
     * Log all FTC Dashboard _Status fields to WPILOG.
     *
     * This reads the actual robot state from the FTC SDK and logs to KoalaLog,
     * ensuring offline WPILOG files contain the same metadata as live AdvantageScope Lite view.
     *
     * @param opMode The current LinearOpMode instance
     * @param hardwareMap The hardware map for accessing voltage sensor
     * @param running Whether the OpMode is actively running (use opModeIsActive())
     */
    public static void logStatus(LinearOpMode opMode, HardwareMap hardwareMap, boolean running) {
        long nowMs = System.currentTimeMillis();

        // RUNNING field at root level (for AdvantageScope timeline) - always log (changes frequently)
        KoalaLog.log("RUNNING", running, true);

        // Get OpMode name from the class
        // This matches what FTC Dashboard does via opModeManager.getActiveOpModeName()
        String opModeName = opMode.getClass().getSimpleName();

        // Determine status - INIT or RUNNING based on running flag
        String status = running ? "RUNNING" : "INIT";

        // Log _Status fields that change frequently (every loop)
        KoalaLog.log("_Status/enabled", !opMode.isStopRequested(), true);
        KoalaLog.log("_Status/activeOpmode", opModeName, true);
        KoalaLog.log("_Status/activeOpModeStatus", status, true);

        // Throttle slow-changing status fields (errors, warnings, battery)
        if (nowMs - lastStatusLogMs >= STATUS_LOG_INTERVAL_MS) {
            // Read error/warning messages from FTC SDK's RobotLog (same source as FTC Dashboard)
            cachedErrorMsg = RobotLog.getGlobalErrorMsg();
            cachedWarningMsg = "";
            try {
                RobotLog.GlobalWarningMessage warningSource = RobotLog.getGlobalWarningMessage();
                if (warningSource != null && warningSource.message != null) {
                    cachedWarningMsg = warningSource.message;
                }
            } catch (Exception e) {
                // Ignore if warning message not available
            }

            // Battery voltage from hardware
            try {
                if (hardwareMap != null && hardwareMap.voltageSensor.iterator().hasNext()) {
                    cachedBatteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
                }
            } catch (Exception e) {
                // Ignore if voltage sensor not available
            }

            lastStatusLogMs = nowMs;
        }

        // Log cached values (updated at 500ms interval)
        KoalaLog.log("_Status/errorMessage", cachedErrorMsg != null ? cachedErrorMsg : "", true);
        KoalaLog.log("_Status/warningMessage", cachedWarningMsg, true);
        KoalaLog.log("_Status/batteryVoltage", cachedBatteryVoltage, true);
    }
}
