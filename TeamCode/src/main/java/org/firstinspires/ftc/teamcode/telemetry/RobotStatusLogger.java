package org.firstinspires.ftc.teamcode.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

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
        // RUNNING field at root level (for AdvantageScope timeline)
//        KoalaLog.log("RUNNING", running, true);
//
//        // Get OpMode name from the class
//        // This matches what FTC Dashboard does via opModeManager.getActiveOpModeName()
//        String opModeName = opMode.getClass().getSimpleName();
//
//        // Determine status - INIT or RUNNING based on running flag
//        String status = running ? "RUNNING" : "INIT";
//
//        // Log _Status fields (exact match to FTC Dashboard)
//        KoalaLog.log("_Status/enabled", !opMode.isStopRequested(), true);
//        KoalaLog.log("_Status/activeOpmode", opModeName, true);
//        KoalaLog.log("_Status/activeOpModeStatus", status, true);
//
//        // Read error/warning messages from FTC SDK's RobotLog (same source as FTC Dashboard)
//        String errorMsg = RobotLog.getGlobalErrorMsg();
//        String warningMsg = "";
//        try {
//            RobotLog.GlobalWarningMessage warningSource = RobotLog.getGlobalWarningMessage();
//            if (warningSource != null && warningSource.message != null) {
//                warningMsg = warningSource.message;
//            }
//        } catch (Exception e) {
//            // Ignore if warning message not available
//        }
//        KoalaLog.log("_Status/errorMessage", errorMsg != null ? errorMsg : "", true);
//        KoalaLog.log("_Status/warningMessage", warningMsg, true);
//
//        // Battery voltage from hardware
//        try {
//            if (hardwareMap != null && hardwareMap.voltageSensor.iterator().hasNext()) {
//                double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//                KoalaLog.log("_Status/batteryVoltage", voltage, true);
//            }
//        } catch (Exception e) {
//            // Ignore if voltage sensor not available
//        }
    }
}
