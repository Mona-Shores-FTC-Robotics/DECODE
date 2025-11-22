package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.WifiManager;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * Utility class for identifying the robot by retrieving its unique SSID and mapping it to RobotType.
 */
public class ControlHubIdentifierUtil {

    // Cached Method instance to optimize reflection performance
    private static Method getWifiApConfigMethod = null;
    private static Telemetry telemetry = null;

    /**
     * Retrieves the RobotType based on the Control Hub's SSID.
     * Retries up to 5 times with delays if initial retrieval fails.
     * This handles the case where WiFi isn't fully initialized on first boot after battery removal.
     *
     * @param hardwareMap The HardwareMap instance from the OpMode.
     * @param opModeTelemetry The telemetry object for logging (optional, can be null)
     */
    public static void setRobotName(HardwareMap hardwareMap, Telemetry opModeTelemetry) {
        telemetry = opModeTelemetry;
        final int MAX_RETRIES = 5;
        final long RETRY_DELAY_MS = 500; // 500ms between retries (increased from 200ms)

        log("Detecting robot by SSID...");

        String ssid = null;
        for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
            ssid = retrieveSSID(hardwareMap);
            if (ssid != null) {
                mapSSIDToRobotType(ssid);
                String robotName = RobotState.getRobotName();
                RobotState.packet.put("_Config/Robot Name", robotName);

                if (attempt > 0) {
                    log("✓ SSID found on attempt " + (attempt + 1) + ": " + robotName);
                } else {
                    log("✓ Robot detected: " + robotName);
                }
                return;
            }

            // If not last attempt, wait before retrying
            if (attempt < MAX_RETRIES - 1) {
                log("⟳ SSID not ready, retrying (" + (attempt + 1) + "/" + MAX_RETRIES + ")...");
                try {
                    Thread.sleep(RETRY_DELAY_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    logError("Interrupted during SSID retrieval retry");
                    return;
                }
            }
        }

        // All retries failed
        logError("✗ Failed to retrieve SSID after " + MAX_RETRIES + " attempts. Using default (19429) config.");
        RobotState.packet.put("_Config/Robot Name", "UNKNOWN (using 19429 default)");
    }

    /**
     * Retrieves the Control Hub's SSID using reflection.
     *
     * @param hardwareMap The HardwareMap instance from the OpMode.
     * @return The sanitized SSID string, or null if retrieval fails.
     */
    private static String retrieveSSID(HardwareMap hardwareMap) {
        try {
            Context context = hardwareMap.appContext;
            WifiManager wifiManager = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);

            if (wifiManager == null) {
                logError("WifiManager not available.");
                return null;
            }

            WifiConfiguration wifiConfig = getWifiApConfiguration(wifiManager);
            if (wifiConfig != null && wifiConfig.SSID != null) {
                return sanitizeSSID(wifiConfig.SSID);
            } else {
                logError("SSID not found in WifiConfiguration.");
                return null;
            }

        } catch (NoSuchMethodException e) {
            logError("getWifiApConfiguration method not found: " + e.getMessage());
            return null;
        } catch (IllegalAccessException | InvocationTargetException e) {
            logError("Failed to invoke getWifiApConfiguration: " + e.getMessage());
            return null;
        } catch (Exception e) {
            logError("Unexpected error while retrieving SSID: " + e.getMessage());
            return null;
        }
    }

    /**
     * Uses reflection to access the hidden getWifiApConfiguration method.
     *
     * @param wifiManager The WifiManager instance.
     * @return The WifiConfiguration object, or null if reflection fails.
     * @throws NoSuchMethodException     If the method does not exist.
     * @throws IllegalAccessException    If the method cannot be accessed.
     * @throws InvocationTargetException If the underlying method throws an exception.
     */
    private static WifiConfiguration getWifiApConfiguration(WifiManager wifiManager)
            throws NoSuchMethodException, IllegalAccessException, InvocationTargetException {
        if (getWifiApConfigMethod == null) {
            getWifiApConfigMethod = WifiManager.class.getDeclaredMethod("getWifiApConfiguration");
            getWifiApConfigMethod.setAccessible(true);
        }
        return (WifiConfiguration) getWifiApConfigMethod.invoke(wifiManager);
    }

    /**
     * Removes surrounding quotes from the SSID if present.
     *
     * @param ssid The raw SSID string.
     * @return The sanitized SSID without surrounding quotes.
     */
    private static String sanitizeSSID(String ssid) {
        if (ssid.startsWith("\"") && ssid.endsWith("\"") && ssid.length() > 1) {
            return ssid.substring(1, ssid.length() - 1);
        }
        return ssid;
    }

    /**
     * Maps the SSID string to a corresponding RobotType.
     *
     * @param ssid The sanitized SSID string.
     */
    private static void mapSSIDToRobotType(String ssid) {
        if (ssid.contains("19429")) {
            RobotState.setRobotName("DECODE_19429");
        } else if (ssid.contains("20245")) {
           RobotState.setRobotName("DECODE_20245");
        } else {
            logError("Unrecognized SSID: " + ssid);
        }
    }

    /**
     * Logs informational messages to both telemetry and packet.
     *
     * @param message The message to log.
     */
    private static void log(String message) {
        if (telemetry != null) {
            telemetry.addData("Robot ID", message);
            telemetry.update();
        }
        RobotState.packet.put("_Config/Status", message);
    }

    /**
     * Logs error messages to both telemetry and packet.
     *
     * @param message The error message to log.
     */
    private static void logError(String message) {
        if (telemetry != null) {
            telemetry.addData("Robot ID ERROR", message);
            telemetry.update();
        }
        RobotState.packet.put("_Config/Error", message);
    }
}
