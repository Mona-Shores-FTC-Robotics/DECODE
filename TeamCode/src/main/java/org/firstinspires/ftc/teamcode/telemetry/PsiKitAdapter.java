package org.firstinspires.ftc.teamcode.telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * CSV logger that writes in AdvantageScope-compatible long format.
 *
 * Format: Timestamp,Key,Value
 * - Timestamp in seconds (not milliseconds)
 * - One row per value
 */
public class PsiKitAdapter {

    private FileWriter writer;
    private boolean active;
    private long startTimeMs;

    /**
     * Starts a new logging session.
     */
    public void startSession() {
        stopSession();
        try {
            // Use RoadRunner logs directory - same path AdvantageScope already knows about
            File logDir = new File("/storage/emulated/0/RoadRunner/logs");
            if (!logDir.exists() && !logDir.mkdirs()) {
                return;
            }
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
            // Use .csv extension for CSV format (not .log which is for Road Runner binary format)
            File logFile = new File(logDir, "log_" + timestamp + ".csv");
            writer = new FileWriter(logFile);
            // AdvantageScope requires EXACTLY these column names (capitalized)
            writer.write("Timestamp,Key,Value\n");
            writer.flush();
            startTimeMs = System.currentTimeMillis();
            active = true;
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Records a numeric value.
     */
    public void recordNumber(String key, double value) {
        if (!active) return;
        try {
            // Timestamp in SECONDS (not milliseconds)
            double timestampSec = (System.currentTimeMillis() - startTimeMs) / 1000.0;
            writer.write(String.format("%.3f,%s,%s\n", timestampSec, key, value));
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Records a boolean value (true or false, not 1/0).
     */
    public void recordBoolean(String key, boolean value) {
        if (!active) return;
        try {
            double timestampSec = (System.currentTimeMillis() - startTimeMs) / 1000.0;
            // AdvantageScope wants "true" or "false" strings for booleans
            writer.write(String.format("%.3f,%s,%s\n", timestampSec, key, value ? "true" : "false"));
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Records a string value (quoted).
     */
    public void recordString(String key, String value) {
        if (!active) return;
        try {
            double timestampSec = (System.currentTimeMillis() - startTimeMs) / 1000.0;
            // Escape quotes and wrap in quotes
            String safe = value.replace("\"", "\"\"");
            writer.write(String.format("%.3f,%s,\"%s\"\n", timestampSec, key, safe));
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Flushes buffered data to disk.
     */
    public void flush() {
        try {
            if (writer != null) {
                writer.flush();
            }
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Stops the session and closes the file.
     */
    public void stopSession() {
        try {
            if (writer != null) {
                writer.flush();
                writer.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            writer = null;
            active = false;
        }
    }
}
