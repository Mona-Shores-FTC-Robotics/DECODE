package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.bylazar.telemetry.TelemetryManager;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Unified logging surface that mirrors FTControl Panels, AdvantageScope Lite, and optional PsiKit
 * logs. Metrics are pushed through a non-blocking queue so that disk or network operations never
 * stall the OpMode loop.
 */
public class RobotLogger {

    public interface Source {
        /**
         * @return logical subsystem name used as the metric prefix (e.g. {@code Drive}).
         */
        String subsystem();

        /**
         * Called each time the logger samples sources. Use the provided frame to record numbers,
         * booleans, strings, and events.
         */
        void collect(Frame frame);
    }

    public interface Frame {
        void number(String key, double value);

        void bool(String key, boolean value);

        void string(String key, String value);

        void event(String eventName);
    }

    private enum LogType {
        NUMBER,
        BOOLEAN,
        STRING,
        EVENT,
        TAG
    }

    private static final int QUEUE_CAPACITY = 4096;
    private static final long WORKER_POLL_MS = 40L;

    private final TelemetryService telemetryService;
    private final BlockingQueue<RobotLogEntry> queue = new LinkedBlockingQueue<>(QUEUE_CAPACITY);
    private final List<Source> sources = new CopyOnWriteArrayList<>();

    private volatile boolean sessionActive = false;
    private volatile boolean workerRunning = false;
    private Thread workerThread;

    private String opModeName = "UnknownOpMode";
    private Alliance alliance = Alliance.UNKNOWN;
    private String phase = "";
    private long sessionStartMs = 0L;

    public RobotLogger(TelemetryService telemetryService) {
        this.telemetryService = telemetryService;
    }

    public synchronized void startSession(Context context, String opModeName, Alliance alliance, String phase) {
        if (sessionActive) {
            return;
        }
        this.opModeName = opModeName == null ? "UnknownOpMode" : opModeName;
        this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
        this.phase = phase == null ? "" : phase;
        this.sessionStartMs = System.currentTimeMillis();
        queue.clear();

        telemetryService.startSession();
        AdvLogger.init(context);

        workerRunning = true;
        workerThread = new Thread(this::runWorker, "RobotLogger-Worker");
        workerThread.setPriority(Thread.MIN_PRIORITY);
        workerThread.start();

        sessionActive = true;
        queueTag("OpMode", this.opModeName);
        queueTag("Phase", this.phase);
        queueTag("Alliance", this.alliance.name());
        logEvent("Session", "Begin");
    }

    public void updateAlliance(Alliance alliance) {
        if (alliance == null) {
            return;
        }
        if (!sessionActive) {
            this.alliance = alliance;
            return;
        }
        if (!Objects.equals(this.alliance, alliance)) {
            this.alliance = alliance;
            queueTag("Alliance", alliance.name());
        }
    }

    public void logNumber(String subsystem, String key, double value) {
        enqueue(createEntry(LogType.NUMBER, subsystem, key, value, false, Double.toString(value)));
        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("metrics/" + buildTopic(subsystem, key), value);
        }
    }

    public void logBoolean(String subsystem, String key, boolean value) {
        enqueue(createEntry(LogType.BOOLEAN, subsystem, key, value ? 1.0 : 0.0, value, value ? "true" : "false"));
        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("metrics/" + buildTopic(subsystem, key), value);
        }
    }

    public void logString(String subsystem, String key, String value) {
        enqueue(createEntry(LogType.STRING, subsystem, key, 0.0, false, value));
        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("metrics/" + buildTopic(subsystem, key), value);
        }
    }

    public void logEvent(String subsystem, String eventName) {
        enqueue(createEntry(LogType.EVENT, subsystem, eventName, 0.0, false, eventName));
        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("events/" + (subsystem == null ? "Robot" : subsystem), eventName);
        }
    }

    public synchronized void stopSession() {
        if (!sessionActive) {
            return;
        }
        logEvent("Session", "End");
        sessionActive = false;
        workerRunning = false;
        if (workerThread != null) {
            try {
                workerThread.join(250);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            workerThread = null;
        }
        AdvLogger.close();
        telemetryService.stopSession();
        queue.clear();
    }

    public TelemetryService telemetry() {
        return telemetryService;
    }

    public void registerSource(Source source) {
        if (source != null) {
            sources.add(source);
        }
    }

    public void unregisterSource(Source source) {
        if (source != null) {
            sources.remove(source);
        }
    }

    /**
     * Polls every registered source so they can publish metrics for the current loop.
     * Call this once per OpMode loop after subsystems update.
     */
    public void sampleSources() {
        if (!sessionActive || sources.isEmpty()) {
            return;
        }
        for (Source source : sources) {
            if (source == null) {
                continue;
            }
            try {
                source.collect(new FrameImpl(source.subsystem()));
            } catch (Exception e) {
                logEvent("Logger", "SourceError:" + e.getMessage());
            }
        }
    }

    private void queueTag(String key, String value) {
        enqueue(createEntry(LogType.TAG, "Session", key, 0.0, false, value));
    }

    private RobotLogEntry createEntry(LogType type, String subsystem, String key,
                                      double numeric, boolean boolValue, String stringValue) {
        if (!sessionActive) {
            return null;
        }
        long timestamp = System.currentTimeMillis();
        long relative = timestamp - sessionStartMs;
        String safeSubsystem = subsystem == null || subsystem.isEmpty() ? "Robot" : subsystem;
        String safeKey;
        switch (type) {
            case EVENT:
                safeKey = key == null || key.isEmpty() ? "Event" : key;
                break;
            case TAG:
                safeKey = key == null || key.isEmpty() ? "Tag" : key;
                break;
            default:
                safeKey = key == null || key.isEmpty() ? "Value" : key;
                break;
        }
        String topic = buildTopic(safeSubsystem, safeKey);
        String nonNullString = stringValue == null ? "" : stringValue;
        return new RobotLogEntry(
                type,
                safeSubsystem,
                safeKey,
                topic,
                numeric,
                boolValue,
                nonNullString,
                timestamp,
                relative,
                opModeName,
                alliance,
                phase
        );
    }

    private void enqueue(RobotLogEntry entry) {
        if (entry == null) {
            return;
        }
        while (!queue.offer(entry)) {
            queue.poll();
        }
    }

    private static String buildTopic(String subsystem, String key) {
        String safeSubsystem = subsystem == null || subsystem.isEmpty() ? "Robot" : subsystem;
        String safeKey = key == null || key.isEmpty() ? "Value" : key;
        return safeSubsystem + "/" + safeKey;
    }

    private void runWorker() {
        PsiKitAdapter psiKit = telemetryService.psiKitLogger();
        try {
            while (workerRunning || !queue.isEmpty()) {
                RobotLogEntry entry = queue.poll(WORKER_POLL_MS, TimeUnit.MILLISECONDS);
                if (entry == null) {
                    continue;
                }
                dispatch(entry, psiKit);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            if (psiKit != null) {
                psiKit.flush();
            }
        }
    }

    private void dispatch(RobotLogEntry entry, PsiKitAdapter psiKit) {
        switch (entry.type) {
            case NUMBER:
                AdvLogger.recordNumber(entry.topic, entry.numericValue);
                if (psiKit != null) {
                    psiKit.recordNumber(entry.topic, entry.numericValue);
                }
                break;
            case BOOLEAN:
                AdvLogger.recordBoolean(entry.topic, entry.booleanValue);
                if (psiKit != null) {
                    psiKit.recordBoolean(entry.topic, entry.booleanValue);
                }
                break;
            case STRING:
                AdvLogger.recordString(entry.topic, entry.stringValue);
                if (psiKit != null) {
                    psiKit.recordString(entry.topic, entry.stringValue);
                }
                break;
            case EVENT:
                AdvLogger.recordEvent(entry.topic);
                if (psiKit != null) {
                    psiKit.recordString("event/" + entry.subsystem, entry.stringValue);
                }
                break;
            case TAG:
                AdvLogger.tag(entry.key, entry.stringValue);
                if (psiKit != null) {
                    psiKit.recordString("meta/" + entry.key, entry.stringValue);
                }
                break;
            default:
                break;
        }
    }

    private static final class RobotLogEntry {
        final LogType type;
        final String subsystem;
        final String key;
        final String topic;
        final double numericValue;
        final boolean booleanValue;
        final String stringValue;
        final long timestampMs;
        final long relativeMs;
        final String opModeName;
        final Alliance alliance;
        final String phase;

        RobotLogEntry(LogType type,
                      String subsystem,
                      String key,
                      String topic,
                      double numericValue,
                      boolean booleanValue,
                      String stringValue,
                      long timestampMs,
                      long relativeMs,
                      String opModeName,
                      Alliance alliance,
                      String phase) {
            this.type = type;
            this.subsystem = subsystem;
            this.key = key;
            this.topic = topic;
            this.numericValue = numericValue;
            this.booleanValue = booleanValue;
            this.stringValue = stringValue;
            this.timestampMs = timestampMs;
            this.relativeMs = relativeMs;
            this.opModeName = opModeName;
            this.alliance = alliance;
            this.phase = phase;
        }
    }

    private final class FrameImpl implements Frame {
        private final String subsystem;

        FrameImpl(String subsystem) {
            this.subsystem = subsystem == null || subsystem.isEmpty() ? "Robot" : subsystem;
        }

        @Override
        public void number(String key, double value) {
            logNumber(subsystem, key, value);
        }

        @Override
        public void bool(String key, boolean value) {
            logBoolean(subsystem, key, value);
        }

        @Override
        public void string(String key, String value) {
            logString(subsystem, key, value);
        }

        @Override
        public void event(String eventName) {
            logEvent(subsystem, eventName);
        }
    }
}
