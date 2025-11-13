package org.firstinspires.ftc.teamcode.telemetry;

import android.content.Context;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Unified logging surface that collects subsystem Inputs and publishes them to FTC Dashboard
 * packets and FullPanels telemetry. Topics are stored synchronously in a Map for retrieval
 * by TelemetryService.
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

    private final TelemetryService telemetryService;
    private final Map<String, Object> currentTopics = new ConcurrentHashMap<>();
    private final List<Source> sources = new CopyOnWriteArrayList<>();
    private final Map<Class<?>, Field[]> inputFieldCache = new ConcurrentHashMap<>();

    private volatile boolean sessionActive = false;

    private String opModeName = "UnknownOpMode";
    private Alliance alliance = Alliance.UNKNOWN;
    private String phase = "";

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
        currentTopics.clear();

        telemetryService.startSession();

        sessionActive = true;
        logString("Session", "OpMode", this.opModeName);
        logString("Session", "Phase", this.phase);
        logString("Session", "Alliance", this.alliance.name());
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
            logString("Session", "Alliance", alliance.name());
        }
    }

    public void logNumber(String subsystem, String key, double value) {
        if (!sessionActive) {
            return;
        }
        String topic = buildTopic(subsystem, key);
        currentTopics.put(topic, value);

        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("metrics/" + topic, value);
        }
    }

    public void logBoolean(String subsystem, String key, boolean value) {
        if (!sessionActive) {
            return;
        }
        String topic = buildTopic(subsystem, key);
        currentTopics.put(topic, value);

        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("metrics/" + topic, value);
        }
    }

    public void logString(String subsystem, String key, String value) {
        if (!sessionActive) {
            return;
        }
        String topic = buildTopic(subsystem, key);
        currentTopics.put(topic, value != null ? value : "");

        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("metrics/" + topic, value);
        }
    }

    public void logEvent(String subsystem, String eventName) {
        if (!sessionActive) {
            return;
        }
        TelemetryManager panels = telemetryService.panelsTelemetry();
        if (panels != null) {
            panels.debug("events/" + (subsystem == null ? "Robot" : subsystem), eventName);
        }
    }

    /**
     * Returns a snapshot of all topics collected since the last clear.
     * Used by TelemetryService to publish to FTC Dashboard packets.
     */
    public Map<String, Object> collectAllTopics() {
        return new HashMap<>(currentTopics);
    }

    /**
     * Clears all collected topics. Called after publishing to Dashboard packet.
     */
    public void clearTopics() {
        currentTopics.clear();
    }

    public synchronized void stopSession() {
        if (!sessionActive) {
            return;
        }
        logEvent("Session", "End");
        sessionActive = false;
        telemetryService.stopSession();
        currentTopics.clear();
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

    private static String buildTopic(String subsystem, String key) {
        String safeSubsystem = subsystem == null || subsystem.isEmpty() ? "Robot" : subsystem;
        String safeKey = key == null || key.isEmpty() ? "Value" : key;
        return safeSubsystem + "/" + safeKey;
    }

    public void logInputs(String subsystem, Object inputs) {
        if (inputs == null) {
            return;
        }
        Field[] fields = inputFieldCache.computeIfAbsent(inputs.getClass(), this::introspectFields);
        for (Field field : fields) {
            Object value;
            try {
                value = field.get(inputs);
            } catch (IllegalAccessException e) {
                continue;
            }
            if (value == null) {
                continue;
            }
            String key = field.getName();
            if (value instanceof Number) {
                logNumber(subsystem, key, ((Number) value).doubleValue());
            } else if (value instanceof Boolean) {
                logBoolean(subsystem, key, (Boolean) value);
            } else if (value instanceof CharSequence) {
                logString(subsystem, key, value.toString());
            } else if (value instanceof Enum<?>) {
                Enum<?> enumValue = (Enum<?>) value;
                logString(subsystem, key, enumValue.name());
            } else {
                logString(subsystem, key, String.valueOf(value));
            }
        }
    }

    private Field[] introspectFields(Class<?> type) {
        return Arrays.stream(type.getDeclaredFields())
                .filter(field -> !Modifier.isStatic(field.getModifiers()))
                .filter(field -> !field.isSynthetic())
                .peek(field -> {
                    try {
                        field.setAccessible(true);
                    } catch (Exception ignored) {
                        // Ignore; best-effort exposure.
                    }
                })
                .toArray(Field[]::new);
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
