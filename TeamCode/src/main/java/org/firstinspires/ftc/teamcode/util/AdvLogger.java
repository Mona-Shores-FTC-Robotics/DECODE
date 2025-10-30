package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;

/**
 * Lightweight wrapper around AdvantageScope Lite that gracefully degrades when the dependency
 * is unavailable. All calls fall back to simple console prints so developers can still test
 * without the library on the classpath.
 */
public final class AdvLogger {

    private static boolean initialized = false;
    private static boolean available = false;

    private static Class<?> liteClass;
    private static Method startMethod;
    private static Method stopMethod;
    private static Method recordNumberMethod;
    private static Method recordBooleanMethod;
    private static Method recordEventMethod;

    private AdvLogger() {}

    public static synchronized void init(Context context) {
        if (initialized) {
            return;
        }
        initialized = true;

        try {
            liteClass = Class.forName("org.advantagescope.ftc.AdvantageScopeLite");
            discoverMethods();
            if (startMethod != null) {
                startMethod.invoke(null, context);
                available = true;
                System.out.println("[AdvLogger] AdvantageScope Lite initialized.");
            } else {
                System.out.println("[AdvLogger] AdvantageScope Lite detected but start(context) method not found.");
            }
        } catch (ClassNotFoundException e) {
            System.out.println("[AdvLogger] AdvantageScope Lite dependency not present; falling back to console logging.");
        } catch (IllegalAccessException | InvocationTargetException e) {
            System.out.println("[AdvLogger] Failed to initialise AdvantageScope Lite: " + e.getMessage());
        }
    }

    public static void recordNumber(String topic, double value) {
        if (!initialized) {
            return;
        }
        if (available && recordNumberMethod != null) {
            try {
                recordNumberMethod.invoke(null, topic, value);
                return;
            } catch (IllegalAccessException | InvocationTargetException e) {
                System.out.println("[AdvLogger] record number failed: " + e.getMessage());
            }
        }
        System.out.printf("[AdvLogger] %s = %.3f%n", topic, value);
    }

    /**
     * Backwards-compatible synonym for {@link #recordNumber(String, double)}.
     * Kept so that legacy call sites continue to function.
     */
    public static void record(String topic, double value) {
        recordNumber(topic, value);
    }

    public static void recordBoolean(String topic, boolean value) {
        if (!initialized) {
            return;
        }
        if (available) {
            if (recordBooleanMethod != null) {
                try {
                    recordBooleanMethod.invoke(null, topic, value);
                    return;
                } catch (IllegalAccessException | InvocationTargetException e) {
                    System.out.println("[AdvLogger] record boolean failed: " + e.getMessage());
                }
            }
            record(topic, value ? 1.0 : 0.0);
            return;
        }
        System.out.printf("[AdvLogger] %s = %s%n", topic, value);
    }

    public static void recordEvent(String eventName) {
        if (!initialized) {
            return;
        }
        if (available && recordEventMethod != null) {
            try {
                recordEventMethod.invoke(null, eventName);
                return;
            } catch (IllegalAccessException | InvocationTargetException e) {
                System.out.println("[AdvLogger] record event failed: " + e.getMessage());
            }
        }
        System.out.println("[AdvLogger] Event: " + eventName);
    }

    /**
     * Records a metadata tag for the current log session. AdvantageScope Lite models tags as string
     * key/value pairs that annotate the dataset. When the runtime does not expose a dedicated API,
     * the tag is emitted as an Event so that the information is still captured.
     */
    public static void tag(String key, String value) {
        if (!initialized) {
            return;
        }
        if (key == null) {
            key = "tag";
        }
        if (value == null) {
            value = "";
        }
        String topic = "Tag/" + key;
        if (available) {
            recordString(topic, value);
        } else {
            System.out.printf("[AdvLogger] Tag: %s = %s%n", key, value);
        }
    }

    public static void recordString(String topic, String value) {
        if (!initialized) {
            return;
        }
        if (available) {
            // AdvantageScope Lite treats strings as events; fall back to boolean/number if needed.
            recordEvent(topic + ":" + value);
            return;
        }
        System.out.printf("[AdvLogger] %s = \"%s\"%n", topic, value);
    }

    public static synchronized void close() {
        if (!initialized) {
            return;
        }
        initialized = false;
        if (available && stopMethod != null) {
            try {
                stopMethod.invoke(null);
            } catch (IllegalAccessException | InvocationTargetException e) {
                System.out.println("[AdvLogger] stop() failed: " + e.getMessage());
            }
        }
        available = false;
        System.out.println("[AdvLogger] AdvantageScope Lite stopped.");
    }

    private static void discoverMethods() {
        Method[] methods = liteClass.getMethods();
        for (Method method : methods) {
            Class<?>[] params = method.getParameterTypes();
            if (startMethod == null && params.length == 1 && params[0] == Context.class) {
                startMethod = method;
            } else if (stopMethod == null && params.length == 0 && method.getReturnType() == Void.TYPE) {
                if (method.getName().equalsIgnoreCase("stop") || method.getName().equalsIgnoreCase("shutdown")) {
                    stopMethod = method;
                }
            } else if (recordNumberMethod == null && params.length == 2 && params[0] == String.class && params[1] == double.class) {
                recordNumberMethod = method;
            } else if (recordBooleanMethod == null && params.length == 2 && params[0] == String.class && params[1] == boolean.class) {
                recordBooleanMethod = method;
            } else if (recordEventMethod == null && params.length == 1 && params[0] == String.class
                    && method.getName().toLowerCase().contains("event")) {
                recordEventMethod = method;
            }
        }

        if (recordNumberMethod == null) {
            recordNumberMethod = findByName(methods, new String[]{"recordNumber", "logNumber", "record"});
        }
        if (recordEventMethod == null) {
            recordEventMethod = findByName(methods, new String[]{"recordEvent", "logEvent", "event"});
        }
        if (stopMethod == null) {
            stopMethod = findByName(methods, new String[]{"stop", "shutdown"});
        }
    }

    private static Method findByName(Method[] methods, String[] names) {
        for (String candidate : names) {
            for (Method method : methods) {
                if (method.getName().equalsIgnoreCase(candidate)) {
                    return method;
                }
            }
        }
        System.out.println("[AdvLogger] Unable to resolve methods for " + Arrays.toString(names));
        return null;
    }
}
