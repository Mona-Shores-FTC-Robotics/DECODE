package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.os.Environment;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Parses a Pedro visualizer .pp file (https://visualizer.pedropathing.com/) into a
 * pure-data representation. Caller turns this into a PathChain.
 *
 * <p>What's parsed:
 * <ul>
 *   <li>{@code startPoint.x/y} — chain start position.</li>
 *   <li>{@code sequence[]} — execution order of lines (NOT {@code pathChains[0].lineIds},
 *       which is a UI grouping that may not match runtime order).</li>
 *   <li>Per line: name, endpoint x/y, control points, heading mode (linear or constant),
 *       startDeg / endDeg / degrees.</li>
 * </ul>
 *
 * <p>What's not parsed: visualizer settings (velocity, friction, robot image),
 * waitBeforeMs / waitAfterMs (no Pedro hook yet), shapes. {@code reverse} IS parsed
 * and exposed on {@link ParsedLine#reverse}; caller wires it to
 * {@code PathBuilder.setReversed()}.
 */
public final class PpPathLoader {

    public enum HeadingMode { LINEAR, CONSTANT }

    public static final class ParsedLine {
        public final String name;
        public final double endX, endY;
        public final HeadingMode headingMode;
        /** For LINEAR: heading at start of segment. */
        public final double startDeg;
        /** For LINEAR: heading at end of segment. For CONSTANT: ignored. */
        public final double endDeg;
        /** For CONSTANT: the held heading. For LINEAR: ignored. */
        public final double constantDeg;
        public final List<double[]> controlPoints;
        public final boolean reverse;

        public ParsedLine(String name, double endX, double endY,
                          HeadingMode headingMode, double startDeg, double endDeg, double constantDeg,
                          List<double[]> controlPoints, boolean reverse) {
            this.name = name;
            this.endX = endX;
            this.endY = endY;
            this.headingMode = headingMode;
            this.startDeg = startDeg;
            this.endDeg = endDeg;
            this.constantDeg = constantDeg;
            this.controlPoints = controlPoints;
            this.reverse = reverse;
        }
    }

    public static final class ParsedPp {
        public final double startX, startY;
        /** Heading the robot starts at (from first segment's startDeg, fallback startPoint.endDeg). */
        public final double startHeadingDeg;
        public final List<ParsedLine> lines;

        public ParsedPp(double startX, double startY, double startHeadingDeg, List<ParsedLine> lines) {
            this.startX = startX;
            this.startY = startY;
            this.startHeadingDeg = startHeadingDeg;
            this.lines = lines;
        }
    }

    private PpPathLoader() {}

    public static ParsedPp loadFromAssets(Context ctx, String assetName) throws IOException, JSONException {
        try (InputStream in = ctx.getAssets().open(assetName)) {
            return parse(slurp(in));
        }
    }

    /** Loads from {@code /sdcard/FIRST/<name>}. Returns null if file doesn't exist. */
    public static ParsedPp loadFromSdcardOrNull(String name) {
        File f = new File(Environment.getExternalStorageDirectory(), "FIRST/" + name);
        if (!f.exists() || !f.canRead()) return null;
        try (InputStream in = new FileInputStream(f)) {
            return parse(slurp(in));
        } catch (IOException | JSONException e) {
            return null;
        }
    }

    public static ParsedPp parse(String json) throws JSONException {
        JSONObject root = new JSONObject(json);

        JSONObject startObj = root.getJSONObject("startPoint");
        double startX = startObj.getDouble("x");
        double startY = startObj.getDouble("y");

        // Index every line by id so we can read them in sequence order.
        JSONArray linesArr = root.getJSONArray("lines");
        Map<String, JSONObject> byId = new HashMap<>();
        for (int i = 0; i < linesArr.length(); i++) {
            JSONObject line = linesArr.getJSONObject(i);
            byId.put(line.getString("id"), line);
        }

        // sequence[] is the runtime execution order (pathChains[0].lineIds may be in a
        // different order — that's a UI grouping, not the path the robot follows).
        JSONArray seq = root.getJSONArray("sequence");
        List<ParsedLine> ordered = new ArrayList<>(seq.length());
        for (int i = 0; i < seq.length(); i++) {
            JSONObject entry = seq.getJSONObject(i);
            if (!"path".equals(entry.optString("kind", "path"))) continue;
            String lineId = entry.getString("lineId");
            JSONObject line = byId.get(lineId);
            if (line == null) continue;
            ordered.add(parseLine(line));
        }

        // Start heading: first segment's startDeg is the most reliable source
        // (it's what setLinearHeadingInterpolation will interpolate FROM).
        // Fall back to startPoint.endDeg if no segments.
        double startHeadingDeg = ordered.isEmpty()
                ? startObj.optDouble("endDeg", 0.0)
                : ordered.get(0).startDeg;

        return new ParsedPp(startX, startY, startHeadingDeg, ordered);
    }

    private static ParsedLine parseLine(JSONObject line) throws JSONException {
        String name = line.optString("name", "");
        JSONObject end = line.getJSONObject("endPoint");
        double endX = end.getDouble("x");
        double endY = end.getDouble("y");
        boolean reverse = end.optBoolean("reverse", false);

        String mode = end.optString("heading", "linear");
        HeadingMode headingMode = "constant".equalsIgnoreCase(mode)
                ? HeadingMode.CONSTANT
                : HeadingMode.LINEAR;
        double startDeg = end.optDouble("startDeg", 0.0);
        double endDeg = end.optDouble("endDeg", 0.0);
        double constantDeg = end.optDouble("degrees", endDeg);

        JSONArray ctrlArr = line.optJSONArray("controlPoints");
        List<double[]> controls = new ArrayList<>();
        if (ctrlArr != null) {
            for (int i = 0; i < ctrlArr.length(); i++) {
                JSONObject c = ctrlArr.getJSONObject(i);
                controls.add(new double[]{ c.getDouble("x"), c.getDouble("y") });
            }
        }
        return new ParsedLine(name, endX, endY, headingMode, startDeg, endDeg, constantDeg, controls, reverse);
    }

    private static String slurp(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader r = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
            String line;
            while ((line = r.readLine()) != null) sb.append(line).append('\n');
        }
        return sb.toString();
    }
}
