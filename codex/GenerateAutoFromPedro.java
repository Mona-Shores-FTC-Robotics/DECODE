import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Converts a Pedro Pathing .pp file into a complete autonomous routine with editable actions.
 *
 * Usage:
 *   javac codex/GenerateAutoFromPedro.java
 *   java -cp codex GenerateAutoFromPedro autonomous_close_blue.pp
 *
 * Generates:
 * 1. Waypoint configuration class (paste into CloseAutoWaypoints)
 * 2. Complete buildAutonomousRoutine() method with placeholders for robot actions
 */
public class GenerateAutoFromPedro {

    static final double FIELD_WIDTH = 144.0;

    public static void main(String[] args) {
        if (args.length < 1) {
            System.err.println("Usage: java GenerateAutoFromPedro <path-to-pp-file>");
            System.err.println("Example: java GenerateAutoFromPedro autonomous_close_blue.pp");
            System.exit(1);
        }

        String filename = args[0];
        boolean isRed = filename.toLowerCase().contains("red");

        try {
            String json = new String(Files.readAllBytes(Paths.get(filename)));

            System.out.println("=== Generated Autonomous Code ===");
            System.out.println("// Source: " + filename);
            System.out.println();

            // Extract all path segments
            List<PathSegment> segments = extractAllSegments(json, isRed);

            // Generate waypoint configuration
            generateWaypointConfig(segments, isRed, json);

            System.out.println();
            System.out.println("// ==========================================");
            System.out.println();

            // Generate buildAutonomousRoutine method
            generateRoutineMethod(segments);

        } catch (IOException e) {
            System.err.println("ERROR reading file: " + e.getMessage());
            System.exit(1);
        }
    }

    static List<PathSegment> extractAllSegments(String json, boolean isRed) {
        List<PathSegment> segments = new ArrayList<>();

        // Extract start point
        Pose start = extractStartPoint(json, isRed);

        // Find all path segments in the "lines" array
        Pattern linePattern = Pattern.compile("\\{\"name\":\"([^\"]+)\"[^}]*\"endPoint\":\\{([^}]+)\\}[^}]*\\}");
        Matcher m = linePattern.matcher(json);

        Pose previousEnd = start;
        int index = 0;

        while (m.find()) {
            String name = m.group(1);
            String endPointData = m.group(2);
            Pose end = parseEndPoint(endPointData, isRed);

            segments.add(new PathSegment(index, name, previousEnd, end));
            previousEnd = end;
            index++;
        }

        return segments;
    }

    static void generateWaypointConfig(List<PathSegment> segments, boolean isRed, String json) {
        System.out.println("// ========== STEP 1: Copy into CloseAutoWaypoints ==========");
        System.out.println();

        // Start point
        Pose start = extractStartPoint(json, isRed);
        System.out.println("public double startX = " + start.x + ";");
        System.out.println("public double startY = " + start.y + ";");
        System.out.println("public double startHeading = " + start.heading + ";");
        System.out.println();

        // All waypoints
        for (PathSegment seg : segments) {
            String varName = "waypoint" + seg.index;
            System.out.println("// " + seg.name);
            System.out.println("public double " + varName + "X = " + seg.end.x + ";");
            System.out.println("public double " + varName + "Y = " + seg.end.y + ";");
            System.out.println("public double " + varName + "Heading = " + seg.end.heading + ";");
            System.out.println();
        }
    }

    static void generateRoutineMethod(List<PathSegment> segments) {
        System.out.println("// ========== STEP 2: Copy into DecodeAutonomousCloseCommand ==========");
        System.out.println("// Replace buildAutonomousRoutine() method with this:");
        System.out.println();
        System.out.println("private Command buildAutonomousRoutine() {");
        System.out.println("    if (currentLayout == null) {");
        System.out.println("        return new Delay(0.01);");
        System.out.println("    }");
        System.out.println();
        System.out.println("    return new SequentialGroup(");

        for (int i = 0; i < segments.size(); i++) {
            PathSegment seg = segments.get(i);
            String startWaypoint = i == 0 ? "START" : "waypoint" + (i - 1);
            String endWaypoint = "waypoint" + i;

            System.out.println();
            System.out.println("            // " + seg.name);

            // Check if this is a movement to a collection point
            boolean isCollecting = seg.name.toLowerCase().contains("collect") ||
                                   seg.name.toLowerCase().contains("pickup") ||
                                   seg.name.toLowerCase().contains("artifact");

            // Check if this is returning to score
            boolean isScoring = seg.name.toLowerCase().contains("score") ||
                               seg.name.toLowerCase().contains("launch") ||
                               seg.name.toLowerCase().contains("basket");

            if (isCollecting) {
                System.out.println("            new ParallelGroup(");
                System.out.println("                followPath(pose(Waypoint." + startWaypoint + "), pose(Waypoint." + endWaypoint + ")),");
                System.out.println("                // TODO: Add intake/collection commands here");
                System.out.println("                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))");
                System.out.println("            ),");
                System.out.println("            new Delay(config.intakeDelaySeconds), // Wait for collection");
            } else if (isScoring) {
                System.out.println("            new ParallelGroup(");
                System.out.println("                followPath(pose(Waypoint." + startWaypoint + "), pose(Waypoint." + endWaypoint + ")),");
                System.out.println("                // TODO: Add launcher spin-up commands here");
                System.out.println("                spinUpLauncher()");
                System.out.println("            ),");
                System.out.println("            // TODO: Add scoring sequence here");
                System.out.println("            scoreSequence(),");
            } else {
                System.out.println("            followPath(pose(Waypoint." + startWaypoint + "), pose(Waypoint." + endWaypoint + ")),");
                System.out.println("            // TODO: Add any commands for this segment");
            }

            if (i < segments.size() - 1) {
                System.out.println();
            }
        }

        System.out.println("    );");
        System.out.println("}");
        System.out.println();
        System.out.println("// ========== STEP 3: Add Waypoint Enum ==========");
        System.out.println("// Add these to your Waypoint enum:");
        System.out.println();
        System.out.println("public enum Waypoint {");
        System.out.println("    START,");
        for (int i = 0; i < segments.size(); i++) {
            System.out.println("    WAYPOINT" + i + (i < segments.size() - 1 ? "," : ""));
        }
        System.out.println("}");
        System.out.println();
        System.out.println("// ========== STEP 4: Update pose() method ==========");
        System.out.println("// Add these cases to your pose() switch statement:");
        System.out.println();
        System.out.println("case START:");
        System.out.println("    return poseForAlliance(w.startX, w.startY, w.startHeading, activeAlliance);");
        for (int i = 0; i < segments.size(); i++) {
            System.out.println("case WAYPOINT" + i + ":");
            System.out.println("    return poseForAlliance(w.waypoint" + i + "X, w.waypoint" + i + "Y, w.waypoint" + i + "Heading, activeAlliance);");
        }
    }

    static Pose extractStartPoint(String json, boolean isRed) {
        String pattern = "\"startPoint\":\\{([^}]+)\\}";
        Pattern p = Pattern.compile(pattern);
        Matcher m = p.matcher(json);

        if (m.find()) {
            String data = m.group(1);
            double x = extractNumber(data, "\"x\"");
            double y = extractNumber(data, "\"y\"");
            double heading = extractNumber(data, "\"startDeg\"");

            if (isRed) {
                x = FIELD_WIDTH - x;
                heading = 180.0 - heading;
            }

            return new Pose(x, y, heading);
        }

        return new Pose(0, 0, 0);
    }

    static Pose parseEndPoint(String data, boolean isRed) {
        double x = extractNumber(data, "\"x\"");
        double y = extractNumber(data, "\"y\"");
        double heading = extractNumber(data, "\"endDeg\"");

        if (isRed) {
            x = FIELD_WIDTH - x;
            heading = 180.0 - heading;
        }

        return new Pose(x, y, heading);
    }

    static double extractNumber(String data, String key) {
        String pattern = key + ":([-\\d.]+)";
        Pattern p = Pattern.compile(pattern);
        Matcher m = p.matcher(data);

        if (m.find()) {
            return Double.parseDouble(m.group(1));
        }

        return 0.0;
    }

    static class Pose {
        double x, y, heading;
        Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    static class PathSegment {
        int index;
        String name;
        Pose start;
        Pose end;

        PathSegment(int index, String name, Pose start, Pose end) {
            this.index = index;
            this.name = name;
            this.start = start;
            this.end = end;
        }
    }
}
