import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Converts a Pedro Pathing .pp file back into CloseAutoWaypoints Java code.
 *
 * Usage:
 *   javac codex/ConvertPedroPath.java
 *   java -cp codex ConvertPedroPath autonomous_close_blue.pp
 *
 * Outputs Java code you can copy into CloseAutoWaypoints class.
 */
public class ConvertPedroPath {

    static final double FIELD_WIDTH = 144.0;

    public static void main(String[] args) {
        if (args.length < 1) {
            System.err.println("Usage: java ConvertPedroPath <path-to-pp-file>");
            System.err.println("Example: java ConvertPedroPath autonomous_close_blue.pp");
            System.exit(1);
        }

        String filename = args[0];
        boolean isRed = filename.toLowerCase().contains("red");

        try {
            String json = new String(Files.readAllBytes(Paths.get(filename)));

            System.out.println("=== Converted Waypoints ===");
            System.out.println();
            System.out.println("// Copy these values into CloseAutoWaypoints class:");
            System.out.println("// (Converted from: " + filename + ")");
            System.out.println();

            // Extract start point
            Pose start = extractPose(json, "\"startPoint\"", isRed);
            System.out.println("public double startX = " + start.x + ";");
            System.out.println("public double startY = " + start.y + ";");
            System.out.println("public double startHeading = " + start.heading + ";");
            System.out.println();

            // Extract waypoints from path names
            extractWaypoint(json, "To Launch", "launch", isRed);
            extractWaypoint(json, "To Pre-Artifacts", "preArtifacts", isRed);
            extractWaypoint(json, "Collect Set 1", "artifacts1", isRed);
            extractWaypoint(json, "To Transition 2", "transition2", isRed);
            extractWaypoint(json, "Collect Set 2", "artifacts2", isRed);
            extractWaypoint(json, "To Transition 3", "transition3", isRed);
            extractWaypoint(json, "Collect Set 3", "artifacts3", isRed);
            extractWaypoint(json, "To Final", "final", isRed);

            System.out.println();
            System.out.println("=== DONE ===");
            System.out.println();
            System.out.println("Note: If this was a RED alliance .pp file, coordinates have been");
            System.out.println("mirrored back to BLUE coordinates (which is what the code stores).");

        } catch (IOException e) {
            System.err.println("ERROR reading file: " + e.getMessage());
            System.exit(1);
        }
    }

    static void extractWaypoint(String json, String pathName, String varName, boolean isRed) {
        // Find the path with this name
        String pattern = "\"name\":\"" + Pattern.quote(pathName) + "\".*?\"endPoint\":\\{([^}]+)\\}";
        Pattern p = Pattern.compile(pattern);
        Matcher m = p.matcher(json);

        if (m.find()) {
            String endPointData = m.group(1);
            Pose pose = parseEndPoint(endPointData, isRed);

            System.out.println("public double " + varName + "X = " + pose.x + ";");
            System.out.println("public double " + varName + "Y = " + pose.y + ";");
            System.out.println("public double " + varName + "Heading = " + pose.heading + ";");
            System.out.println();
        } else {
            System.out.println("// WARNING: Could not find path: " + pathName);
            System.out.println();
        }
    }

    static Pose extractPose(String json, String key, boolean isRed) {
        // Extract the start point or endpoint object
        String pattern = key + ":\\{([^}]+)\\}";
        Pattern p = Pattern.compile(pattern);
        Matcher m = p.matcher(json);

        if (m.find()) {
            String data = m.group(1);
            return parseEndPoint(data, isRed);
        }

        return new Pose(0, 0, 0);
    }

    static Pose parseEndPoint(String data, boolean isRed) {
        double x = extractNumber(data, "\"x\"");
        double y = extractNumber(data, "\"y\"");
        double heading = extractNumber(data, "\"endDeg\"");

        // If this is from a red .pp file, mirror back to blue coordinates
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
}
