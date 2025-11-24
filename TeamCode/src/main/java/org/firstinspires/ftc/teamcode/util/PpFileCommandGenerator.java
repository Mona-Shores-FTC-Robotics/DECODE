package org.firstinspires.ftc.teamcode.util;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Generates autonomous command Java files from Pedro Pathing .pp files.
 *
 * Usage:
 * PpFileCommandGenerator.generate(
 *     "trajectory.pp",
 *     "TrajectoryAuto",
 *     "/path/to/TeamCode/src/main/assets/",
 *     "/path/to/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/generated/"
 * );
 */
public class PpFileCommandGenerator {

    private static class WaypointData {
        String name;
        double x, y, headingDeg;

        WaypointData(String name, double x, double y, double headingDeg) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }
    }

    private static class PathData {
        String name;
        WaypointData start;
        WaypointData end;
        String headingMode;  // "linear", "tangential", "constant"
        boolean reverse;
        List<double[]> controlPoints;

        PathData(String name, WaypointData start, WaypointData end, String headingMode, boolean reverse) {
            this.name = name;
            this.start = start;
            this.end = end;
            this.headingMode = headingMode;
            this.reverse = reverse;
            this.controlPoints = new ArrayList<>();
        }
    }

    /**
     * Generate autonomous command files from a .pp file
     *
     * @param ppFileName Name of .pp file (e.g., "trajectory.pp")
     * @param commandName Base name for generated command (e.g., "TrajectoryAuto")
     * @param ppFilePath Path to directory containing .pp file
     * @param outputPath Path to output directory for generated files
     */
    public static void generate(String ppFileName, String commandName, String ppFilePath, String outputPath)
            throws IOException, JSONException {

        // Read .pp file
        File ppFile = new File(ppFilePath, ppFileName);
        StringBuilder jsonContent = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new FileReader(ppFile))) {
            String line;
            while ((line = reader.readLine()) != null) {
                jsonContent.append(line);
            }
        }

        // Parse .pp file
        List<PathData> paths = parsePpFile(jsonContent.toString());

        // Generate config file
        String configCode = generateConfigFile(commandName, paths);
        File configFile = new File(outputPath, commandName + "Config.java");
        try (FileWriter writer = new FileWriter(configFile)) {
            writer.write(configCode);
        }

        // Generate command file
        String commandCode = generateCommandFile(commandName, paths);
        File commandFile = new File(outputPath, commandName + "Command.java");
        try (FileWriter writer = new FileWriter(commandFile)) {
            writer.write(commandCode);
        }

        System.out.println("Generated files:");
        System.out.println("  " + configFile.getAbsolutePath());
        System.out.println("  " + commandFile.getAbsolutePath());
    }

    private static List<PathData> parsePpFile(String jsonContent) throws JSONException {
        JSONObject root = new JSONObject(jsonContent);
        JSONObject startPointJson = root.getJSONObject("startPoint");
        JSONArray linesJson = root.getJSONArray("lines");

        List<PathData> paths = new ArrayList<>();

        // Parse start point
        double startX = startPointJson.getDouble("x");
        double startY = startPointJson.getDouble("y");
        double startHeadingDeg = startPointJson.optDouble("startDeg", startPointJson.optDouble("endDeg", 0));
        WaypointData currentWaypoint = new WaypointData("start", startX, startY, startHeadingDeg);

        // Parse each path segment
        for (int i = 0; i < linesJson.length(); i++) {
            JSONObject lineJson = linesJson.getJSONObject(i);

            String name = lineJson.optString("name", "path" + (i + 1));
            JSONObject endPointJson = lineJson.getJSONObject("endPoint");

            double endX = endPointJson.getDouble("x");
            double endY = endPointJson.getDouble("y");
            double endHeadingDeg = endPointJson.optDouble("degrees",
                                   endPointJson.optDouble("endDeg",
                                   endPointJson.optDouble("startDeg", 0)));

            String headingMode = endPointJson.optString("heading", "linear");
            boolean reverse = endPointJson.optBoolean("reverse", false);

            // Sanitize name to be valid Java identifier
            String sanitizedName = sanitizeName(name);
            WaypointData endWaypoint = new WaypointData(sanitizedName, endX, endY, endHeadingDeg);

            PathData pathData = new PathData(sanitizedName, currentWaypoint, endWaypoint, headingMode, reverse);

            // Parse control points if any
            JSONArray controlPointsJson = lineJson.optJSONArray("controlPoints");
            if (controlPointsJson != null) {
                for (int j = 0; j < controlPointsJson.length(); j++) {
                    JSONObject cpJson = controlPointsJson.getJSONObject(j);
                    double cpX = cpJson.getDouble("x");
                    double cpY = cpJson.getDouble("y");
                    pathData.controlPoints.add(new double[]{cpX, cpY});
                }
            }

            paths.add(pathData);
            currentWaypoint = endWaypoint;
        }

        return paths;
    }

    private static String sanitizeName(String name) {
        // Convert to camelCase Java identifier
        String sanitized = name.replaceAll("[^a-zA-Z0-9_]", "_");

        // Convert to camelCase
        StringBuilder result = new StringBuilder();
        boolean capitalizeNext = false;
        for (int i = 0; i < sanitized.length(); i++) {
            char c = sanitized.charAt(i);
            if (c == '_') {
                capitalizeNext = true;
            } else {
                if (capitalizeNext) {
                    result.append(Character.toUpperCase(c));
                    capitalizeNext = false;
                } else {
                    result.append(i == 0 ? Character.toLowerCase(c) : c);
                }
            }
        }

        return result.toString();
    }

    private static String generateConfigFile(String commandName, List<PathData> paths) {
        StringBuilder sb = new StringBuilder();

        sb.append("package org.firstinspires.ftc.teamcode.commands.generated;\n\n");
        sb.append("import com.bylazar.configurables.annotations.Configurable;\n");
        sb.append("import com.pedropathing.geometry.Pose;\n\n");
        sb.append("/**\n");
        sb.append(" * AUTO-GENERATED configuration for ").append(commandName).append("\n");
        sb.append(" * \n");
        sb.append(" * Waypoints and path settings are Dashboard-tunable.\n");
        sb.append(" * Regenerate this file when you change the path in Pedro GUI.\n");
        sb.append(" */\n");
        sb.append("@Configurable\n");
        sb.append("public class ").append(commandName).append("Config {\n\n");

        // Generate Waypoints class
        sb.append("    // ===== WAYPOINTS (Dashboard tunable) =====\n");
        sb.append("    @Configurable\n");
        sb.append("    public static class Waypoints {\n");

        // Start waypoint
        WaypointData startWaypoint = paths.get(0).start;
        sb.append("        \n");
        sb.append("        @Configurable\n");
        sb.append("        public static class Start {\n");
        sb.append("            public double x = ").append(startWaypoint.x).append(";\n");
        sb.append("            public double y = ").append(startWaypoint.y).append(";\n");
        sb.append("            public double headingDeg = ").append(startWaypoint.headingDeg).append(";\n");
        sb.append("            \n");
        sb.append("            public Pose toPose() {\n");
        sb.append("                return new Pose(x, y, Math.toRadians(headingDeg));\n");
        sb.append("            }\n");
        sb.append("        }\n");
        sb.append("        public static Start start = new Start();\n");

        // End waypoints
        for (PathData path : paths) {
            String className = capitalize(path.end.name);
            sb.append("        \n");
            sb.append("        @Configurable\n");
            sb.append("        public static class ").append(className).append(" {\n");
            sb.append("            public double x = ").append(path.end.x).append(";\n");
            sb.append("            public double y = ").append(path.end.y).append(";\n");
            sb.append("            public double headingDeg = ").append(path.end.headingDeg).append(";\n");
            sb.append("            \n");
            sb.append("            public Pose toPose() {\n");
            sb.append("                return new Pose(x, y, Math.toRadians(headingDeg));\n");
            sb.append("            }\n");
            sb.append("        }\n");
            sb.append("        public static ").append(className).append(" ").append(path.end.name).append(" = new ").append(className).append("();\n");
        }

        sb.append("    }\n\n");

        // Generate PathPower class
        sb.append("    // ===== PATH POWER SETTINGS (Dashboard tunable) =====\n");
        sb.append("    @Configurable\n");
        sb.append("    public static class PathPower {\n");
        for (PathData path : paths) {
            sb.append("        public double ").append(path.name).append(" = 0.8;\n");
        }
        sb.append("    }\n");
        sb.append("    public static PathPower pathPower = new PathPower();\n");

        sb.append("}\n");

        return sb.toString();
    }

    private static String generateCommandFile(String commandName, List<PathData> paths) {
        StringBuilder sb = new StringBuilder();

        sb.append("package org.firstinspires.ftc.teamcode.commands.generated;\n\n");
        sb.append("import static org.firstinspires.ftc.teamcode.commands.generated.").append(commandName).append("Config.*;\n\n");
        sb.append("import com.pedropathing.geometry.BezierLine;\n");
        sb.append("import com.pedropathing.geometry.BezierCurve;\n");
        sb.append("import com.pedropathing.geometry.Pose;\n\n");
        sb.append("import org.firstinspires.ftc.teamcode.Robot;\n");
        sb.append("import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;\n");
        sb.append("import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;\n\n");
        sb.append("import dev.nextftc.core.commands.Command;\n");
        sb.append("import dev.nextftc.core.commands.groups.ParallelGroup;\n");
        sb.append("import dev.nextftc.core.commands.groups.SequentialGroup;\n");
        sb.append("import dev.nextftc.extensions.pedro.FollowPath;\n\n");
        sb.append("/**\n");
        sb.append(" * AUTO-GENERATED autonomous command from .pp file\n");
        sb.append(" * \n");
        sb.append(" * Add robot actions where indicated.\n");
        sb.append(" * Waypoints are in ").append(commandName).append("Config.java\n");
        sb.append(" */\n");
        sb.append("public class ").append(commandName).append("Command extends Command {\n\n");
        sb.append("    private final SequentialGroup sequence;\n\n");
        sb.append("    public ").append(commandName).append("Command(\n");
        sb.append("            Robot robot,\n");
        sb.append("            IntakeCommands intakeCommands,\n");
        sb.append("            LauncherCommands launcherCommands) {\n\n");
        sb.append("        sequence = new SequentialGroup(\n");

        // Generate path following commands
        for (int i = 0; i < paths.size(); i++) {
            PathData path = paths.get(i);

            sb.append("            // ======================================\n");
            sb.append("            // SEGMENT ").append(i + 1).append(": ").append(path.name).append("\n");
            sb.append("            // ").append(path.start.name).append(" → ").append(path.end.name);
            sb.append(" (").append(path.headingMode).append(" heading)\n");
            sb.append("            // ======================================\n");
            sb.append("            new ParallelGroup(\n");
            sb.append("                new FollowPath(\n");
            sb.append("                    robot.drive.getFollower().pathBuilder()\n");

            // Add path type (line or curve)
            if (path.controlPoints.isEmpty()) {
                sb.append("                        .addPath(new BezierLine(\n");
                sb.append("                            Waypoints.").append(path.start.name).append(".toPose(),\n");
                sb.append("                            Waypoints.").append(path.end.name).append(".toPose()\n");
                sb.append("                        ))\n");
            } else {
                sb.append("                        .addPath(new BezierCurve(\n");
                sb.append("                            Waypoints.").append(path.start.name).append(".toPose(),\n");
                for (double[] cp : path.controlPoints) {
                    sb.append("                            new Pose(").append(cp[0]).append(", ").append(cp[1]).append(", 0),\n");
                }
                sb.append("                            Waypoints.").append(path.end.name).append(".toPose()\n");
                sb.append("                        ))\n");
            }

            // Add heading interpolation
            switch (path.headingMode.toLowerCase()) {
                case "tangential":
                    sb.append("                        .setTangentialHeadingInterpolation()\n");
                    break;
                case "constant":
                    sb.append("                        .setConstantHeadingInterpolation(\n");
                    sb.append("                            Waypoints.").append(path.start.name).append(".toPose().getHeading()\n");
                    sb.append("                        )\n");
                    break;
                case "linear":
                default:
                    sb.append("                        .setLinearHeadingInterpolation(\n");
                    sb.append("                            Waypoints.").append(path.start.name).append(".toPose().getHeading(),\n");
                    sb.append("                            Waypoints.").append(path.end.name).append(".toPose().getHeading()\n");
                    sb.append("                        )\n");
                    break;
            }

            sb.append("                        .build(),\n");
            sb.append("                    ").append(path.reverse ? "true" : "false").append(",  // reverse\n");
            sb.append("                    pathPower.").append(path.name).append("\n");
            sb.append("                )\n");
            sb.append("                // TODO: Add robot actions during ").append(path.name).append("\n");
            sb.append("            )");

            if (i < paths.size() - 1) {
                sb.append(",\n");
                sb.append("            // TODO: Add actions after reaching ").append(path.end.name).append("\n\n");
            } else {
                sb.append("\n");
                sb.append("            // TODO: Add actions after reaching ").append(path.end.name).append("\n");
            }
        }

        sb.append("        );\n");
        sb.append("    }\n\n");
        sb.append("    @Override\n");
        sb.append("    public void start() {\n");
        sb.append("        sequence.start();\n");
        sb.append("    }\n\n");
        sb.append("    @Override\n");
        sb.append("    public void update() {\n");
        sb.append("        sequence.update();\n");
        sb.append("    }\n\n");
        sb.append("    @Override\n");
        sb.append("    public boolean isDone() {\n");
        sb.append("        return sequence.isDone();\n");
        sb.append("    }\n\n");
        sb.append("    @Override\n");
        sb.append("    public void end(boolean interrupted) {\n");
        sb.append("        sequence.end(interrupted);\n");
        sb.append("    }\n");
        sb.append("}\n");

        return sb.toString();
    }

    private static String capitalize(String str) {
        if (str == null || str.isEmpty()) {
            return str;
        }
        return Character.toUpperCase(str.charAt(0)) + str.substring(1);
    }

    // Main method for command-line usage
    public static void main(String[] args) {
        if (args.length < 2) {
            System.out.println("Usage: PpFileCommandGenerator <ppFile> <commandName> [assetsPath] [outputPath]");
            System.out.println("Example: PpFileCommandGenerator trajectory.pp TrajectoryAuto");
            return;
        }

        String ppFileName = args[0];
        String commandName = args[1];
        String assetsPath = args.length > 2 ? args[2] : "TeamCode/src/main/assets/";
        String outputPath = args.length > 3 ? args[3] : "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/generated/";

        try {
            generate(ppFileName, commandName, assetsPath, outputPath);
            System.out.println("Success!");
        } catch (Exception e) {
            System.err.println("Error generating files: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
