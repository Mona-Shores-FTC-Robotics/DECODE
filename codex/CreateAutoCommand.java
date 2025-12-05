import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.io.FileWriter;

/**
 * Creates a Command class from a Pedro .pp file for easy autonomous integration.
 *
 * Usage:
 *   javac codex/CreateAutoCommand.java
 *   java -cp codex CreateAutoCommand my_auto.pp MyCustomAuto
 *
 * Generates a Command class you can use in any OpMode:
 *   Command auto = MyCustomAutoCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 */
public class CreateAutoCommand {

    static final double FIELD_WIDTH = 144.0;

    public static void main(String[] args) {
        if (args.length < 2) {
            System.err.println("Usage: java CreateAutoCommand <pp-file> <command-name>");
            System.err.println("Example: java CreateAutoCommand my_auto.pp MyCustomAuto");
            System.err.println();
            System.err.println("This generates a Command class you can use in any OpMode:");
            System.err.println("  Command auto = MyCustomAutoCommand.create(robot, activeAlliance);");
            System.exit(1);
        }

        String ppFile = args[0];
        String commandName = args[1];
        boolean isRed = ppFile.toLowerCase().contains("red");

        // Generate class name (add "Command" suffix if not present)
        String className = commandName.replaceAll("[^a-zA-Z0-9]", "");
        if (!className.endsWith("Command")) {
            className += "Command";
        }

        // Generate in current directory for easy copying
        String outputFile = className + ".java";

        try {
            String json = new String(Files.readAllBytes(Paths.get(ppFile)));
            List<PathSegment> segments = extractAllSegments(json, isRed);

            String javaCode = generateCommandClass(className, segments, json, isRed);

            // Write to file
            writeFile(outputFile, javaCode);

            System.out.println("=== SUCCESS ===");
            System.out.println();
            System.out.println("Generated: " + outputFile);
            System.out.println();
            System.out.println("Usage in your OpMode:");
            System.out.println("  Command autoRoutine = " + className + ".create(robot, activeAlliance);");
            System.out.println("  CommandManager.INSTANCE.scheduleCommand(autoRoutine);");
            System.out.println();
            System.out.println("Next steps:");
            System.out.println("1. Move " + outputFile + " to TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/");
            System.out.println("2. Edit TODO comments to customize robot actions");
            System.out.println("3. Build and deploy");

        } catch (IOException e) {
            System.err.println("ERROR: " + e.getMessage());
            System.exit(1);
        }
    }

    static String generateCommandClass(String className, List<PathSegment> segments, String json, boolean isRed) {
        Pose start = extractStartPoint(json, isRed);

        StringBuilder code = new StringBuilder();
        code.append("package org.firstinspires.ftc.teamcode.commands;\n\n");

        // Imports
        code.append("import static org.firstinspires.ftc.teamcode.util.AutoField.poseForAlliance;\n\n");
        code.append("import com.bylazar.configurables.annotations.Configurable;\n");
        code.append("import com.pedropathing.geometry.BezierCurve;\n");
        code.append("import com.pedropathing.geometry.BezierLine;\n");
        code.append("import com.pedropathing.geometry.Pose;\n");
        code.append("import com.pedropathing.paths.PathChain;\n");
        code.append("import com.qualcomm.robotcore.util.Range;\n\n");

        code.append("import org.firstinspires.ftc.teamcode.Robot;\n");
        code.append("import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;\n");
        code.append("import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;\n");
        code.append("import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.Alliance;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.AutoField;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.LauncherRange;\n\n");

        code.append("import dev.nextftc.core.commands.Command;\n");
        code.append("import dev.nextftc.core.commands.delays.Delay;\n");
        code.append("import dev.nextftc.core.commands.groups.ParallelGroup;\n");
        code.append("import dev.nextftc.core.commands.groups.SequentialGroup;\n");
        code.append("import dev.nextftc.core.commands.utility.InstantCommand;\n");
        code.append("import dev.nextftc.extensions.pedro.FollowPath;\n\n");

        // Class declaration
        code.append("/**\n");
        code.append(" * Generated autonomous command from Pedro Pathing .pp file\n");
        code.append(" * \n");
        code.append(" * Usage in OpMode:\n");
        code.append(" *   Command auto = ").append(className).append(".create(robot, activeAlliance);\n");
        code.append(" *   CommandManager.INSTANCE.scheduleCommand(auto);\n");
        code.append(" * \n");
        code.append(" * TODO: Fill in robot actions at each segment (search for TODO comments)\n");
        code.append(" */\n");
        code.append("@Configurable\n");
        code.append("public class ").append(className).append(" {\n\n");

        // Config class
        code.append("    public static class Config {\n");
        code.append("        public double maxPathPower = 0.79;\n");
        code.append("        public double intakeDelaySeconds = 2.5;\n");
        code.append("    }\n\n");

        // Waypoints config
        code.append("    public static class Waypoints {\n");
        code.append("        public double startX = ").append(start.x).append(";\n");
        code.append("        public double startY = ").append(start.y).append(";\n");
        code.append("        public double startHeading = ").append(start.heading).append(";\n\n");

        // Track generated waypoints to avoid duplicates
        Set<String> generatedWaypoints = new HashSet<>();
        Set<String> generatedControlPoints = new HashSet<>();

        for (int i = 0; i < segments.size(); i++) {
            PathSegment seg = segments.get(i);
            String varName = toVariableName(seg.name);

            // Only generate waypoint if not already generated
            if (!generatedWaypoints.contains(varName)) {
                generatedWaypoints.add(varName);
                code.append("        // ").append(seg.name).append("\n");
                code.append("        public double ").append(varName).append("X = ").append(seg.end.x).append(";\n");
                code.append("        public double ").append(varName).append("Y = ").append(seg.end.y).append(";\n");
                code.append("        public double ").append(varName).append("Heading = ").append(seg.end.heading).append(";\n\n");
            }

            // Generate control points even if waypoint was already generated
            // (different segments can have different control points to same endpoint)
            if (!seg.controlPoints.isEmpty()) {
                for (int j = 0; j < seg.controlPoints.size(); j++) {
                    String controlPointName = varName + "Control" + j;
                    if (!generatedControlPoints.contains(controlPointName)) {
                        generatedControlPoints.add(controlPointName);
                        Pose cp = seg.controlPoints.get(j);
                        code.append("        // Control point for segment: ").append(seg.name).append("\n");
                        code.append("        public double ").append(controlPointName).append("X = ").append(cp.x).append(";\n");
                        code.append("        public double ").append(controlPointName).append("Y = ").append(cp.y).append(";\n\n");
                    }
                }
            }
        }
        code.append("    }\n\n");

        code.append("    public static Config config = new Config();\n");
        code.append("    public static Waypoints waypoints = new Waypoints();\n\n");

        // Private constructor to prevent instantiation
        code.append("    private ").append(className).append("() {}\n\n");

        // Factory method
        code.append("    public static Command create(Robot robot, Alliance alliance) {\n");
        code.append("        return new SequentialGroup(\n");

        // Build command sequence
        for (int i = 0; i < segments.size(); i++) {
            PathSegment seg = segments.get(i);
            String startIdx = i == 0 ? "start" : toVariableName(segments.get(i - 1).name);
            String endIdx = toVariableName(seg.name);

            code.append("                // ").append(seg.name).append("\n");

            boolean isCollecting = seg.name.toLowerCase().contains("collect") ||
                                   seg.name.toLowerCase().contains("pickup") ||
                                   seg.name.toLowerCase().contains("artifact");
            boolean isScoring = seg.name.toLowerCase().contains("score") ||
                               seg.name.toLowerCase().contains("launch") ||
                               seg.name.toLowerCase().contains("basket");

            // Build followPath call with control points if they exist
            String followPathCall;
            if (!seg.controlPoints.isEmpty()) {
                StringBuilder controlArgs = new StringBuilder();
                for (int j = 0; j < seg.controlPoints.size(); j++) {
                    controlArgs.append(", ").append(endIdx).append("Control").append(j).append("()");
                }
                followPathCall = "followPath(robot, alliance, " + startIdx + "(), " + endIdx + "()" + controlArgs + ")";
            } else {
                followPathCall = "followPath(robot, alliance, " + startIdx + "(), " + endIdx + "())";
            }

            if (isCollecting) {
                code.append("                new ParallelGroup(\n");
                code.append("                    ").append(followPathCall).append(",\n");
                code.append("                    // TODO: Customize intake for ").append(seg.name).append("\n");
                code.append("                    new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))\n");
                code.append("                ),\n");
                code.append("                new Delay(config.intakeDelaySeconds)");
            } else if (isScoring) {
                code.append("                new ParallelGroup(\n");
                code.append("                    ").append(followPathCall).append(",\n");
                code.append("                    // TODO: Customize launcher for ").append(seg.name).append("\n");
                code.append("                    spinUpLauncher(robot)\n");
                code.append("                ),\n");
                code.append("                // TODO: Customize scoring for ").append(seg.name).append("\n");
                code.append("                scoreSequence(robot)");
            } else {
                code.append("                ").append(followPathCall);
                if (i < segments.size() - 1) {
                    code.append(",\n");
                } else {
                    code.append("\n");
                }
                code.append("                // TODO: Add commands for ").append(seg.name).append("\n");
            }

            // For collecting and scoring, add comma if not last
            if ((isCollecting || isScoring) && i < segments.size() - 1) {
                code.append(",\n");
            } else if (isCollecting || isScoring) {
                code.append("\n");
            }
        }

        code.append("        );\n");
        code.append("    }\n\n");

        // Helper methods for waypoints
        code.append("    private static Pose start() {\n");
        code.append("        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));\n");
        code.append("    }\n\n");

        // Reuse the same sets to track generated helpers (avoid duplicate method declarations)
        generatedWaypoints.clear();
        generatedControlPoints.clear();

        for (int i = 0; i < segments.size(); i++) {
            PathSegment seg = segments.get(i);
            String varName = toVariableName(seg.name);

            // Only generate waypoint helper if not already generated
            if (!generatedWaypoints.contains(varName)) {
                generatedWaypoints.add(varName);
                code.append("    private static Pose ").append(varName).append("() {\n");
                code.append("        return new Pose(waypoints.").append(varName).append("X, waypoints.").append(varName).append("Y, Math.toRadians(waypoints.").append(varName).append("Heading));\n");
                code.append("    }\n\n");
            }

            // Generate control point helper methods even if waypoint was already generated
            if (!seg.controlPoints.isEmpty()) {
                for (int j = 0; j < seg.controlPoints.size(); j++) {
                    String controlPointName = varName + "Control" + j;
                    if (!generatedControlPoints.contains(controlPointName)) {
                        generatedControlPoints.add(controlPointName);
                        code.append("    private static Pose ").append(controlPointName).append("() {\n");
                        code.append("        return new Pose(waypoints.").append(controlPointName).append("X, waypoints.").append(controlPointName).append("Y, 0);\n");
                        code.append("    }\n\n");
                    }
                }
            }
        }

        // Helper methods - followPath with varargs for control points
        code.append("    private static Command followPath(Robot robot, Alliance alliance, Pose startPose, Pose endPose, Pose... controlPoses) {\n");
        code.append("        // Mirror for red alliance\n");
        code.append("        Pose start = poseForAlliance(startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()), alliance);\n");
        code.append("        Pose end = poseForAlliance(endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading()), alliance);\n\n");
        code.append("        PathChain path;\n");
        code.append("        if (controlPoses.length > 0) {\n");
        code.append("            // Use BezierCurve with control points\n");
        code.append("            Pose control = poseForAlliance(controlPoses[0].getX(), controlPoses[0].getY(), 0, alliance);\n");
        code.append("            path = robot.drive.getFollower().pathBuilder()\n");
        code.append("                    .addPath(new com.pedropathing.geometry.BezierCurve(start, control, end))\n");
        code.append("                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), 0.7)\n");
        code.append("                    .build();\n");
        code.append("        } else {\n");
        code.append("            // Use straight line\n");
        code.append("            path = robot.drive.getFollower().pathBuilder()\n");
        code.append("                    .addPath(new BezierLine(start, end))\n");
        code.append("                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), 0.7)\n");
        code.append("                    .build();\n");
        code.append("        }\n");
        code.append("        double maxPower = Range.clip(config.maxPathPower, 0.0, 1.0);\n");
        code.append("        return new FollowPath(path, false, maxPower);\n");
        code.append("    }\n\n");

        code.append("    private static Command spinUpLauncher(Robot robot) {\n");
        code.append("        // TODO: Customize launcher spin-up\n");
        code.append("        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake);\n");
        code.append("        return launcherCommands.spinUpForPosition(AutoField.FieldPoint.LAUNCH_CLOSE);\n");
        code.append("    }\n\n");

        code.append("    private static Command scoreSequence(Robot robot) {\n");
        code.append("        // TODO: Customize scoring sequence\n");
        code.append("        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake);\n");
        code.append("        return launcherCommands.launchAllAtRangePreset(LauncherRange.SHORT, false);\n");
        code.append("    }\n");

        code.append("}\n");

        return code.toString();
    }

    static List<PathSegment> extractAllSegments(String json, boolean isRed) {
        List<PathSegment> segments = new ArrayList<>();
        Pose start = extractStartPoint(json, isRed);

        // Updated pattern to capture control points array
        Pattern linePattern = Pattern.compile("\\{\"name\":\"([^\"]+)\"[^}]*\"endPoint\":\\{([^}]+)\\}[^}]*\"controlPoints\":\\[([^\\]]*)\\]");
        Matcher m = linePattern.matcher(json);

        Pose previousEnd = start;
        int index = 0;

        while (m.find()) {
            String name = m.group(1);
            String endPointData = m.group(2);
            String controlPointsData = m.group(3);

            Pose end = parseEndPoint(endPointData, isRed);
            List<Pose> controlPoints = parseControlPoints(controlPointsData, isRed);

            segments.add(new PathSegment(index, name, previousEnd, end, controlPoints));
            previousEnd = end;
            index++;
        }

        return segments;
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

    static List<Pose> parseControlPoints(String data, boolean isRed) {
        List<Pose> controlPoints = new ArrayList<>();

        if (data == null || data.trim().isEmpty()) {
            return controlPoints;
        }

        // Extract individual control point objects: {"x":29,"y":101}
        Pattern controlPattern = Pattern.compile("\\{\"x\":([-\\d.]+),\"y\":([-\\d.]+)\\}");
        Matcher m = controlPattern.matcher(data);

        while (m.find()) {
            double x = Double.parseDouble(m.group(1));
            double y = Double.parseDouble(m.group(2));

            if (isRed) {
                x = FIELD_WIDTH - x;
            }

            // Control points don't have heading, use 0
            controlPoints.add(new Pose(x, y, 0));
        }

        return controlPoints;
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

    static void writeFile(String filename, String content) throws IOException {
        try (FileWriter writer = new FileWriter(filename)) {
            writer.write(content);
        }
    }

    /**
     * Converts a path name from Pedro into a valid Java variable name.
     * Examples:
     *   "Launch Close" -> "launchClose"
     *   "Pre-Artifacts" -> "preArtifacts"
     *   "Gate Artifacts" -> "gateArtifacts"
     */
    static String toVariableName(String pathName) {
        // Remove special characters and split on spaces/hyphens/underscores
        String[] words = pathName.split("[\\s\\-_]+");

        StringBuilder result = new StringBuilder();
        for (int i = 0; i < words.length; i++) {
            String word = words[i].replaceAll("[^a-zA-Z0-9]", "");
            if (word.isEmpty()) continue;

            if (i == 0) {
                // First word: lowercase first letter
                result.append(Character.toLowerCase(word.charAt(0)));
                if (word.length() > 1) {
                    result.append(word.substring(1));
                }
            } else {
                // Subsequent words: uppercase first letter (camelCase)
                result.append(Character.toUpperCase(word.charAt(0)));
                if (word.length() > 1) {
                    result.append(word.substring(1));
                }
            }
        }

        // If result is empty or starts with number, prefix with 'waypoint'
        String varName = result.toString();
        if (varName.isEmpty() || Character.isDigit(varName.charAt(0))) {
            varName = "waypoint" + varName;
        }

        return varName;
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
        List<Pose> controlPoints;

        PathSegment(int index, String name, Pose start, Pose end, List<Pose> controlPoints) {
            this.index = index;
            this.name = name;
            this.start = start;
            this.end = end;
            this.controlPoints = controlPoints;
        }
    }
}
