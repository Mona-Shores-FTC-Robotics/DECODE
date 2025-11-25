import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.io.FileWriter;

/**
 * Creates a Command class from a Pedro .pp file for easy autonomous integration.
 *
 * Usage:
 *   javac codex/CreateAutoFromPedro.java
 *   java -cp codex CreateAutoFromPedro my_auto.pp "MyCustomAuto"
 *
 * Generates a Command class you can use in any OpMode:
 *   Command auto = new MyCustomAutoCommand(robot);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 */
public class CreateAutoFromPedro {

    static final double FIELD_WIDTH = 144.0;

    public static void main(String[] args) {
        if (args.length < 2) {
            System.err.println("Usage: java CreateAutoFromPedro <pp-file> <command-name>");
            System.err.println("Example: java CreateAutoFromPedro my_auto.pp MyCustomAuto");
            System.err.println();
            System.err.println("This generates a Command class you can use in any OpMode:");
            System.err.println("  Command auto = new MyCustomAutoCommand(robot, activeAlliance);");
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

        String outputFile = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/" + className + ".java";

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
            System.out.println("  Command autoRoutine = new " + className + "(robot, activeAlliance);");
            System.out.println("  CommandManager.INSTANCE.scheduleCommand(autoRoutine);");
            System.out.println();
            System.out.println("Next steps:");
            System.out.println("1. Edit TODO comments to customize robot actions");
            System.out.println("2. Use in any autonomous OpMode");
            System.out.println("3. Build and deploy");

        } catch (IOException e) {
            System.err.println("ERROR: " + e.getMessage());
            System.exit(1);
        }
    }

    static String generateCommandClass(String className, List<PathSegment> segments, String json, boolean isRed) {
        Pose start = extractStartPoint(json, isRed);

        StringBuilder code = new StringBuilder();
        code.append("package org.firstinspires.ftc.teamcode.opmodes;\n\n");

        // Imports
        code.append("import static org.firstinspires.ftc.teamcode.util.AutoField.poseForAlliance;\n\n");
        code.append("import com.bylazar.configurables.annotations.Configurable;\n");
        code.append("import com.pedropathing.follower.Follower;\n");
        code.append("import com.pedropathing.geometry.BezierLine;\n");
        code.append("import com.pedropathing.geometry.Pose;\n");
        code.append("import com.pedropathing.paths.PathChain;\n");
        code.append("import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n");
        code.append("import com.qualcomm.robotcore.util.Range;\n\n");

        code.append("import org.firstinspires.ftc.teamcode.Robot;\n");
        code.append("import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;\n");
        code.append("import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;\n");
        code.append("import org.firstinspires.ftc.teamcode.pedroPathing.Constants;\n");
        code.append("import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;\n");
        code.append("import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.Alliance;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.AllianceSelector;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.LauncherMode;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.LauncherRange;\n");
        code.append("import org.firstinspires.ftc.teamcode.util.RobotState;\n\n");

        code.append("import dev.nextftc.bindings.BindingManager;\n");
        code.append("import dev.nextftc.core.commands.Command;\n");
        code.append("import dev.nextftc.core.commands.CommandManager;\n");
        code.append("import dev.nextftc.core.commands.delays.Delay;\n");
        code.append("import dev.nextftc.core.commands.groups.ParallelGroup;\n");
        code.append("import dev.nextftc.core.commands.groups.SequentialGroup;\n");
        code.append("import dev.nextftc.core.commands.utility.InstantCommand;\n");
        code.append("import dev.nextftc.core.components.BindingsComponent;\n");
        code.append("import dev.nextftc.core.components.SubsystemComponent;\n");
        code.append("import dev.nextftc.extensions.pedro.FollowPath;\n");
        code.append("import dev.nextftc.extensions.pedro.PedroComponent;\n");
        code.append("import dev.nextftc.ftc.GamepadEx;\n");
        code.append("import dev.nextftc.ftc.NextFTCOpMode;\n");
        code.append("import dev.nextftc.ftc.components.BulkReadComponent;\n\n");

        // Class declaration
        code.append("/**\n");
        code.append(" * Generated autonomous routine from Pedro Pathing .pp file\n");
        code.append(" * \n");
        code.append(" * TODO: Fill in robot actions at each segment (search for TODO comments)\n");
        code.append(" */\n");
        code.append("@Autonomous(name = \"").append(autoName).append("\", group = \"Custom\")\n");
        code.append("@Configurable\n");
        code.append("public class ").append(className).append(" extends NextFTCOpMode {\n\n");

        // Waypoint enum
        code.append("    public enum Waypoint {\n");
        code.append("        START,\n");
        for (int i = 0; i < segments.size(); i++) {
            code.append("        WAYPOINT").append(i);
            if (i < segments.size() - 1) code.append(",");
            code.append("\n");
        }
        code.append("    }\n\n");

        // Config classes
        code.append("    @Configurable\n");
        code.append("    public static class AutoMotionConfig {\n");
        code.append("        public double maxPathPower = 0.79;\n");
        code.append("        public double intakeDelaySeconds = 2.5;\n");
        code.append("        public int relocalizeMaxAttempts = 10;\n");
        code.append("        public LauncherMode startingLauncherMode = LauncherMode.THROUGHPUT;\n");
        code.append("    }\n\n");

        // Waypoints config
        code.append("    @Configurable\n");
        code.append("    public static class Waypoints {\n");
        code.append("        public double startX = ").append(start.x).append(";\n");
        code.append("        public double startY = ").append(start.y).append(";\n");
        code.append("        public double startHeading = ").append(start.heading).append(";\n\n");

        for (int i = 0; i < segments.size(); i++) {
            PathSegment seg = segments.get(i);
            code.append("        // ").append(seg.name).append("\n");
            code.append("        public double waypoint").append(i).append("X = ").append(seg.end.x).append(";\n");
            code.append("        public double waypoint").append(i).append("Y = ").append(seg.end.y).append(";\n");
            code.append("        public double waypoint").append(i).append("Heading = ").append(seg.end.heading).append(";\n\n");
        }
        code.append("    }\n\n");

        code.append("    public static AutoMotionConfig config = new AutoMotionConfig();\n");
        code.append("    public static Waypoints waypoints = new Waypoints();\n\n");

        // Instance variables
        code.append("    private Robot robot;\n");
        code.append("    private AllianceSelector allianceSelector;\n");
        code.append("    private Alliance activeAlliance = Alliance.BLUE;\n");
        code.append("    private IntakeCommands intakeCommands;\n");
        code.append("    private LauncherCommands launcherCommands;\n\n");

        // Component registration
        code.append("    {\n");
        code.append("        addComponents(\n");
        code.append("                BulkReadComponent.INSTANCE,\n");
        code.append("                new PedroComponent(Constants::createFollower),\n");
        code.append("                BindingsComponent.INSTANCE,\n");
        code.append("                CommandManager.INSTANCE\n");
        code.append("        );\n");
        code.append("    }\n\n");

        // onInit
        code.append("    @Override\n");
        code.append("    public void onInit() {\n");
        code.append("        BindingManager.reset();\n");
        code.append("        robot = new Robot(hardwareMap);\n");
        code.append("        robot.attachPedroFollower();\n");
        code.append("        robot.telemetry.startSession();\n");
        code.append("        robot.initializeForAuto();\n\n");
        code.append("        intakeCommands = new IntakeCommands(robot.intake);\n");
        code.append("        launcherCommands = new LauncherCommands(robot.launcher, robot.intake);\n\n");
        code.append("        GamepadEx driverPad = new GamepadEx(() -> gamepad1);\n");
        code.append("        allianceSelector = new AllianceSelector(driverPad, Alliance.UNKNOWN);\n");
        code.append("        activeAlliance = allianceSelector.getSelectedAlliance();\n");
        code.append("        robot.setAlliance(activeAlliance);\n\n");
        code.append("        Pose startPose = pose(Waypoint.START);\n");
        code.append("        robot.drive.getFollower().setStartingPose(startPose);\n");
        code.append("        robot.drive.getFollower().setPose(startPose);\n\n");
        code.append("        addComponents(\n");
        code.append("                new SubsystemComponent(robot.drive),\n");
        code.append("                new SubsystemComponent(robot.launcher),\n");
        code.append("                new SubsystemComponent(robot.intake),\n");
        code.append("                new SubsystemComponent(robot.lighting),\n");
        code.append("                new SubsystemComponent(robot.vision)\n");
        code.append("        );\n");
        code.append("    }\n\n");

        // onWaitForStart
        code.append("    @Override\n");
        code.append("    public void onWaitForStart() {\n");
        code.append("        BindingManager.update();\n");
        code.append("        robot.intake.periodic();\n");
        code.append("        robot.vision.periodic();\n");
        code.append("        allianceSelector.updateFromVision(robot.vision);\n");
        code.append("        allianceSelector.applySelection(robot, robot.lighting);\n\n");
        code.append("        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();\n");
        code.append("        if (selectedAlliance != activeAlliance) {\n");
        code.append("            activeAlliance = selectedAlliance;\n");
        code.append("            robot.setAlliance(activeAlliance);\n");
        code.append("        }\n\n");
        code.append("        telemetry.addData(\"Alliance\", activeAlliance.displayName());\n");
        code.append("        telemetry.addLine(\"Press START when ready\");\n");
        code.append("        telemetry.update();\n");
        code.append("    }\n\n");

        // onStartButtonPressed
        code.append("    @Override\n");
        code.append("    public void onStartButtonPressed() {\n");
        code.append("        BindingManager.reset();\n");
        code.append("        allianceSelector.lockSelection();\n");
        code.append("        RobotState.setLauncherMode(config.startingLauncherMode);\n");
        code.append("        RobotState.resetMotifTail();\n");
        code.append("        robot.launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);\n\n");
        code.append("        Command autoRoutine = buildAutonomousRoutine();\n");
        code.append("        CommandManager.INSTANCE.scheduleCommand(autoRoutine);\n\n");
        code.append("        robot.intake.forwardRoller();\n");
        code.append("        robot.intake.setGateAllowArtifacts();\n");
        code.append("    }\n\n");

        // onUpdate
        code.append("    @Override\n");
        code.append("    public void onUpdate() {\n");
        code.append("        publishTelemetry();\n");
        code.append("    }\n\n");

        // onStop
        code.append("    @Override\n");
        code.append("    public void onStop() {\n");
        code.append("        allianceSelector.unlockSelection();\n");
        code.append("        BindingManager.reset();\n");
        code.append("        robot.launcher.abort();\n");
        code.append("        robot.drive.stop();\n");
        code.append("        robot.vision.stop();\n");
        code.append("        RobotState.setHandoffPose(robot.drive.getFollower().getPose());\n");
        code.append("    }\n\n");

        // pose method
        code.append("    private Pose pose(Waypoint waypoint) {\n");
        code.append("        Waypoints w = waypoints;\n");
        code.append("        switch (waypoint) {\n");
        code.append("            case START:\n");
        code.append("                return poseForAlliance(w.startX, w.startY, w.startHeading, activeAlliance);\n");
        for (int i = 0; i < segments.size(); i++) {
            code.append("            case WAYPOINT").append(i).append(":\n");
            code.append("                return poseForAlliance(w.waypoint").append(i).append("X, w.waypoint").append(i).append("Y, w.waypoint").append(i).append("Heading, activeAlliance);\n");
        }
        code.append("            default:\n");
        code.append("                throw new IllegalArgumentException(\"Unknown waypoint: \" + waypoint);\n");
        code.append("        }\n");
        code.append("    }\n\n");

        // relocalize method
        code.append("    private Command relocalize() {\n");
        code.append("        return new Command() {\n");
        code.append("            private int attempts = 0;\n");
        code.append("            private boolean success = false;\n\n");
        code.append("            @Override\n");
        code.append("            public void start() {\n");
        code.append("                attempts = 0;\n");
        code.append("                success = false;\n");
        code.append("            }\n\n");
        code.append("            @Override\n");
        code.append("            public void update() {\n");
        code.append("                if (robot.vision.hasValidTag()) {\n");
        code.append("                    success = robot.drive.forceRelocalizeFromVision();\n");
        code.append("                    if (success) {\n");
        code.append("                        attempts = config.relocalizeMaxAttempts;\n");
        code.append("                    }\n");
        code.append("                }\n");
        code.append("                attempts++;\n");
        code.append("            }\n\n");
        code.append("            @Override\n");
        code.append("            public boolean isDone() {\n");
        code.append("                return attempts >= config.relocalizeMaxAttempts || success;\n");
        code.append("            }\n");
        code.append("        };\n");
        code.append("    }\n\n");

        // buildAutonomousRoutine method
        code.append("    private Command buildAutonomousRoutine() {\n");
        code.append("        return new SequentialGroup(\n");

        for (int i = 0; i < segments.size(); i++) {
            PathSegment seg = segments.get(i);
            String startWaypoint = i == 0 ? "START" : "WAYPOINT" + (i - 1);
            String endWaypoint = "WAYPOINT" + i;

            code.append("                // ").append(seg.name).append("\n");

            boolean isCollecting = seg.name.toLowerCase().contains("collect") ||
                                   seg.name.toLowerCase().contains("pickup") ||
                                   seg.name.toLowerCase().contains("artifact");
            boolean isScoring = seg.name.toLowerCase().contains("score") ||
                               seg.name.toLowerCase().contains("launch") ||
                               seg.name.toLowerCase().contains("basket");

            if (isCollecting) {
                code.append("                new ParallelGroup(\n");
                code.append("                    followPath(pose(Waypoint.").append(startWaypoint).append("), pose(Waypoint.").append(endWaypoint).append(")),\n");
                code.append("                    // TODO: Customize intake behavior for ").append(seg.name).append("\n");
                code.append("                    new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))\n");
                code.append("                ),\n");
                code.append("                new Delay(config.intakeDelaySeconds)");
            } else if (isScoring) {
                code.append("                new ParallelGroup(\n");
                code.append("                    followPath(pose(Waypoint.").append(startWaypoint).append("), pose(Waypoint.").append(endWaypoint).append(")),\n");
                code.append("                    // TODO: Customize launcher behavior for ").append(seg.name).append("\n");
                code.append("                    spinUpLauncher()\n");
                code.append("                ),\n");
                code.append("                // TODO: Customize scoring sequence for ").append(seg.name).append("\n");
                code.append("                scoreSequence()");
            } else {
                code.append("                followPath(pose(Waypoint.").append(startWaypoint).append("), pose(Waypoint.").append(endWaypoint).append("))");
                code.append("\n                // TODO: Add any commands for ").append(seg.name);
            }

            if (i < segments.size() - 1) {
                code.append(",\n");
            }
            code.append("\n");
        }

        code.append("        );\n");
        code.append("    }\n\n");

        // Helper methods
        code.append("    private Command followPath(Pose startPose, Pose endPose) {\n");
        code.append("        PathChain path = robot.drive.getFollower().pathBuilder()\n");
        code.append("                .addPath(new BezierLine(startPose, endPose))\n");
        code.append("                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)\n");
        code.append("                .build();\n");
        code.append("        double maxPower = Range.clip(config.maxPathPower, 0.0, 1.0);\n");
        code.append("        return new FollowPath(path, false, maxPower);\n");
        code.append("    }\n\n");

        code.append("    private Command spinUpLauncher() {\n");
        code.append("        // TODO: Customize launcher spin-up behavior\n");
        code.append("        return launcherCommands.spinUpForPosition(org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint.LAUNCH_CLOSE);\n");
        code.append("    }\n\n");

        code.append("    private Command scoreSequence() {\n");
        code.append("        // TODO: Customize scoring sequence\n");
        code.append("        return launcherCommands.launchAllAtRangePreset(LauncherRange.SHORT, false);\n");
        code.append("    }\n\n");

        code.append("    private void publishTelemetry() {\n");
        code.append("        robot.telemetry.publishLoopTelemetry(\n");
        code.append("                robot.drive, robot.launcher, robot.intake, robot.vision, robot.lighting,\n");
        code.append("                null, gamepad1, gamepad2, RobotState.getAlliance(),\n");
        code.append("                getRuntime(), Math.max(0.0, 150.0 - getRuntime()),\n");
        code.append("                telemetry, \"Auto\", true, null, 0, 0\n");
        code.append("        );\n");
        code.append("    }\n");

        code.append("}\n");

        return code.toString();
    }

    static List<PathSegment> extractAllSegments(String json, boolean isRed) {
        List<PathSegment> segments = new ArrayList<>();
        Pose start = extractStartPoint(json, isRed);

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

    static void writeFile(String filename, String content) throws IOException {
        // Create parent directories if needed
        java.io.File file = new java.io.File(filename);
        file.getParentFile().mkdirs();

        try (FileWriter writer = new FileWriter(filename)) {
            writer.write(content);
        }
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
