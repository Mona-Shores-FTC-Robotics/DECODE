package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.telemetry.RobotStatusLogger;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Clean example of using @AutoLog for WPILOG logging without any hardware.
 *
 * This OpMode demonstrates:
 * - KoalaLog.setup() for WPILOG file generation
 * - @AutoLog subsystem (DemoAutoLogSubsystem) with automatic logging
 * - AutoLogManager.periodic() to sample logged data
 * - KoalaLog.logPose2d() for robot pose field visualization
 * - RobotStatusLogger.logStatus() for FTC Dashboard _Status fields
 *
 * Robot state metadata (via RobotStatusLogger - matches FTC Dashboard):
 *   - RUNNING: OpMode active state (root level boolean)
 *   - _Status/enabled: Stop requested state
 *   - _Status/activeOpmode: Current OpMode name (from class name)
 *   - _Status/activeOpModeStatus: INIT/RUNNING state
 *   - _Status/errorMessage: From RobotLog.getGlobalErrorMsg()
 *   - _Status/warningMessage: From RobotLog.getGlobalWarningMessage()
 *   - _Status/batteryVoltage: Battery voltage from hardware
 *
 * Custom fields (separate namespaces):
 *   - OpMode/*: Custom fields for this specific OpMode (Alliance, Phase, CurrentStep, etc.)
 *   - Simulation/*: Simulation-specific state (stepName, stepType, progress, etc.)
 *   - Robot/Pose: Robot pose for field visualization (Pose2d struct)
 *
 * Perfect for testing on a testbench without expansion hub!
 */
@Autonomous(name = "Test: Path Visualization Codex", group = "Test")
public class TestPathVisualizationCodex extends LinearOpMode {

    @Configurable
    public static class SimulationConfig {
        public double simulatedDriveSpeedIps = 30.0;
        public double spinUpTimeSec = 1.4;
        public double scoreBurstTimeSec = 1.0;
        public double intakeTimeSec = 1.2;
        public double transferPauseSec = 0.35;
        public double dwellBetweenLoopsSec = 0.25;
        public double missingPathHoldSec = 0.25;
    }

    TestPathVisualizationCodex.SimulationConfig config = new TestPathVisualizationCodex.SimulationConfig();

    private static final String[] PATH_SEQUENCE = {
            "Start → Launch",
            "Launch → Alliance Wall",
            "Alliance Wall → Launch",
            "Launch → Parking",
            "Parking → Launch",
            "Launch → Gate Far",
            "Gate Far → Launch"
    };

    private Alliance activeAlliance = Alliance.BLUE;
    private SimulationMode simulationMode = SimulationMode.FULL_AUTO;

    private final PathSimInputs simInputs = new PathSimInputs();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Follower follower;
    private SimulationContext simulationContext;
    private CommandRunner commandRunner;

    private boolean prevLeftBumper;
    private boolean prevRightBumper;
    private boolean prevDpadLeft;
    private boolean prevDpadRight;
    private boolean prevX;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        rebuildSimulationContext(true);
        sendDashboardPacket();
        updateInitTelemetry();

        while (!isStarted() && !isStopRequested()) {
            boolean rebuildRequested = handleInitInput();
            if (rebuildRequested) {
                rebuildSimulationContext(false);
            }

            // Update follower to get latest pose
            follower.update();

            // ========================================================================
            // OPTIONAL: Log Pose2d for field visualization in AdvantageScope
            // This creates a draggable robot on the 2D field view
            // Must convert inches to meters for AdvantageScope
            // ========================================================================
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                double xMeters = currentPose.getX() * 0.0254;
                double yMeters = currentPose.getY() * 0.0254;
                double headingRad = currentPose.getHeading();
            }

            // Log FTC Dashboard _Status fields (matches AdvantageScope Lite)
            RobotStatusLogger.logStatus(this, hardwareMap, false);

            updateInitTelemetry();
            sendDashboardPacket();

            // ========================================================================
            // STEP 3: Call AutoLogManager.periodic() in your loop
            // This samples all @AutoLog subsystems and writes to WPILOG
            // ========================================================================
            sleep(25);
        }

        if (isStopRequested()) {
            cleanup();
            return;
        }

        waitForStart();

        if (commandRunner != null) {
            commandRunner.restart(simulationContext.startPose);
        }

        ElapsedTime simTimer = new ElapsedTime();
        double lastTime = 0.0;

        while (opModeIsActive() && commandRunner != null && commandRunner.shouldContinue()) {
            double now = simTimer.seconds();
            double dt = Math.min(now - lastTime, 0.1);
            lastTime = now;

            commandRunner.update(dt, simInputs);
            simInputs.runtimeSec = now;

            // Log robot pose for field visualization
            Pose currentPose = new Pose(simInputs.poseX, simInputs.poseY, simInputs.poseHeading);
            double xMeters = currentPose.getX() * 0.0254;
            double yMeters = currentPose.getY() * 0.0254;
            double headingRad = currentPose.getHeading();

            // Log FTC Dashboard _Status fields (matches AdvantageScope Lite)
            RobotStatusLogger.logStatus(this, hardwareMap, opModeIsActive());



            sendDashboardPacket();
            updateRuntimeTelemetry();
        }

        simInputs.running = false;
        simInputs.overallProgress = 1.0;
        sendDashboardPacket();

        telemetry.addLine("Simulation complete!");
        telemetry.addLine("Download WPILOG file with FTCLogPuller.exe");
        telemetry.addLine("Open in AdvantageScope to view all logged data");
        telemetry.update();
        sleep(750);

        cleanup();
    }

    private void cleanup() {

    }

    private boolean handleInitInput() {
        boolean changed = false;
        if (gamepad1.left_bumper && !prevLeftBumper) {
            activeAlliance = Alliance.BLUE;
            changed = true;
        }
        if (gamepad1.right_bumper && !prevRightBumper) {
            activeAlliance = Alliance.RED;
            changed = true;
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            simulationMode = cycleMode(-1);
            changed = true;
        }
        if (gamepad1.dpad_right && !prevDpadRight) {
            simulationMode = cycleMode(1);
            changed = true;
        }
        if (gamepad1.x && !prevX) {
            changed = true; // manual rebuild to apply dashboard tunables
        }

        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevX = gamepad1.x;

        return changed;
    }

    private void rebuildSimulationContext(boolean firstBuild) {
        FieldLayout layout = AutoField.layoutForAlliance(activeAlliance);
        Pose startPose = layout.pose(FieldPoint.START_FAR);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        PathChain[] pathChains;
        try {
            pathChains = DecodeAutonomousFarCommand.buildAllPathsForVisualization(follower, activeAlliance);
        } catch (Exception e) {
            telemetry.addData("Path error", e.getMessage());
            pathChains = new PathChain[0];
        }

        VirtualPath[] virtualPaths = buildVirtualPaths(pathChains);
        List<SimulatedStep> steps = buildStepsForMode(virtualPaths);
        if (steps.isEmpty()) {
            steps.add(new SimulatedActionStep("No paths available", config.missingPathHoldSec));
        }

        simulationContext = new SimulationContext(
                copyPose(startPose),
                virtualPaths
        );

        commandRunner = new CommandRunner(steps, simulationMode.loops, startPose);

        Pose pose = simulationContext.startPose;
        simInputs.poseX = pose.getX();
        simInputs.poseY = pose.getY();
        simInputs.poseHeading = pose.getHeading();
        simInputs.stepName = firstBuild ? "Init" : "Preview";
        simInputs.stepType = "IDLE";
        simInputs.stepProgress = 0.0;
        simInputs.overallProgress = 0.0;
        simInputs.running = false;

        PanelsBridge.drawPreview(pathChains, pose, activeAlliance == Alliance.RED);
    }

    private VirtualPath[] buildVirtualPaths(PathChain[] pathChains) {
        VirtualPath[] paths = new VirtualPath[PATH_SEQUENCE.length];
        for (int i = 0; i < PATH_SEQUENCE.length; i++) {
            PathChain chain = pathChains != null && i < pathChains.length ? pathChains[i] : null;
            paths[i] = chain == null ? null : new VirtualPath(PATH_SEQUENCE[i], chain);
        }
        return paths;
    }

    private List<SimulatedStep> buildStepsForMode(VirtualPath[] virtualPaths) {
        List<SimulatedStep> steps = new ArrayList<>();
        switch (simulationMode) {
            case FULL_AUTO:
                buildFullAutoSequence(virtualPaths, steps);
                break;
            case WALL_LOOP:
                buildWallLoopSequence(virtualPaths, steps);
                break;
            case PARK_SWEEP:
                buildParkingSweepSequence(virtualPaths, steps);
                break;
            default:
                buildFullAutoSequence(virtualPaths, steps);
                break;
        }
        return steps;
    }

    private void buildFullAutoSequence(VirtualPath[] paths, List<SimulatedStep> steps) {
        steps.add(action("Spin Up Launcher", config.spinUpTimeSec));
        steps.add(pathStep(paths[0]));
        steps.add(action("Score Preload", config.scoreBurstTimeSec));
        steps.add(pathStep(paths[1]));
        steps.add(action("Collect Alliance Wall", config.intakeTimeSec));
        steps.add(pathStep(paths[2]));
        steps.add(action("Score Alliance Wall", config.scoreBurstTimeSec));
        steps.add(action("Transfer", config.transferPauseSec));
        steps.add(pathStep(paths[3]));
        steps.add(action("Collect Parking", config.intakeTimeSec));
        steps.add(pathStep(paths[4]));
        steps.add(action("Score Parking", config.scoreBurstTimeSec));
        steps.add(action("Transfer", config.transferPauseSec));
        steps.add(pathStep(paths[5]));
        steps.add(action("Collect Gate Far", config.intakeTimeSec));
        steps.add(pathStep(paths[6]));
        steps.add(action("Final Score", config.scoreBurstTimeSec));
    }

    private void buildWallLoopSequence(VirtualPath[] paths, List<SimulatedStep> steps) {
        steps.add(action("Loop Prep", config.transferPauseSec));
        steps.add(pathStep(paths[1]));
        steps.add(action("Collect @ Wall", config.intakeTimeSec));
        steps.add(pathStep(paths[2]));
        steps.add(action("Score @ Launch", config.scoreBurstTimeSec));
        steps.add(action("Loop Dwell", config.dwellBetweenLoopsSec));
    }

    private void buildParkingSweepSequence(VirtualPath[] paths, List<SimulatedStep> steps) {
        steps.add(action("Parking Prep", config.transferPauseSec));
        steps.add(pathStep(paths[3]));
        steps.add(action("Collect Parking", config.intakeTimeSec));
        steps.add(pathStep(paths[4]));
        steps.add(action("Score Parking", config.scoreBurstTimeSec));
    }

    private SimulatedStep pathStep(VirtualPath path) {
        if (path == null || !path.hasSamples()) {
            return action("Missing path", config.missingPathHoldSec);
        }
        return new SimulatedPathStep(path, config.simulatedDriveSpeedIps);
    }

    private SimulatedStep action(String name, double durationSec) {
        return new SimulatedActionStep(name, durationSec);
    }

    private void updateInitTelemetry() {
        telemetry.addData("Alliance", activeAlliance);
        telemetry.addData("Mode", simulationMode.displayName);
        telemetry.addLine("LB/RB = switch alliance");
        telemetry.addLine("DPAD L/R = change scenario");
        telemetry.addLine("X = rebuild with new Pedro tunables");
        telemetry.addLine("Press START to stream to AdvantageScope Lite.");
        telemetry.update();
    }

    private void updateRuntimeTelemetry() {
        telemetry.addData("Alliance", activeAlliance);
        telemetry.addData("Mode", simulationMode.displayName);
        telemetry.addData("Step", "%s (%.0f%%)", simInputs.stepName, simInputs.stepProgress * 100.0);
        telemetry.addData("Overall", "%.0f%%", simInputs.overallProgress * 100.0);
        telemetry.addData("Loops", commandRunner.getCompletedLoops());
        telemetry.addData("Pose", "(%.1f, %.1f, %.0f°)", simInputs.poseX, simInputs.poseY, Math.toDegrees(simInputs.poseHeading));
        telemetry.update();
    }

    private void sendDashboardPacket() {
        if (dashboard == null || simulationContext == null) {
            return;
        }
        TelemetryPacket packet = new TelemetryPacket();
        Pose pedroPose = new Pose(simInputs.poseX, simInputs.poseY, simInputs.poseHeading);
        packet.put("Pose/Pose x", pedroPose.getX());
        packet.put("Pose/Pose y", pedroPose.getY());
        packet.put("Pose/Pose heading", pedroPose.getHeading());
        packet.put("Simulation/Alliance", activeAlliance.name());
        packet.put("Simulation/Mode", simulationMode.displayName);
        packet.put("Simulation/StepName", simInputs.stepName);
        packet.put("Simulation/StepType", simInputs.stepType);
        packet.put("Simulation/StepProgress", simInputs.stepProgress);
        packet.put("Simulation/OverallProgress", simInputs.overallProgress);
        packet.put("Simulation/LoopCount", simInputs.loopCount);
        packet.put("Simulation/Running", simInputs.running);

        Canvas overlay = packet.fieldOverlay();
        drawPathsOnOverlay(overlay,
                simulationContext.virtualPaths,
                commandRunner != null ? commandRunner.getActivePath() : null);
        drawRobotOnOverlay(overlay, simInputs.poseX, simInputs.poseY, simInputs.poseHeading);

        dashboard.sendTelemetryPacket(packet);
    }

    private void drawPathsOnOverlay(Canvas overlay, VirtualPath[] paths, VirtualPath highlight) {
        if (paths == null) {
            return;
        }
        String baseColor = activeAlliance == Alliance.RED ? "#EF5350" : "#64B5F6";
        for (VirtualPath path : paths) {
            if (path == null || !path.hasSamples()) {
                continue;
            }
            boolean isActive = highlight != null && path == highlight;
            overlay.setStroke(isActive ? "#FFC107" : baseColor);
            overlay.setStrokeWidth(isActive ? 3 : 1);
            List<Point2d> pts = path.getPoints();
            for (int i = 1; i < pts.size(); i++) {
                Point2d prev = pts.get(i - 1);
                Point2d curr = pts.get(i);
                overlay.strokeLine(prev.x, prev.y, curr.x, curr.y);
            }
        }
        Pose start = simulationContext.startPose;
        if (start != null) {
            overlay.setStroke("#FFFFFF");
            overlay.setStrokeWidth(2);
            overlay.strokeCircle(start.getX(), start.getY(), 1.5);
        }
    }

    private void drawRobotOnOverlay(Canvas overlay, double x, double y, double heading) {
        double length = 5.0;
        double endX = x + Math.cos(heading) * length;
        double endY = y + Math.sin(heading) * length;
        overlay.setStroke("#FFFFFF");
        overlay.setStrokeWidth(2);
        overlay.strokeCircle(x, y, 2.0);
        overlay.strokeLine(x, y, endX, endY);
    }

    private SimulationMode cycleMode(int delta) {
        SimulationMode[] modes = SimulationMode.values();
        int index = simulationMode.ordinal();
        int next = (index + delta) % modes.length;
        if (next < 0) {
            next += modes.length;
        }
        return modes[next];
    }

    private static Pose copyPose(Pose pose) {
        if (pose == null) {
            return new Pose(0, 0, 0);
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private enum SimulationMode {
        FULL_AUTO("Full Far Routine", false),
        WALL_LOOP("Alliance Wall Loop", true),
        PARK_SWEEP("Parking Sweep", false);

        final String displayName;
        final boolean loops;

        SimulationMode(String displayName, boolean loops) {
            this.displayName = displayName;
            this.loops = loops;
        }
    }

    private static class SimulationContext {
        final Pose startPose;
        final VirtualPath[] virtualPaths;

        SimulationContext(Pose startPose,
                          VirtualPath[] virtualPaths) {
            this.startPose = copyPose(startPose);
            this.virtualPaths = virtualPaths;
        }
    }

    public static class PathSimInputs {
        public double poseX;
        public double poseY;
        public double poseHeading;
        public double stepProgress;
        public double overallProgress;
        public double runtimeSec;
        public int loopCount;
        public int stepIndex;
        public boolean running;
        public String stepName = "";
        public String stepType = "";
    }

    private abstract static class SimulatedStep {
        private final String name;
        private final StepType type;
        protected Pose currentPose = new Pose(0, 0, 0);

        SimulatedStep(String name, StepType type) {
            this.name = name == null ? "Step" : name;
            this.type = type;
        }

        void start(Pose anchorPose) {
            currentPose = copyPose(anchorPose);
            onReset();
        }

        protected abstract void onReset();

        abstract void update(double dt);

        abstract boolean isFinished();

        abstract double getProgress();

        double getCommandedSpeedIps() {
            return 0.0;
        }

        Pose getPose() {
            return currentPose;
        }

        String getName() {
            return name;
        }

        StepType getType() {
            return type;
        }
    }

    private static final class SimulatedActionStep extends SimulatedStep {
        private final double durationSec;
        private double elapsed;

        SimulatedActionStep(String name, double durationSec) {
            super(name, StepType.ACTION);
            this.durationSec = Math.max(0.0, durationSec);
        }

        @Override
        protected void onReset() {
            elapsed = 0.0;
        }

        @Override
        void update(double dt) {
            elapsed += Math.max(0.0, dt);
        }

        @Override
        boolean isFinished() {
            return elapsed >= durationSec;
        }

        @Override
        double getProgress() {
            if (durationSec <= 0.0) {
                return 1.0;
            }
            return Math.min(1.0, elapsed / durationSec);
        }
    }

    private static final class SimulatedPathStep extends SimulatedStep {
        private final VirtualPath virtualPath;
        private final double speedIps;
        private double traveled;

        SimulatedPathStep(VirtualPath path, double speedIps) {
            super(path.getName(), StepType.PATH);
            this.virtualPath = path;
            this.speedIps = Math.max(0.1, speedIps);
        }

        @Override
        protected void onReset() {
            traveled = 0.0;
            currentPose = virtualPath.sample(0.0);
        }

        @Override
        void update(double dt) {
            double cappedDt = Math.max(0.0, dt);
            traveled = Math.min(virtualPath.length(), traveled + speedIps * cappedDt);
            currentPose = virtualPath.sample(traveled);
        }

        @Override
        boolean isFinished() {
            return traveled >= virtualPath.length();
        }

        @Override
        double getProgress() {
            if (virtualPath.length() <= 0.0) {
                return 1.0;
            }
            return traveled / virtualPath.length();
        }

        @Override
        double getCommandedSpeedIps() {
            return speedIps;
        }

        VirtualPath getVirtualPath() {
            return virtualPath;
        }
    }

    private enum StepType {
        PATH,
        ACTION
    }

    private static final class CommandRunner {
        private final List<SimulatedStep> steps;
        private final boolean loop;
        private Pose lastPose;
        private SimulatedStep activeStep;
        private int activeIndex = -1;
        private int completedLoops = 0;

        CommandRunner(List<SimulatedStep> sourceSteps, boolean loop, Pose startPose) {
            this.steps = new ArrayList<>(sourceSteps);
            this.loop = loop;
            this.lastPose = copyPose(startPose);
        }

        void restart(Pose startPose) {
            this.lastPose = copyPose(startPose);
            this.completedLoops = 0;
            this.activeIndex = -1;
            this.activeStep = null;
            advance();
        }

        boolean shouldContinue() {
            if (steps.isEmpty()) {
                return false;
            }
            if (loop) {
                return true;
            }
            return activeStep != null;
        }

        void update(double dt, PathSimInputs inputs) {
            if (activeStep == null) {
                inputs.running = false;
                return;
            }
            activeStep.update(dt);
            Pose pose = activeStep.getPose();
            if (pose != null) {
                lastPose = copyPose(pose);
            }

            inputs.poseX = lastPose.getX();
            inputs.poseY = lastPose.getY();
            inputs.poseHeading = lastPose.getHeading();
            inputs.stepName = activeStep.getName();
            inputs.stepType = activeStep.getType().name();
            inputs.stepIndex = activeIndex;
            inputs.loopCount = completedLoops;
            inputs.running = true;

            double stepProgress = Range.clip(activeStep.getProgress(), 0.0, 1.0);
            inputs.stepProgress = stepProgress;
            inputs.overallProgress = calculateOverallProgress(stepProgress);

            if (activeStep.isFinished()) {
                advance();
            }
        }

        int getCompletedLoops() {
            return completedLoops;
        }

        VirtualPath getActivePath() {
            if (activeStep instanceof SimulatedPathStep) {
                return ((SimulatedPathStep) activeStep).getVirtualPath();
            }
            return null;
        }

        private double calculateOverallProgress(double stepProgress) {
            if (steps.isEmpty()) {
                return 1.0;
            }
            double base = Math.max(0, activeIndex);
            if (loop) {
                double cycle = (base + stepProgress) / steps.size();
                return Math.min(1.0, cycle);
            }
            double raw = Math.min(steps.size(), base + stepProgress);
            return raw / steps.size();
        }

        private void advance() {
            if (steps.isEmpty()) {
                activeStep = null;
                return;
            }
            activeIndex++;
            if (activeIndex >= steps.size()) {
                if (loop) {
                    activeIndex = 0;
                    completedLoops++;
                } else {
                    activeStep = null;
                    return;
                }
            }
            activeStep = steps.get(activeIndex);
            activeStep.start(lastPose);
        }
    }

    private static final class VirtualPath {
        private static final double EPSILON = 1e-3;
        private static final double HEADING_LOOKAHEAD = 1.0;
        private final String name;
        private final List<Point2d> points;
        private final double[] cumulative;
        private final double totalLength;

        VirtualPath(String name, PathChain chain) {
            this.name = name == null ? "Path" : name;
            this.points = flattenChain(chain);
            this.cumulative = buildCumulative(points);
            this.totalLength = cumulative.length == 0 ? 0.0 : cumulative[cumulative.length - 1];
        }

        String getName() {
            return name;
        }

        boolean hasSamples() {
            return !points.isEmpty();
        }

        List<Point2d> getPoints() {
            return points;
        }

        double length() {
            return totalLength;
        }

        Pose sample(double distance) {
            if (points.isEmpty()) {
                return new Pose(0, 0, 0);
            }
            if (points.size() == 1 || totalLength <= 0.0) {
                Point2d point = points.get(0);
                return new Pose(point.x, point.y, 0.0);
            }
            double target = Range.clip(distance, 0.0, totalLength);
            Point2d position = samplePosition(target);
            double aheadDist = Math.min(totalLength, target + HEADING_LOOKAHEAD);
            double behindDist = Math.max(0.0, target - HEADING_LOOKAHEAD);
            Point2d ahead = samplePosition(aheadDist);
            Point2d behind = samplePosition(behindDist);
            double heading = headingBetween(behind, ahead);
            if (Double.isNaN(heading)) {
                heading = headingBetween(position, ahead);
                if (Double.isNaN(heading)) {
                    heading = 0.0;
                }
            }
            return new Pose(position.x, position.y, heading);
        }

        private Point2d samplePosition(double distance) {
            if (points.isEmpty()) {
                return new Point2d(0.0, 0.0);
            }
            double target = Range.clip(distance, 0.0, totalLength);
            int idx = Arrays.binarySearch(cumulative, target);
            if (idx >= 0 && idx < points.size()) {
                return points.get(idx);
            }
            int insertion = -idx - 1;
            int upper = Math.min(insertion, points.size() - 1);
            int lower = Math.max(0, upper - 1);
            double lowerDist = cumulative[lower];
            double upperDist = cumulative[upper];
            double span = Math.max(EPSILON, upperDist - lowerDist);
            double t = Range.clip((target - lowerDist) / span, 0.0, 1.0);
            Point2d start = points.get(lower);
            Point2d end = points.get(upper);
            double x = lerp(start.x, end.x, t);
            double y = lerp(start.y, end.y, t);
            return new Point2d(x, y);
        }

        private static List<Point2d> flattenChain(PathChain chain) {
            List<Point2d> pts = new ArrayList<>();
            if (chain == null) {
                return pts;
            }
            for (int i = 0; i < chain.size(); i++) {
                Path path = chain.getPath(i);
                if (path == null) {
                    continue;
                }
                double[][] raw = path.getPanelsDrawingPoints();
                appendPoints(pts, raw);
            }
            return pts;
        }

        private static void appendPoints(List<Point2d> pts, double[][] raw) {
            if (raw == null || raw.length == 0) {
                return;
            }
            if (isComponentArray(raw)) {
                double[] xs = raw[0];
                double[] ys = raw[1];
                int len = Math.min(xs.length, ys.length);
                for (int i = 0; i < len; i++) {
                    addPoint(pts, xs[i], ys[i]);
                }
            } else {
                for (double[] point : raw) {
                    if (point == null || point.length < 2) {
                        continue;
                    }
                    addPoint(pts, point[0], point[1]);
                }
            }
        }

        private static void addPoint(List<Point2d> pts, double rawX, double rawY) {
            double x = Double.isNaN(rawX) ? 0.0 : rawX;
            double y = Double.isNaN(rawY) ? 0.0 : rawY;
            Point2d next = new Point2d(x, y);
            if (pts.isEmpty() || distance(pts.get(pts.size() - 1), next) > EPSILON) {
                pts.add(next);
            }
        }

        private static boolean isComponentArray(double[][] raw) {
            if (raw.length < 2) {
                return false;
            }
            if (raw.length == 2) {
                return true;
            }
            return raw[0] != null && raw[1] != null;
        }

        private static double[] buildCumulative(List<Point2d> pts) {
            if (pts.isEmpty()) {
                return new double[0];
            }
            double[] cumulative = new double[pts.size()];
            cumulative[0] = 0.0;
            double sum = 0.0;
            for (int i = 1; i < pts.size(); i++) {
                sum += distance(pts.get(i - 1), pts.get(i));
                cumulative[i] = sum;
            }
            return cumulative;
        }

        private static double distance(Point2d a, Point2d b) {
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            return Math.hypot(dx, dy);
        }

        private static double headingBetween(Point2d a, Point2d b) {
            return Math.atan2(b.y - a.y, b.x - a.x);
        }

        private static double lerp(double start, double end, double t) {
            return start + (end - start) * t;
        }
    }

    private static final class Point2d {
        final double x;
        final double y;

        Point2d(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
