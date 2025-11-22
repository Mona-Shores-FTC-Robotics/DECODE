package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

/**
 * Fluent API for building command-based autonomous routines from .pp files.
 *
 * Provides granular control over:
 * - Per-segment speed/power
 * - Heading interpolation (linear, tangential, constant)
 * - Actions before/during/after each segment
 * - Skipping segments
 *
 * Example usage:
 * <pre>
 * AutoRoutineBuilder.fromPpFile("trajectory.pp", robot, Alliance.BLUE)
 *     .followNext()
 *         .withPower(0.9)
 *         .during(spinUpLauncher())
 *     .followNext()
 *         .withPower(0.5)  // Slow down the 2nd path
 *         .withHeadingInterpolation(0.8)  // More tangential
 *         .after(scoreSequence())
 *     .build();
 * </pre>
 */
public class AutoRoutineBuilder {

    private final List<PedroPathLoader.PathSegment> segments;
    private final Follower follower;
    private final List<Command> commandSequence;
    private int currentSegmentIndex = 0;

    // Current segment configuration
    private double currentPower = 0.8;
    private double currentHeadingInterpolation = 0.5;
    private Command beforeAction = null;
    private Command duringAction = null;
    private Command afterAction = null;

    private AutoRoutineBuilder(List<PedroPathLoader.PathSegment> segments, Follower follower) {
        this.segments = segments;
        this.follower = follower;
        this.commandSequence = new ArrayList<>();
    }

    /**
     * Start building an autonomous routine from a .pp file in assets
     *
     * @param ppFileName Name of .pp file in TeamCode/src/main/assets (e.g., "trajectory.pp")
     * @param robot Robot instance (provides follower)
     * @param hardwareMap HardwareMap from OpMode (for asset access)
     * @param alliance Alliance for mirroring
     * @return Builder instance
     */
    public static AutoRoutineBuilder fromPpFile(String ppFileName, Robot robot,
                                                 com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
                                                 Alliance alliance) {
        try {
            List<PedroPathLoader.PathSegment> segments =
                PedroPathLoader.loadFromAssets(ppFileName, hardwareMap, alliance);
            return new AutoRoutineBuilder(segments, robot.drive.getFollower());
        } catch (Exception e) {
            throw new RuntimeException("Failed to load .pp file: " + ppFileName, e);
        }
    }

    /**
     * Creates a builder from pre-loaded segments (for testing)
     */
    public static AutoRoutineBuilder fromSegments(List<PedroPathLoader.PathSegment> segments,
                                                   Follower follower) {
        return new AutoRoutineBuilder(segments, follower);
    }

    /**
     * Follow the next path segment from the .pp file
     * @return SegmentConfigurator for fine-tuning this segment
     */
    public SegmentConfigurator followNext() {
        if (currentSegmentIndex >= segments.size()) {
            throw new IllegalStateException("No more segments to follow! File has " +
                segments.size() + " segments, tried to follow segment " + (currentSegmentIndex + 1));
        }
        return new SegmentConfigurator(this);
    }

    /**
     * Follow a specific segment by name
     * @param segmentName Name of the segment (from .pp file)
     * @return SegmentConfigurator for fine-tuning this segment
     */
    public SegmentConfigurator followSegment(String segmentName) {
        for (int i = 0; i < segments.size(); i++) {
            if (segments.get(i).name.equals(segmentName)) {
                currentSegmentIndex = i;
                return new SegmentConfigurator(this);
            }
        }
        throw new IllegalArgumentException("No segment found with name: " + segmentName);
    }

    /**
     * Skip the next N segments (useful for testing partial routines)
     */
    public AutoRoutineBuilder skip(int count) {
        currentSegmentIndex += count;
        return this;
    }

    /**
     * Insert a raw command into the sequence
     */
    public AutoRoutineBuilder then(Command command) {
        commandSequence.add(command);
        return this;
    }

    /**
     * Insert a delay (in seconds)
     */
    public AutoRoutineBuilder delay(double seconds) {
        commandSequence.add(new Delay(seconds));
        return this;
    }

    /**
     * Build the final command sequence
     * @return Command ready to schedule with CommandManager
     */
    public Command build() {
        if (commandSequence.isEmpty()) {
            return new Delay(0.01); // Empty command
        }
        return new SequentialGroup(commandSequence.toArray(new Command[0]));
    }

    /**
     * Fluent configurator for individual path segments
     */
    public class SegmentConfigurator {
        private final AutoRoutineBuilder builder;

        private SegmentConfigurator(AutoRoutineBuilder builder) {
            this.builder = builder;
        }

        /**
         * Set maximum power for this segment (0.0 - 1.0)
         */
        public SegmentConfigurator withPower(double power) {
            builder.currentPower = Range.clip(power, 0.0, 1.0);
            return this;
        }

        /**
         * Set heading interpolation for this segment
         * @param interpolation 0.0 = more linear, 1.0 = more tangential
         */
        public SegmentConfigurator withHeadingInterpolation(double interpolation) {
            builder.currentHeadingInterpolation = Range.clip(interpolation, 0.0, 1.0);
            return this;
        }

        /**
         * Use linear heading interpolation (default)
         */
        public SegmentConfigurator withLinearHeading() {
            builder.currentHeadingInterpolation = 0.5;
            return this;
        }

        /**
         * Use tangential heading interpolation (follows path curve)
         */
        public SegmentConfigurator withTangentialHeading() {
            builder.currentHeadingInterpolation = 1.0;
            return this;
        }

        /**
         * Use constant heading (maintain start heading)
         */
        public SegmentConfigurator withConstantHeading() {
            builder.currentHeadingInterpolation = 0.0;
            return this;
        }

        /**
         * Execute command BEFORE following this segment
         */
        public SegmentConfigurator before(Command action) {
            builder.beforeAction = action;
            return this;
        }

        /**
         * Execute command IN PARALLEL while following this segment
         */
        public SegmentConfigurator during(Command action) {
            builder.duringAction = action;
            return this;
        }

        /**
         * Execute command AFTER following this segment
         */
        public SegmentConfigurator after(Command action) {
            builder.afterAction = action;
            return this;
        }

        /**
         * Finish configuring this segment and add it to the routine
         * @return Builder for chaining next segment
         */
        public AutoRoutineBuilder andThen() {
            // Build the command for this segment
            PedroPathLoader.PathSegment segment = builder.segments.get(builder.currentSegmentIndex);
            PathChain path = segment.buildPathChain(builder.follower, builder.currentHeadingInterpolation);
            FollowPath followCommand = new FollowPath(path, false, builder.currentPower);

            // Add before action if specified
            if (builder.beforeAction != null) {
                builder.commandSequence.add(builder.beforeAction);
            }

            // Add path following (with or without parallel action)
            if (builder.duringAction != null) {
                builder.commandSequence.add(new ParallelGroup(followCommand, builder.duringAction));
            } else {
                builder.commandSequence.add(followCommand);
            }

            // Add after action if specified
            if (builder.afterAction != null) {
                builder.commandSequence.add(builder.afterAction);
            }

            // Reset for next segment
            builder.currentSegmentIndex++;
            builder.currentPower = 0.8;
            builder.currentHeadingInterpolation = 0.5;
            builder.beforeAction = null;
            builder.duringAction = null;
            builder.afterAction = null;

            return builder;
        }
    }

    /**
     * Get segment count for this auto
     */
    public int getSegmentCount() {
        return segments.size();
    }

    /**
     * Get segment names for debugging
     */
    public List<String> getSegmentNames() {
        List<String> names = new ArrayList<>();
        for (PedroPathLoader.PathSegment segment : segments) {
            names.add(segment.name);
        }
        return names;
    }
}
