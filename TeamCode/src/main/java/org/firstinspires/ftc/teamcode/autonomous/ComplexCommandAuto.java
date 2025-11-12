package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.command.CommandScheduler;
import dev.nextftc.ftc.command.InstantCommand;
import dev.nextftc.ftc.command.ParallelCommandGroup;
import dev.nextftc.ftc.command.ParallelDeadlineGroup;
import dev.nextftc.ftc.command.ParallelRaceGroup;
import dev.nextftc.ftc.command.SequentialCommandGroup;
import dev.nextftc.pedro.PedroOpMode;
import dev.nextftc.pedro.command.FollowPathCommand;
import dev.nextftc.pedro.command.WaitCommand;

/**
 * Complex Autonomous using Sequential and Parallel Command Groups
 *
 * This demonstrates a realistic autonomous routine that:
 * 1. Scores a preloaded specimen on the high chamber
 * 2. Collects 3 samples from the field
 * 3. Scores each sample in the high basket
 * 4. Parks in the observation zone
 *
 * This showcases:
 * - Nested command groups (parallel inside sequential, sequential inside parallel)
 * - ParallelDeadlineGroup (all commands run until the deadline command finishes)
 * - ParallelRaceGroup (all commands run until ANY command finishes)
 * - Creating reusable command methods
 * - Complex autonomous logic with clean, readable code
 */
@Autonomous(name = "Complex Command Auto", group = "Examples")
public class ComplexCommandAuto extends PedroOpMode {

    // Field positions (adjust these to match your actual field positions)
    private static final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));
    private static final Pose SPECIMEN_SCORE_POSE = new Pose(10, 32, Math.toRadians(90));
    private static final Pose SAMPLE_1_POSE = new Pose(24, 24, Math.toRadians(0));
    private static final Pose SAMPLE_2_POSE = new Pose(36, 24, Math.toRadians(0));
    private static final Pose SAMPLE_3_POSE = new Pose(48, 24, Math.toRadians(0));
    private static final Pose BASKET_SCORE_POSE = new Pose(12, 48, Math.toRadians(135));
    private static final Pose PARK_POSE = new Pose(48, 48, Math.toRadians(90));

    @Override
    public void onInit() {
        follower.setStartingPose(START_POSE);

        // Build the complete autonomous routine
        SequentialCommandGroup autoRoutine = new SequentialCommandGroup(
            // ===== Phase 1: Score Preloaded Specimen =====
            scorePreloadedSpecimen(),

            // ===== Phase 2: Three-Sample Scoring Cycle =====
            // Collect and score sample 1
            collectAndScoreSample(SAMPLE_1_POSE, BASKET_SCORE_POSE),

            // Collect and score sample 2
            collectAndScoreSample(SAMPLE_2_POSE, BASKET_SCORE_POSE),

            // Collect and score sample 3
            collectAndScoreSample(SAMPLE_3_POSE, BASKET_SCORE_POSE),

            // ===== Phase 3: Park =====
            parkInObservationZone()
        );

        // Schedule the autonomous routine to run on start
        onStartPressed(() -> CommandScheduler.scheduleForState(autoRoutine, OpModeState.RUN));
    }

    /**
     * Scores the preloaded specimen on the high chamber
     */
    private SequentialCommandGroup scorePreloadedSpecimen() {
        Path toSpecimenBar = new Path(new BezierLine(
            START_POSE,
            SPECIMEN_SCORE_POSE
        ));
        toSpecimenBar.setHeadingInterpolation(HeadingInterpolator.linearFrom(
            START_POSE.getHeading(),
            SPECIMEN_SCORE_POSE.getHeading()
        ));

        return new SequentialCommandGroup(
            // Drive to specimen scoring position while raising arm
            new ParallelCommandGroup(
                new FollowPathCommand(follower, follower.pathBuilder()
                    .addPath(toSpecimenBar)
                    .build()),
                // Simulate raising arm to high position
                simulateArmToHigh()
            ),

            // Score the specimen
            new InstantCommand(() ->
                telemetry.addData("Phase 1", "Scoring specimen...")),
            new WaitCommand(0.5),
            new InstantCommand(() ->
                telemetry.addData("Phase 1", "Specimen scored!"))
        );
    }

    /**
     * Collects a sample from the field and scores it in the basket
     */
    private SequentialCommandGroup collectAndScoreSample(Pose samplePose, Pose scorePose) {
        // Create smooth curved path to sample
        Path toSample = new Path(new BezierCurve(
            new Point(follower.getPose()),
            new Point(samplePose.getX() - 10, samplePose.getY()),
            new Point(samplePose)
        ));
        toSample.setHeadingInterpolation(HeadingInterpolator.linearFrom(
            follower.getHeading(),
            samplePose.getHeading()
        ));

        // Create path to basket
        Path toBasket = new Path(new BezierCurve(
            new Point(samplePose),
            new Point((samplePose.getX() + scorePose.getX()) / 2, scorePose.getY()),
            new Point(scorePose)
        ));
        toBasket.setHeadingInterpolation(HeadingInterpolator.linearFrom(
            samplePose.getHeading(),
            scorePose.getHeading()
        ));

        return new SequentialCommandGroup(
            // Drive to sample while lowering intake
            new ParallelDeadlineGroup(
                // Deadline: drive to sample (when this finishes, others are interrupted)
                new FollowPathCommand(follower, follower.pathBuilder()
                    .addPath(toSample)
                    .build()),
                // This runs in parallel until the drive finishes
                simulateIntakeDown()
            ),

            // Collect the sample (using ParallelRaceGroup - finishes when FIRST command completes)
            new ParallelRaceGroup(
                // This is the "winner" - finishes after timeout
                new WaitCommand(1.0),
                // This runs until the wait finishes
                new InstantCommand(() ->
                    telemetry.addData("Action", "Collecting sample..."))
            ),
            new InstantCommand(() ->
                telemetry.addData("Action", "Sample collected!")),

            // Drive to basket while raising arm
            new ParallelCommandGroup(
                new FollowPathCommand(follower, follower.pathBuilder()
                    .addPath(toBasket)
                    .build()),
                // Start raising after a brief delay
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    simulateArmToHigh()
                )
            ),

            // Score in basket
            new InstantCommand(() ->
                telemetry.addData("Action", "Scoring in basket...")),
            new WaitCommand(0.5),
            new InstantCommand(() ->
                telemetry.addData("Action", "Sample scored!"))
        );
    }

    /**
     * Parks the robot in the observation zone
     */
    private SequentialCommandGroup parkInObservationZone() {
        Path toPark = new Path(new BezierLine(
            BASKET_SCORE_POSE,
            PARK_POSE
        ));
        toPark.setHeadingInterpolation(HeadingInterpolator.linearFrom(
            BASKET_SCORE_POSE.getHeading(),
            PARK_POSE.getHeading()
        ));

        return new SequentialCommandGroup(
            // Drive to park while retracting mechanisms
            new ParallelCommandGroup(
                new FollowPathCommand(follower, follower.pathBuilder()
                    .addPath(toPark)
                    .build()),
                new SequentialCommandGroup(
                    simulateArmToLow(),
                    simulateIntakeUp()
                )
            ),
            new InstantCommand(() ->
                telemetry.addData("Status", "Parked!"))
        );
    }

    // ===== Simulated Subsystem Commands =====
    // In a real robot, these would be actual commands controlling your mechanisms

    private SequentialCommandGroup simulateArmToHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() ->
                telemetry.addData("Arm", "Moving to HIGH...")),
            new WaitCommand(0.8),
            new InstantCommand(() ->
                telemetry.addData("Arm", "At HIGH position"))
        );
    }

    private SequentialCommandGroup simulateArmToLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() ->
                telemetry.addData("Arm", "Moving to LOW...")),
            new WaitCommand(0.6),
            new InstantCommand(() ->
                telemetry.addData("Arm", "At LOW position"))
        );
    }

    private SequentialCommandGroup simulateIntakeDown() {
        return new SequentialCommandGroup(
            new WaitCommand(0.2),
            new InstantCommand(() ->
                telemetry.addData("Intake", "Lowering...")),
            new WaitCommand(0.4),
            new InstantCommand(() ->
                telemetry.addData("Intake", "Down and active"))
        );
    }

    private SequentialCommandGroup simulateIntakeUp() {
        return new SequentialCommandGroup(
            new InstantCommand(() ->
                telemetry.addData("Intake", "Raising...")),
            new WaitCommand(0.3),
            new InstantCommand(() ->
                telemetry.addData("Intake", "Retracted"))
        );
    }
}
