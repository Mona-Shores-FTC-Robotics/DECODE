package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;
import java.util.Optional;

@Configurable
public class DriveSubsystem implements Subsystem {

    private static final double AIM_KP = 1.5;
    private static final double VISION_TIMEOUT_MS = 500.0;
    private static final double STATIONARY_SPEED_THRESHOLD_IN_PER_SEC = 1.0;
    private static final double ODOMETRY_RELOCALIZE_DISTANCE_IN = 6.0;
    private static final String LOG_TAG = "DriveSubsystem";

    public enum DriveMode {
        NORMAL,
        SLOW
    }

    private static final double NORMAL_MULTIPLIER = 1.0;
    public static double slowMultiplier = 0.35;

    private final Follower follower;
    private final Constants.Motors driveMotors;
    private final DcMotorEx motorLf;
    private final DcMotorEx motorRf;
    private final DcMotorEx motorLb;
    private final DcMotorEx motorRb;
    private final VisionSubsystemLimelight vision;
    private final ElapsedTime clock = new ElapsedTime();

    private double lastGoodVisionAngle = Double.NaN;
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;

    private boolean robotCentric = false;
    private DriveMode activeMode = DriveMode.NORMAL;
    private double lastRequestFieldX = 0.0;
    private double lastRequestFieldY = 0.0;
    private double lastRequestRotation = 0.0;
    private boolean lastRequestSlowMode = false;
    private double lastCommandForward = 0.0;
    private double lastCommandStrafeLeft = 0.0;
    private double lastCommandTurn = 0.0;
    private RobotMode robotMode = RobotMode.DEBUG;

    public static final class Inputs {
        public double poseXInches;
        public double poseYInches;
        public double poseHeadingDeg;
        public boolean robotCentric;
        public String driveMode = DriveMode.NORMAL.name();
        public double requestFieldX;
        public double requestFieldY;
        public double requestRotation;
        public boolean requestSlowMode;
        public double commandForward;
        public double commandStrafeLeft;
        public double commandTurn;
        public boolean followerBusy;
        public double lfPower;
        public double rfPower;
        public double lbPower;
        public double rbPower;
        public double lfVelocityIps;
        public double rfVelocityIps;
        public double lbVelocityIps;
        public double rbVelocityIps;
        public double followerSpeedIps;
        public double lastVisionAngleDeg;
        public double visionSampleAgeMs;
    }

    public DriveSubsystem(HardwareMap hardwareMap , VisionSubsystemLimelight vision) {
        follower = Constants.createFollower(hardwareMap);
        driveMotors = new Constants.Motors(hardwareMap);
        driveMotors.setRunUsingEncoder();

        motorLf = driveMotors.lf;
        motorRf = driveMotors.rf;
        motorLb = driveMotors.lb;
        motorRb = driveMotors.rb;
        this.vision = vision;
    }

    public void populateInputs(Inputs inputs) {
        if (inputs == null) {
            return;
        }
        Pose pose = follower.getPose();
        if (pose != null) {
            inputs.poseXInches = pose.getX();
            inputs.poseYInches = pose.getY();
            inputs.poseHeadingDeg = Math.toDegrees(follower.getHeading());
        } else {
            inputs.poseXInches = 0.0;
            inputs.poseYInches = 0.0;
            inputs.poseHeadingDeg = 0.0;
        }
        inputs.robotCentric = robotCentric;
        inputs.driveMode = activeMode.name();
        inputs.requestFieldX = lastRequestFieldX;
        inputs.requestFieldY = lastRequestFieldY;
        inputs.requestRotation = lastRequestRotation;
        inputs.requestSlowMode = lastRequestSlowMode;
        inputs.commandForward = lastCommandForward;
        inputs.commandStrafeLeft = lastCommandStrafeLeft;
        inputs.commandTurn = lastCommandTurn;
        inputs.followerBusy = follower.isBusy();
        inputs.lfPower = motorLf.getPower();
        inputs.rfPower = motorRf.getPower();
        inputs.lbPower = motorLb.getPower();
        inputs.rbPower = motorRb.getPower();
        inputs.lfVelocityIps = ticksToInchesPerSecond(motorLf.getVelocity());
        inputs.rfVelocityIps = ticksToInchesPerSecond(motorRf.getVelocity());
        inputs.lbVelocityIps = ticksToInchesPerSecond(motorLb.getVelocity());
        inputs.rbVelocityIps = ticksToInchesPerSecond(motorRb.getVelocity());
        inputs.followerSpeedIps = followerVelocityIps();
        inputs.lastVisionAngleDeg = Double.isNaN(lastGoodVisionAngle) ? Double.NaN : Math.toDegrees(lastGoodVisionAngle);
        inputs.visionSampleAgeMs = lastVisionTimestamp == Double.NEGATIVE_INFINITY
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0 , clock.milliseconds() - lastVisionTimestamp);
    }

    public void setRobotCentric(boolean enabled) {
        robotCentric = enabled;
    }

    public void setRobotMode(RobotMode mode) {
        robotMode = RobotMode.orDefault(mode);
    }

    @Override
    public void initialize() {
        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        Pose seed = RobotState.takeHandoffPose();
        if (seed == null) {
            seed = new Pose();
        }

        follower.setStartingPose(seed);
        follower.update();
        follower.startTeleopDrive();
    }

    private double lastPeriodicMs = 0.0;

    @Override
    public void periodic() {
        long start = System.nanoTime();
        follower.update();
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void stop() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0 , 0 , 0 , robotCentric);
        driveMotors.stop();

        activeMode = DriveMode.NORMAL;
        lastCommandForward = 0.0;
        lastCommandStrafeLeft = 0.0;
        lastCommandTurn = 0.0;
    }

    /**
     * Applies scaled translation/rotation commands that have already been sampled from an input device.
     *
     * @param fieldX        translation along the field X axis (+right from the driver wall)
     * @param fieldY        translation along the field Y axis (+forward away from driver wall)
     * @param rotationInput rotation command with positive meaning counter-clockwise
     * @param slowMode      when true applies the configured slow multiplier
     */
    public void driveScaled(double fieldX , double fieldY , double rotationInput , boolean slowMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestRotation = rotationInput;
        lastRequestSlowMode = slowMode;
        double multiplier = slowMode ? Range.clip(slowMultiplier , 0.0 , 1.0) : NORMAL_MULTIPLIER;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);
        double turnCW = Range.clip(- rotationInput * multiplier , - 1.0 , 1.0);
        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turnCW;
        follower.setTeleOpDrive(forward , strafeLeft , turnCW , robotCentric);
        activeMode = slowMode ? DriveMode.SLOW : DriveMode.NORMAL;
    }

    public void aimAndDrive(double fieldX , double fieldY , boolean slowMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double multiplier = slowMode ? Range.clip(slowMultiplier , 0.0 , 1.0) : NORMAL_MULTIPLIER;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);
        double nowMs = clock.milliseconds();
        Optional<Double> visionAngle = vision.getAimAngle();

        double targetHeading;
        if (visionAngle.isPresent()) {
            targetHeading = visionAngle.get();
            lastGoodVisionAngle = targetHeading;
            lastVisionTimestamp = nowMs;
            maybeRelocalizeFromVision();
        } else if (! Double.isNaN(lastGoodVisionAngle) && nowMs - lastVisionTimestamp <= VISION_TIMEOUT_MS) {
            targetHeading = lastGoodVisionAngle;
        } else {
            Pose pose = follower.getPose();
            Pose targetPose = vision.getTargetGoalPose().orElse(FieldConstants.BLUE_GOAL_TAG);
            targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);
        }

        double headingError = normalizeAngle(targetHeading - follower.getHeading());
        double turn = Range.clip(AIM_KP * headingError , - 1.0 , 1.0);
        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward , strafeLeft , turn , robotCentric);
    }

    // --- Telemetry accessors ------------------------------------------------
    public DriveMode getDriveMode() {
        return activeMode;
    }

    public Pose2D getPose() {
        Pose pose = follower.getPose();
        if (pose == null) {
            return new Pose2D(DistanceUnit.INCH , 0.0 , 0.0 , AngleUnit.RADIANS , 0.0);
        }
        return new Pose2D(
                DistanceUnit.INCH ,
                pose.getX() ,
                pose.getY() ,
                AngleUnit.RADIANS ,
                follower.getHeading()
        );
    }

    public Pose getFollowerPose() {
        return follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public void updateFollower() {
        follower.update();
    }

    public void followPath(PathChain pathChain , boolean resetPose) {
        follower.followPath(pathChain , resetPose);
    }

    public boolean isFollowerBusy() {
        return follower.isBusy();
    }

    public double getLfPower() {
        return motorLf.getPower();
    }

    public double getRfPower() {
        return motorRf.getPower();
    }

    public double getLbPower() {
        return motorLb.getPower();
    }

    public double getRbPower() {
        return motorRb.getPower();
    }

    public double getLfVelocityTicksPerSec() {
        return motorLf.getVelocity();
    }

    public double getRfVelocityTicksPerSec() {
        return motorRf.getVelocity();
    }

    public double getLbVelocityTicksPerSec() {
        return motorLb.getVelocity();
    }

    public double getRbVelocityTicksPerSec() {
        return motorRb.getVelocity();
    }

    public void drawPoseOnPanels() {
        PanelsBridge.drawCurrentPose(follower);
    }

    public void drawPoseWithHistoryOnPanels() {
        PanelsBridge.drawCurrentPoseWithHistory(follower);
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle) , Math.cos(angle));
    }

    private void maybeRelocalizeFromVision() {
        if (! vision.shouldUpdateOdometry()) {
            return;
        }

        double speed = getRobotSpeedInchesPerSecond();
        if (speed > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            return;
        }

        Optional<Pose> tagPoseOpt = vision.getRobotPoseFromTag();
        if (! tagPoseOpt.isPresent()) {
            return;
        }

        Pose tagPose = tagPoseOpt.get();
        Pose odomPose = follower.getPose();
        double distance = distanceBetween(odomPose , tagPose);
        if (distance > ODOMETRY_RELOCALIZE_DISTANCE_IN) {
            return;
        }

        follower.setPose(tagPose);
        vision.markOdometryUpdated();
        RobotLog.dd(LOG_TAG , "Re-localized from AprilTag: (%.2f, %.2f, %.2f)" , tagPose.getX() , tagPose.getY() , tagPose.getHeading());
    }

    private double getRobotSpeedInchesPerSecond() {
        try {
            Vector velocity = follower.getVelocity();
            if (velocity == null) {
                return Double.POSITIVE_INFINITY;
            }
            return Math.abs(velocity.getMagnitude());
        } catch (Exception ignored) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private static double distanceBetween(Pose a , Pose b) {
        return Math.hypot(a.getX() - b.getX() , a.getY() - b.getY());
    }

    private double ticksToInchesPerSecond(double ticksPerSecond) {
        double mps = Constants.Speed.ticksPerSecToMps(ticksPerSecond);
        return DistanceUnit.METER.toInches(mps);
    }

    private double followerVelocityIps() {
        try {
            Vector velocity = follower.getVelocity();
            if (velocity == null) {
                return 0.0;
            }
            return velocity.getMagnitude();
        } catch (Exception ignored) {
            return 0.0;
        }
    }
}
