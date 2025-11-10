package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseFusion;
import org.firstinspires.ftc.teamcode.util.PoseTransforms;
import org.firstinspires.ftc.teamcode.telemetry.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;
import java.util.Optional;

@Configurable
public class DriveSubsystem implements Subsystem {

    private static final double VISION_TIMEOUT_MS = 500.0;
    private static final double STATIONARY_SPEED_THRESHOLD_IN_PER_SEC = 1.0;
    private static final double ODOMETRY_RELOCALIZE_DISTANCE_IN = 6.0;
    private static final String LOG_TAG = "DriveSubsystem";

    @Configurable
    public static class TeleOpDriveConfig {
        public static double slowMultiplier = 0.07;
        public static double rotationOverrideThreshold = 0.05;
    }

    @Configurable
    public static class AimAssistConfig {
        public static double kP = .38;
        public static double kMaxTurn = .6;
    }

    @Configurable
    public static class RampConfig {
        public static double forwardRatePerSec = 0.3;
        public static double strafeRatePerSec = 0.3;
        public static double turnRatePerSec = 0.6;
        public static double fallbackDtSeconds = 0.02;
    }

    public enum DriveMode {
        NORMAL,
        SLOW
    }

    private static final double NORMAL_MULTIPLIER = 1.0;

    private final Follower follower;
    private final Constants.Motors driveMotors;
    private final DcMotorEx motorLf;
    private final DcMotorEx motorRf;
    private final DcMotorEx motorLb;
    private final DcMotorEx motorRb;
    private final VisionSubsystemLimelight vision;
    private final PoseFusion poseFusion = new PoseFusion();
    private final ElapsedTime clock = new ElapsedTime();

    private double lastGoodVisionAngle = Double.NaN;
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    private long lastFusionVisionTimestampMs = 0L;

    public static boolean robotCentricConfig = false;
    private boolean robotCentric = robotCentricConfig;
    private static final double ROTATION_OVERRIDE_FALLBACK = 0.05;
    private DriveMode activeMode = DriveMode.NORMAL;
    private double lastRequestFieldX = 0.0;
    private double lastRequestFieldY = 0.0;
    private double lastRequestRotation = 0.0;
    private boolean lastRequestSlowMode = false;
    private double lastCommandForward = 0.0;
    private double lastCommandStrafeLeft = 0.0;
    private double lastCommandTurn = 0.0;
    private double lastAimErrorRad = Double.NaN;
    private RobotMode robotMode = RobotMode.DEBUG;
    private boolean rampModeActive = false;
    private double rampForward = 0.0;
    private double rampStrafeLeft = 0.0;
    private double rampTurn = 0.0;
    private long lastRampUpdateNs = 0L;
    private boolean teleOpControlEnabled = false;
    private boolean visionRelocalizationEnabled = false;

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
        public boolean headingLockActive;
        public double headingLockTargetDeg;
        public double headingLockErrorDeg;
        public double headingLockOutput;
        public boolean followerBusy;
        public double lfPower;
        public double rfPower;
        public double lbPower;
        public double rbPower;
        public double lfCurrentAmps;
        public double rfCurrentAmps;
        public double lbCurrentAmps;
        public double rbCurrentAmps;
        public double driveTotalCurrentAmps;
        public double lfVelocityIps;
        public double rfVelocityIps;
        public double lbVelocityIps;
        public double rbVelocityIps;
        public double followerSpeedIps;
        public double lastVisionAngleDeg;
        public double visionSampleAgeMs;
        public boolean fusionHasPose;
        public double fusionPoseXInches;
        public double fusionPoseYInches;
        public double fusionPoseHeadingDeg;
        public double fusionDeltaXYInches;
        public double fusionDeltaHeadingDeg;
        public double fusionVisionWeight;
        public boolean fusionVisionAccepted;
        public double fusionVisionErrorInches;
        public double fusionVisionHeadingErrorDeg;
        public double fusionVisionRangeInches;
        public double fusionVisionDecisionMargin;
        public double fusionLastVisionAgeMs;
        public double fusionOdometryDtMs;
    }

    public DriveSubsystem(HardwareMap hardwareMap , VisionSubsystemLimelight vision) {
        follower = Constants.createFollower(hardwareMap);
        driveMotors = new Constants.Motors(hardwareMap);
        driveMotors.setRunWithoutEncoder();

        motorLf = driveMotors.lf;
        motorRf = driveMotors.rf;
        motorLb = driveMotors.lb;
        motorRb = driveMotors.rb;
        this.vision = vision;
    }

    public void setRobotCentric(boolean enabled) {
        robotCentric = enabled;
        robotCentricConfig = enabled;
    }

    public void setRobotMode(RobotMode mode) {
        robotMode = RobotMode.orDefault(mode);
    }

    public void setTeleOpControlEnabled(boolean enabled) {
        teleOpControlEnabled = enabled;
    }

    public void setVisionRelocalizationEnabled(boolean enabled) {
        visionRelocalizationEnabled = enabled;
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
        follower.setPose(seed);
        follower.update();
        if (teleOpControlEnabled) {
            follower.startTeleopDrive();
        } else {
            follower.breakFollowing();
        }
        poseFusion.reset(seed, System.currentTimeMillis());
        lastFusionVisionTimestampMs = vision.getLastPoseTimestampMs();
    }

    private double lastPeriodicMs = 0.0;

    @Override
    public void periodic() {
        long start = System.nanoTime();
        follower.update();
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
        updatePoseFusion();
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    public void stop() {
        if (teleOpControlEnabled) {
            follower.startTeleopDrive(true);
            follower.setTeleOpDrive(0 , 0 , 0 , robotCentric);
        } else {
            follower.breakFollowing();
        }
        driveMotors.stop();

        activeMode = DriveMode.NORMAL;
        lastCommandForward = 0.0;
        lastCommandStrafeLeft = 0.0;
        lastCommandTurn = 0.0;
        lastAimErrorRad = Double.NaN;
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
        inputs.lfCurrentAmps = readCurrentAmps(motorLf);
        inputs.rfCurrentAmps = readCurrentAmps(motorRf);
        inputs.lbCurrentAmps = readCurrentAmps(motorLb);
        inputs.rbCurrentAmps = readCurrentAmps(motorRb);
        inputs.driveTotalCurrentAmps = sumCurrentAmps(
                inputs.lfCurrentAmps,
                inputs.rfCurrentAmps,
                inputs.lbCurrentAmps,
                inputs.rbCurrentAmps
        );

        PoseFusion.State fusionState = poseFusion.getStateSnapshot();
        inputs.fusionHasPose = fusionState.hasFusedPose;
        if (fusionState.hasFusedPose && Double.isFinite(fusionState.fusedXInches) && Double.isFinite(fusionState.fusedYInches)) {
            inputs.fusionPoseXInches = fusionState.fusedXInches;
            inputs.fusionPoseYInches = fusionState.fusedYInches;
            inputs.fusionPoseHeadingDeg = Math.toDegrees(fusionState.fusedHeadingRad);
        } else {
            inputs.fusionPoseXInches = Double.NaN;
            inputs.fusionPoseYInches = Double.NaN;
            inputs.fusionPoseHeadingDeg = Double.NaN;
        }

        if (fusionState.hasFusedPose
                && Double.isFinite(fusionState.odometryXInches)
                && Double.isFinite(fusionState.odometryYInches)
                && Double.isFinite(fusionState.odometryHeadingRad)) {
            inputs.fusionDeltaXYInches = Math.hypot(
                    fusionState.fusedXInches - fusionState.odometryXInches,
                    fusionState.fusedYInches - fusionState.odometryYInches);
            inputs.fusionDeltaHeadingDeg = Math.toDegrees(
                    normalizeAngle(fusionState.fusedHeadingRad - fusionState.odometryHeadingRad));
        } else {
            inputs.fusionDeltaXYInches = Double.NaN;
            inputs.fusionDeltaHeadingDeg = Double.NaN;
        }

        inputs.fusionVisionWeight = fusionState.lastVisionWeight;
        inputs.fusionVisionAccepted = fusionState.lastVisionAccepted;
        inputs.fusionVisionErrorInches = fusionState.lastVisionTranslationErrorInches;
        inputs.fusionVisionHeadingErrorDeg = fusionState.lastVisionHeadingErrorDeg;
        inputs.fusionVisionRangeInches = fusionState.lastVisionRangeInches;
        inputs.fusionVisionDecisionMargin = fusionState.lastVisionDecisionMargin;
        inputs.fusionLastVisionAgeMs = fusionState.ageOfLastVisionMs;
        inputs.fusionOdometryDtMs = fusionState.lastOdometryDtMs;
    }

    private double readCurrentAmps(DcMotorEx motor) {
        if (motor == null) {
            return Double.NaN;
        }
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    private double sumCurrentAmps(double... values) {
        double total = 0.0;
        for (double value : values) {
            if (Double.isFinite(value)) {
                total += value;
            }
        }
        return total;
    }

    public void driveTeleOp(double fieldX,
                            double fieldY,
                            double rotationInput,
                            boolean slowMode,
                            boolean rampMode
                            ) {
        if (robotCentric != robotCentricConfig) {
            setRobotCentric(robotCentricConfig);
        }
        driveScaled(fieldX, fieldY, rotationInput, slowMode, rampMode);
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
        driveScaled(fieldX, fieldY, rotationInput, slowMode, false);
    }

    public void driveScaled(double fieldX , double fieldY , double rotationInput , boolean slowMode, boolean rampMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestRotation = rotationInput;
        lastRequestSlowMode = slowMode;
        double slowMultiplier = Range.clip(TeleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : NORMAL_MULTIPLIER;
        double targetForward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double targetStrafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);
        double targetTurnCW = Range.clip(- rotationInput * multiplier , - 1.0 , 1.0);

        double appliedForward = targetForward;
        double appliedStrafeLeft = targetStrafeLeft;
        double appliedTurn = targetTurnCW;

        if (rampMode) {
            long now = System.nanoTime();
            if (!rampModeActive) {
                rampForward = lastCommandForward;
                rampStrafeLeft = lastCommandStrafeLeft;
                rampTurn = lastCommandTurn;
                rampModeActive = true;
                lastRampUpdateNs = now;
            }
            double dt = lastRampUpdateNs == 0L ? 0.0 : (now - lastRampUpdateNs) / 1_000_000_000.0;
            lastRampUpdateNs = now;
            if (dt <= 0.0) {
                dt = Math.max(0.0, RampConfig.fallbackDtSeconds);
            }
            double forwardRate = Math.max(0.0, RampConfig.forwardRatePerSec);
            double strafeRate = Math.max(0.0, RampConfig.strafeRatePerSec);
            double turnRate = Math.max(0.0, RampConfig.turnRatePerSec);
            double maxForwardDelta = forwardRate * dt;
            double maxStrafeDelta = strafeRate * dt;
            double maxTurnDelta = turnRate * dt;
            rampForward = moveToward(rampForward, targetForward, maxForwardDelta);
            rampStrafeLeft = moveToward(rampStrafeLeft, targetStrafeLeft, maxStrafeDelta);
            rampTurn = moveToward(rampTurn, targetTurnCW, maxTurnDelta);
            appliedForward = rampForward;
            appliedStrafeLeft = rampStrafeLeft;
            appliedTurn = rampTurn;
        } else {
            rampModeActive = false;
            rampForward = appliedForward;
            rampStrafeLeft = appliedStrafeLeft;
            rampTurn = appliedTurn;
            lastRampUpdateNs = System.nanoTime();
        }


        lastCommandForward = appliedForward;
        lastCommandStrafeLeft = appliedStrafeLeft;
        lastCommandTurn = appliedTurn;
        follower.setTeleOpDrive(appliedForward , appliedStrafeLeft , appliedTurn , robotCentric);
        activeMode = slowMode ? DriveMode.SLOW : DriveMode.NORMAL;
    }

    private static double moveToward(double current, double target, double maxDelta) {
        double delta = target - current;
        if (Math.abs(delta) <= maxDelta) {
            return target;
        }
        return current + Math.copySign(maxDelta, delta);
    }

    public void aimAndDrive(double fieldX , double fieldY , boolean slowMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double slowMultiplier = Range.clip(TeleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : NORMAL_MULTIPLIER;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);
        double nowMs = clock.milliseconds();
        Optional<Double> visionAngle = vision.getAimAngle();

        double targetHeading;
        if (visionAngle.isPresent()) {
            targetHeading = visionAngle.get();
            lastGoodVisionAngle = targetHeading;
            lastVisionTimestamp = nowMs;
            if (visionRelocalizationEnabled) {
                maybeRelocalizeFromVision();
            }
        } else if (! Double.isNaN(lastGoodVisionAngle) && nowMs - lastVisionTimestamp <= VISION_TIMEOUT_MS) {
            targetHeading = lastGoodVisionAngle;
        } else {
            Pose pose = follower.getPose();
            Pose targetPose = vision.getTargetGoalPose().orElse(FieldConstants.BLUE_GOAL_TAG);
            targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);
        }

        double headingError = normalizeAngle(targetHeading - follower.getHeading());
        lastAimErrorRad = headingError;
        double maxTurn = Math.max(0.0, AimAssistConfig.kMaxTurn);
        double turn = Range.clip(AimAssistConfig.kP * headingError , - maxTurn , maxTurn);
        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward , strafeLeft , turn , robotCentric);
    }

    public double getLastAimErrorRadians() {
        return lastAimErrorRad;
    }

    public boolean isRobotStationary() {
        return getRobotSpeedInchesPerSecond() <= STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
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

    public void followPath(PathChain pathChain , boolean holdPositionAtEnd) {
        follower.followPath(pathChain , holdPositionAtEnd);
    }

    public void followPath(PathChain pathChain, double maxPower, boolean holdPositionAtEnd) {
        double clippedPower = Range.clip(maxPower, 0.0, 1.0);
        follower.followPath(pathChain, clippedPower, holdPositionAtEnd);
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

    public double getLastCommandTurn() {
        return lastCommandTurn;
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
        if (!visionRelocalizationEnabled) {
            return;
        }
        if (! vision.shouldUpdateOdometry()) {
            return;
        }

        double speed = getRobotSpeedInchesPerSecond();
        if (speed > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            return;
        }

        if (forceRelocalizeFromVision()) {
            // Successful re-localization already recorded inside the helper.
        }
    }

    /**
     * Forces the follower pose to match the latest Limelight pose when available.
     *
     * @return {@code true} when the pose was updated.
     */
    public boolean forceRelocalizeFromVision() {
        vision.findAllianceSnapshot(vision.getAlliance());
        Optional<Pose> tagPoseOpt = vision.getRobotPoseFromTag();
        if (!tagPoseOpt.isPresent()) {
            return false;
        }

        Pose tagPose = tagPoseOpt.get();
        double currentHeading = follower.getHeading();
        Pose adjustedPose = new Pose(tagPose.getX(), tagPose.getY(), currentHeading);
        follower.setPose(adjustedPose);
        vision.markOdometryUpdated();
        vision.overrideRobotPose(adjustedPose);
        poseFusion.reset(adjustedPose, System.currentTimeMillis());
        lastFusionVisionTimestampMs = vision.getLastPoseTimestampMs();
        // Keep the driver-facing heading to what was already tracked.
        lastGoodVisionAngle = currentHeading;
        lastVisionTimestamp = clock.milliseconds();
        RobotLog.dd(LOG_TAG, "Forced re-localization from AprilTag (heading preserved): (%.2f, %.2f, %.2f)", tagPose.getX(), tagPose.getY(), Math.toDegrees(currentHeading));
        return true;
    }


    private void updatePoseFusion() {
        long timestampMs = System.currentTimeMillis();
        poseFusion.updateWithOdometry(follower.getPose(), timestampMs);

        long visionTimestamp = vision.getLastPoseTimestampMs();
        if (visionTimestamp > 0L && visionTimestamp > lastFusionVisionTimestampMs) {
            if (!visionRelocalizationEnabled) {
                lastFusionVisionTimestampMs = visionTimestamp;
                return;
            }
            Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = vision.getLastSnapshot();
            if (snapshotOpt.isPresent()) {
                VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
                Optional<Pose> visionPoseOpt = snapshot.getRobotPose();
                if (visionPoseOpt.isPresent()) {
                    poseFusion.addVisionMeasurement(
                            visionPoseOpt.get(),
                            visionTimestamp,
                            snapshot.getFtcRange(),
                            snapshot.getDecisionMargin()
                    );
                }
            } else {
                Optional<Pose> visionPoseOpt = vision.getRobotPoseFromTag();
                if (visionPoseOpt.isPresent()) {
                    poseFusion.addVisionMeasurement(
                            visionPoseOpt.get(),
                            visionTimestamp,
                            Double.NaN,
                            Double.NaN
                    );
                }
            }
            lastFusionVisionTimestampMs = visionTimestamp;
        } else if (visionTimestamp == 0L) {
            lastFusionVisionTimestampMs = 0L;
        }
    }

    public PoseFusion.State getPoseFusionStateSnapshot() {
        return poseFusion.getStateSnapshot();
    }

    public void logPoseFusion(RobotLogger robotLogger) {
        if (robotLogger == null) {
            return;
        }
        PoseFusion.State state = poseFusion.getStateSnapshot();
        if (!state.hasFusedPose) {
            return;
        }
        if (Double.isFinite(state.fusedXInches)) {
            robotLogger.logNumber("FusionPose", "poseX", DistanceUnit.INCH.toMeters(state.fusedXInches));
        }
        if (Double.isFinite(state.fusedYInches)) {
            robotLogger.logNumber("FusionPose", "poseY", DistanceUnit.INCH.toMeters(state.fusedYInches));
        }
        if (Double.isFinite(state.fusedHeadingRad)) {
            robotLogger.logNumber("FusionPose", "poseHeading", state.fusedHeadingRad);
        }
        Pose fusedPedroPose = (Double.isFinite(state.fusedXInches) && Double.isFinite(state.fusedYInches) && Double.isFinite(state.fusedHeadingRad))
                ? new Pose(state.fusedXInches, state.fusedYInches, state.fusedHeadingRad)
                : null;
        Pose fusedFtcPose = PoseTransforms.toFtcPose(fusedPedroPose);
        if (fusedFtcPose != null) {
            robotLogger.logNumber("FusionPoseFtc", "poseX", DistanceUnit.INCH.toMeters(fusedFtcPose.getX()));
            robotLogger.logNumber("FusionPoseFtc", "poseY", DistanceUnit.INCH.toMeters(fusedFtcPose.getY()));
            robotLogger.logNumber("FusionPoseFtc", "poseHeading", fusedFtcPose.getHeading());
        }
        if (Double.isFinite(state.odometryXInches) && Double.isFinite(state.odometryYInches)) {
            robotLogger.logNumber("OdometryPose", "poseX", DistanceUnit.INCH.toMeters(state.odometryXInches));
            robotLogger.logNumber("OdometryPose", "poseY", DistanceUnit.INCH.toMeters(state.odometryYInches));
            robotLogger.logNumber("OdometryPose", "poseHeading", state.odometryHeadingRad);
            Pose odomPedroPose = new Pose(state.odometryXInches, state.odometryYInches, state.odometryHeadingRad);
            Pose odomFtcPose = PoseTransforms.toFtcPose(odomPedroPose);
            if (odomFtcPose != null) {
                robotLogger.logNumber("OdometryPoseFtc", "poseX", DistanceUnit.INCH.toMeters(odomFtcPose.getX()));
                robotLogger.logNumber("OdometryPoseFtc", "poseY", DistanceUnit.INCH.toMeters(odomFtcPose.getY()));
                robotLogger.logNumber("OdometryPoseFtc", "poseHeading", odomFtcPose.getHeading());
            }
        }
    }

    public double getRobotSpeedInchesPerSecond() {
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
