package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseFusion;
import org.firstinspires.ftc.teamcode.util.PoseTransforms;
import org.firstinspires.ftc.teamcode.util.RobotState;
import java.util.Optional;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Configurable
public class DriveSubsystem implements Subsystem {

    private static final double VISION_TIMEOUT_MS = 100;
    private static final double STATIONARY_SPEED_THRESHOLD_IN_PER_SEC = 1.5;
    private static final double ODOMETRY_RELOCALIZE_DISTANCE_IN = 6.0;
    private static final String LOG_TAG = "DriveSubsystem";

    @Configurable
    public static class TeleOpDriveConfig {
        /** Max power multiplier for normal (non-slow) teleop driving (0.0-1.0) */
        public double normalMultiplier = .8;
        /** Max power multiplier for slow mode teleop driving (0.0-1.0) */
        public double slowMultiplier = 0.2;
        /** Turn multiplier for normal mode */
        public double normalTurnMultiplier = .7;
        /** Turn multiplier for slow mode */
        public double slowTurnMultiplier = 0.4;
        /** Rotation override threshold for aim assist */
        public double rotationOverrideThreshold = 0.05;
    }

    @Configurable
    public static class AimAssistConfig {
        /** Proportional gain for geometry-based aiming - higher = faster response to error */
        public double kP = 0.5;
        /** Max turn speed when aiming (0.0-1.0) */
        public double kMaxTurn = 0.7;
    }

    @Configurable
    public static class VisionCenteredAimConfig {
        /** Proportional gain for vision-centered aiming (turn per degree of tx offset) */
        public double kP = 0.03;
        /** Max turn speed when aiming (0.0-1.0) */
        public double kMaxTurn = 0.7;
        /** Deadband - stop turning when tx error is below this (degrees) */
        public double deadbandDeg = 1.0;
    }

    @Configurable
    public static class FixedAngleAimConfig {
        /** Fixed target heading for blue alliance (degrees, 0=forward, 90=left) */
        public double blueHeadingDeg = 60.0;
        /** Fixed target heading for red alliance (degrees, 0=forward, 90=left) */
        public double redHeadingDeg = 120.0;
        /** Proportional gain for fixed-angle aiming */
        public double kP = 0.5;
        /** Max turn speed when aiming (0.0-1.0) */
        public double kMaxTurn = 0.7;
    }

    @Configurable
    public static class RampConfig {
        public double forwardRatePerSec = 0.3;
        public double strafeRatePerSec = 0.3;
        public double turnRatePerSec = 0.6;
        public double fallbackDtSeconds = 0.02;
    }

    public static TeleOpDriveConfig teleOpDriveConfig = new TeleOpDriveConfig();
    public static AimAssistConfig aimAssistConfig = new AimAssistConfig();
    public static VisionCenteredAimConfig visionCenteredAimConfig = new VisionCenteredAimConfig();
    public static FixedAngleAimConfig fixedAngleAimConfig = new FixedAngleAimConfig();
    public static RampConfig rampConfig = new RampConfig();

    public String visionRelocalizeStatus = "Press A to re-localize";
    public long visionRelocalizeStatusMs = 0L;

    public enum DriveMode {
        NORMAL,
        SLOW
    }

    // Removed NORMAL_MULTIPLIER constant - now uses teleOpDriveConfig.normalMultiplier

    private Follower follower;
    private final Constants.Motors driveMotors;
    private final DcMotorEx motorLf;
    private final DcMotorEx motorRf;
    private final DcMotorEx motorLb;
    private final DcMotorEx motorRb;
    private final VisionSubsystemLimelight vision;
    private final PoseFusion poseFusion = new PoseFusion();
    private final ElapsedTime clock = new ElapsedTime();

    // Default command for NextFTC command scheduler
    private Command driveDefaultCommand = null;

    private double lastGoodVisionAngle = Double.NaN;
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    private long lastFusionVisionTimestampMs = 0L;
    private double fieldHeadingOffsetRad = DESIRED_FIELD_HEADING_RAD;
    private static final double DESIRED_FIELD_HEADING_RAD = Math.PI / 2.0;
    private boolean visionHeadingCalibrated = false;

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
    private boolean rampModeActive = false;
    private double rampForward = 0.0;
    private double rampStrafeLeft = 0.0;
    private double rampTurn = 0.0;
    private long lastRampUpdateNs = 0L;
    private boolean teleOpControlEnabled = false;
    private boolean visionRelocalizationEnabled = false;

    public DriveSubsystem(HardwareMap hardwareMap , VisionSubsystemLimelight vision) {
//        follower = Constants.createFollower(hardwareMap);
        driveMotors = new Constants.Motors(hardwareMap);
        driveMotors.setRunWithoutEncoder();

        motorLf = driveMotors.lf;
        motorRf = driveMotors.rf;
        motorLb = driveMotors.lb;
        motorRb = driveMotors.rb;
        this.vision = vision;
    }

    public void attachFollower() {
        this.follower = follower();
    }

    /**
     * Sets the default command for this subsystem.
     * The default command runs whenever no other command requires this subsystem.
     */
    public void setDefaultCommand(Command command) {
        this.driveDefaultCommand = command;
    }

    /**
     * Gets the default command for this subsystem.
     * Overrides the Subsystem interface property to return our configured command.
     */
    @NonNull
    @Override
    public Command getDefaultCommand() {
        return driveDefaultCommand != null ? driveDefaultCommand : Subsystem.super.getDefaultCommand();
    }

    public void setRobotCentric(boolean enabled) {
        robotCentric = enabled;
        robotCentricConfig = enabled;
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
        // Always break following during init to prevent driving before match starts
        // Call startTeleopDrive() explicitly when ready to enable motors
        follower.breakFollowing();
        fieldHeadingOffsetRad = DESIRED_FIELD_HEADING_RAD;
        visionHeadingCalibrated = false;
        poseFusion.reset(seed , System.currentTimeMillis());
        lastFusionVisionTimestampMs = vision.getLastPoseTimestampMs();
    }

    /**
     * Starts teleop drive mode, enabling motor control.
     * Should be called when the match starts (after init phase).
     */
    public void startTeleopDrive() {
        if (teleOpControlEnabled) {
            follower.startTeleopDrive();
        }
    }

    private double lastPeriodicMs = 0.0;

    @Override
    public void periodic() {
        long start = System.nanoTime();
        follower.update();
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;
        updatePoseFusion();

        // Log robot pose for AdvantageScope field visualization (required for WPILOG replay)
        Pose pose = follower.getPose();
        if (pose != null) {
            // Convert inches to meters for AdvantageScope (SI units required)
            double xMeters = pose.getX() * 0.0254;
            double yMeters = pose.getY() * 0.0254;
            double headingRadians = follower.getHeading();
//            KoalaLog.logPose2d("Robot/Pose", xMeters, yMeters, headingRadians, true);
        }
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

    protected double readCurrentAmps(DcMotorEx motor) {
        if (motor == null) {
            return Double.NaN;
        }
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    protected double sumCurrentAmps(double... values) {
        double total = 0.0;
        for (double value : values) {
            if (Double.isFinite(value)) {
                total += value;
            }
        }
        return total;
    }

    public void driveTeleOp(double fieldX ,
                            double fieldY ,
                            double rotationInput ,
                            boolean slowMode ,
                            boolean rampMode
    ) {
        if (robotCentric != robotCentricConfig) {
            setRobotCentric(robotCentricConfig);
        }
        double[] rotated = rotateFieldInput(fieldX , fieldY);
        driveScaled(rotated[0] , rotated[1] , rotationInput , slowMode , rampMode);
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
        driveScaled(fieldX , fieldY , rotationInput , slowMode , false);
    }

    public void driveScaled(double fieldX , double fieldY , double rotationInput , boolean slowMode , boolean rampMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestRotation = rotationInput;
        lastRequestSlowMode = slowMode;

        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double slowTurnMultiplier = Range.clip(teleOpDriveConfig.slowTurnMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double normalTurnMultiplier = Range.clip(teleOpDriveConfig.normalTurnMultiplier , 0.0 , 1.0);
        double driveMultiplier = slowMode ? slowMultiplier : normalMultiplier;
        double turnMultiplier = slowMode ? slowTurnMultiplier : normalTurnMultiplier;
        double targetForward = Range.clip(fieldY * driveMultiplier , - 1.0 , 1.0);
        double targetStrafeLeft = Range.clip(- fieldX * driveMultiplier , - 1.0 , 1.0);
        double targetTurnCW = Range.clip(- rotationInput * turnMultiplier , - 1.0 , 1.0);

        double appliedForward = targetForward;
        double appliedStrafeLeft = targetStrafeLeft;
        double appliedTurn = targetTurnCW;

        if (rampMode) {
            long now = System.nanoTime();
            if (! rampModeActive) {
                rampForward = lastCommandForward;
                rampStrafeLeft = lastCommandStrafeLeft;
                rampTurn = lastCommandTurn;
                rampModeActive = true;
                lastRampUpdateNs = now;
            }
            double dt = lastRampUpdateNs == 0L ? 0.0 : (now - lastRampUpdateNs) / 1_000_000_000.0;
            lastRampUpdateNs = now;
            if (dt <= 0.0) {
                dt = Math.max(0.0 , rampConfig.fallbackDtSeconds);
            }
            double forwardRate = Math.max(0.0 , rampConfig.forwardRatePerSec);
            double strafeRate = Math.max(0.0 , rampConfig.strafeRatePerSec);
            double turnRate = Math.max(0.0 , rampConfig.turnRatePerSec);
            double maxForwardDelta = forwardRate * dt;
            double maxStrafeDelta = strafeRate * dt;
            double maxTurnDelta = turnRate * dt;
            rampForward = moveToward(rampForward , targetForward , maxForwardDelta);
            rampStrafeLeft = moveToward(rampStrafeLeft , targetStrafeLeft , maxStrafeDelta);
            rampTurn = moveToward(rampTurn , targetTurnCW , maxTurnDelta);
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

    private static double moveToward(double current , double target , double maxDelta) {
        double delta = target - current;
        if (Math.abs(delta) <= maxDelta) {
            return target;
        }
        return current + Math.copySign(maxDelta , delta);
    }

    /**
     * Geometry-based aiming: Calculates angle from robot pose to basket centroid using atan2.
     * Uses odometry pose and basket target coordinates (tunable via FTC Dashboard).
     * Falls back to vision-based angle if available and relocalization is enabled.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDrive(double fieldX , double fieldY , boolean slowMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double[] rotated = rotateFieldInput(fieldX , fieldY);
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(rotated[1] * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- rotated[0] * multiplier , - 1.0 , 1.0);
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
            // Use tunable basket target coordinates
            Pose pose = follower.getPose();
            Alliance alliance = vision.getAlliance();
            Pose targetPose = (alliance == Alliance.RED)
                ? FieldConstants.getRedBasketTarget()
                : FieldConstants.getBlueBasketTarget();
            targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);
        }

        double headingError = normalizeAngle(targetHeading - follower.getHeading());
        lastAimErrorRad = headingError;

        // Simple P controller
        double maxTurn = Math.max(0.0, aimAssistConfig.kMaxTurn);
        double turn = Range.clip(aimAssistConfig.kP * headingError, -maxTurn, maxTurn);

        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);
    }

    /**
     * Vision-centered aiming: Uses Limelight tx (horizontal offset) to center the AprilTag.
     * This approach directly uses the camera's measurement without coordinate calculations.
     * Similar to the original FTC RobotAutoDriveToAprilTagOmni example.
     *
     * When tx = 0, the target is centered in the camera view.
     * tx > 0 means target is to the right, tx < 0 means target is to the left.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDriveVisionCentered(double fieldX , double fieldY , boolean slowMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double[] rotated = rotateFieldInput(fieldX , fieldY);
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(rotated[1] * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- rotated[0] * multiplier , - 1.0 , 1.0);

        // Get horizontal offset from Limelight (tx)
        double txDegrees = vision.getLastTxDegrees();

        // If no valid tag, hold current heading (zero turn)
        if (Double.isNaN(txDegrees)) {
            lastCommandForward = forward;
            lastCommandStrafeLeft = strafeLeft;
            lastCommandTurn = 0.0;
            lastRequestRotation = 0.0;
            lastAimErrorRad = Double.NaN;
            follower.setTeleOpDrive(forward, strafeLeft, 0.0, robotCentric);
            return;
        }

        // Deadband - stop turning if error is small enough
        if (Math.abs(txDegrees) < visionCenteredAimConfig.deadbandDeg) {
            txDegrees = 0.0;
        }

        // Convert to turn command
        // tx > 0 (target right) -> turn clockwise (negative in Pedro convention)
        // tx < 0 (target left) -> turn counter-clockwise (positive)
        double maxTurn = Math.max(0.0, visionCenteredAimConfig.kMaxTurn);
        double turn = Range.clip(-txDegrees * visionCenteredAimConfig.kP, -maxTurn, maxTurn);

        lastAimErrorRad = Math.toRadians(txDegrees); // Store for telemetry
        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);

        // Optionally relocalize if enabled
        if (visionRelocalizationEnabled && vision.hasValidTag()) {
            maybeRelocalizeFromVision();
        }
    }

    /**
     * Fixed-angle aiming: Rotates to a fixed heading based on alliance.
     * Simple and predictable - works well from specific field positions (e.g., launch line).
     * No pose or vision calculations needed.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDriveFixedAngle(double fieldX , double fieldY , boolean slowMode) {
        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double[] rotated = rotateFieldInput(fieldX , fieldY);
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(rotated[1] * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- rotated[0] * multiplier , - 1.0 , 1.0);

        // Get fixed target heading based on alliance
        Alliance alliance = vision.getAlliance();
        double targetHeadingDeg = (alliance == Alliance.RED)
            ? fixedAngleAimConfig.redHeadingDeg
            : fixedAngleAimConfig.blueHeadingDeg;
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        // Calculate heading error
        double headingError = normalizeAngle(targetHeadingRad - follower.getHeading());
        lastAimErrorRad = headingError;

        // Simple P controller
        double maxTurn = Math.max(0.0, fixedAngleAimConfig.kMaxTurn);
        double turn = Range.clip(fixedAngleAimConfig.kP * headingError, -maxTurn, maxTurn);

        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);
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

    public void followPath(PathChain pathChain , double maxPower , boolean holdPositionAtEnd) {
        double clippedPower = Range.clip(maxPower , 0.0 , 1.0);
        follower.followPath(pathChain , clippedPower , holdPositionAtEnd);
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

    public double getLastCommandDrive() {
        return lastCommandForward;
    }

    public double getLastCommandStrafe() {
        return lastCommandStrafeLeft;
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

    protected double[] rotateFieldInput(double fieldX , double fieldY) {
        double cos = Math.cos(- fieldHeadingOffsetRad);
        double sin = Math.sin(- fieldHeadingOffsetRad);
        return new double[]{
                cos * fieldX - sin * fieldY ,
                sin * fieldX + cos * fieldY
        };
    }

    private void maybeRelocalizeFromVision() {
        if (! visionRelocalizationEnabled) {
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
    public void tryRelocalize() {
        boolean tagVisible = vision.hasValidTag();

        if (tagVisible &&
                (vision.getCurrentTagId() == FieldConstants.BLUE_GOAL_TAG_ID ||
                        vision.getCurrentTagId() == FieldConstants.RED_GOAL_TAG_ID)) {
            boolean success = forceRelocalizeFromVision();
            if (success) {
                visionRelocalizeStatus = "Pose updated from Limelight";
            } else {
                visionRelocalizeStatus = "Failed to update pose from Limelight";
            }
        } else if (tagVisible) {
            visionRelocalizeStatus = "No Goal AprilTag visible";
        } else {
            visionRelocalizeStatus = "No AprilTag visible";
        }
        visionRelocalizeStatusMs = System.currentTimeMillis();
    }

    public boolean forceRelocalizeFromVision() {
        vision.findAllianceSnapshot(vision.getAlliance());
        Optional<Pose> tagPoseOpt = vision.getRobotPoseFromTag();
        if (!tagPoseOpt.isPresent()) {
            return false;
        }

        Pose tagPose = tagPoseOpt.get();
        double currentHeading = follower.getHeading();
        double visionHeading = tagPose.getHeading();
        Pose adjustedPose = new Pose(tagPose.getX(), tagPose.getY(), visionHeading);
        follower.setPose(adjustedPose);
        vision.markOdometryUpdated();
        vision.overrideRobotPose(adjustedPose);
        poseFusion.reset(adjustedPose, System.currentTimeMillis());
        lastFusionVisionTimestampMs = vision.getLastPoseTimestampMs();
        // Keep the driver-facing heading in sync with vision so offsets keep controls stable.
        if (!visionHeadingCalibrated) {
            fieldHeadingOffsetRad = 0.0;
            visionHeadingCalibrated = true;
        }
        lastGoodVisionAngle = visionHeading;
        lastVisionTimestamp = clock.milliseconds();
        RobotLog.dd(LOG_TAG, "Forced re-localization from AprilTag: vision heading %.1f°, odometry heading %.1f°, offset %.1f° -> adjusted heading %.1f°",
                Math.toDegrees(visionHeading),
                Math.toDegrees(currentHeading),
                Math.toDegrees(fieldHeadingOffsetRad),
                Math.toDegrees(adjustedPose.getHeading()));
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

    public double getRawPinpointHeadingDeg() {
        try {
            if (follower.getPoseTracker() != null && follower.getPoseTracker().getLocalizer() != null) {
                Pose rawPose = follower.getPoseTracker().getLocalizer().getPose();
                if (rawPose != null) {
                    return Math.toDegrees(rawPose.getHeading());
                }
            }
        } catch (Exception ignored) {
            // Some localizers may not expose raw heading
        }
        return Double.NaN;
    }

//    public boolean correctInitialHeadingFromVision() {
//        if (!vision.hasValidTag()) {
//            RobotLog.dd(LOG_TAG, "No AprilTag visible - cannot correct heading");
//            return false;
//        }
//        boolean success = forceRelocalizeFromVision();
//        if (!success) {
//            RobotLog.dd(LOG_TAG, "Vision pose unavailable - cannot correct heading");
//        }
//        return success;
//    }

    private static double distanceBetween(Pose a , Pose b) {
        return Math.hypot(a.getX() - b.getX() , a.getY() - b.getY());
    }

    protected double ticksToInchesPerSecond(double ticksPerSecond) {
        double mps = Constants.Speed.ticksPerSecToMps(ticksPerSecond);
        return DistanceUnit.METER.toInches(mps);
    }

    protected double followerVelocityIps() {
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

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    public double getPoseXInches() {
        Pose pose = follower.getPose();
        return pose != null ? pose.getX() : 0.0;
    }

    public double getPoseYInches() {
        Pose pose = follower.getPose();
        return pose != null ? pose.getY() : 0.0;
    }

    public double getPoseHeadingDeg() {
        return Math.toDegrees(follower.getHeading());
    }

    public boolean isRobotCentric() {
        return robotCentric;
    }

    public String getDriveModeString() {
        return activeMode.name();
    }

    public double getRequestFieldX() {
        return lastRequestFieldX;
    }

    public double getRequestFieldY() {
        return lastRequestFieldY;
    }

    public double getRequestRotation() {
        return lastRequestRotation;
    }

    public boolean getRequestSlowMode() {
        return lastRequestSlowMode;
    }

    public double getCommandForward() {
        return lastCommandForward;
    }

    public double getCommandStrafeLeft() {
        return lastCommandStrafeLeft;
    }

    public double getCommandTurn() {
        return lastCommandTurn;
    }

    public boolean isFollowerBusyLogged() {
        return follower.isBusy();
    }

    public double getLfPowerLogged() {
        return motorLf.getPower();
    }

    public double getRfPowerLogged() {
        return motorRf.getPower();
    }

    public double getLbPowerLogged() {
        return motorLb.getPower();
    }

    public double getRbPowerLogged() {
        return motorRb.getPower();
    }

    public double getLfCurrentAmps() {
        return readCurrentAmps(motorLf);
    }

    public double getRfCurrentAmps() {
        return readCurrentAmps(motorRf);
    }

    public double getLbCurrentAmps() {
        return readCurrentAmps(motorLb);
    }

    public double getRbCurrentAmps() {
        return readCurrentAmps(motorRb);
    }

    public double getDriveTotalCurrentAmps() {
        return sumCurrentAmps(
                getLfCurrentAmps(),
                getRfCurrentAmps(),
                getLbCurrentAmps(),
                getRbCurrentAmps()
        );
    }

    public double getLfVelocityIps() {
        return ticksToInchesPerSecond(motorLf.getVelocity());
    }

    public double getRfVelocityIps() {
        return ticksToInchesPerSecond(motorRf.getVelocity());
    }

    public double getLbVelocityIps() {
        return ticksToInchesPerSecond(motorLb.getVelocity());
    }

    public double getRbVelocityIps() {
        return ticksToInchesPerSecond(motorRb.getVelocity());
    }

    public double getFollowerSpeedIps() {
        return followerVelocityIps();
    }

    public double getLastVisionAngleDeg() {
        return Double.isNaN(lastGoodVisionAngle) ? Double.NaN : Math.toDegrees(lastGoodVisionAngle);
    }

    public double getVisionSampleAgeMs() {
        return lastVisionTimestamp == Double.NEGATIVE_INFINITY
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0, clock.milliseconds() - lastVisionTimestamp);
    }

    public boolean getFusionHasPose() {
        return poseFusion.getStateSnapshot().hasFusedPose;
    }

    public double getFusionPoseXInches() {
        PoseFusion.State state = poseFusion.getStateSnapshot();
        return (state.hasFusedPose && Double.isFinite(state.fusedXInches)) ? state.fusedXInches : Double.NaN;
    }

    public double getFusionPoseYInches() {
        PoseFusion.State state = poseFusion.getStateSnapshot();
        return (state.hasFusedPose && Double.isFinite(state.fusedYInches)) ? state.fusedYInches : Double.NaN;
    }

    public double getFusionPoseHeadingDeg() {
        PoseFusion.State state = poseFusion.getStateSnapshot();
        return (state.hasFusedPose && Double.isFinite(state.fusedHeadingRad)) ? Math.toDegrees(state.fusedHeadingRad) : Double.NaN;
    }

    public double getFusionDeltaXYInches() {
        PoseFusion.State state = poseFusion.getStateSnapshot();
        if (state.hasFusedPose
                && Double.isFinite(state.odometryXInches)
                && Double.isFinite(state.odometryYInches)) {
            return Math.hypot(
                    state.fusedXInches - state.odometryXInches,
                    state.fusedYInches - state.odometryYInches);
        }
        return Double.NaN;
    }

    public double getFusionDeltaHeadingDeg() {
        PoseFusion.State state = poseFusion.getStateSnapshot();
        if (state.hasFusedPose
                && Double.isFinite(state.odometryXInches)
                && Double.isFinite(state.odometryYInches)
                && Double.isFinite(state.odometryHeadingRad)) {
            return Math.toDegrees(normalizeAngle(state.fusedHeadingRad - state.odometryHeadingRad));
        }
        return Double.NaN;
    }

    public double getFusionVisionWeight() {
        return poseFusion.getStateSnapshot().lastVisionWeight;
    }

    public boolean getFusionVisionAccepted() {
        return poseFusion.getStateSnapshot().lastVisionAccepted;
    }

    public double getFusionVisionErrorInches() {
        return poseFusion.getStateSnapshot().lastVisionTranslationErrorInches;
    }

    public double getFusionVisionHeadingErrorDeg() {
        return poseFusion.getStateSnapshot().lastVisionHeadingErrorDeg;
    }

    public double getFusionVisionRangeInches() {
        return poseFusion.getStateSnapshot().lastVisionRangeInches;
    }

    public double getFusionVisionDecisionMargin() {
        return poseFusion.getStateSnapshot().lastVisionDecisionMargin;
    }

    public double getFusionLastVisionAgeMs() {
        return poseFusion.getStateSnapshot().ageOfLastVisionMs;
    }

    public double getFusionOdometryDtMs() {
        return poseFusion.getStateSnapshot().lastOdometryDtMs;
    }
}
