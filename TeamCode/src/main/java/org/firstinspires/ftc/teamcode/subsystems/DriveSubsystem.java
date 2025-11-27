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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseFrames;
import org.firstinspires.ftc.teamcode.util.PoseFusion;
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
        public double normalMultiplier = 0.7;
        /** Max power multiplier for slow mode teleop driving (0.0-1.0) */
        public double slowMultiplier = 0.2;
        /** Turn multiplier for normal mode */
        public double normalTurnMultiplier = 0.4;
        /** Turn multiplier for slow mode */
        public double slowTurnMultiplier = 0.2;
        /** Rotation override threshold for aim assist */
        public double rotationOverrideThreshold = 0.05;
    }

    @Configurable
    public static class AimAssistConfig {
        /** Proportional gain for geometry-based aiming - higher = faster response to error */
        public double kP = .5;
        /** Max turn speed when aiming (0.0-1.0) */
        public double kMaxTurn = 0.7;

        public double MT2DistanceThreshold = 12;
    }

    @Configurable
    public static class VisionCenteredAimConfig {
        /** Proportional gain for vision-centered aiming (motor power per radian) */
        public double kP = 1.7;  // Equivalent to 0.03 per degree
        /** Max turn speed when aiming (0.0-1.0) */
        public double kMaxTurn = 0.7;
        /** Deadband - stop turning when tx error is below this (degrees) */
        public double deadbandDeg = 1.0;
    }

    @Configurable
    public static class FixedAngleAimConfig {
        /** Fixed target heading for blue alliance (degrees, 0=forward, 90=left) */
        public double blueHeadingDeg = 133.9;
        /** Fixed target heading for red alliance (degrees, 0=forward, 90=left) */
        public double redHeadingDeg = 45.2;
        /** Proportional gain for fixed-angle aiming */
        public double kP = 0.5;
        /** Max turn speed when aiming (0.0-1.0) */
        public double kMaxTurn = 0.7;
    }

    @Configurable
    public static class InitialPoseConfig {
        /** Starting X position in Pedro coordinates (inches) */
        public double startX = 56;
        /** Starting Y position in Pedro coordinates (inches) */
        public double startY = 8;
        /** Starting heading in degrees */
        public double startHeadingDeg = 90;
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
    public static RampConfig rampConfig = new RampConfig();

    // Robot-specific configs - visible in Panels for tuning
    public static FixedAngleAimConfig fixedAngleAimConfig19429 = createFixedAngleAimConfig19429();
    public static FixedAngleAimConfig fixedAngleAimConfig20245 = createFixedAngleAimConfig20245();
    public static InitialPoseConfig initialPoseConfig19429 = createInitialPoseConfig19429();
    public static InitialPoseConfig initialPoseConfig20245 = createInitialPoseConfig20245();

    /**
     * Helper to create 19429-specific fixed angle aim configuration.
     */
    private static FixedAngleAimConfig createFixedAngleAimConfig19429() {
        FixedAngleAimConfig config = new FixedAngleAimConfig();
        config.blueHeadingDeg = 133.9;
        config.redHeadingDeg = 45.2;
        config.kP = 0.7;
        config.kMaxTurn = 0.7;
        return config;
    }

    /**
     * Helper to create 20245-specific fixed angle aim configuration.
     */
    private static FixedAngleAimConfig createFixedAngleAimConfig20245() {
        FixedAngleAimConfig config = new FixedAngleAimConfig();
        config.blueHeadingDeg = 135.2;
        config.redHeadingDeg = 45.2;
        config.kP = 0.5;
        config.kMaxTurn = 0.7;
        return config;
    }

    /**
     * Helper to create 19429-specific initial pose configuration.
     */
    private static InitialPoseConfig createInitialPoseConfig19429() {
        InitialPoseConfig config = new InitialPoseConfig();
        config.startX = 56;
        config.startY = 9;
        config.startHeadingDeg = 90;
        return config;
    }

    /**
     * Helper to create 20245-specific initial pose configuration.
     */
    private static InitialPoseConfig createInitialPoseConfig20245() {
        InitialPoseConfig config = new InitialPoseConfig();
        config.startX = 0;
        config.startY = 0;
        config.startHeadingDeg = 90;
        return config;
    }

    /**
     * Gets the robot-specific FixedAngleAimConfig based on RobotState.getRobotName().
     * @return fixedAngleAimConfig19429 or fixedAngleAimConfig20245
     */
    public static FixedAngleAimConfig fixedAngleAimConfig() {
        return org.firstinspires.ftc.teamcode.util.RobotConfigs.getFixedAngleAimConfig();
    }

    /**
     * Gets the robot-specific InitialPoseConfig based on RobotState.getRobotName().
     * @return initialPoseConfig19429 or initialPoseConfig20245
     */
    public static InitialPoseConfig initialPoseConfig() {
        return org.firstinspires.ftc.teamcode.util.RobotConfigs.getInitialPoseConfig();
    }

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
    private boolean visionRelocalizationEnabled = true; // Enable auto-relocalize during aiming

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
        // Log which config set is being used for diagnostics
        RobotState.packet.put("_Config/Active Config Set", org.firstinspires.ftc.teamcode.util.RobotConfigs.getActiveConfigName());
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

    @Override
    public void initialize() {
        // Defensive check: ensure follower is attached before initialization
        if (follower == null) {
            throw new IllegalStateException(
                "DriveSubsystem: Follower not attached! Call attachFollower() before initialize(). " +
                "This typically happens in Robot.attachPedroFollower() during OpMode init.");
        }

        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        Pose pedroFollowerSeed = RobotState.takeHandoffPose();

        if (pedroFollowerSeed == null) {
            InitialPoseConfig poseConfig = initialPoseConfig();
            pedroFollowerSeed = new Pose(poseConfig.startX, poseConfig.startY, Math.toRadians(poseConfig.startHeadingDeg));

//            Optional<Pose> poseFromVision = vision.getRobotPoseFromTagPedro();
//            pedroFollowerSeed = poseFromVision.orElseGet(() -> new Pose(0, 0, Math.toRadians(90.0)));
        }
        Pose ftcSeed = PoseFrames.pedroToFtc(pedroFollowerSeed);
        RobotState.putPose("PoseDiag/FTC Coord Seed Pose",ftcSeed );
        Pose pedroSeed = PoseFrames.ftcToPedro(ftcSeed);
        RobotState.putPose("PoseDiag/Pedro Coord Seed Pose", pedroSeed);

        RobotState.putPose("PoseDiag/redonetoFTC", PoseFrames.pedroToFtc(pedroSeed));


        // Follower initialization
        follower.setStartingPose(pedroFollowerSeed);
        follower.setPose(pedroFollowerSeed);
        follower.update();
        follower.breakFollowing();

        fieldHeadingOffsetRad = DESIRED_FIELD_HEADING_RAD;
        visionHeadingCalibrated = false;

        poseFusion.reset(pedroFollowerSeed, System.currentTimeMillis());
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

        // MegaTag2: Update vision subsystem with current heading for IMU-fused localization
        // This must happen before vision.periodic() processes AprilTag detections
        if (vision != null) {
            double headingRad = follower.getHeading();
            vision.setRobotHeading(headingRad);
        }

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
        driveScaled(fieldX , fieldY , rotationInput , slowMode , rampMode);
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
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = vision.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

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
     * Automatically relocalizes when seeing basket AprilTag with good confidence (MT1/MT2 agreement).
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDrive(double fieldX , double fieldY , boolean slowMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = vision.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
//        double[] rotated = rotateFieldInput(fieldX , fieldY);
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Calculate target heading using pinpoint odometry + basket target coordinates
        Pose pose = follower.getPose();
        Pose targetPose = (alliance == Alliance.RED)
            ? FieldConstants.getRedBasketTarget()
            : FieldConstants.getBlueBasketTarget();
        double targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);

        double headingError = normalizeAngle(targetHeading - follower.getHeading());
        lastAimErrorRad = headingError;
        RobotState.packet.put("Aim Error", Math.toDegrees(headingError));


        // Simple P controller
        double maxTurn = Math.max(0.0, aimAssistConfig.kMaxTurn);
        double turn = Range.clip(aimAssistConfig.kP * headingError, -maxTurn, maxTurn);

        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);

        // Auto-relocalize when aiming at basket with reliable vision
        // Uses MT1/MT2 agreement as safety check (falls back to MT1 if heading is bad)
        if (vision.hasValidTag() && isBasketTag(vision.getCurrentTagId())) {
            maybeRelocalizeFromVision();
        }
    }

    /**
     * Vision-centered aiming: Uses Limelight tx (horizontal offset) to center the AprilTag.
     * This approach directly uses the camera's measurement without coordinate calculations.
     * Similar to the original FTC RobotAutoDriveToAprilTagOmni example.
     *
     * When tx = 0, the target is centered in the camera view.
     * Positive error means robot needs to turn counter-clockwise (left),
     * negative error means turn clockwise (right).
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDriveVisionCentered(double fieldX , double fieldY , boolean slowMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = vision.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
//        double[] rotated = rotateFieldInput(fieldX , fieldY);
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Get vision aiming error (robot-relative, in radians)
        Optional<Double> visionErrorOpt = vision.getVisionAimErrorRad();

        // If no valid tag, hold current heading (zero turn)
        if (!visionErrorOpt.isPresent()) {
            lastCommandForward = forward;
            lastCommandStrafeLeft = strafeLeft;
            lastCommandTurn = 0.0;
            lastRequestRotation = 0.0;
            lastAimErrorRad = Double.NaN;
            follower.setTeleOpDrive(forward, strafeLeft, 0.0, robotCentric);
            return;
        }

        double headingError = visionErrorOpt.get();

        // Deadband - stop turning if error is small enough
        double deadbandRad = Math.toRadians(visionCenteredAimConfig.deadbandDeg);
        if (Math.abs(headingError) < deadbandRad) {
            headingError = 0.0;
        }

        // Convert to turn command using P controller
        double maxTurn = Math.max(0.0, visionCenteredAimConfig.kMaxTurn);
        // kP units: (motor power / radian) - multiply error by gain
        double turn = Range.clip(visionCenteredAimConfig.kP * headingError, -maxTurn, maxTurn);

        lastAimErrorRad = headingError; // Store for telemetry
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
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = vision.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
//        double[] rotated = rotateFieldInput(fieldX , fieldY);
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Get fixed target heading based on alliance
        FixedAngleAimConfig aimConfig = fixedAngleAimConfig();
        double targetHeadingDeg = (alliance == Alliance.RED)
            ? aimConfig.redHeadingDeg
            : aimConfig.blueHeadingDeg;
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        // Calculate heading error
        double headingError = normalizeAngle(targetHeadingRad - follower.getHeading());
        lastAimErrorRad = headingError;

        // Simple P controller
        double maxTurn = Math.max(0.0, aimConfig.kMaxTurn);
        double turn = Range.clip(aimConfig.kP * headingError, -maxTurn, maxTurn);

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
            // Successful re-localization - update status for telemetry
            visionRelocalizeStatus = "Auto-relocated (aiming)";
            visionRelocalizeStatusMs = System.currentTimeMillis();
        }
    }

    /**
     * Helper to check if a tag ID is a basket AprilTag (safe for relocalization).
     */
    private boolean isBasketTag(int tagId) {
        return tagId == FieldConstants.BLUE_GOAL_TAG_ID || tagId == FieldConstants.RED_GOAL_TAG_ID;
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
        Optional<VisionSubsystemLimelight.TagSnapshot> snapOpt = vision.getLastSnapshot();
        if (!snapOpt.isPresent()) return false;

        VisionSubsystemLimelight.TagSnapshot snap = snapOpt.get();

        long ageMs = System.currentTimeMillis() - vision.getLastPoseTimestampMs();
        if (ageMs > 250) {
            // Stale data, do not relocalize
            return false;
        }
        Pose pedroPoseMT1 = snap.pedroPoseMT1; //only use MT1 for relocalization
        Pose pedroPoseMT2 = snap.pedroPoseMT2; //only use MT1 for relocalization

        // MT1 and MT2 positions are in meters in Pedro space
        double dx = pedroPoseMT1.getX() - pedroPoseMT2.getX();
        double dy = pedroPoseMT1.getY() - pedroPoseMT2.getY();

        // Euclidean distance between the two poses
        double distanceMeters = Math.hypot(dx, dy);
        boolean relocalizeWithMT2 = distanceMeters < aimAssistConfig.MT2DistanceThreshold;
        Pose pedroPose = pedroPoseMT1;

        if (relocalizeWithMT2) {
            pedroPose = pedroPoseMT2;
        }
        RobotState.packet.put("/vision/Poses/distanceBetweenMT1MT2", distanceMeters);
        RobotState.packet.put("/vision/Poses/relocalizeWithMT2", relocalizeWithMT2);
        RobotState.packet.put("/vision/Poses/RelocalizePedro",pedroPose);
        RobotState.putPose("/vision/Poses/Relocalize", PoseFrames.pedroToFtc(pedroPose));

        follower.setPose(pedroPose);
        poseFusion.reset(pedroPose, System.currentTimeMillis());
        vision.markOdometryUpdated();

        return true;
    }

    private void updatePoseFusion() {
        long timestampMs = System.currentTimeMillis();

        // 1. Always update with odometry
        poseFusion.updateWithOdometry(follower.getPose(), timestampMs);

        // 2. Get the latest LL tag snapshot
        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = vision.getLastSnapshot();
        if (!snapshotOpt.isPresent()) {
            return;
        }

        VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
        Pose visionPose = snapshot.getRobotPosePedroMT1().orElse(null);
        //  Pose visionPose = snapshot.getRobotPosePedroMT2().orElse(null);  // try this once we think mt2 is working

        if (visionPose == null) {
            return;
        }

        // 3. Only use fresh measurements
        long visionTimestamp = vision.getLastPoseTimestampMs();
        if (visionTimestamp <= lastFusionVisionTimestampMs) {
            return;
        }

        // 4. Use vision measurement
        poseFusion.addVisionMeasurement(
                visionPose,
                visionTimestamp,
                snapshot.getFtcRange(),
                snapshot.getDecisionMargin()
        );

        lastFusionVisionTimestampMs = visionTimestamp;
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
