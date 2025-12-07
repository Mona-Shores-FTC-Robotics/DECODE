package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.VisionCalibrationState.HEADING_KNOWN;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.VisionCalibrationState.HEADING_UNKNOWN;

import androidx.annotation.NonNull;

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
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveAimAssistConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveFixedAngleAimConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveInitialPoseConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveRampConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveRightTriggerFixedAngleConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveTeleOpConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveVisionCenteredAimConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveVisionRelocalizeConfig;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseFrames;
import org.firstinspires.ftc.teamcode.util.PoseFusion;
import org.firstinspires.ftc.teamcode.util.RobotState;
import java.util.Optional;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

public class DriveSubsystem implements Subsystem {

    private static final double VISION_TIMEOUT_MS = 100;
    public static final double STATIONARY_SPEED_THRESHOLD_IN_PER_SEC = 1.5;
    private static final double ODOMETRY_RELOCALIZE_DISTANCE_IN = 6.0;
    private static final long MAX_VISION_AGE_MS = 250L;
    private static final String LOG_TAG = "DriveSubsystem";

    enum VisionCalibrationState {
        HEADING_UNKNOWN,
        HEADING_KNOWN
    }
    private VisionCalibrationState visionState = HEADING_UNKNOWN;

    // Global configuration instances
    public static DriveTeleOpConfig teleOpDriveConfig = new DriveTeleOpConfig();
    public static DriveAimAssistConfig aimAssistConfig = new DriveAimAssistConfig();
    public static DriveVisionCenteredAimConfig visionCenteredAimConfig = new DriveVisionCenteredAimConfig();
    public static DriveRampConfig rampConfig = new DriveRampConfig();
    public static DriveVisionRelocalizeConfig visionRelocalizeConfig = new DriveVisionRelocalizeConfig();

    // Robot-specific configs - visible in Panels for tuning
    public static DriveFixedAngleAimConfig fixedAngleAimConfig19429 = createFixedAngleAimConfig19429();
    public static DriveFixedAngleAimConfig fixedAngleAimConfig20245 = createFixedAngleAimConfig20245();

    /**
     * Helper to create 19429-specific fixed angle aim configuration.
     */
    private static DriveFixedAngleAimConfig createFixedAngleAimConfig19429() {
        DriveFixedAngleAimConfig config = new DriveFixedAngleAimConfig();
        config.blueHeadingDeg = 135.2; // Match 20245 (closer to tuned value)
        config.redHeadingDeg = 45.2;
        config.kP = 0.5;               // Match 20245 for more predictable aim response
        config.kMaxTurn = 0.7;
        return config;
    }

    /**
     * Helper to create 20245-specific fixed angle aim configuration.
     */
    private static DriveFixedAngleAimConfig createFixedAngleAimConfig20245() {
        DriveFixedAngleAimConfig config = new DriveFixedAngleAimConfig();
        config.blueHeadingDeg = 135.2;
        config.redHeadingDeg = 45.2;
        config.kP = 0.5;
        config.kMaxTurn = 0.7;
        return config;
    }

    /**
     * Gets the robot-specific FixedAngleAimConfig based on RobotState.getRobotName().
     * @return fixedAngleAimConfig19429 or fixedAngleAimConfig20245
     */
    public static DriveFixedAngleAimConfig fixedAngleAimConfig() {
        return org.firstinspires.ftc.teamcode.util.RobotConfigs.getFixedAngleAimConfig();
    }

    // Robot-specific configs for right trigger fixed angle aiming - visible in Panels for tuning
    public static DriveRightTriggerFixedAngleConfig rightTriggerFixedAngleConfig19429 = createRightTriggerFixedAngleConfig19429();
    public static DriveRightTriggerFixedAngleConfig rightTriggerFixedAngleConfig20245 = createRightTriggerFixedAngleConfig20245();

    /**
     * Helper to create 19429-specific right trigger fixed angle aim configuration.
     */
    private static DriveRightTriggerFixedAngleConfig createRightTriggerFixedAngleConfig19429() {
        DriveRightTriggerFixedAngleConfig config = new DriveRightTriggerFixedAngleConfig();
        config.blueParkHeadingDeg = 265.0;
        config.redParkHeadingDeg = 275.0;
        config.kP = 0.5;
        config.kMaxTurn = 0.7;
        return config;
    }

    /**
     * Helper to create 20245-specific right trigger fixed angle aim configuration.
     */
    private static DriveRightTriggerFixedAngleConfig createRightTriggerFixedAngleConfig20245() {
        DriveRightTriggerFixedAngleConfig config = new DriveRightTriggerFixedAngleConfig();
        config.blueParkHeadingDeg = 265.0;
        config.redParkHeadingDeg = 275.0;
        config.kP = 0.5;
        config.kMaxTurn = 0.7;
        return config;
    }

    /**
     * Gets the robot-specific RightTriggerFixedAngleConfig based on RobotState.getRobotName().
     * @return rightTriggerFixedAngleConfig19429 or rightTriggerFixedAngleConfig20245
     */
    public static DriveRightTriggerFixedAngleConfig rightTriggerFixedAngleConfig() {
        return org.firstinspires.ftc.teamcode.util.RobotConfigs.getRightTriggerFixedAngleConfig();
    }

    /**
     * Gets the robot-specific InitialPoseConfig based on RobotState.getRobotName().
     * @return initialPoseConfig19429 or initialPoseConfig20245
     */
    public static DriveInitialPoseConfig initialPoseConfig = new DriveInitialPoseConfig();

    public String visionRelocalizeStatus = "Press A to re-localize";
    public long visionRelocalizeStatusMs = 0L;
    private String lastRelocalizeSource = "none";

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
    private LightingSubsystem lighting;
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
    private long lastAimUpdateNs = 0L;
    private double lastAimTurnCommand = 0.0;
    private boolean rampModeActive = false;
    private double rampForward = 0.0;
    private double rampStrafeLeft = 0.0;
    private double rampTurn = 0.0;
    private long lastRampUpdateNs = 0L;
    private boolean teleOpControlEnabled = false;
    private boolean visionRelocalizationEnabled = false;
    private long lastAutoRelocalizeMs = 0L;

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

    public void setLightingSubsystem(LightingSubsystem lighting) {
        this.lighting = lighting;
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
            DriveInitialPoseConfig poseConfig = initialPoseConfig;
            pedroFollowerSeed = new Pose(poseConfig.startX, poseConfig.startY, Math.toRadians(poseConfig.startHeadingDeg));

//            Optional<Pose> poseFromVision = vision.getRobotPoseFromTagPedro();
//            pedroFollowerSeed = poseFromVision.orElseGet(() -> new Pose(0, 0, Math.toRadians(90.0)));
        }
        Pose ftcSeed = PoseFrames.pedroToFtc(pedroFollowerSeed);
        RobotState.putPose("Pose/FTC Coord Seed Pose",ftcSeed );

        // Follower initialization
        follower.setStartingPose(pedroFollowerSeed);
        follower.setPose(pedroFollowerSeed);
        follower.update();
        follower.breakFollowing();

        fieldHeadingOffsetRad = DESIRED_FIELD_HEADING_RAD;
        visionHeadingCalibrated = false;

        poseFusion.reset(pedroFollowerSeed, System.currentTimeMillis());
        lastFusionVisionTimestampMs = vision.getLastPoseTimestampMs();
        visionRelocalizationEnabled = true;
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
        RobotState.packet.put("/vision/state", visionState.name());

        // Log robot pose for AdvantageScope field visualization (required for WPILOG replay)
        Pose pose = follower.getPose();
        if (pose != null) {
            // Convert inches to meters for AdvantageScope (SI units required)
            double xMeters = pose.getX() * 0.0254;
            double yMeters = pose.getY() * 0.0254;
            double headingRadians = follower.getHeading();
//            KoalaLog.logPose2d("Robot/Pose", xMeters, yMeters, headingRadians, true);
        }

        // Heading diagnostics for waggle investigation (follower target vs current)
        if (pose != null) {
            // Pedro follower doesn't expose target heading directly; approximate using current path heading
            double currentHeading = follower.getHeading();
            RobotState.packet.put("HeadingDiag/current_deg", Math.toDegrees(currentHeading));
            RobotState.packet.put("HeadingDiag/mode", follower.isBusy() ? "path_follow" : "idle/hold");
            RobotState.packet.put("HeadingDiag/speed_ips", getRobotSpeedInchesPerSecond());
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
     * Falls back to vision-based angle if available and relocalization is enabled.
     * This is an open-loop turn controller (kP + static). It does **not** use Pedro's heading PID
     * so you can see the raw turn command in telemetry when tuning.
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

        double headingErrorRad = normalizeAngle(targetHeading - follower.getHeading());
        double headingErrorDeg = Math.toDegrees(headingErrorRad);
        double absHeadingErrorDeg = Math.abs(headingErrorDeg);
        boolean withinDeadband = absHeadingErrorDeg <= aimAssistConfig.deadbandDeg;
        long now = System.nanoTime();
        double dt = lastAimUpdateNs == 0L ? 0.0 : (now - lastAimUpdateNs) / 1_000_000_000.0;

        // Apply a small deadband to keep the robot still when settled; telemetry matches what is commanded
        if (withinDeadband) {
            headingErrorRad = 0.0;
            headingErrorDeg = 0.0;
        }

        // PD controller with optional inner-zone P, static feedforward, and slew limit
        double pGain = absHeadingErrorDeg <= aimAssistConfig.innerZoneDeg ? aimAssistConfig.kPInner : aimAssistConfig.kP;
        double pTerm = pGain * headingErrorRad;
        double errorRateRadPerSec = (dt > 0.0 && !Double.isNaN(lastAimErrorRad)) ? (headingErrorRad - lastAimErrorRad) / dt : 0.0;
        double dTerm = aimAssistConfig.kD * errorRateRadPerSec;
        double turnRaw = pTerm + dTerm;

        boolean staticApplied = false;
        if (headingErrorRad != 0.0
                && Math.abs(turnRaw) < aimAssistConfig.kStatic
                && absHeadingErrorDeg > aimAssistConfig.staticApplyAboveDeg) {
            turnRaw = Math.copySign(aimAssistConfig.kStatic, headingErrorRad);
            staticApplied = true;
        }

        double maxTurn = Math.max(0.0, aimAssistConfig.kMaxTurn);
        double turnClipped = Range.clip(turnRaw, -maxTurn, maxTurn);

        double turn = turnClipped;
        if (aimAssistConfig.turnSlewRatePerSec > 0.0 && dt > 0.0) {
            double maxDelta = aimAssistConfig.turnSlewRatePerSec * dt;
            double delta = turnClipped - lastAimTurnCommand;
            if (delta > maxDelta) {
                turn = lastAimTurnCommand + maxDelta;
            } else if (delta < -maxDelta) {
                turn = lastAimTurnCommand - maxDelta;
            }
        }

        lastAimErrorRad = headingErrorRad;
        lastAimUpdateNs = now;
        lastAimTurnCommand = turn;
        double errorRateDegPerSec = Math.toDegrees(errorRateRadPerSec);

        RobotState.packet.put("Command/AimAndDrive/Aim Error (deg)", headingErrorDeg);
        RobotState.packet.put("Command/AimAndDrive/Aim Error Rate (deg/s)", errorRateDegPerSec);
        RobotState.packet.put("Command/AimAndDrive/Aim Target Heading (deg)", Math.toDegrees(targetHeading));
        RobotState.packet.put("Command/AimAndDrive/Aim Odo Heading (deg)", Math.toDegrees(follower.getHeading()));
        RobotState.packet.put("Command/AimAndDrive/P Gain", pGain);
        RobotState.packet.put("Command/AimAndDrive/P Term", pTerm);
        RobotState.packet.put("Command/AimAndDrive/D Term", dTerm);
        RobotState.packet.put("Command/AimAndDrive/Static Applied", staticApplied);
        RobotState.packet.put("Command/AimAndDrive/Turn Cmd Raw", turnRaw);
        RobotState.packet.put("Command/AimAndDrive/Turn Cmd (slewed)", turn);

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
        DriveFixedAngleAimConfig aimConfig = fixedAngleAimConfig();
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

    /**
     * Right trigger fixed-angle aiming: Rotates to a fixed heading based on alliance.
     * Uses different target angles than triangle button (265° blue / 275° red).
     * Driver retains full translation control while holding right trigger.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDriveRightTriggerFixedAngle(double fieldX , double fieldY , boolean slowMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = vision.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Get right trigger fixed target heading based on alliance
        DriveRightTriggerFixedAngleConfig aimConfig = rightTriggerFixedAngleConfig();
        double targetHeadingDeg = (alliance == Alliance.RED)
            ? aimConfig.redParkHeadingDeg
            : aimConfig.blueParkHeadingDeg;
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

    /**
     * Returns true if the robot is effectively aimed at the goal based on the active alliance and pose.
     * Uses odometry pose and a tight heading deadband.
     */
    public boolean isAimSettled(double headingDeadbandDeg) {
        Pose pose = follower.getPose();
        if (pose == null) {
            return false;
        }

        // Require low chassis speed
        if (getRobotSpeedInchesPerSecond() > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            return false;
        }

        Alliance alliance = vision.getAlliance();
        Pose targetPose = (alliance == Alliance.RED)
            ? FieldConstants.getRedBasketTarget()
            : FieldConstants.getBlueBasketTarget();
        double targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);
        double headingErrorRad = normalizeAngle(targetHeading - follower.getHeading());
        double headingErrorDeg = Math.toDegrees(Math.abs(headingErrorRad));
        return headingErrorDeg <= headingDeadbandDeg;
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

    private boolean isGoalAprilTag(int tagId) {
        return tagId == FieldConstants.BLUE_GOAL_TAG_ID || tagId == FieldConstants.RED_GOAL_TAG_ID;
    }

    private boolean isVisionPoseFresh() {
        long ageMs = System.currentTimeMillis() - vision.getLastPoseTimestampMs();
        return ageMs <= MAX_VISION_AGE_MS;
    }

    private double mt2DistanceThresholdInches() { return visionRelocalizeConfig.MT2DistanceThreshold; }

    private double headingErrorDegrees(Pose mt1 , Pose mt2) {
        double headingErrorRad = normalizeAngle(mt1.getHeading() - mt2.getHeading());
        return Math.abs(Math.toDegrees(headingErrorRad));
    }

    private boolean posesAgreeForMt2(Pose mt1 , Pose mt2) {
        double distanceInches = distanceBetween(mt1 , mt2);
        double headingErrorDeg = headingErrorDegrees(mt1 , mt2);

        return distanceInches < mt2DistanceThresholdInches()
                && headingErrorDeg < visionRelocalizeConfig.MT2HeadingThresholdDeg;
    }

    private void recordVisionAgreementTelemetry(double distance , double headingErrorDeg , boolean mt2Safe) {
        RobotState.packet.put("/vision/Poses/distanceBetweenMT1MT2", distance);
        RobotState.packet.put("/vision/Poses/relocalizeWithMT2", mt2Safe);
        RobotState.packet.put("/vision/Poses/headingErrorMT1MT2", headingErrorDeg);
        RobotState.packet.put("/vision/Poses/mt2DistanceThreshold", mt2DistanceThresholdInches());
        RobotState.packet.put("/vision/Poses/mt2HeadingThresholdDeg", visionRelocalizeConfig.MT2HeadingThresholdDeg);
    }

    private void recordVisionPoseTelemetry(Pose mt1 , Pose mt2) {
        double mt1HeadingDeg = mt1 == null ? Double.NaN : Math.toDegrees(mt1.getHeading());
        double mt2HeadingDeg = mt2 == null ? Double.NaN : Math.toDegrees(mt2.getHeading());

        RobotState.packet.put("/vision/Poses/mt1/x", mt1 == null ? Double.NaN : mt1.getX());
        RobotState.packet.put("/vision/Poses/mt1/y", mt1 == null ? Double.NaN : mt1.getY());
        RobotState.packet.put("/vision/Poses/mt1/heading_deg", mt1HeadingDeg);

        RobotState.packet.put("/vision/Poses/mt2/x", mt2 == null ? Double.NaN : mt2.getX());
        RobotState.packet.put("/vision/Poses/mt2/y", mt2 == null ? Double.NaN : mt2.getY());
        RobotState.packet.put("/vision/Poses/mt2/heading_deg", mt2HeadingDeg);
    }

    private void recordRelocalizeSource(String source , boolean success , long timestampMs , int tagId) {
        lastRelocalizeSource = source;
        RobotState.packet.put("/vision/relocalize/last/source", source);
        RobotState.packet.put("/vision/relocalize/last/success", success);
        RobotState.packet.put("/vision/relocalize/last/timestamp_ms", timestampMs);
        RobotState.packet.put("/vision/relocalize/last/tag", tagId);
    }

    private void recordAutoRelocalizePacket(boolean success , long timestampMs , int tagId , String status , String source) {
        RobotState.packet.put("/vision/relocalize/auto/success", success);
        RobotState.packet.put("/vision/relocalize/auto/timestamp_ms", timestampMs);
        RobotState.packet.put("/vision/relocalize/auto/tag", tagId);
        RobotState.packet.put("/vision/relocalize/auto/status", status);
        RobotState.packet.put("/vision/relocalize/auto/source", source);
    }

    private boolean poseWithinField(Pose pose) {
        if (pose == null) {
            return false;
        }
        double x = pose.getX();
        double y = pose.getY();
        double min = -6.0; // allow small margin
        double max = FieldConstants.FIELD_WIDTH_INCHES + 6.0;
        return x >= min && x <= max && y >= min && y <= max;
    }

    private boolean headingFacesTag(Pose pose, int tagId, double maxHeadingErrorDeg) {
        if (pose == null) {
            return false;
        }
        Pose tagPosePedro;
        if (tagId == FieldConstants.BLUE_GOAL_TAG_ID) {
            tagPosePedro = FieldConstants.BLUE_GOAL_TAG_APRILTAG; // Pedro frame already
        } else if (tagId == FieldConstants.RED_GOAL_TAG_ID) {
            tagPosePedro = FieldConstants.RED_GOAL_TAG_APRILTAG; // Pedro frame already
        } else {
            return false;
        }
        double bearing = Math.atan2(tagPosePedro.getY() - pose.getY(), tagPosePedro.getX() - pose.getX());
        double headingErr = normalizeAngle(pose.getHeading() - bearing);
        double headingErrDeg = Math.toDegrees(Math.abs(headingErr));
        return headingErrDeg <= maxHeadingErrorDeg;
    }

    private void flashRelocalizeFeedback() {
        if (lighting != null) {
            lighting.flashAimAligned();
        }
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
     * Attempts a single relocalization tied to a launch event.
     * Gated to moments where the robot should be steady and facing the goal tag.
     */
    public void tryRelocalizeForShot() {
        if (!visionRelocalizationEnabled) {
            recordAutoRelocalizePacket(false , System.currentTimeMillis() , -1 , "launch relocalize failed" , "disabled");
            return;
        }
        if (vision == null || !vision.hasValidTag()) {
            recordAutoRelocalizePacket(false , System.currentTimeMillis() , -1 , "launch relocalize failed" , "no_tag");
            return;
        }

        long nowMs = System.currentTimeMillis();
        if (nowMs - lastAutoRelocalizeMs < 300) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "cooldown");
            return; // simple cooldown
        }

        // Require low chassis speed and minimal commanded turn to avoid relocalizing while spinning
        if (getRobotSpeedInchesPerSecond() > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "too_fast");
            return;
        }
        if (Math.abs(lastCommandTurn) > 0.05) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "turning");
            return;
        }

        long ageMs = nowMs - vision.getLastPoseTimestampMs();
        if (ageMs > MAX_VISION_AGE_MS) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "stale_pose");
            return;
        }

        Optional<VisionSubsystemLimelight.TagSnapshot> snapOpt = vision.getLastSnapshot();
        if (!snapOpt.isPresent()) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "no_snapshot");
            return;
        }

        VisionSubsystemLimelight.TagSnapshot snapshot = snapOpt.get();
        if (!isGoalAprilTag(snapshot.tagId)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "not_goal_tag");
            return;
        }

        Pose pedroPoseMT1 = snapshot.pedroPoseMT1;
        Pose pedroPoseMT2 = snapshot.pedroPoseMT2;
        if (pedroPoseMT1 == null && pedroPoseMT2 == null) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "no_pose");
            return;
        }

        // Geometry-based sanity checks (field bounds, facing tag, distance)
        Pose primaryPose = pedroPoseMT2 != null ? pedroPoseMT2 : pedroPoseMT1;
        if (!poseWithinField(primaryPose)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "off_field");
            return;
        }
        if (!headingFacesTag(primaryPose, snapshot.tagId, 120.0)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "heading_not_facing_tag");
            return;
        }
        double distToTag = Double.NaN;
        Pose tagPosePedro = snapshot.tagId == FieldConstants.BLUE_GOAL_TAG_ID
                ? FieldConstants.BLUE_GOAL_TAG_APRILTAG
                : FieldConstants.RED_GOAL_TAG_APRILTAG;
        if (tagPosePedro != null && primaryPose != null) {
            distToTag = distanceBetween(primaryPose, tagPosePedro);
        }
        if (!Double.isNaN(distToTag) && (distToTag < 6.0 || distToTag > 200.0)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "distance_out_of_range");
            return;
        }

        boolean mt2Safe = pedroPoseMT1 != null && pedroPoseMT2 != null && posesAgreeForMt2(pedroPoseMT1, pedroPoseMT2);
        if (!mt2Safe && pedroPoseMT1 != null && pedroPoseMT2 != null) {
            recordVisionAgreementTelemetry(distanceBetween(pedroPoseMT1, pedroPoseMT2), headingErrorDegrees(pedroPoseMT1, pedroPoseMT2), mt2Safe);
        }

        if (forceRelocalizeFromVision()) {
            lastAutoRelocalizeMs = nowMs;
            visionRelocalizeStatus = "Re-localized on launch";
            visionRelocalizeStatusMs = nowMs;
            recordAutoRelocalizePacket(true , nowMs , snapshot.tagId , visionRelocalizeStatus , lastRelocalizeSource);
//            flashRelocalizeFeedback();
        }
        else {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "blocked_conditions");
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

    /**
     * Re-centers odometry heading to the configured field-forward angle.
     * Driver should square the robot to the field wall (start-heading direction)
     * before triggering this for a quick recovery after a bad auto handoff.
     */
    public void resetHeadingToFieldForward() {
        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return;
        }

        double targetHeading = Math.toRadians(initialPoseConfig.startHeadingDeg);
        Pose correctedPose = new Pose(currentPose.getX() , currentPose.getY() , targetHeading);

        follower.setPose(correctedPose);
        poseFusion.reset(correctedPose , System.currentTimeMillis());
        vision.markOdometryUpdated();

        visionRelocalizeStatus = "Heading reset to field-forward (square to wall)";
        visionRelocalizeStatusMs = System.currentTimeMillis();
    }
    public boolean forceRelocalizeFromVision() {
        Optional<VisionSubsystemLimelight.TagSnapshot> snapOpt = vision.getLastSnapshot();
        long nowMs = System.currentTimeMillis();
        if (!snapOpt.isPresent()) {
            recordRelocalizeSource("none", false, nowMs, -1);
            return false;
        }

        VisionSubsystemLimelight.TagSnapshot snap = snapOpt.get();

        // 1. Use MT1 whenever heading is unknown
        Pose pedroPoseMT1 = snap.pedroPoseMT1;
        Pose pedroPoseMT2 = snap.pedroPoseMT2;

        if (visionState == HEADING_UNKNOWN) {
            if (pedroPoseMT1 == null) {
                recordRelocalizeSource("mt1_missing", false, nowMs, snap.tagId);
                return false;
            }

            follower.setPose(pedroPoseMT1);
            poseFusion.reset(pedroPoseMT1, System.currentTimeMillis());
            vision.markOdometryUpdated();

            // Now heading is calibrated
            visionState = HEADING_KNOWN;
            recordRelocalizeSource("mt1_seed", true, nowMs, snap.tagId);
            return true;
        }

        // 2. If heading is known, prefer MT2 when it agrees with MT1 (or MT1 missing)
        boolean mt2Safe = pedroPoseMT1 != null && pedroPoseMT2 != null && posesAgreeForMt2(pedroPoseMT1, pedroPoseMT2);
        if (pedroPoseMT2 != null && (mt2Safe || pedroPoseMT1 == null)) {
            follower.setPose(pedroPoseMT2);
            poseFusion.reset(pedroPoseMT2, System.currentTimeMillis());
            vision.markOdometryUpdated();
            recordRelocalizeSource("mt2", true, nowMs, snap.tagId);
            return true;
        }

        // 3. Fallback to MT1 if MT2 unavailable or unsafe
        if (pedroPoseMT1 != null) {
            follower.setPose(pedroPoseMT1);
            poseFusion.reset(pedroPoseMT1, System.currentTimeMillis());
            vision.markOdometryUpdated();
            recordRelocalizeSource("mt1", true, nowMs, snap.tagId);
            return true;
        }

        recordRelocalizeSource("none", false, nowMs, snap.tagId);
        return false;
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
