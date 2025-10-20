package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.Tuning;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Wraps the PedroPathing {@link Follower} to provide field-centric driving for TeleOp.
 * The subsystem owns the follower lifecycle so OpModes can simply call {@link #driveFieldCentric}.
 */
public class DriveSubsystem implements Subsystem {

    public enum DriveMode {
        NORMAL,
        PRECISION,
        VELOCITY,
        VELOCITY_PRECISION,
        SLOW // legacy alias for slow mode support
    }

    private static final double ROTATION_DEADBAND = 0.03;

    private final Follower follower;
    private final Constants.Motors velocityMotors;
    private final DcMotorEx motorLf;
    private final DcMotorEx motorRf;
    private final DcMotorEx motorLb;
    private final DcMotorEx motorRb;

    public double slowMultiplier = 0.20; // legacy compatibility for older TeleOp

    private boolean poseIsMeters = false;
    private Pose teleopSeedPose = null;
    private DriveMode defaultMode = DriveMode.NORMAL;
    private boolean isRobotCentric = false;
    private boolean autoHeadingEnabled = true;

    private double headingHoldTarget = 0.0;
    private boolean headingHoldActive = false;

    private double lastForwardCommand = 0.0;
    private double lastStrafeCommand = 0.0;
    private double lastTurnCommand = 0.0;
    private DriveMode activeMode = DriveMode.NORMAL;
    private boolean velocityControlActive = false;
    private double lastVxCommand = 0.0;
    private double lastVyCommand = 0.0;
    private double lastOmegaCommand = 0.0;
    private long lastVelocityTimestampNanos = System.nanoTime();
    private double lastTargetForwardMps = 0.0;
    private double lastTargetStrafeMps = 0.0;
    private double lastMeasuredForwardMps = 0.0;
    private double lastMeasuredStrafeMps = 0.0;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
        this.velocityMotors = new Constants.Motors(hardwareMap);
        this.velocityMotors.setRunUsingEncoder();

        this.motorLf = velocityMotors.lf;
        this.motorRf = velocityMotors.rf;
        this.motorLb = velocityMotors.lb;
        this.motorRb = velocityMotors.rb;
    }

    public void setTeleopSeedPose(Pose pose) {
        this.teleopSeedPose = pose;
    }

    public void setPoseIsMeters(boolean poseIsMeters) {
        this.poseIsMeters = poseIsMeters;
    }

    public boolean isPoseMeters() {
        return poseIsMeters;
    }

    public void setDefaultMode(DriveMode mode) {
        this.defaultMode = mode;
        this.activeMode = mode;
    }

    /**
     * Enables or disables the automatic heading hold behaviour that keeps the robot
     * pointed in its last direction of travel when the driver releases the rotation
     * stick. Disable this for fully manual rotation control.
     */
    public void setAutoHeadingEnabled(boolean enabled) {
        autoHeadingEnabled = enabled;
        if (!enabled) {
            headingHoldActive = false;
        } else {
            headingHoldTarget = follower.getHeading();
        }
    }

    /**
     * Historical API used by earlier OpModes. Internally the Pedro follower expects
     * {@code true} for field-centric mode, matching the behaviour of the legacy code.
     */
    public void setRobotCentric(boolean value) {
        this.isRobotCentric = value;
    }

    public boolean isRobotCentric() {
        return isRobotCentric;
    }

    @Override
    public void initialize() {
        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        Pose seed = RobotState.takeHandoffPose();
        if (seed == null) {
            seed = teleopSeedPose != null ? teleopSeedPose : new Pose();
        }

        follower.setStartingPose(seed);
        follower.update();
        follower.startTeleopDrive();

        headingHoldTarget = follower.getHeading();
        headingHoldActive = false;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void stop() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
        velocityMotors.stop();
        lastVxCommand = 0.0;
        lastVyCommand = 0.0;
        lastOmegaCommand = 0.0;
        velocityControlActive = false;
        lastTargetForwardMps = 0.0;
        lastTargetStrafeMps = 0.0;
        lastMeasuredForwardMps = 0.0;
        lastMeasuredStrafeMps = 0.0;
    }

    /** Legacy alias for older OpModes. */
    public void shutdown() {
        stop();
    }

    /**
     * Drive the follower using field-centric inputs.
     *
     * @param fieldX         translation along the field X axis (+right from driver perspective)
     * @param fieldY         translation along the field Y axis (+forward away from driver wall)
     * @param rotationInput  rotation command with positive meaning counter-clockwise
     * @param precisionMode  when true applies {@link Tuning#SLOW_MULTIPLIER}
     */
    public void driveFieldCentric(double fieldX, double fieldY, double rotationInput, boolean precisionMode) {
        double multiplier = precisionMode ? Tuning.SLOW_MULTIPLIER : 1.0;
        DriveMode mode = precisionMode ? DriveMode.PRECISION : DriveMode.NORMAL;
        driveFieldCentricScaled(fieldX, fieldY, rotationInput, multiplier, mode);
    }

    private void driveFieldCentricScaled(double fieldX, double fieldY, double rotationInput,
                                         double multiplier, DriveMode mode) {
        velocityControlActive = false;
        double forward = Range.clip(fieldY * multiplier, -1.0, 1.0);
        double strafeLeft = Range.clip(-fieldX * multiplier, -1.0, 1.0);
        double turnCW = Range.clip(-computeRotationCommand(rotationInput) * multiplier, -1.0, 1.0);

        follower.setTeleOpDrive(forward, strafeLeft, turnCW, isRobotCentric);

        lastForwardCommand = forward;
        lastStrafeCommand = strafeLeft;
        lastTurnCommand = turnCW;
        activeMode = mode;
        lastVxCommand = 0.0;
        lastVyCommand = 0.0;
        lastOmegaCommand = 0.0;
        lastTargetForwardMps = 0.0;
        lastTargetStrafeMps = 0.0;
        lastMeasuredForwardMps = 0.0;
        lastMeasuredStrafeMps = 0.0;
    }

    /**
     * Minimal field-centric velocity control that maps joystick inputs straight to motor velocity.
     * No slew limiting or odometry feedback is applied so it is easier to debug drivetrain issues.
     */
    public void driveFieldCentricVelocitySimple(double fieldX, double fieldY,
                                                double rotationInput, boolean precisionMode) {
        double multiplier = precisionMode ? Tuning.SLOW_MULTIPLIER : 1.0;
        DriveMode mode = precisionMode ? DriveMode.VELOCITY_PRECISION : DriveMode.VELOCITY;

        double fieldForward = Range.clip(fieldY, -1.0, 1.0);
        double fieldLeft = Range.clip(-fieldX, -1.0, 1.0);

        double heading = follower.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward;
        double robotLeft;
        if (isRobotCentric) {
            robotForward = fieldForward;
            robotLeft = fieldLeft;
        } else {
            robotForward = fieldForward * cos + fieldLeft * sin;
            robotLeft = -fieldForward * sin + fieldLeft * cos;
        }

        double maxLinearMps = Math.min(DistanceUnit.INCH.toMeters(Tuning.MAX_LINEAR_SPEED_IPS), Constants.Speed.MAX_VEL_MPS);
        double maxAngularRad = Math.min(Math.toRadians(Tuning.MAX_ANGULAR_SPEED_DEG_PER_SEC), Constants.Speed.MAX_ANG_VEL_RADPS);

        double vxCmd = robotForward * maxLinearMps * multiplier;
        double vyCmd = robotLeft * maxLinearMps * multiplier;
        double omegaCmd = rotationInput * maxAngularRad * multiplier;

        Constants.MecanumIK.WheelSpeeds speeds = Constants.MecanumIK.wheelSpeedsFromChassis(vxCmd, vyCmd, omegaCmd);
        Constants.MecanumIK.TicksPerSec ticks = speeds.toTicksPerSec();
        velocityMotors.setAllVelocityTps(ticks.tFL, ticks.tFR, ticks.tBL, ticks.tBR);

        lastForwardCommand = Range.clip(robotForward * multiplier, -1.0, 1.0);
        lastStrafeCommand = Range.clip(robotLeft * multiplier, -1.0, 1.0);
        lastTurnCommand = Range.clip(-rotationInput * multiplier, -1.0, 1.0);
        lastTargetForwardMps = vxCmd;
        lastTargetStrafeMps = vyCmd;
        lastMeasuredForwardMps = vxCmd;
        lastMeasuredStrafeMps = vyCmd;
        velocityControlActive = true;
        lastVxCommand = vxCmd;
        lastVyCommand = vyCmd;
        lastOmegaCommand = omegaCmd;
        lastVelocityTimestampNanos = System.nanoTime();
        activeMode = mode;
    }

    /**
     * Field-centric drive that targets wheel velocity instead of open-loop motor power.
     *
     * @param fieldX        translation along the field X axis (+right from driver perspective)
     * @param fieldY        translation along the field Y axis (+forward away from driver wall)
     * @param rotationInput rotation command with positive meaning counter-clockwise
     * @param precisionMode when true applies {@link Tuning#SLOW_MULTIPLIER}
     */
    public void driveFieldCentricVelocity(double fieldX, double fieldY,
                                          double rotationInput, boolean precisionMode) {
        double multiplier = precisionMode ? Tuning.SLOW_MULTIPLIER : 1.0;
        DriveMode mode = precisionMode ? DriveMode.VELOCITY_PRECISION : DriveMode.VELOCITY;

        double fieldForward = Range.clip(fieldY, -1.0, 1.0);
        double fieldLeft = Range.clip(-fieldX, -1.0, 1.0);

        double robotForward;
        double robotLeft;
        if (isRobotCentric) {
            robotForward = fieldForward;
            robotLeft = fieldLeft;
        } else {
            double heading = follower.getHeading();
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            robotForward = fieldForward * cos + fieldLeft * sin;
            robotLeft = -fieldForward * sin + fieldLeft * cos;
        }

        double magnitude = Math.hypot(robotForward, robotLeft);
        if (magnitude > 1.0) {
            robotForward /= magnitude;
            robotLeft /= magnitude;
        }

        robotForward = Range.clip(robotForward, -1.0, 1.0);
        robotLeft = Range.clip(robotLeft, -1.0, 1.0);

        double maxLinearMps = DistanceUnit.INCH.toMeters(Tuning.MAX_LINEAR_SPEED_IPS);
        maxLinearMps = Math.min(maxLinearMps, Constants.Speed.MAX_VEL_MPS);
        double maxAngularRad = Math.toRadians(Tuning.MAX_ANGULAR_SPEED_DEG_PER_SEC);
        maxAngularRad = Math.min(maxAngularRad, Constants.Speed.MAX_ANG_VEL_RADPS);

        double rotationCommand = computeRotationCommand(rotationInput);
        double targetVx = robotForward * maxLinearMps * multiplier;
        double targetVy = robotLeft * maxLinearMps * multiplier;
        double targetOmega = rotationCommand * maxAngularRad * multiplier;

        long now = System.nanoTime();
        double dtSeconds = (now - lastVelocityTimestampNanos) / 1e9;
        if (velocityControlActive) {
            dtSeconds = Range.clip(dtSeconds, 1e-3, 0.05);
        } else {
            dtSeconds = 0.02; // assume ~20ms loop on first entry to honour ramp settings
            follower.setTeleOpDrive(0, 0, 0, isRobotCentric);
            velocityMotors.setRunUsingEncoder();
            lastVxCommand = 0.0;
            lastVyCommand = 0.0;
            lastOmegaCommand = 0.0;
        }

        double linearRampTime = Math.max(Tuning.LINEAR_RAMP_TIME_SEC, 0.0);
        double angularRampTime = Math.max(Tuning.ANGULAR_RAMP_TIME_SEC, 0.0);
        double linearRate = linearRampTime <= 0.0 ? Double.POSITIVE_INFINITY : maxLinearMps / linearRampTime;
        double angularRate = angularRampTime <= 0.0 ? Double.POSITIVE_INFINITY : maxAngularRad / angularRampTime;

        double vxCmd = applySlew(targetVx, lastVxCommand, linearRate, dtSeconds);
        double vyCmd = applySlew(targetVy, lastVyCommand, linearRate, dtSeconds);
        double omegaCmd = applySlew(targetOmega, lastOmegaCommand, angularRate, dtSeconds);

        Constants.MecanumIK.WheelSpeeds speeds = Constants.MecanumIK.wheelSpeedsFromChassis(vxCmd, vyCmd, omegaCmd);
        Constants.MecanumIK.TicksPerSec ticks = speeds.toTicksPerSec();
        velocityMotors.setAllVelocityTps(ticks.tFL, ticks.tFR, ticks.tBL, ticks.tBR);

        lastVxCommand = vxCmd;
        lastVyCommand = vyCmd;
        lastOmegaCommand = omegaCmd;
        lastTargetForwardMps = vxCmd;
        lastTargetStrafeMps = vyCmd;
        Pose velocity = follower.poseTracker.getLocalizer().getVelocity();
        double vxField = 0.0;
        double vyField = 0.0;
        if (velocity != null) {
            vxField = velocity.getX();
            vyField = velocity.getY();
        }
        if (!poseIsMeters) {
            vxField = DistanceUnit.INCH.toMeters(vxField);
            vyField = DistanceUnit.INCH.toMeters(vyField);
        }
        if (isRobotCentric) {
            lastMeasuredForwardMps = vxField;
            lastMeasuredStrafeMps = vyField;
        } else {
            double cos = Math.cos(follower.getHeading());
            double sin = Math.sin(follower.getHeading());
            lastMeasuredForwardMps = vxField * cos + vyField * sin;
            lastMeasuredStrafeMps = -vxField * sin + vyField * cos;
        }
        lastVelocityTimestampNanos = now;
        velocityControlActive = true;

        lastForwardCommand = Range.clip(robotForward * multiplier, -1.0, 1.0);
        lastStrafeCommand = Range.clip(robotLeft * multiplier, -1.0, 1.0);
        lastTurnCommand = Range.clip(-rotationCommand * multiplier, -1.0, 1.0);
        activeMode = mode;
    }

    /**
     * Field-centric drive that regulates chassis velocity based on Pinpoint odometry feedback.
     * This path is useful when motor encoders are unavailable or unreliable.
     */
    public void driveFieldCentricVelocityPinpoint(double fieldX, double fieldY,
                                                  double rotationInput, boolean precisionMode) {
        double multiplier = precisionMode ? Tuning.SLOW_MULTIPLIER : 1.0;
        DriveMode mode = precisionMode ? DriveMode.VELOCITY_PRECISION : DriveMode.VELOCITY;

        double fieldForward = Range.clip(fieldY, -1.0, 1.0);
        double fieldLeft = Range.clip(-fieldX, -1.0, 1.0);

        double heading = follower.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward;
        double robotLeft;
        if (isRobotCentric) {
            robotForward = fieldForward;
            robotLeft = fieldLeft;
        } else {
            robotForward = fieldForward * cos + fieldLeft * sin;
            robotLeft = -fieldForward * sin + fieldLeft * cos;
        }

        double magnitude = Math.hypot(robotForward, robotLeft);
        if (magnitude > 1.0) {
            robotForward /= magnitude;
            robotLeft /= magnitude;
        }

        double rotationCommand = computeRotationCommand(rotationInput);
        double maxLinearMps = Math.min(DistanceUnit.INCH.toMeters(Tuning.MAX_LINEAR_SPEED_IPS), Constants.Speed.MAX_VEL_MPS);

        long now = System.nanoTime();
        double dtSeconds = (now - lastVelocityTimestampNanos) / 1e9;
        if (velocityControlActive) {
            dtSeconds = Range.clip(dtSeconds, 1e-3, 0.05);
        } else {
            dtSeconds = 0.02;
        }

        double linearRampTime = Math.max(Tuning.LINEAR_RAMP_TIME_SEC, 0.0);
        double linearRate = linearRampTime <= 0.0 ? Double.POSITIVE_INFINITY : maxLinearMps / linearRampTime;

        double forwardBase = Range.clip(robotForward * multiplier, -1.0, 1.0);
        double strafeBase = Range.clip(robotLeft * multiplier, -1.0, 1.0);
        double turnBase = Range.clip(-rotationCommand * multiplier, -1.0, 1.0);

        double targetVx = applySlew(forwardBase * maxLinearMps, lastVxCommand, linearRate, dtSeconds);
        double targetVy = applySlew(strafeBase * maxLinearMps, lastVyCommand, linearRate, dtSeconds);

        Pose velocity = follower.poseTracker.getLocalizer().getVelocity();
        double vxField = 0.0;
        double vyField = 0.0;
        if (velocity != null) {
            vxField = velocity.getX();
            vyField = velocity.getY();
        }
        if (!poseIsMeters) {
            vxField = DistanceUnit.INCH.toMeters(vxField);
            vyField = DistanceUnit.INCH.toMeters(vyField);
        }

        double measuredForward;
        double measuredLeft;
        if (isRobotCentric) {
            measuredForward = vxField;
            measuredLeft = vyField;
        } else {
            measuredForward = vxField * cos + vyField * sin;
            measuredLeft = -vxField * sin + vyField * cos;
        }

        double forwardErrorNorm = maxLinearMps > 0 ? (targetVx - measuredForward) / maxLinearMps : 0.0;
        double strafeErrorNorm = maxLinearMps > 0 ? (targetVy - measuredLeft) / maxLinearMps : 0.0;

        double forwardPower = Range.clip(forwardBase + Tuning.PINPOINT_LINEAR_KP * forwardErrorNorm, -1.0, 1.0);
        double strafePower = Range.clip(strafeBase + Tuning.PINPOINT_LINEAR_KP * strafeErrorNorm, -1.0, 1.0);
        double turnCwPower = turnBase;

        lastTargetForwardMps = targetVx;
        lastTargetStrafeMps = targetVy;
        lastMeasuredForwardMps = measuredForward;
        lastMeasuredStrafeMps = measuredLeft;

        follower.setTeleOpDrive(forwardPower, strafePower, turnCwPower, isRobotCentric);

        lastForwardCommand = forwardPower;
        lastStrafeCommand = strafePower;
        lastTurnCommand = turnCwPower;
        activeMode = mode;
        velocityControlActive = true;

        lastVxCommand = targetVx;
        lastVyCommand = targetVy;
        lastOmegaCommand = 0.0;
        lastVelocityTimestampNanos = now;
    }

    private double computeRotationCommand(double rotationInput) {
        if (!autoHeadingEnabled) {
            headingHoldActive = false;
            headingHoldTarget = follower.getHeading();
            return Range.clip(rotationInput, -1.0, 1.0);
        }

        if (Math.abs(rotationInput) > ROTATION_DEADBAND) {
            headingHoldTarget = follower.getHeading();
            headingHoldActive = false;
            return rotationInput;
        }

        if (!headingHoldActive) {
            headingHoldTarget = follower.getHeading();
            headingHoldActive = true;
        }

        double error = AngleUnit.normalizeRadians(headingHoldTarget - follower.getHeading());
        return Range.clip(error * Tuning.AUTO_HEADING_KP, -1.0, 1.0);
    }

    /**
     * Compatibility helper for older OpModes that still call the legacy API.
     */
    public void driveWithModeHolds(double lx, double ly, double rx,
                                   boolean rightBumperHeld, boolean leftBumperHeld) {
        DriveMode requested = defaultMode;
        if (rightBumperHeld) {
            requested = DriveMode.SLOW;
        } else if (leftBumperHeld) {
            requested = DriveMode.NORMAL;
        }

        double multiplier = requested == DriveMode.SLOW ? slowMultiplier : 1.0;
        driveFieldCentricScaled(-lx, ly, -rx, multiplier,
                requested == DriveMode.SLOW ? DriveMode.SLOW : requested);
    }

    public void driveSimple(double lx, double ly, double rx) {
        driveFieldCentricScaled(-lx, ly, -rx, 1.0, DriveMode.NORMAL);
    }

    public DriveMode getDriveMode() {
        return activeMode;
    }

    public Pose2D getPose() {
        Pose pose = follower.getPose();
        DistanceUnit unit = poseIsMeters ? DistanceUnit.METER : DistanceUnit.INCH;
        return new Pose2D(unit, pose.getX(), pose.getY(), AngleUnit.RADIANS, follower.getHeading());
    }

    public double getHeadingRad() {
        return follower.getHeading();
    }

    public double getLastForwardCommand() {
        return lastForwardCommand;
    }

    public double getLastStrafeCommand() {
        return lastStrafeCommand;
    }

    public double getLastTurnCommand() {
        return lastTurnCommand;
    }

    public double getLastTargetForwardIps() {
        return DistanceUnit.METER.toInches(lastTargetForwardMps);
    }

    public double getLastTargetStrafeIps() {
        return DistanceUnit.METER.toInches(lastTargetStrafeMps);
    }

    public double getLastMeasuredForwardIps() {
        return DistanceUnit.METER.toInches(lastMeasuredForwardMps);
    }

    public double getLastMeasuredStrafeIps() {
        return DistanceUnit.METER.toInches(lastMeasuredStrafeMps);
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

    public void setPose(double x, double y, double headingRad) {
        follower.setStartingPose(new Pose(x, y, headingRad));
        follower.startTeleopDrive();
    }

    private static double applySlew(double target, double previous, double ratePerSecond, double dtSeconds) {
        if (!Double.isFinite(ratePerSecond) || ratePerSecond <= 0) {
            return target;
        }
        double maxDelta = ratePerSecond * dtSeconds;
        return Range.clip(target, previous - maxDelta, previous + maxDelta);
    }
}
