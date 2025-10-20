package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
        SLOW // legacy alias for slow mode support
    }

    private static final double ROTATION_DEADBAND = 0.03;

    private final Follower follower;

    public double slowMultiplier = 0.20; // legacy compatibility for older TeleOp

    private boolean poseIsMeters = false;
    private Pose teleopSeedPose = null;
    private DriveMode defaultMode = DriveMode.NORMAL;
    private boolean fieldCentricFlag = true;

    private double headingHoldTarget = 0.0;
    private boolean headingHoldActive = false;

    private double lastForwardCommand = 0.0;
    private double lastStrafeCommand = 0.0;
    private double lastTurnCommand = 0.0;
    private DriveMode activeMode = DriveMode.NORMAL;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
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
     * Historical API used by earlier OpModes. Internally the Pedro follower expects
     * {@code true} for field-centric mode, matching the behaviour of the legacy code.
     */
    public void setRobotCentric(boolean value) {
        this.fieldCentricFlag = value;
    }

    public boolean isRobotCentric() {
        return fieldCentricFlag;
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
        double forward = Range.clip(-fieldY * multiplier, -1.0, 1.0);
        double strafeLeft = Range.clip(-fieldX * multiplier, -1.0, 1.0);
        double turnCW = Range.clip(-computeRotationCommand(rotationInput) * multiplier, -1.0, 1.0);

        follower.setTeleOpDrive(forward, strafeLeft, turnCW, fieldCentricFlag);

        lastForwardCommand = forward;
        lastStrafeCommand = strafeLeft;
        lastTurnCommand = turnCW;
        activeMode = mode;
    }

    private double computeRotationCommand(double rotationInput) {
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

    public void setPose(double x, double y, double headingRad) {
        follower.setStartingPose(new Pose(x, y, headingRad));
        follower.startTeleopDrive();
    }
}
