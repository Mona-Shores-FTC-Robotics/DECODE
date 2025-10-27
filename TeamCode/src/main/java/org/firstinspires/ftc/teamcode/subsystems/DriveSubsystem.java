package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Owns the Pedro follower and exposes a minimal API for commanding the drivetrain.
 * OpModes choose the frame (field vs robot-centric) and pass scaled stick values here.
 */
@Configurable
public class DriveSubsystem implements Subsystem {

    public enum DriveMode {
        NORMAL,
        SLOW
    }

    private static final double NORMAL_MULTIPLIER = 1.0;

    /**
     * Multiplier applied when slow mode is requested.
     * Tunable via the live config system so drivers can adjust on the fly.
     */
    public static double slowMultiplier = 0.35;

    private final Follower follower;
    private final Constants.Motors driveMotors;
    private final DcMotorEx motorLf;
    private final DcMotorEx motorRf;
    private final DcMotorEx motorLb;
    private final DcMotorEx motorRb;

    private boolean robotCentric = false;
    private DriveMode activeMode = DriveMode.NORMAL;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        driveMotors = new Constants.Motors(hardwareMap);
        driveMotors.setRunUsingEncoder();

        motorLf = driveMotors.lf;
        motorRf = driveMotors.rf;
        motorLb = driveMotors.lb;
        motorRb = driveMotors.rb;
    }

    public void setRobotCentric(boolean enabled) {
        robotCentric = enabled;
    }

    public boolean isRobotCentric() {
        return robotCentric;
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

    @Override
    public void periodic() {
        follower.update();
    }

    public void stop() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
        driveMotors.stop();

        activeMode = DriveMode.NORMAL;
    }

    /**
     * Applies scaled translation/rotation commands that have already been sampled from an input device.
     *
     * @param fieldX        translation along the field X axis (+right from the driver wall)
     * @param fieldY        translation along the field Y axis (+forward away from driver wall)
     * @param rotationInput rotation command with positive meaning counter-clockwise
     * @param slowMode      when true applies the configured slow multiplier
     */
    public void driveScaled(double fieldX, double fieldY, double rotationInput, boolean slowMode) {
        double multiplier = slowMode ? Range.clip(slowMultiplier, 0.0, 1.0) : NORMAL_MULTIPLIER;
        double forward = Range.clip(fieldY * multiplier, -1.0, 1.0);
        double strafeLeft = Range.clip(-fieldX * multiplier, -1.0, 1.0);
        double turnCW = Range.clip(-rotationInput * multiplier, -1.0, 1.0);
        follower.setTeleOpDrive(forward, strafeLeft, turnCW, robotCentric);
        activeMode = slowMode ? DriveMode.SLOW : DriveMode.NORMAL;
    }

    // --- Telemetry accessors ------------------------------------------------
    public DriveMode getDriveMode() {
        return activeMode;
    }

    public Pose2D getPose() {
        Pose pose = follower.getPose();
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, follower.getHeading());
    }

    public Pose getFollowerPose() {
        return follower.getPose();
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

}
