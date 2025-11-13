package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.logging.Filter;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0))

            .headingPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0,0,0,0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0.0,.6, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,.6, 0))

            .mass(5);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(-2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)

                .build();
    }

    /**
     * Speed conversion utilities for drive motors.
     * Used to convert between motor encoder ticks and real-world velocities.
     */
    public static class Speed {
        // Geometry (meters)
        public static double WHEEL_RADIUS_M = 0.048;           // 96 mm diameter wheels
        public static double TRACKWIDTH_M   = DistanceUnit.INCH.toMeters(14.25);  // left-right spacing
        public static double WHEELBASE_M    = DistanceUnit.INCH.toMeters(4.75);   // front-back spacing

        // Motor model
        public static double GEAR_RATIO    = 1.0;              // wheel revs per motor rev
        public static double TICKS_PER_REV = 383.6;            // goBILDA 435 rpm motor

        // Limits
        public static double MAX_VEL_MPS        = 1.6;
        public static double MAX_ACCEL_MPS2     = 3.0;
        public static double MAX_ANG_VEL_RAD_S  = 2.0 * Math.PI;
        public static double MAX_ANG_ACCEL_RAD_S2 = 4.0 * Math.PI;

        /**
         * Convert motor encoder ticks/sec to meters/sec linear wheel velocity.
         */
        public static double ticksPerSecToMps(double ticksPerSec) {
            return ticksPerSec * (2.0 * Math.PI * WHEEL_RADIUS_M) / (TICKS_PER_REV * GEAR_RATIO);
        }

        /**
         * Convert meters/sec linear wheel velocity to motor encoder ticks/sec.
         */
        public static double mpsToTicksPerSec(double mps) {
            return mps * (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI * WHEEL_RADIUS_M);
        }

        /**
         * Effective radius for yaw coupling.
         */
        public static double kYaw() {
            return 0.5 * (TRACKWIDTH_M + WHEELBASE_M);
        }
    }
}
