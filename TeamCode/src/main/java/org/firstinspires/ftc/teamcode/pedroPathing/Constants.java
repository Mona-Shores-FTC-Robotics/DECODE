package org.firstinspires.ftc.teamcode.pedroPathing;

import android.health.connect.datatypes.units.Mass;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    /** Centralized hardware names. Use these everywhere. */
    public static class HardwareNames {
        public static final String LF = "lf";
        public static final String RF = "rf";
        public static final String LB = "lb";
        public static final String RB = "rb";
        public static final String PINPOINT = "pinpoint";
    }

    /** Centralized naming conventions for consistency. */
    public static final class Naming {
        private Naming() {} // prevent instantiation

        /**
         * Prefix for all internal modules.
         */
        public static final String MODULE_PREFIX = "PP"; // PedroPathing

        /**
         * Names for different collections or groups of items.
         */
        public static final class CollectionNames {
            private CollectionNames() {}

            public static final String AUTONOMOUS_ROUTINES = "autonomous_routines";
            public static final String TELEOP_CONFIGS = "teleop_configs";
        }

        /**
         * Standardized field names used in data logging, telemetry, or configuration.
         */
        public static final class FieldNames {
            private FieldNames() {}

            public static final String MOTOR_POWER = "motor_power";
            public static final String SERVO_POSITION = "servo_position";
            public static final String SENSOR_VALUE = "sensor_value";
        }
    }

    // ========= Pedro follower and localization =========
    public static FollowerConstants followerConstants = new FollowerConstants()
//            .forwardZeroPowerAcceleration(-29.294739328)
//            .lateralZeroPowerAcceleration(-77.6639)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
//            .translationalPIDFCoefficients(new PIDFCoefficients(.030, 0, .0001, .08))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.01, 0, 0.0001, .05))
//            .headingPIDFCoefficients(new PIDFCoefficients(.6, .0001, .035, .065))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.4, 0, .015, .029))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.0058, 0, .001, 0.6, .075)) //was .005p
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(.0002, 0.01, 0.001, 0.6, 0.001))
            .centripetalScaling(0.00005)
            .mass(15.4221); //34 pounds 11/16/25

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
//            .xVelocity(72.58829912801427) //tuned 11/16/25
//            .yVelocity(43.5633) //tuned 11/16/25
            .rightFrontMotorName(HardwareNames.RF)
            .rightRearMotorName(HardwareNames.RB)
            .leftRearMotorName(HardwareNames.LB)
            .leftFrontMotorName(HardwareNames.LF)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.25)
            .strafePodX(0) //19429 6.25
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(HardwareNames.PINPOINT)
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

    // ========= Speed control configuration and helpers =========
    public static class Speed {
        // Geometry (meters)
        public static double WHEEL_RADIUS_M = 0.048;           // 96 mm diameter
        public static double TRACKWIDTH_M   = DistanceUnit.INCH.toMeters(14.25);         // left-right spacin
        public static double WHEELBASE_M    = DistanceUnit.INCH.toMeters(4.75); // front-back spacing

        // Motor model
        public static double GEAR_RATIO    = 1.0;              // wheel revs per motor rev
        public static double TICKS_PER_REV = 537.7;         // goBILDA 435 rpm motor (FTC encoder counts per wheel rev)

        // Limits
        public static double MAX_VEL_MPS        = 1.6;
        public static double MAX_ANG_VEL_RADPS  = Math.toRadians(360);

        // Slew limits for speed mode
        public static double LINEAR_SLEW_MPS2   = 3.0;
        public static double ANGULAR_SLEW_RAD2  = Math.toRadians(720);

        /** Convert wheel linear speed to motor ticks per second. */
        public static double mpsToTicksPerSec(double mps) {
            return mps * (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI * WHEEL_RADIUS_M);
        }

        public static double ticksPerSecToMps(double ticksPerSec) {
            return ticksPerSec * (2.0 * Math.PI * WHEEL_RADIUS_M) / (TICKS_PER_REV * GEAR_RATIO);
        }

        /** Effective radius for yaw coupling. */
        public static double kYaw() {
            return 0.5 * (TRACKWIDTH_M + WHEELBASE_M);
        }
    }

    /** Simple holder for DcMotorEx references and helpers. */
    public static class Motors {
        public final DcMotorEx lf, rf, lb, rb;

        public Motors(HardwareMap hw) {
            this.lf = hw.get(DcMotorEx.class, HardwareNames.LF);
            this.rf = hw.get(DcMotorEx.class, HardwareNames.RF);
            this.lb = hw.get(DcMotorEx.class, HardwareNames.LB);
            this.rb = hw.get(DcMotorEx.class, HardwareNames.RB);

            // Match the Pedro follower assumptions so direct velocity control shares the same axes.
            this.lf.setDirection(DcMotorSimple.Direction.REVERSE);
            this.lb.setDirection(DcMotorSimple.Direction.REVERSE);
            this.rf.setDirection(DcMotorSimple.Direction.FORWARD);
            this.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void setRunUsingEncoder() {
            safeSetMode(lf, DcMotor.RunMode.RUN_USING_ENCODER);
            safeSetMode(rf, DcMotor.RunMode.RUN_USING_ENCODER);
            safeSetMode(lb, DcMotor.RunMode.RUN_USING_ENCODER);
            safeSetMode(rb, DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void setRunWithoutEncoder() {
            safeSetMode(lf, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            safeSetMode(rf, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            safeSetMode(lb, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            safeSetMode(rb, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setAllVelocityTps(double tpsLF, double tpsRF, double tpsLB, double tpsRB) {
            lf.setVelocity(tpsLF);
            rf.setVelocity(tpsRF);
            lb.setVelocity(tpsLB);
            rb.setVelocity(tpsRB);
        }

        public void stop() {
            lf.setVelocity(0);
            rf.setVelocity(0);
            lb.setVelocity(0);
            rb.setVelocity(0);
        }

        private static void safeSetMode(DcMotorEx m, DcMotor.RunMode mode) {
            try { m.setMode(mode); } catch (Throwable ignored) { }
        }
    }

    /** Mecanum inverse kinematics. Inputs are robot frame: vx forward, vy left, wz CCW. */
    public static class MecanumIK {
        public static WheelSpeeds wheelSpeedsFromChassis(double vx, double vy, double wz) {
            double k = Speed.kYaw();
            double vFL = vx + vy - k * wz;
            double vFR = vx - vy + k * wz;
            double vBL = vx - vy - k * wz;
            double vBR = vx + vy + k * wz;
            return new WheelSpeeds(vFL, vFR, vBL, vBR);
        }

        public static class WheelSpeeds {
            public final double vFL, vFR, vBL, vBR; // m/s
            public WheelSpeeds(double vFL, double vFR, double vBL, double vBR) {
                this.vFL = vFL; this.vFR = vFR; this.vBL = vBL; this.vBR = vBR;
            }
            public TicksPerSec toTicksPerSec() {
                double tFL = Speed.mpsToTicksPerSec(vFL);
                double tFR = Speed.mpsToTicksPerSec(vFR);
                double tBL = Speed.mpsToTicksPerSec(vBL);
                double tBR = Speed.mpsToTicksPerSec(vBR);
                return new TicksPerSec(tFL, tFR, tBL, tBR);
            }
        }

        public static class TicksPerSec {
            public final double tFL, tFR, tBL, tBR;
            public TicksPerSec(double tFL, double tFR, double tBL, double tBR) {
                this.tFL = tFL; this.tFR = tFR; this.tBL = tBL; this.tBR = tBR;
            }
        }
    }
}
