package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotProfile;

public class Constants {

    public static FusionLocalizer activeFusionLocalizer;

    /** Pedro follower constants for the active robot. See {@code util/RobotProfile.java} for per-robot tuning. */
    public static FollowerConstants followerConstants() {
        return RobotProfile.forCurrent().follower;
    }

    /** Pinpoint localizer constants for the active robot. See {@code util/RobotProfile.java} for per-robot tuning. */
    public static PinpointConstants localizerConstants() {
        return RobotProfile.forCurrent().pinpoint;
    }

    /** Mecanum drive constants for the active robot. See {@code util/RobotProfile.java} for per-robot tuning. */
    public static MecanumConstants driveConstants() {
        return RobotProfile.forCurrent().drive;
    }

    /** Centralized hardware names. Use these everywhere. */
    public static class HardwareNames {
        public static final String LF = "lf";
        public static final String RF = "rf";
        public static final String LB = "lb";
        public static final String RB = "rb";
        public static final String PINPOINT = "pinpoint";
    }

    // PathConstraints parameter order:
    // tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint,
    // timeoutConstraint, brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, // tValueConstraint: Pedro default — end-settling starts at 99.5% of path, not 95%
            0.1,   // velocityConstraint: Pedro default — was 0.08
            .1,
            Math.toRadians(.5),
            400, // timeoutConstraint (ms): settle window AFTER reaching path end — give the
                 // heading PID time to pull the last few degrees in before completing. Was 100.
            1.0,
            10,
            1);

    private static Follower _follower;

    /**
     * Builds the Pedro Follower for the current robot. Caches the instance so
     * {@link #follower()} can return it without rebuilding. Call exactly once
     * from OpMode init (e.g. {@code Robot.attachPedroFollower()}).
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, localizerConstants());
        activeFusionLocalizer = new FusionLocalizer(
                pinpoint,
                new Pose(0.25, 0.25, Math.toRadians(2.0)),
                new Pose(1.0, 1.0, Math.toRadians(0.5) / 60.0),
                new Pose(2.1561, 2.6065, 0.0248),
                100
        );
        _follower = new FollowerBuilder(followerConstants(), hardwareMap)
                .setLocalizer(activeFusionLocalizer)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants())
                .build();
        return _follower;
    }

    /** Returns the follower built by {@link #createFollower}. */
    public static Follower follower() {
        return _follower;
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
            this.lf = org.firstinspires.ftc.teamcode.util.CachedHardware.motor(hw, HardwareNames.LF);
            this.rf = org.firstinspires.ftc.teamcode.util.CachedHardware.motor(hw, HardwareNames.RF);
            this.lb = org.firstinspires.ftc.teamcode.util.CachedHardware.motor(hw, HardwareNames.LB);
            this.rb = org.firstinspires.ftc.teamcode.util.CachedHardware.motor(hw, HardwareNames.RB);

            // Match the Pedro follower assumptions so direct velocity control shares the same axes.
            this.lf.setDirection(DcMotorSimple.Direction.REVERSE);
            this.lb.setDirection(DcMotorSimple.Direction.REVERSE);
            this.rf.setDirection(DcMotorSimple.Direction.FORWARD);
            this.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void setRunWithoutEncoder() {
            safeSetMode(lf, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            safeSetMode(rf, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            safeSetMode(lb, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            safeSetMode(rb, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

}
