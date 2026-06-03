package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveAimAssistConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveFixedAngleAimConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveRightTriggerFixedAngleConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeGateConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeLaneSensorConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFeederConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFlywheelConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherHoodConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherTimingConfig;

/**
 * Per-robot configuration bundle. THE single source of truth for what differs
 * between robot 19429 and robot 20245.
 *
 * <p>How to use this file:
 * <ul>
 *   <li>Want to know what's different between the two robots? Read this file
 *       top to bottom. Every per-robot value lives here.</li>
 *   <li>Want a config in your subsystem? Call {@link #forCurrent()} and read
 *       the field you need (e.g. {@code RobotProfile.forCurrent().aimAssist}).</li>
 *   <li>Adding a third robot? Add a branch in {@link #buildFor(String)} and
 *       any new per-robot factories you need.</li>
 * </ul>
 *
 * <p>Only the active robot's profile is constructed. The inactive robot's
 * config objects are never created, so Dashboard / Panels show one tuning
 * tree, not two.
 */
public final class RobotProfile {

    // ============================================================
    // Public profile shape — instance fields filled at construction
    // ============================================================

    public final String name;
    public final LauncherFeederConfig feeder;
    public final LauncherHoodConfig hood;
    public final LauncherFlywheelConfig flywheel;
    public final LauncherTimingConfig timing;
    public final PinpointConstants pinpoint;
    public final FollowerConstants follower;
    public final MecanumConstants drive;
    public final DriveAimAssistConfig aimAssist;
    public final DriveFixedAngleAimConfig fixedAngleAim;
    public final DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle;
    public final IntakeGateConfig gate;
    public final IntakeLaneSensorConfig.LanePresenceConfig lanePresence;
    public final CommandRangeConfig commandRange;

    private RobotProfile(
            String name,
            LauncherFeederConfig feeder,
            LauncherHoodConfig hood,
            LauncherFlywheelConfig flywheel,
            LauncherTimingConfig timing,
            PinpointConstants pinpoint,
            FollowerConstants follower,
            MecanumConstants drive,
            DriveAimAssistConfig aimAssist,
            DriveFixedAngleAimConfig fixedAngleAim,
            DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle,
            IntakeGateConfig gate,
            IntakeLaneSensorConfig.LanePresenceConfig lanePresence,
            CommandRangeConfig commandRange) {
        this.name = name;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheel = flywheel;
        this.timing = timing;
        this.pinpoint = pinpoint;
        this.follower = follower;
        this.drive = drive;
        this.aimAssist = aimAssist;
        this.fixedAngleAim = fixedAngleAim;
        this.rightTriggerFixedAngle = rightTriggerFixedAngle;
        this.gate = gate;
        this.lanePresence = lanePresence;
        this.commandRange = commandRange;
    }

    // ============================================================
    // Lazy single-instance cache
    // ============================================================

    private static volatile RobotProfile cached;

    /**
     * Returns the profile for the robot currently identified by
     * {@link RobotState#getRobotName()}. Builds it on first call; the same
     * instance is returned thereafter. Unknown robots fall back to 20245.
     */
    public static RobotProfile forCurrent() {
        RobotProfile local = cached;
        if (local == null) {
            local = buildFor(RobotState.getRobotName());
            cached = local;
        }
        return local;
    }

    /**
     * Drops the cached profile so the next {@link #forCurrent()} re-resolves the
     * robot identity from {@link RobotState#getRobotName()}.
     *
     * <p>Needed because the {@code @Configurable} annotation scan at app boot loads
     * the subsystem classes — running their {@code static} config-field initializers,
     * which call {@code forCurrent()} — <em>before</em> the WiFi SSID has identified
     * the robot. That caches the 20245 fallback. OpMode init calls this once the
     * SSID is known (see {@code Robot}'s constructor) so the correct robot's config
     * is rebuilt. Pair it with each subsystem's {@code reloadProfileConfigs()} to
     * re-bind the already-captured static field references.
     */
    public static void invalidate() {
        cached = null;
    }

    /** Human-readable identifier for telemetry. Marks "(default)" if unknown. */
    public static String activeName() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) return "19429";
        if ("DECODE_20245".equals(robotName)) return "20245";
        return "20245 (default)";
    }

    private static RobotProfile buildFor(String robotName) {
        boolean is19429 = "DECODE_19429".equals(robotName);
        String displayName = is19429 ? "19429" : "20245";
        return new RobotProfile(
                displayName,
                is19429 ? feederConfig19429() : feederConfig20245(),
                hoodConfig(),
                is19429 ? flywheelConfig19429() : flywheelConfig20245(),
                timingConfig(),
                pinpointConstants(),
                is19429 ? followerConstants19429() : followerConstants20245(),
                is19429 ? driveConstants19429() : driveConstants20245(),
                aimAssistConfig(),
                new DriveFixedAngleAimConfig(),
                new DriveRightTriggerFixedAngleConfig(),
                new IntakeGateConfig(),
                is19429 ? lanePresenceConfig19429() : lanePresenceConfig20245(),
                is19429 ? rangeConfig19429() : rangeConfig20245());
    }

    // ============================================================
    // Shared factories — same values for both robots today.
    // If a robot ever needs a different value, split into 19429/20245.
    // ============================================================

    private static LauncherHoodConfig hoodConfig() {
        LauncherHoodConfig config = new LauncherHoodConfig();
        config.hoodLeft.shortPosition = .3;
        config.hoodCenter.shortPosition = .3;
        config.hoodRight.shortPosition = .3;
        config.hoodLeft.midPosition = .05;
        config.hoodCenter.midPosition = .05;
        config.hoodRight.midPosition = .05;
        config.hoodLeft.longPosition = 0;
        config.hoodCenter.longPosition = 0;
        config.hoodRight.longPosition = 0;
        return config;
    }

    private static LauncherTimingConfig timingConfig() {
        return new LauncherTimingConfig();
    }

    private static LauncherFlywheelConfig flywheelConfig19429() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.flywheelLeft.reversed = false;
        config.flywheelLeft.idleRpm = 1500;
        config.flywheelLeft.kS = 0.10;
        config.flywheelLeft.kV = 0.00017;
        config.flywheelLeft.kP = .001;

        config.flywheelCenter.reversed = false;
        config.flywheelCenter.idleRpm = 1500;
        config.flywheelCenter.kS = 0.10;
        config.flywheelCenter.kV = 0.00017;
        config.flywheelCenter.kP = .001;

        config.flywheelRight.reversed = false;
        config.flywheelRight.idleRpm = 1500;
        config.flywheelRight.kS = 0.10;
        config.flywheelRight.kV = 0.00017;
        config.flywheelRight.kP = .001;
        return config;
    }

    private static LauncherFlywheelConfig flywheelConfig20245() {
        LauncherFlywheelConfig config = new LauncherFlywheelConfig();
        config.flywheelLeft.reversed = false;
        config.flywheelLeft.idleRpm = 1500;
        config.flywheelLeft.kS = 0.10;
        config.flywheelLeft.kV = 0.00017;
        config.flywheelLeft.kP = .001;

        config.flywheelCenter.reversed = false;
        config.flywheelCenter.idleRpm = 1500;
        config.flywheelCenter.kS = 0.10;
        config.flywheelCenter.kV = 0.00017;
        config.flywheelCenter.kP = .001;

        config.flywheelRight.reversed = true;
        config.flywheelRight.idleRpm = 1500;
        config.flywheelRight.kS = 0.10;
        config.flywheelRight.kV = 0.00017;
        config.flywheelRight.kP = .001;
        return config;
    }

    private static DriveAimAssistConfig aimAssistConfig() {
        DriveAimAssistConfig config = new DriveAimAssistConfig();
        config.kP = 0.35;
        config.kPInner = 2.85;
        config.innerZoneDeg = 3.5;
        config.kD = 0.05;
        config.kMaxTurn = 0.6;
        config.kStatic = 0.1;
        config.staticApplyAboveDeg = 3.0;
        config.turnSlewRatePerSec = 8.0;
        config.deadbandDeg = 2.0;
        return config;
    }

    // ============================================================
    // Per-robot factories — values genuinely differ between robots.
    // Side-by-side here so a reader can diff them in one editor window.
    // ============================================================

    private static LauncherFeederConfig feederConfig19429() {
        LauncherFeederConfig config = new LauncherFeederConfig();
        config.center.loadPosition  = .93;
        config.center.pinchPosition = .87;
        config.center.firePosition  = .75;

        config.left.loadPosition  = .85;
        config.left.pinchPosition = .74;
        config.left.firePosition  = .61;

        config.right.loadPosition  = .75;
        config.right.pinchPosition = .69;
        config.right.firePosition  = .56;
        return config;
    }

    private static LauncherFeederConfig feederConfig20245() {
        LauncherFeederConfig config = new LauncherFeederConfig();
        config.center.loadPosition  = .18;
        config.center.pinchPosition = .12;
        config.center.firePosition  = .03;

        config.left.loadPosition  = .33;
        config.left.pinchPosition = .27;
        config.left.firePosition  = .18;

        config.right.loadPosition  = .78;
        config.right.pinchPosition = .72;
        config.right.firePosition  = .58;
        return config;
    }

    private static FollowerConstants followerConstants19429() {
        return new FollowerConstants()
                .forwardZeroPowerAcceleration(-29.294739328)
                .lateralZeroPowerAcceleration(-77.6639)
                .useSecondaryTranslationalPIDF(true)
                .useSecondaryHeadingPIDF(true)
                .useSecondaryDrivePIDF(true)
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(.005, 0, 0.0005, 0.6, 0.0001))
                .drivePIDFSwitch(5)
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(.015, 0, .0001, 0.6, 0.0001))
                .headingPIDFCoefficients(new PIDFCoefficients(.6, .01, .0001, .0005))
                .headingPIDFSwitch(Math.toRadians(3.5))
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.0, 0.0001, .05, 0.0003))
                .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0, .05))
                .translationalPIDFSwitch(5)
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.1, 0.0001, .0006, .00015))
                .centripetalScaling(0)
                .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016))
                .mass(15.4221); // 34 pounds 11/16/25
    }

    private static FollowerConstants followerConstants20245() {
        return new FollowerConstants()
                .forwardZeroPowerAcceleration(-31.2)
                .lateralZeroPowerAcceleration(-82)
                .useSecondaryTranslationalPIDF(true)
                .useSecondaryHeadingPIDF(true)
                .useSecondaryDrivePIDF(true)
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(.005, 0, 0.0005, 0.6, .0001)) // tuned 12/4
                .drivePIDFSwitch(3)
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(.02, 0, 0, 0.6, .0001))
                .headingPIDFCoefficients(new PIDFCoefficients(.48, 0.01, .001, .0005))
                // Switch 3° → 7° → 10°. Above the switch the weak primary (P=.48) can't break
                // in-place turning friction and STALLS (e.g. #3 parked at 7.1°, angVel≈0, right
                // at the 7° boundary). The secondary is now well-damped (P=1.0/D=.09 settled
                // #5/#7 cleanly), so widening its range to 10° lets it own the 7-10° stall zone.
                .headingPIDFSwitch(Math.toRadians(10))
                // Tuning the <3° settle regime for launch-pose accuracy:
                //   P 3.0 → 1.5 → 1.0 → 0.7 (each step cuts overshoot; over the 10° band at full
                //                  battery, 1.0 still over-drove and swung ±40 dps even with the
                //                  base stopped — easing drive so D=.09 can actually damp it.
                //                  I=.01 keeps it from stalling at the lower P.)
                //   D .001 → .02 → .05 → .09 (under-damped at FULL battery voltage — confirmed
                //                  fresh-pack run still swung ±22-36 dps while a sagging pack
                //                  settled. Matches start full, so tuning for full voltage:
                //                  D=.05 braking (~.03 power) lost to P drive (~.09); .09 to win.)
                //   I .0001 → .01 (it was stalling ~4-5° short with ang vel ≈ 0; no integral
                //                  authority to overcome static friction. I builds power to
                //                  close the steady-state offset.) If it now drifts past and
                //                  slowly swings back, that's I-windup — back I off.
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.7, 0.01, .09, .0003))
                .translationalPIDFCoefficients(new PIDFCoefficients(.06, 0, 0, .05))
                .translationalPIDFSwitch(5)
                // Secondary (<5" regime): I .0001 → .01 closed the ~0.7" stall. D .0006 → .006
                // → .03: position error (tErr) looks fine but the robot whips THROUGH the point
                // at vel 2-5 ips and never stops — and that residual translation disturbs heading
                // (when the base actually stops, vel≈0, heading settles instantly: see #3). D=.006
                // braked only ~.01 power at 2 ips; .03 to actually arrest the velocity. If it eases
                // in sluggishly / undershoots position, back D off.
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.1, 0.01, .03, .00015))
                .centripetalScaling(0)
                // FOLLOW-UP (predictive braking): likely gains here. Predictive braking IS enabled
                // (the .predictiveBrakingCoefficients(...) builder auto-sets usePredictiveBraking=true).
                // Coeffs are (kLinearBraking, kQuadraticFriction, P). Decelerating the approach harder
                // — bump kLinearBraking / P here, or brakingStrength/brakingStart in Constants
                // pathConstraints — could let the robot arrive slower and settle even faster, instead
                // of leaning on the secondary translational D to arrest velocity at the point.
                .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.0655, 0.0015))
                .mass(15.4221);
    }

    private static MecanumConstants driveConstants19429() {
        return driveConstants(59.1365506697527, 53.16551100362942);
    }

    private static MecanumConstants driveConstants20245() {
        return driveConstants(58.5, 50.4); // tuned 12/4
    }

    private static MecanumConstants driveConstants(double xVelocity, double yVelocity) {
        return new MecanumConstants()
                .maxPower(1.0)
                .xVelocity(xVelocity)
                .yVelocity(yVelocity)
                .rightFrontMotorName(Constants.HardwareNames.RF)
                .rightRearMotorName(Constants.HardwareNames.RB)
                .leftRearMotorName(Constants.HardwareNames.LB)
                .leftFrontMotorName(Constants.HardwareNames.LF)
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .useBrakeModeInTeleOp(true);
    }

    private static PinpointConstants pinpointConstants() {
        return new PinpointConstants()
                .forwardPodY(6.25)
                .strafePodX(0)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName(Constants.HardwareNames.PINPOINT)
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    private static IntakeLaneSensorConfig.LanePresenceConfig lanePresenceConfig19429() {
        return lanePresenceConfig(0.09, 0.09, 0.09, 6.5, 7.0, 5.0);
    }

    private static IntakeLaneSensorConfig.LanePresenceConfig lanePresenceConfig20245() {
        return lanePresenceConfig(0.04, 0.02, 0.04, 4.0, 4.0, 4.5);
    }

    private static IntakeLaneSensorConfig.LanePresenceConfig lanePresenceConfig(
            double leftValue, double centerValue, double rightValue,
            double leftCm, double centerCm, double rightCm) {
        IntakeLaneSensorConfig.LanePresenceConfig config = new IntakeLaneSensorConfig.LanePresenceConfig();
        config.useDistance = false;
        config.useSaturation = false;
        config.saturationThreshold = 0.25;
        config.useValue = true;
        config.useHue = false;
        config.hueThreshold = 130.0;
        config.leftValueThreshold = leftValue;
        config.centerValueThreshold = centerValue;
        config.rightValueThreshold = rightValue;
        config.leftThresholdCm = leftCm;
        config.centerThresholdCm = centerCm;
        config.rightThresholdCm = rightCm;
        return config;
    }

    private static CommandRangeConfig rangeConfig19429() {
        CommandRangeConfig config = new CommandRangeConfig();
        // Teleop ranges:                     all-lanes RPM,  hood
        config.teleop.shortRange.set(2400, 1);             // RPM was 1900 (matched to 20245)
        config.teleop.midRange.set(3350, 0.05);            // RPM was 2500 (matched to 20245)
        config.teleop.longRange.set(3875, 4000, 0.0);      // min/max RPM was 2725/2900 (matched to 20245)
        // Auto ranges:
        config.auto.shortRange.set(2500, .55);             // RPM was 2000 (matched to 20245)
        config.auto.midRange.set(3350, 0.1);               // RPM was 2400 (matched to 20245)
        config.auto.farRange.set(3900, 0.0);               // RPM was 2725 (matched to 20245)
        config.timeoutSeconds = 3.5;
        return config;
    }

    private static CommandRangeConfig rangeConfig20245() {
        CommandRangeConfig config = new CommandRangeConfig();
        // Teleop ranges:                     all-lanes RPM,  hood
        config.teleop.shortRange.set(2400, .95);
        config.teleop.midRange.set(3350, 0.1);
        config.teleop.longRange.set(3875, 4000, 0.0); // min RPM, max RPM, hood
        // Auto ranges:
        config.auto.shortRange.set(2500, .55);
        config.auto.midRange.set(3350, 0.1);
        config.auto.farRange.set(3900, 0.0);
        config.timeoutSeconds = 2.5;
        return config;
    }
}
