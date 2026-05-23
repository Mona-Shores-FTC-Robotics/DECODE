package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ControlHubIdentifierUtil;
import org.firstinspires.ftc.teamcode.util.RobotProfile;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryService;

/**
 * Robot is the hardware container — it holds every subsystem and wires them together.
 * Think of it as the robot's "parts list": motors, sensors, and servos all get built here
 * and then the rest of the code asks Robot for what it needs.
 *
 * To add a new mechanism:
 *   1. Create a subsystem class in the subsystems/ folder.
 *   2. Add a public field for it here (e.g. "public final ArmSubsystem arm;").
 *   3. Build it in the constructor below (e.g. "arm = new ArmSubsystem(hardwareMap);").
 *   4. Call arm.initialize() in both initializeForTeleOp() and initializeForAuto().
 */
public class Robot {
    public final DriveSubsystem drive;
    public final LauncherSubsystem launcher;
    public final IntakeSubsystem intake;
    public final LightingSubsystem lighting;
    public final VisionSubsystemLimelight vision;
    public final TelemetryService telemetry;

    private final HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new TelemetryService());
    }

    public Robot(HardwareMap hardwareMap, TelemetryService telemetryService) {
        this.hardwareMap = hardwareMap;
        // Note: setRobotName() is now called from OpMode.onInit() BEFORE attachPedroFollower()
        // to give WiFi more time to initialize on first boot
        telemetry = telemetryService == null ? new TelemetryService() : telemetryService;

        // Ensure robot identity is resolved before subsystems pull config defaults (motor directions, etc.)
        if ("UNKNOWN".equals(RobotState.getRobotName())) {
            ControlHubIdentifierUtil.setRobotName(hardwareMap, null);
        }

        // The boot-time @Configurable scan loads the subsystem classes and runs their
        // static config-field initializers (which call RobotProfile.forCurrent())
        // BEFORE the SSID identifies the robot, caching the 20245 fallback. Now that
        // the name is resolved, drop that cache and re-bind the static fields so both
        // they and the constructor-injected instances below see the correct robot.
        RobotProfile.invalidate();
        LauncherSubsystem.reloadProfileConfigs();
        DriveSubsystem.reloadProfileConfigs();
        IntakeSubsystem.reloadProfileConfigs();

        // Resolve per-robot configs once at boot; subsystems take what they need explicitly.
        RobotProfile profile = RobotProfile.forCurrent();

        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision,
                profile.aimAssist, profile.fixedAngleAim, profile.rightTriggerFixedAngle);
        launcher = new LauncherSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, profile.gate, profile.lanePresence);
        lighting = new LightingSubsystem(hardwareMap);

        drive.setLightingSubsystem(lighting);
    }

    public void setAlliance(Alliance alliance) {
        RobotState.setAlliance(alliance);
        vision.setAlliance(alliance);
        lighting.setAlliance(alliance);
    }

    public void initializeForAuto() {
        drive.setTeleOpControlEnabled(false);
        initializeAllSubsystems();
    }

    public void initializeForTeleOp() {
        drive.setTeleOpControlEnabled(true);
        initializeAllSubsystems();
    }

    /** Shared subsystem-init sequence used by both Auto and TeleOp. */
    private void initializeAllSubsystems() {
        vision.initialize();
        drive.initialize();
        launcher.initialize();
        lighting.initialize();
        intake.initialize();

        // Wire lighting to receive lane color updates from intake
        intake.addLaneColorListener(lighting);
    }

    public void attachPedroFollower() {
        // Build the Follower (previously done by NextFTC PedroComponent.preInit).
        Constants.createFollower(hardwareMap);
        drive.attachFollower();
    }
}
