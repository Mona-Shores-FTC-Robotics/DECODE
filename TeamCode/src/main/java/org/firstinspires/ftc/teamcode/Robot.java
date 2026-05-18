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

        // Resolve per-robot configs once at boot; subsystems take what they need explicitly.
        RobotProfile profile = RobotProfile.forCurrent();

        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision,
                profile.aimAssist, profile.fixedAngleAim, profile.rightTriggerFixedAngle);
        launcher = new LauncherSubsystem(hardwareMap,
                profile.flywheel, profile.feeder, profile.hood, profile.timing);
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
        vision.initialize();
        drive.initialize();
        launcher.initialize();
        lighting.initialize();
        intake.initialize();

        // Wire lighting to receive lane color updates from intake
        intake.addLaneColorListener(lighting);
    }

    public void initializeForTeleOp() {
        drive.setTeleOpControlEnabled(true);
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
