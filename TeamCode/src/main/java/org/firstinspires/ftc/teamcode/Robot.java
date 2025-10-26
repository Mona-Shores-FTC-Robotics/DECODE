// src/main/java/org/firstinspires/ftc/teamcode/Robot.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/** Simple container for all of the robot's subsystems. */
public class Robot {
    public final DriveSubsystem drive;
    public final FlywheelSubsystem flywheel;
    public final IntakeSubsystem intake;
    public final LightingSubsystem lighting;
    public final VisionSubsystem vision;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null, null);
    }

    public Robot(HardwareMap hardwareMap,
                 Position cameraPosition,
                 YawPitchRollAngles cameraOrientation) {
        drive = new DriveSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        if (cameraPosition != null && cameraOrientation != null) {
            vision = new VisionSubsystem(hardwareMap, cameraPosition, cameraOrientation);
        } else {
            vision = null;
        }
    }

    public void initializeAutonomous() {
        intake.initialize();
        lighting.initialize();
        if (vision != null) {
            vision.initialize();
        }
    }

    public void periodic() {
        drive.periodic();
        flywheel.periodic();
        intake.periodic();
        lighting.periodic();
        if (vision != null) {
            vision.periodic();
        }
    }

    public void stop() {
        drive.stop();
        flywheel.stop();
        intake.stop();
        lighting.indicateIdle();
        if (vision != null) {
            vision.stopStreaming();
        }
    }

    public void setAlliance(Alliance alliance) {
//        drive.setAlliance(alliance);
        intake.setAlliance(alliance);
        lighting.setAlliance(alliance);
    }
}
