package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

/**
 * Wrapper around the FTC AprilTag pipeline so OpModes can query detections
 * without performing hardware setup directly.
 */
@Configurable
public class VisionSubsystem implements Subsystem {

    public static Position CAMERA_POSITION = new Position(
            DistanceUnit.INCH,
            3.0,   // X right
            2.0,   // Y forward
            9.2,   // Z up
            0
    );
    public static YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0.0,   // yaw facing forward
            -90.0, // pitch to horizontal
            0.0,
            0
    );


    private final HardwareMap hardwareMap;

    public enum VisionState {
        OFF,
        STREAMING
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private VisionState state = VisionState.OFF;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initialize() {
        if (visionPortal != null) {
            return;
        }
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder().addProcessor(aprilTagProcessor);
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcam != null) {
            builder.setCamera(webcam);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        visionPortal = builder.build();
        state = VisionState.STREAMING;
    }

    @Override
    public void periodic() {
        // Nothing to do – the SDK handles processing on a background thread.
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTagProcessor = null;
        state = VisionState.OFF;
    }

    public VisionState getState() {
        return state;
    }

    public boolean isStreaming() {
        return state == VisionState.STREAMING;
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) {
            return Collections.emptyList();
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return Collections.emptyList();
        }
        return Collections.unmodifiableList(detections);
    }

}
