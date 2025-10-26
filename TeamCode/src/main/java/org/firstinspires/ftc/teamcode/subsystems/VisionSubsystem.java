package org.firstinspires.ftc.teamcode.subsystems;

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
public class VisionSubsystem implements Subsystem {

    public enum VisionState {
        OFF,
        STREAMING
    }

    private final HardwareMap hardwareMap;
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private VisionState state = VisionState.OFF;

    public VisionSubsystem(HardwareMap hardwareMap,
                           Position cameraPosition,
                           YawPitchRollAngles cameraOrientation) {
        this.hardwareMap = hardwareMap;
        this.cameraPosition = cameraPosition;
        this.cameraOrientation = cameraOrientation;
    }

    @Override
    public void initialize() {
        start();
    }

    @Override
    public void periodic() {
        // Nothing to do – the SDK handles processing on a background thread.
    }

    public void start() {
        if (visionPortal != null) {
            return;
        }
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
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

    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTagProcessor = null;
        state = VisionState.OFF;
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) {
            return Collections.emptyList();
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections == null ? Collections.emptyList() : detections;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public VisionState getState() {
        return state;
    }
}
