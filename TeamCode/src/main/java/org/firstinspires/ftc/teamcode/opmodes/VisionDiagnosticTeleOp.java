package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.AprilTagPoseUtil;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Minimal driving OpMode that leaves the vision portal running and mirrors its pose estimates to
 * the driver station telemetry and FTControl Panels.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
        name = "Vision Diagnostic TeleOp",
        group = "Diagnostics"
)
public class VisionDiagnosticTeleOp extends NextFTCOpMode {

    private Robot robot;
    private DriverBindings driverBindings;
    private TelemetryManager panelsTelemetry;

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);

        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        driverBindings = new DriverBindings(driverPad);
        robot.drive.setRobotCentric(false);

        panelsTelemetry = PanelsBridge.preparePanels();

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.vision)
        );
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        DriverBindings.DriveRequest request = driverBindings.sampleDriveRequest();
        robot.drive.driveScaled(request.fieldX, request.fieldY, request.rotation, request.slowMode);

        pushVisionTelemetry();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }

    private void pushVisionTelemetry() {
        VisionSubsystem vision = robot.vision;
        List<AprilTagDetection> detections = vision.getDetections();

        telemetry.addData("Vision state", vision.getState());
        telemetry.addData("Detections", detections.size());

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Vision state", vision.getState().name());
            panelsTelemetry.debug("Detection count", detections.size());
        }

        for (int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            String ftcString = formatFtcPose(detection);
            String pedroString = formatPedroPose(detection);

            telemetry.addLine(String.format("Tag %d id=%d  FTC=%s  Pedro=%s",
                    i,
                    detection.id,
                    ftcString,
                    pedroString));

            if (panelsTelemetry != null) {
                panelsTelemetry.debug(
                        "Tag " + detection.id,
                        "FTC " + ftcString + "  Pedro " + pedroString
                );
            }
        }

        if (panelsTelemetry != null) {
            panelsTelemetry.update(telemetry);
        }
        telemetry.update();
    }

    private static String formatFtcPose(AprilTagDetection detection) {
        if (detection == null) {
            return "(n/a)";
        }
        if (detection.ftcPose != null) {
            return String.format("(%.1f, %.1f, %.1f°)",
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    detection.ftcPose.yaw);
        }
        if (detection.robotPose != null) {
            return String.format("(%.1f, %.1f, %.1f°)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
        }
        return "(n/a)";
    }

    private static String formatPedroPose(AprilTagDetection detection) {
        Pose pose = AprilTagPoseUtil.toPedroPose(detection);
        if (pose == null) {
            return "(n/a)";
        }
        return String.format("(%.1f, %.1f, %.1f°)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
    }
}
