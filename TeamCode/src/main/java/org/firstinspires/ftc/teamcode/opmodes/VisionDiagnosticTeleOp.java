package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

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
        VisionSubsystemLimelight vision = robot.vision;
        vision.findAllianceSnapshot(Alliance.UNKNOWN);

        telemetry.addData("Vision state", vision.getState());
        telemetry.addData("Streaming", vision.isStreaming());
        telemetry.addData("Has tag", vision.hasValidTag());
        telemetry.addData("Current tag", vision.getCurrentTagId());
        telemetry.addData("Odometry pending", vision.shouldUpdateOdometry());

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Vision state", vision.getState().name());
            panelsTelemetry.debug("Streaming", Boolean.toString(vision.isStreaming()));
            panelsTelemetry.debug("Has tag", Boolean.toString(vision.hasValidTag()));
            panelsTelemetry.debug("Current tag", Integer.toString(vision.getCurrentTagId()));
        }

        vision.getLastSnapshot().ifPresent(snapshot -> {
            telemetry.addData("Snapshot alliance", snapshot.getAlliance().displayName());
            telemetry.addData("Snapshot id", snapshot.getTagId());
            telemetry.addData("Range (in)", format(snapshot.getFtcRange()));
            telemetry.addData("Bearing (deg)", format(snapshot.getFtcBearing()));
            telemetry.addData("Yaw (deg)", format(snapshot.getFtcYaw()));
            telemetry.addData("Robot pose", String.format("(%.1f, %.1f, %.1f°)",
                    snapshot.getRobotX(),
                    snapshot.getRobotY(),
                    snapshot.getRobotYaw()));
            telemetry.addData("tx/ty/ta", String.format("(%.2f°, %.2f°, %.2f)",
                    snapshot.getTxDegrees(),
                    snapshot.getTyDegrees(),
                    snapshot.getTargetAreaPercent()));

            if (panelsTelemetry != null) {
                panelsTelemetry.debug("Snapshot alliance", snapshot.getAlliance().displayName());
                panelsTelemetry.debug("Snapshot id", Integer.toString(snapshot.getTagId()));
                panelsTelemetry.debug("Range (in)", format(snapshot.getFtcRange()));
                panelsTelemetry.debug("Bearing (deg)", format(snapshot.getFtcBearing()));
                panelsTelemetry.debug("Yaw (deg)", format(snapshot.getFtcYaw()));
                panelsTelemetry.debug("Robot pose",
                        String.format("(%.1f, %.1f, %.1f°)",
                                snapshot.getRobotX(),
                                snapshot.getRobotY(),
                                snapshot.getRobotYaw()));
            }
        });

        if (panelsTelemetry != null) {
            panelsTelemetry.update(telemetry);
        }
        telemetry.update();
    }

    private static String format(double value) {
        return Double.isNaN(value) ? "--" : String.format("%.2f", value);
    }
}
