package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Mirrors the reference heading-lock logic so you can tune gains live through Panels.
 * Hold X to engage heading lock toward {@link LockConfig#targetHeadingDeg}, release to go back to manual rotation.
 */
@Configurable
@TeleOp(name = "Heading Lock Diagnostic", group = "Debug")
public class HeadingLockDiagnostic extends OpMode {

    @Configurable
    public static class LockConfig {
        public static double targetHeadingDeg = 180.0;
        public static double toleranceDeg = 2.0;
        public static double kP = 0.8;
        public static double kI = 0.0;
        public static double kD = 0.05;
        public static double maxTurn = 1.0;
    }

    private Follower follower;
    private final PIDFController headingPid = new PIDFController(new PIDFCoefficients(0, 0, 0, 0));
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(LockConfig.class);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        boolean headingLock = gamepad1.x;
        double forward = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0);
        double strafeLeft = Range.clip(-gamepad1.left_stick_x, -1.0, 1.0);
        double rotationCw = Range.clip(-gamepad1.right_stick_x, -1.0, 1.0);
        double headingCommandCw = rotationCw;

        double targetHeadingRad = Math.toRadians(LockConfig.targetHeadingDeg);
        double toleranceRad = Math.toRadians(Math.max(0.0, LockConfig.toleranceDeg));
        double maxTurn = Range.clip(LockConfig.maxTurn, 0.0, 1.0);

        double currentHeading = follower.getHeading();
        double headingError = Math.IEEEremainder(targetHeadingRad - currentHeading, 2 * Math.PI);

        if (headingLock) {
            headingPid.setCoefficients(new PIDFCoefficients(LockConfig.kP, LockConfig.kI, LockConfig.kD, 0.0));
            if (Math.abs(headingError) < toleranceRad) {
                headingPid.reset();
                headingCommandCw = 0.0;
            } else {
                headingPid.updateError(headingError);
                double pidOutputCcw = Range.clip(headingPid.run(), -maxTurn, maxTurn);
                headingCommandCw = -pidOutputCcw;
            }
        } else {
            headingPid.reset();
        }

        follower.setTeleOpDrive(forward, strafeLeft, headingCommandCw, true);

        double headingErrorDeg = Math.toDegrees(headingError);
        telemetry.addData("Heading lock", headingLock ? "ON" : "OFF");
        telemetry.addData("Target heading (deg)", LockConfig.targetHeadingDeg);
        telemetry.addData("Current heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("Heading error (deg)", headingErrorDeg);
        telemetry.addData("Command turn (cw)", headingCommandCw);
        telemetry.update();

        if (panelsTelemetry != null) {
            panelsTelemetry.debug(String.format("HeadingLock/lock=%s", headingLock));
            panelsTelemetry.debug(String.format("HeadingLock/currentDeg=%.2f", Math.toDegrees(currentHeading)));
            panelsTelemetry.debug(String.format("HeadingLock/targetDeg=%.2f", LockConfig.targetHeadingDeg));
            panelsTelemetry.debug(String.format("HeadingLock/errorDeg=%.2f", headingErrorDeg));
            panelsTelemetry.debug(String.format("HeadingLock/commandCw=%.2f", headingCommandCw));
            panelsTelemetry.update(telemetry);
        }
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }
    }
}

