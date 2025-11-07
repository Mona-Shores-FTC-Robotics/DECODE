package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Diagnostic OpMode that queues +/- turn commands with Pedro's holdPoint and lets you tune inputs live.
 * A rotates left by {@link TurnConfig#turnDegrees}, B rotates right, and X aborts any active turn.
 */
@Configurable
@TeleOp(name = "Heading Turn Diagnostic", group = "Debug")
public class HeadingTurnDiagnostic extends OpMode {

    @Configurable
    public static class TurnConfig {
        public static double turnDegrees = 90.0;
        public static double manualForwardScale = 1.0;
        public static double manualStrafeScale = 1.0;
        public static double manualTurnScale = 1.0;
    }

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;

    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(TurnConfig.class);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.a && !prevA) {
            turn(TurnConfig.turnDegrees);
        }
        if (gamepad1.b && !prevB) {
            turn(-TurnConfig.turnDegrees);
        }
        if (gamepad1.x && !prevX) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;

        if (!follower.isBusy()) {
            double forward = Range.clip(-gamepad1.left_stick_y * TurnConfig.manualForwardScale, -1.0, 1.0);
            double strafeLeft = Range.clip(-gamepad1.left_stick_x * TurnConfig.manualStrafeScale, -1.0, 1.0);
            double turnCw = Range.clip(-gamepad1.right_stick_x * TurnConfig.manualTurnScale, -1.0, 1.0);
            follower.setTeleOpDrive(forward, strafeLeft, turnCw, true);
        }

        Pose pose = follower.getPose();
        double headingDeg = pose == null ? Double.NaN : Math.toDegrees(pose.getHeading());

        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Heading (deg)", Double.isNaN(headingDeg) ? "(unknown)" : headingDeg);
        telemetry.addData("Turn step (deg)", TurnConfig.turnDegrees);
        telemetry.update();

        if (panelsTelemetry != null) {
            panelsTelemetry.debug(String.format("HeadingTurn/Busy=%s", follower.isBusy()));
            panelsTelemetry.debug(String.format("HeadingTurn/HeadingDeg=%.2f", headingDeg));
            panelsTelemetry.debug(String.format("HeadingTurn/TurnStepDeg=%.1f", TurnConfig.turnDegrees));
            panelsTelemetry.update(telemetry);
        }
    }

    private void turn(double degreesLeft) {
        Pose current = follower.getPose();
        if (current == null) {
            current = new Pose();
        }
        Pose target = new Pose(
                current.getX(),
                current.getY(),
                current.getHeading() + Math.toRadians(degreesLeft)
        );
        follower.holdPoint(target);
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
            follower.startTeleopDrive(true);
            follower.setTeleOpDrive(0, 0, 0, true);
        }
    }
}

