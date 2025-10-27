package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


/**
 * TelemetryPublisher centralises the construction of telemetry packets for the
 * FTC Dashboard and AdvantageScope.  It publishes drive and shooter state
 * variables on every loop so that they can be graphed or visualised.
 *
 * <p>The FTC Dashboard library will automatically aggregate packets from
 * multiple calls in the same loop, so publishing from both drive and
 * shooter in one OpMode is safe.</p>
 */
public class TelemetryPublisher {

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final PsiKitAdapter logger;

    /**
     * Creates a TelemetryPublisher.  If a {@link PsiKitAdapter} is provided the
     * publisher will log all values to a CSV file in addition to sending them
     * to the FTC Dashboard.  Passing {@code null} disables file logging.
     *
     * @param logger an optional logger for persistent recordings
     */
    public TelemetryPublisher(PsiKitAdapter logger) {
        this.logger = logger;
    }

    /**
     * Convenience constructor that does not record to a file.  Only live
     * dashboard telemetry will be produced.
     */
    public TelemetryPublisher() {
        this(null);
    }

    /**
     * Publish drivetrain telemetry to the dashboard.  Includes joystick inputs,
     * the current pose estimate and slow mode state.
     *
     * @param drive    the drive subsystem to read the pose from
     * @param lx       left stick x
     * @param ly       left stick y
     * @param rx       right stick x (rotation)
     * @param slowMode whether slow mode is active
     */
    public void publishDrive(DriveSubsystem drive,
                             double lx, double ly, double rx,
                             boolean slowMode) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("lx", lx);
        p.put("ly", ly);
        p.put("rx", rx);
        p.put("drive_mode", drive.getDriveMode().name());
        p.put("slow_mode", slowMode);

        double lfPower = drive.getLfPower();
        double rfPower = drive.getRfPower();
        double lbPower = drive.getLbPower();
        double rbPower = drive.getRbPower();
        double lfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLfVelocityTicksPerSec()));
        double rfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRfVelocityTicksPerSec()));
        double lbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLbVelocityTicksPerSec()));
        double rbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRbVelocityTicksPerSec()));
        p.put("lf_power", lfPower);
        p.put("rf_power", rfPower);
        p.put("lb_power", lbPower);
        p.put("rb_power", rbPower);
        p.put("lf_vel_ips", lfVelIps);
        p.put("rf_vel_ips", rfVelIps);
        p.put("lb_vel_ips", lbVelIps);
        p.put("rb_vel_ips", rbVelIps);

        Pose2D pose = drive.getPose();
        double xIn = pose.getX(DistanceUnit.INCH);
        double yIn = pose.getY(DistanceUnit.INCH);
        double headingRad = pose.getHeading(AngleUnit.RADIANS);
        p.put("x_in", xIn);
        p.put("y_in", yIn);
        p.put("heading_deg", Math.toDegrees(headingRad));

//        var field = p.fieldOverlay();
//        field.setStroke("yellow");
//        field.strokeCircle(xIn, yIn, 1.5);
//        double arrowLen = 6.0;
//        double arrowX = xIn + arrowLen * Math.cos(headingRad);
//        double arrowY = yIn + arrowLen * Math.sin(headingRad);
//        field.strokeLine(xIn, yIn, arrowX, arrowY);
//        double leftX = arrowX - 2 * Math.cos(headingRad - Math.PI / 6);
//        double leftY = arrowY - 2 * Math.sin(headingRad - Math.PI / 6);
//        double rightX = arrowX - 2 * Math.cos(headingRad + Math.PI / 6);
//        double rightY = arrowY - 2 * Math.sin(headingRad + Math.PI / 6);
//        field.strokeLine(arrowX, arrowY, leftX, leftY);
//        field.strokeLine(arrowX, arrowY, rightX, rightY);

        // Send to the live dashboard
        dash.sendTelemetryPacket(p);

        // Record to persistent log if enabled
        if (logger != null) {
            logger.recordNumber("drive_lx", lx);
            logger.recordNumber("drive_ly", ly);
            logger.recordNumber("drive_rx", rx);
            logger.recordBoolean("drive_slowMode", slowMode);
            logger.recordString("drive_mode", drive.getDriveMode().name());
            logger.recordNumber("drive_x_in", pose.getX(DistanceUnit.INCH));
            logger.recordNumber("drive_y_in", pose.getY(DistanceUnit.INCH));
            logger.recordNumber("drive_heading_deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));
            logger.recordNumber("drive_lf_power", lfPower);
            logger.recordNumber("drive_rf_power", rfPower);
            logger.recordNumber("drive_lb_power", lbPower);
            logger.recordNumber("drive_rb_power", rbPower);
            logger.recordNumber("drive_lf_vel_ips", lfVelIps);
            logger.recordNumber("drive_rf_vel_ips", rfVelIps);
            logger.recordNumber("drive_lb_vel_ips", lbVelIps);
            logger.recordNumber("drive_rb_vel_ips", rbVelIps);
        }
    }

    /**
     * Publish shooter telemetry.  Includes the target RPM, measured RPM,
     * control error and applied power.
     *
     * @param targetRpm desired wheel speed
     * @param rpm       current measured speed
     * @param power     last power sent to the motors
     * @param error     current error (target – measured)
     */
    public void publishShooter(double targetRpm, double rpm, double power, double error) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("shooter_target_rpm", targetRpm);
        p.put("shooter_rpm", rpm);
        p.put("shooter_err", error);
        p.put("shooter_power", power);
        dash.sendTelemetryPacket(p);
        if (logger != null) {
            logger.recordNumber("shooter_target_rpm", targetRpm);
            logger.recordNumber("shooter_rpm", rpm);
            logger.recordNumber("shooter_err", error);
            logger.recordNumber("shooter_power", power);
            // Flush immediately to minimise data loss if the robot shuts down
            logger.flush();
        }
    }

    /** @deprecated Use {@link #publishShooter(double, double, double, double)} instead. */
    @Deprecated
    public void publishFlywheel(double targetRpm, double rpm, double power, double error) {
        publishShooter(targetRpm, rpm, power, error);
    }
}
