package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * TelemetryPublisher centralises the construction of telemetry lines for FTControl Panels
 * (and optional PsiKit logging).  It publishes drive and shooter state values each loop so
 * they can be graphed or inspected without keeping the FTC Dashboard dependency.
 */
public class TelemetryPublisher {

    private TelemetryManager panelsTelemetry;
    private final PsiKitAdapter logger;

    public TelemetryPublisher(TelemetryManager panelsTelemetry, PsiKitAdapter logger) {
        this.panelsTelemetry = panelsTelemetry;
        this.logger = logger;
    }

    public TelemetryPublisher(TelemetryManager panelsTelemetry) {
        this(panelsTelemetry, null);
    }

    public TelemetryPublisher() {
        this(null, null);
    }

    public void setTelemetryManager(TelemetryManager panelsTelemetry) {
        this.panelsTelemetry = panelsTelemetry;
    }

    /**
     * Publish drivetrain telemetry to Panels. Includes joystick inputs, pose, and motor stats.
     */
    public void publishDrive(DriveSubsystem drive,
                             double lx, double ly, double rx,
                             boolean slowMode) {
        double lfPower = drive.getLfPower();
        double rfPower = drive.getRfPower();
        double lbPower = drive.getLbPower();
        double rbPower = drive.getRbPower();
        double lfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLfVelocityTicksPerSec()));
        double rfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRfVelocityTicksPerSec()));
        double lbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLbVelocityTicksPerSec()));
        double rbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRbVelocityTicksPerSec()));
        Pose2D pose = drive.getPose();
        if (pose == null) {
            pose = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0);
        }

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("drive/lx", lx);
            panelsTelemetry.debug("drive/ly", ly);
            panelsTelemetry.debug("drive/rx", rx);
            panelsTelemetry.debug("drive/mode", drive.getDriveMode().name());
            panelsTelemetry.debug("drive/slowMode", slowMode);
            panelsTelemetry.debug("drive/lfPower", lfPower);
            panelsTelemetry.debug("drive/rfPower", rfPower);
            panelsTelemetry.debug("drive/lbPower", lbPower);
            panelsTelemetry.debug("drive/rbPower", rbPower);
            panelsTelemetry.debug("drive/lfVelIps", lfVelIps);
            panelsTelemetry.debug("drive/rfVelIps", rfVelIps);
            panelsTelemetry.debug("drive/lbVelIps", lbVelIps);
            panelsTelemetry.debug("drive/rbVelIps", rbVelIps);
            panelsTelemetry.debug("drive/xIn", pose.getX(DistanceUnit.INCH));
            panelsTelemetry.debug("drive/yIn", pose.getY(DistanceUnit.INCH));
            panelsTelemetry.debug("drive/headingDeg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));
        }

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
     * Publish shooter telemetry. Includes target RPM, measured RPM, error, and power.
     */
    public void publishShooter(double targetRpm, double rpm, double power, double error) {
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("shooter/targetRpm", targetRpm);
            panelsTelemetry.debug("shooter/rpm", rpm);
            panelsTelemetry.debug("shooter/error", error);
            panelsTelemetry.debug("shooter/power", power);
        }
        if (logger != null) {
            logger.recordNumber("shooter_target_rpm", targetRpm);
            logger.recordNumber("shooter_rpm", rpm);
            logger.recordNumber("shooter_err", error);
            logger.recordNumber("shooter_power", power);
            logger.flush();
        }
    }

    /** @deprecated Use {@link #publishShooter(double, double, double, double)} instead. */
    @Deprecated
    public void publishFlywheel(double targetRpm, double rpm, double power, double error) {
        publishShooter(targetRpm, rpm, power, error);
    }
}
