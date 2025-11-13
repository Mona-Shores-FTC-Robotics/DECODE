package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * TelemetryPublisher centralises the construction of telemetry lines for FTControl Panels.
 * It publishes drive and launcher state values each loop so they can be graphed or inspected.
 */
public class TelemetryPublisher {

    private TelemetryManager panelsTelemetry;

    public TelemetryPublisher(TelemetryManager panelsTelemetry) {
        this.panelsTelemetry = panelsTelemetry;
    }

    public TelemetryPublisher() {
        this(null);
    }

    public void setTelemetryManager(TelemetryManager panelsTelemetry) {
        this.panelsTelemetry = panelsTelemetry;
    }

    /**
     * Publish drivetrain telemetry to Panels. Includes joystick inputs, pose, and motor stats.
     */
    public void publishDrive(DriveSubsystem drive,
                             double lx, double ly, double rx,
                             boolean slowMode,
                             boolean aimMode
                             ) {
        double lfPower = drive.getLfPower();
        double rfPower = drive.getRfPower();
        double lbPower = drive.getLbPower();
        double rbPower = drive.getRbPower();
        double commandTurn = drive.getLastCommandTurn();
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
            panelsTelemetry.debug("drive/aimMode", aimMode);
            panelsTelemetry.debug("drive/commandTurn", commandTurn);
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
    }

    /**
     * Publish launcher telemetry. Includes target RPM, measured RPM, error, and power.
     */
    public void publishLauncher(double targetRpm, double rpm, double power, double error) {
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("launcher/targetRpm", targetRpm);
            panelsTelemetry.debug("launcher/rpm", rpm);
            panelsTelemetry.debug("launcher/error", error);
            panelsTelemetry.debug("launcher/power", power);
        }
    }

    /** @deprecated Use {@link #publishLauncher(double, double, double, double)} instead. */
    @Deprecated
    public void publishFlywheel(double targetRpm, double rpm, double power, double error) {
        publishLauncher(targetRpm, rpm, power, error);
    }
}
