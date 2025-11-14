package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Drive subsystem telemetry data.
 * Organized with clear four-motor hierarchy.
 */
public class DriveTelemetryData {
    // Drive mode
    public final String driveMode;
    public final boolean aimMode;
    public final boolean slowMode;
    public final double commandTurn;

    // Driver inputs
    public final double requestX;
    public final double requestY;
    public final double requestRot;

    // Motor data (organized by motor)
    public final MotorData leftFront;
    public final MotorData rightFront;
    public final MotorData leftBack;
    public final MotorData rightBack;

    public DriveTelemetryData(
            String driveMode,
            boolean aimMode,
            boolean slowMode,
            double commandTurn,
            double requestX,
            double requestY,
            double requestRot,
            MotorData leftFront,
            MotorData rightFront,
            MotorData leftBack,
            MotorData rightBack
    ) {
        this.driveMode = driveMode;
        this.aimMode = aimMode;
        this.slowMode = slowMode;
        this.commandTurn = commandTurn;
        this.requestX = requestX;
        this.requestY = requestY;
        this.requestRot = requestRot;
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    /**
     * Individual motor telemetry data.
     */
    public static class MotorData {
        public final double power;
        public final double velocityIps;

        public MotorData(double power, double velocityTicksPerSec) {
            this.power = power;
            this.velocityIps = DistanceUnit.METER.toInches(
                    Constants.Speed.ticksPerSecToMps(velocityTicksPerSec)
            );
        }
    }

    public static DriveTelemetryData capture(DriveSubsystem drive, DriverBindings.DriveRequest request) {
        double requestX = request != null ? request.fieldX : 0.0;
        double requestY = request != null ? request.fieldY : 0.0;
        double requestRot = request != null ? request.rotation : 0.0;
        boolean slowMode = request != null && request.slowMode;
        boolean aimMode = request != null && request.aimMode;

        MotorData lf = new MotorData(drive.getLfPower(), drive.getLfVelocityTicksPerSec());
        MotorData rf = new MotorData(drive.getRfPower(), drive.getRfVelocityTicksPerSec());
        MotorData lb = new MotorData(drive.getLbPower(), drive.getLbVelocityTicksPerSec());
        MotorData rb = new MotorData(drive.getRbPower(), drive.getRbVelocityTicksPerSec());

        return new DriveTelemetryData(
                drive.getDriveMode().name(),
                aimMode,
                slowMode,
                drive.getLastCommandTurn(),
                requestX,
                requestY,
                requestRot,
                lf, rf, lb, rb
        );
    }
}
