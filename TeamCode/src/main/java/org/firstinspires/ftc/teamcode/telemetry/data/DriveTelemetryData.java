package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Drive subsystem telemetry data.
 * Shows commanded robot behavior (after processing) and actual motor outputs.
 * <p>
 * For raw driver inputs, see GamepadTelemetryData.
 * </p>
 */
public class DriveTelemetryData {
    // Drive mode and state
    public final String driveMode;
    public final boolean aimMode;
    public final boolean slowMode;

    // Commanded values (after processing)
    public final double commandTurn;

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
            MotorData leftFront,
            MotorData rightFront,
            MotorData leftBack,
            MotorData rightBack
    ) {
        this.driveMode = driveMode;
        this.aimMode = aimMode;
        this.slowMode = slowMode;
        this.commandTurn = commandTurn;
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
                lf, rf, lb, rb
        );
    }
}
