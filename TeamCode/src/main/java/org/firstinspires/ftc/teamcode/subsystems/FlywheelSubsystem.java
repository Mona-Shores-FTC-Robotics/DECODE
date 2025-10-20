package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Simple single-motor flywheel controller that targets RPM using a feedforward + bang-bang blend.
 */
public class FlywheelSubsystem implements Subsystem {

    private static final String MOTOR_NAME = "flywheel";
    private static final double RPM_EPSILON = 75.0;

    private final DcMotorEx motor;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double lastTicks = 0.0;
    private double measuredRpm = 0.0;
    private double targetRpm = 0.0;
    private double lastPower = 0.0;
    private boolean atTarget = false;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        loopTimer.reset();
        lastTicks = motor.getCurrentPosition();
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        return measuredRpm;
    }

    /** Legacy name retained for existing OpModes. */
    public double getRpm() {
        return getCurrentRpm();
    }

    public double getLastPower() {
        return lastPower;
    }

    public boolean atTarget() {
        return atTarget;
    }

    public void stop() {
        setTargetRpm(0.0);
        motor.setPower(0.0);
        lastPower = 0.0;
    }

    @Override
    public void periodic() {
        updateRpmEstimate();
        applyControl();
    }

    private void updateRpmEstimate() {
        double dt = Math.max(1e-3, loopTimer.seconds());
        loopTimer.reset();

        double currentTicks = motor.getCurrentPosition();
        double deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;

        double revolutions = deltaTicks / Constants.TICKS_PER_REV;
        double rpm = revolutions / dt * 60.0;

        // Light exponential smoothing keeps telemetry stable without masking large errors.
        measuredRpm = 0.75 * measuredRpm + 0.25 * rpm;
    }

    private void applyControl() {
        double error = targetRpm - measuredRpm;
        double power;

        if (targetRpm <= 0.0) {
            power = 0.0;
        } else if (measuredRpm < targetRpm - Constants.FLY_BB_THRESH_RPM) {
            power = Constants.FLY_MAX_POWER;
        } else if (measuredRpm > targetRpm + Constants.FLY_BB_THRESH_RPM) {
            power = 0.0;
        } else {
            double ff = Constants.FLY_KF * targetRpm;
            double p = Constants.FLY_KP * error;
            power = ff + p;
        }

        power = Range.clip(power, 0.0, Constants.FLY_MAX_POWER);
        motor.setPower(power);
        lastPower = power;
        atTarget = Math.abs(error) <= RPM_EPSILON;
    }
}
