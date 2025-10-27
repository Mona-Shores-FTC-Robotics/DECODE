package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Coordinated shooter subsystem that manages the flywheel and the gate servo. The subsystem
 * exposes a queue based interface so callers can request bursts of rings while the class handles
 * the timing required to feed them cleanly.
 */
@Config
public class ShooterSubsystem implements Subsystem {

    /** Dashboard-editable presets for common shot distances. */
    @Config
    public static class Presets {
        public static double CLOSE_RPM = 3200.0;
        public static double FAR_RPM = 4200.0;
        public static double WING_RPM = 3600.0;
        public static int DEFAULT_CYCLE_SHOTS = 3;
    }

    /** Timing constants governing how long the feeder remains open between shots. */
    @Config
    public static class FeedTiming {
        public static double feedDurationMs = 220.0;
        public static double recoveryDelayMs = 180.0;
    }


    private final ElapsedTime cycleTimer = new ElapsedTime();

    private boolean spinUpRequested = false;
    private boolean feeding = false;
    private double requestedRpm = Presets.FAR_RPM;
    private double targetHeadingRad = 0.0;
    private int queuedShots = 0;
    private int completedShots = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
         cycleTimer.reset();
    }

    public void setTargetRpm(double rpm) {
        requestedRpm = Math.max(0.0, rpm);
        if (spinUpRequested) {
            flywheel.setTargetRpm(requestedRpm);
        }
    }

    public double getTargetRpm() {
        return flywheel.getTargetRpm();
    }

    public double getCurrentRpm() {
        return flywheel.getCurrentRpm();
    }

    public double getLastPower() {
        return flywheel.getLastPower();
    }

    public boolean atTarget() {
        return flywheel.atTarget();
    }

    public boolean isSpinUpRequested() {
        return spinUpRequested;
    }

    public int getQueuedShots() {
        return queuedShots;
    }

    public void toggleSpinUp() {
        setSpinUp(!spinUpRequested);
    }

    public void setSpinUp(boolean enabled) {
        spinUpRequested = enabled;
        if (enabled) {
            flywheel.setTargetRpm(requestedRpm);
        } else {
            flywheel.stop();
            feeding = false;
            queuedShots = 0;
            gate.close();
        }
    }

    public void selectCloseShot() {
        selectPreset(Presets.CLOSE_RPM);
    }

    public void selectFarShot() {
        selectPreset(Presets.FAR_RPM);
    }

    public void selectPreset(double rpm) {
        setTargetRpm(rpm);
    }

    public void queueDefaultCycle() {
        queueShots(Presets.DEFAULT_CYCLE_SHOTS);
    }

    public void queueShots(int count) {
        if (count <= 0) {
            return;
        }
        queuedShots += count;
        if (!spinUpRequested) {
            setSpinUp(true);
        }
    }

    public void shootLeft() {
        selectCloseShot();
        queueShots(1);
    }

    public void shootMiddle() {
        selectPreset(Presets.WING_RPM);
        queueShots(1);
    }

    public void shootRight() {
        selectFarShot();
        queueShots(1);
    }

    public void shootAll() {
        selectFarShot();
        queueShots(3);
    }

    public void shoot(int count) {
        queueShots(count);
    }

    public void burst(int count) {
        queueShots(count);
    }

    public void update() {
        periodic();
    }

    public void aimToTarget(double headingRad) {
        targetHeadingRad = headingRad;
    }

    public void aim() {
        // Provided for compatibility with older autonomous routines.
    }

    public double getTargetHeading() {
        return targetHeadingRad;
    }

    public boolean atSpeed() {
        return spinUpRequested && flywheel.atTarget();
    }

    public boolean isBursting() {
        return feeding || queuedShots > 0;
    }

    public boolean isBusy() {
        return (spinUpRequested && !flywheel.atTarget()) || feeding;
    }

    public int takeCompletedShots() {
        int fired = completedShots;
        completedShots = 0;
        return fired;
    }

    public void stop() {
        setSpinUp(false);
        completedShots = 0;
    }

    @Override
    public void initialize() {
        flywheel.initialize();
        gate.close();
        cycleTimer.reset();
    }

    @Override
    public void periodic() {
        flywheel.periodic();
        gate.update();

        if (!spinUpRequested) {
            return;
        }

        if (feeding) {
            if (!gate.isBusy() && !gate.isOpen()) {
                feeding = false;
                if (queuedShots > 0) {
                    queuedShots--;
                }
                completedShots++;
                cycleTimer.reset();
            }
            return;
        }

        if (queuedShots > 0 && flywheel.atTarget()
                && cycleTimer.milliseconds() >= FeedTiming.recoveryDelayMs) {
            gate.extendForMs(FeedTiming.feedDurationMs);
            feeding = true;
        }
    }

    private static class FlywheelSubsystem {
        private static final double SPOOL_RATE = 0.2;

        private double targetRpm = 0.0;
        private double currentRpm = 0.0;
        private double lastPower = 0.0;

        FlywheelSubsystem(HardwareMap hardwareMap) {
            // Real implementation would bind the flywheel motor.
        }

        void setTargetRpm(double rpm) {
            targetRpm = Math.max(0.0, rpm);
            lastPower = targetRpm > 0.0 ? 1.0 : 0.0;
        }

        double getTargetRpm() {
            return targetRpm;
        }

        double getCurrentRpm() {
            return currentRpm;
        }

        double getLastPower() {
            return lastPower;
        }

        boolean atTarget() {
            return Math.abs(currentRpm - targetRpm) <= 75.0;
        }

        void stop() {
            targetRpm = 0.0;
            lastPower = 0.0;
        }

        void initialize() {
            currentRpm = 0.0;
            targetRpm = 0.0;
            lastPower = 0.0;
        }

        void periodic() {
            currentRpm += (targetRpm - currentRpm) * SPOOL_RATE;
        }
    }

    private static class Gate {
        private final ElapsedTime timer = new ElapsedTime();
        private double durationMs = 0.0;
        private boolean open = false;

        Gate() {
            timer.reset();
        }

        void extendForMs(double ms) {
            durationMs = Math.max(0.0, ms);
            open = true;
            timer.reset();
        }

        void close() {
            open = false;
        }

        boolean isOpen() {
            return open;
        }

        boolean isBusy() {
            return open;
        }

        void update() {
            if (open && timer.milliseconds() >= durationMs) {
                open = false;
            }
        }
    }
}
