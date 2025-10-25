package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Placeholder shooter that simply tracks whether we intend to spin up and logs how many
 * shots have been "fired". No real hardware interactions occur.
 */
@Config
public class Shooter {

    public static double spinUpTimeMs = 0.0;

    private final ElapsedTime spinTimer = new ElapsedTime();

    private boolean spinUpRequested = false;
    private boolean atSpeed = true;
    private int completedShots = 0;
    private double targetHeadingRad = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        // Real implementation would bind motors/servos here.
    }

    public void update() {
        if (spinUpRequested && !atSpeed && spinTimer.milliseconds() >= spinUpTimeMs) {
            atSpeed = true;
        }
    }

    public void setSpinUp(boolean enabled) {
        if (enabled) {
            if (!spinUpRequested) {
                spinUpRequested = true;
                spinTimer.reset();
                if (spinUpTimeMs <= 0.0) {
                    atSpeed = true;
                } else {
                    atSpeed = false;
                }
            }
        } else {
            spinUpRequested = false;
            atSpeed = false;
        }
    }

    public boolean atSpeed() {
        return atSpeed;
    }

    public void aimToTarget(double headingRad) {
        targetHeadingRad = headingRad;
    }

    public double getTargetHeading() {
        return targetHeadingRad;
    }

    public void burst(int count) {
        if (count <= 0) {
            return;
        }
        if (!spinUpRequested) {
            setSpinUp(true);
        }
        if (atSpeed) {
            completedShots += count;
        }
    }

    public void shoot(int count) {
        burst(count);
    }

    public boolean isBursting() {
        return false;
    }

    public boolean isBusy() {
        return !atSpeed && spinUpRequested;
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

    public void aim() {
        // Provided for compatibility with older autos that call aim().
    }
}
