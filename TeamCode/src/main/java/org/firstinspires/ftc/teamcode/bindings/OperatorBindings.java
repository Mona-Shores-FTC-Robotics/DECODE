package org.firstinspires.ftc.teamcode.bindings;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Operator bindings for secondary gamepad. Handles intake toggles and shooter requests.
 */
public class OperatorBindings {

    @Configurable
    public static class OperatorConfig {
        public static double burstSpacingMs = 200.0;
    }

    private final Robot robot;
    private final LauncherCoordinator launcherCoordinator;
    private final Button intakeToggle;
    private final Button spinUpToggle;
    private final Button shootLeft;
    private final Button shootMiddle;
    private final Button shootRight;
    private final Button shootAll;

    private boolean intakeActive = false;

    public OperatorBindings(GamepadEx operator, Robot robot) {
        this(operator, robot, null);
    }

    public OperatorBindings(GamepadEx operator, Robot robot, LauncherCoordinator launcherCoordinator) {
        this.robot = robot;
        this.launcherCoordinator = launcherCoordinator;

        intakeToggle = operator.rightBumper();
        spinUpToggle = operator.leftBumper();
        shootLeft = operator.x();
        shootMiddle = operator.y();
        shootRight = operator.b();
        shootAll = operator.a();

        intakeToggle.whenBecomesTrue(this::toggleIntake);
        spinUpToggle.whenBecomesTrue(robot.shooter::toggleSpinUp);
        shootLeft.whenBecomesTrue(() -> fireLane(LauncherLane.LEFT));
        shootMiddle.whenBecomesTrue(() -> fireLane(LauncherLane.CENTER));
        shootRight.whenBecomesTrue(() -> fireLane(LauncherLane.RIGHT));
        shootAll.whenBecomesTrue(this::fireBurst);
    }

    private void toggleIntake() {
        intakeActive = !intakeActive;
        if (intakeActive) {
            robot.intake.requestIntake();
            robot.lighting.indicateBusy();
        } else {
            robot.intake.stop();
            robot.lighting.indicateIdle();
        }
    }

    private void fireLane(LauncherLane lane) {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestKick(lane);
            return;
        }
        switch (lane) {
            case LEFT:
                robot.shooter.shootLeft();
                break;
            case CENTER:
                robot.shooter.shootMiddle();
                break;
            case RIGHT:
            default:
                robot.shooter.shootRight();
                break;
        }
    }

    private void fireBurst() {
        if (launcherCoordinator != null) {
            launcherCoordinator.requestBurst(OperatorConfig.burstSpacingMs);
        } else {
            robot.shooter.shootAll();
        }
    }

    public void reset() {
        intakeActive = false;
        robot.intake.stop();
        robot.lighting.indicateIdle();
    }
}
