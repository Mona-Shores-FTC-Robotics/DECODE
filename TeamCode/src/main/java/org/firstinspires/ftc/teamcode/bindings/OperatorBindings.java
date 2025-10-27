package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Operator bindings for secondary gamepad. Handles intake toggles and shooter requests.
 */
public class OperatorBindings {

    private final Robot robot;
    private final Button intakeToggle;
    private final Button spinUpToggle;
    private final Button shootLeft;
    private final Button shootMiddle;
    private final Button shootRight;
    private final Button shootAll;

    private boolean intakeActive = false;

    public OperatorBindings(GamepadEx operator, Robot robot) {
        this.robot = robot;

        intakeToggle = operator.rightBumper();
        spinUpToggle = operator.leftBumper();
        shootLeft = operator.x();
        shootMiddle = operator.y();
        shootRight = operator.b();
        shootAll = operator.a();

        intakeToggle.whenBecomesTrue(this::toggleIntake);
        spinUpToggle.whenBecomesTrue(robot.shooter::toggleSpinUp);
        shootLeft.whenBecomesTrue(robot.shooter::shootLeft);
        shootMiddle.whenBecomesTrue(robot.shooter::shootMiddle);
        shootRight.whenBecomesTrue(robot.shooter::shootRight);
        shootAll.whenBecomesTrue(robot.shooter::shootAll);
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

    public void reset() {
        intakeActive = false;
        robot.intake.stop();
        robot.lighting.indicateIdle();
    }
}
