package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.MatchOperatorControls;
import org.firstinspires.ftc.teamcode.bindings.OperatorControls;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.RobotMode;

@TeleOp(name = "Decode Teleop", group = "TeleOp")
public class DecodeTeleOp extends DecodeTeleOpBase {

    @Override
    protected RobotMode getDesiredRobotMode() {
        return RobotMode.MATCH;
    }

    @Override
    protected OperatorControls buildOperatorControls(GamepadEx operatorPad,
                                                     Robot robot,
                                                     LauncherCoordinator launcherCoordinator,
                                                     RobotMode mode) {
        return new MatchOperatorControls(operatorPad, robot, launcherCoordinator);
    }

    @Override
    protected void configureForMode(Robot robot, LauncherCoordinator coordinator, RobotMode mode) {
        coordinator.enableAutoSpin(false);
    }
}
