package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.bindings.DebugOperatorControls;
import org.firstinspires.ftc.teamcode.bindings.OperatorControls;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.RobotMode;

@TeleOp(name = "Decode Teleop (Debug)", group = "TeleOp")
public class DecodeTeleOpDebug extends DecodeTeleOpBase {

    @Override
    protected RobotMode getDesiredRobotMode() {
        return RobotMode.DEBUG;
    }

    @Override
    protected OperatorControls buildOperatorControls(GamepadEx operatorPad,
                                                     Robot robot,
                                                     LauncherCoordinator launcherCoordinator,
                                                     RobotMode mode) {
        return new DebugOperatorControls(operatorPad, robot, launcherCoordinator);
    }
}
