package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class RelocalizeFromVisionCommand extends Command {

    private final Robot robot;
    private final int maxAttempts;

    private int attempts;
    private boolean success;

    public RelocalizeFromVisionCommand(Robot robot, int maxAttempts) {
        this.robot = robot;
        this.maxAttempts = maxAttempts;
    }

    @Override
    public void start() {
        attempts = 0;
        success = false;
    }

    @Override
    public void update() {
        if (robot.vision.hasValidTag()) {
            success = robot.drive.forceRelocalizeFromVision();
            if (success) {
                // Fast exit
                attempts = maxAttempts;
            }
        }

        RobotState.packet.put("Auto/Relocalized", success);

        attempts++;
    }

    @Override
    public boolean isDone() {
        return attempts >= maxAttempts || success;
    }
}
