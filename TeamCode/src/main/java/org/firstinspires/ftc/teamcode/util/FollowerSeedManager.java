package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;

import java.util.Optional;

public final class FollowerSeedManager {

    private FollowerSeedManager() {}

    /**
     * Gets the best available starting pose:
     * 1) Handoff from auto → teleop 
     * 2) Limelight MT1 / MT2 vision pose
     * 3) Fallback origin
     */
    public static Pose getStartingPose(VisionSubsystemLimelight vision) {

        // 1) If Auto handed off a pose (teleop start), use that.
        Pose handoff = RobotState.takeHandoffPose();
        if (handoff != null) {
            RobotState.putPose("Seed/Handoff", handoff);
            return handoff;
        }

        // 2) If Limelight has a valid pose, use it.
        Optional<Pose> visionPose = vision.getRobotPoseFromTagPedro();
        if (visionPose.isPresent()) {
            Pose pedro = visionPose.get();
            RobotState.putPose("Seed/VisionPedro", pedro);
            RobotState.putPose("Seed/VisionFTC", vision.getRobotPoseFromTagFtc().orElse(null));
            return pedro;
        }

        // 3) Fallback to origin facing up-field
        Pose fallback = new Pose(0.0, 0.0, Math.toRadians(90));
        RobotState.putPose("Seed/Fallback", fallback);
        return fallback;
    }
}
