package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;

/**
 * Static holder for the Pedro Follower instance.
 * This allows PedroComponent to access the follower that was created
 * AFTER the robot name was set, ensuring correct robot-specific configs.
 */
public class FollowerHolder {
    private static Follower follower = null;

    /**
     * Set the follower instance (called from DriveSubsystem.attachFollower())
     */
    public static void setFollower(Follower f) {
        follower = f;
    }

    /**
     * Get the follower instance (used by PedroComponent)
     */
    public static Follower getFollower() {
        return follower;
    }

    /**
     * Clear the follower reference (called when OpMode stops)
     */
    public static void clear() {
        follower = null;
    }
}
