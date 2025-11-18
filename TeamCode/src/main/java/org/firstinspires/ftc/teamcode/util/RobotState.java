// org.firstinspires.ftc.teamcode.util.RobotState
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.concurrent.atomic.AtomicReference;

public class RobotState {
    private static final AtomicReference<Pose> handoffPose = new AtomicReference<>(null);
    private static final AtomicReference<Alliance> alliance = new AtomicReference<>(Alliance.UNKNOWN);

    public static void setHandoffPose(Pose p) { handoffPose.set(p); }
    public static Pose takeHandoffPose()      { return handoffPose.getAndSet(null); } // consume-once

    public static void setAlliance(Alliance value) { alliance.set(value == null ? Alliance.UNKNOWN : value); }
    public static Alliance getAlliance() { return alliance.get(); }
    public static TelemetryPacket packet = new TelemetryPacket();

}
