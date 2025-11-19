// org.firstinspires.ftc.teamcode.util.RobotState
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.MotifPattern;

import java.util.concurrent.atomic.AtomicReference;

public class RobotState {
    private static final AtomicReference<Pose> handoffPose = new AtomicReference<>(null);
    private static final AtomicReference<Alliance> alliance = new AtomicReference<>(Alliance.UNKNOWN);
    private static final AtomicReference<MotifPattern> motif = new AtomicReference<>(MotifPattern.UNKNOWN);
    private static final AtomicReference<LauncherMode> launcherMode = new AtomicReference<>(LauncherMode.THROUGHPUT);
    private static final AtomicReference<Integer> motifTail = new AtomicReference<>(0);

    public static void setHandoffPose(Pose p) { handoffPose.set(p); }
    public static Pose takeHandoffPose()      { return handoffPose.getAndSet(null); } // consume-once

    public static void setAlliance(Alliance value) { alliance.set(value == null ? Alliance.UNKNOWN : value); }
    public static Alliance getAlliance() { return alliance.get(); }

    public static MotifPattern getMotif() { return motif.get(); }
    public static void setMotif(MotifPattern pattern) { motif.set(pattern == null ? MotifPattern.UNKNOWN : pattern); }

    public static LauncherMode getLauncherMode() { return launcherMode.get(); }
    public static void setLauncherMode(LauncherMode mode) { launcherMode.set(mode == null ? LauncherMode.THROUGHPUT : mode); }

    /**
     * Gets the current motif tail value (0, 1, or 2).
     * This is manually set by the operator based on visual count of artifacts in field ramp.
     */
    public static int getMotifTail() {
        Integer value = motifTail.get();
        return value == null ? 0 : value;
    }

    /**
     * Sets the motif tail value (0, 1, or 2).
     * Automatically clamps to valid range.
     */
    public static void setMotifTail(int value) {
        // Clamp to 0-2 range
        int clamped = Math.max(0, Math.min(2, value));
        motifTail.set(clamped);
    }

    /**
     * Increments motif tail (wraps from 2 to 0).
     * Called when operator presses increment button.
     */
    public static void incrementMotifTail() {
        int current = getMotifTail();
        setMotifTail((current + 1) % 3);
    }

    /**
     * Decrements motif tail (wraps from 0 to 2).
     * Called when operator presses decrement button.
     */
    public static void decrementMotifTail() {
        int current = getMotifTail();
        setMotifTail((current + 2) % 3); // +2 mod 3 is same as -1 mod 3
    }

    /**
     * Resets motif tail to 0.
     * Called when starting fresh scoring or after completing a full motif.
     */
    public static void resetMotifTail() {
        setMotifTail(0);
    }

    public static TelemetryPacket packet = new TelemetryPacket();

    public static void putPose(String label, Pose pose) {
        packet.put(label + " x", pose.getX());
        packet.put(label + " y", pose.getY());
        packet.put(label + " heading", pose.getHeading());
        packet.put(label + " heading (deg)", Math.toDegrees(pose.getHeading()));
    }
}
