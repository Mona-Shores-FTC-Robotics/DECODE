package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.IFollower.PoseLike;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/**
 * Wraps the Pedro {@link Follower} in the light-weight {@link IFollower} interface so
 * state machines can remain non-blocking. If Pedro is unavailable (for example when
 * running on a laptop without the native libraries) we fall back to {@link DummyFollower}
 * that simply "arrives" after a short timeout. This keeps student code compiling.
 */
public class PedroFollowerAdapter implements IFollower {

    private static final String TAG = "PedroFollowerAdapter";

    private final Follower follower;
    private final DummyFollower dummy;
    private final Queue<Path> pendingPaths = new ArrayDeque<>();
    private ConstraintSnapshot baselineConstraints;
    private Double temporaryVelCap = null;

    public PedroFollowerAdapter(HardwareMap hardwareMap) {
        Follower instance = null;
        try {
            instance = Constants.createFollower(hardwareMap);
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Pedro follower not available - falling back to dummy implementation");
        }

        if (instance != null) {
            follower = instance;
            dummy = null;
        } else {
            follower = null;
            dummy = new DummyFollower();
        }

        baselineConstraints = ConstraintSnapshot.from(Constants.pathConstraints);
    }

    @Override
    public void setStartingPose(Pose pose) {
        if (follower != null) {
            follower.setStartingPose(pose);
        } else {
            dummy.setStartingPose(pose);
        }
    }

    @Override
    public void update() {
        if (follower != null) {
            follower.update();
            if (!pendingPaths.isEmpty() && !follower.isBusy()) {
                followNextSegment();
            }
        } else {
            dummy.update();
        }
    }

    @Override
    public void goTo(PoseLike... targets) {
        if (targets == null || targets.length == 0) {
            return;
        }

        if (follower != null) {
            Pose current = getPose();
            if (current == null) {
                current = new Pose();
            }

            List<Path> newSegments = new ArrayList<>();
            for (PoseLike like : targets) {
                Pose target = like != null ? like.toPose() : null;
                if (target == null) {
                    continue;
                }
                BezierLine line = new BezierLine(current, target);
                Path path = new Path(line);
                path.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());
                path.setReversed(false);
                applyConstraints(path);
                newSegments.add(path);
                current = target;
            }

            if (newSegments.isEmpty()) {
                return;
            }

            pendingPaths.addAll(newSegments);
            if (!follower.isBusy()) {
                followNextSegment();
            }
        } else {
            dummy.goTo(targets);
        }
    }

    @Override
    public boolean isBusy() {
        if (follower != null) {
            return follower.isBusy() || !pendingPaths.isEmpty();
        } else {
            return dummy.isBusy();
        }
    }

    @Override
    public Pose getPose() {
        if (follower != null) {
            return follower.getPose();
        } else {
            return dummy.getPose();
        }
    }

    @Override
    public void setConstraints(PathConstraints constraints) {
        if (constraints != null) {
            baselineConstraints = ConstraintSnapshot.from(constraints);
        }
    }

    @Override
    public void setVelCapTemporary(double velCap) {
        temporaryVelCap = velCap;
    }

    @Override
    public void clearVelCap() {
        temporaryVelCap = null;
    }

    private void followNextSegment() {
        Path next = pendingPaths.poll();
        if (next != null) {
            follower.followPath(applyConstraints(next));
        } else {
            clearVelCap();
        }
    }

    private Path applyConstraints(Path path) {
        PathConstraints constraints = buildConstraintsSnapshot();
        if (tryInvoke(path, "setPathConstraints", constraints)) {
            return path;
        }
        tryInvoke(path, "setConstraints", constraints);
        return path;
    }

    private PathConstraints buildConstraintsSnapshot() {
        double maxVel = baselineConstraints.maxVel;
        if (temporaryVelCap != null) {
            maxVel = Math.min(maxVel, temporaryVelCap);
        }

        return new PathConstraints(
                maxVel,
                baselineConstraints.maxAccel,
                baselineConstraints.maxAngVel,
                baselineConstraints.maxAngAccel
        );
    }

    private static boolean tryInvoke(Path path, String methodName, PathConstraints constraints) {
        try {
            Method method = path.getClass().getMethod(methodName, PathConstraints.class);
            method.invoke(path, constraints);
            return true;
        } catch (Throwable ignored) {
            return false;
        }
    }

    /**
     * Reflectively cracks open {@link PathConstraints} so we can clone the values without
     * depending on Pedro internals. If the shape changes we fall back to reasonable defaults.
     */
    private static class ConstraintSnapshot {
        final double maxVel;
        final double maxAccel;
        final double maxAngVel;
        final double maxAngAccel;

        ConstraintSnapshot(double maxVel, double maxAccel, double maxAngVel, double maxAngAccel) {
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
            this.maxAngVel = maxAngVel;
            this.maxAngAccel = maxAngAccel;
        }

        static ConstraintSnapshot from(PathConstraints constraints) {
            if (constraints == null) {
                return new ConstraintSnapshot(0.9, 90.0, 1.0, 1.0);
            }

            double maxVel = readField(constraints, "maxVel", 0.9);
            double maxAccel = readField(constraints, "maxAccel", 90.0);
            double maxAngVel = readField(constraints, "maxAngularVel", 1.0);
            double maxAngAccel = readField(constraints, "maxAngularAccel", 1.0);
            return new ConstraintSnapshot(maxVel, maxAccel, maxAngVel, maxAngAccel);
        }

        private static double readField(PathConstraints constraints, String fieldName, double fallback) {
            try {
                Field field = constraints.getClass().getDeclaredField(fieldName);
                field.setAccessible(true);
                return field.getDouble(constraints);
            } catch (Throwable t) {
                RobotLog.ww(TAG, "Unable to read PathConstraints.%s, using fallback %.2f", fieldName, fallback);
                return fallback;
            }
        }
    }
}
