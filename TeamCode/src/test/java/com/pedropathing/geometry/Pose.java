package com.pedropathing.geometry;

/** Test stub — mirrors the Pedro Pathing Pose API used by PoseFusion. */
public class Pose {
    private final double x;
    private final double y;
    private final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
}
