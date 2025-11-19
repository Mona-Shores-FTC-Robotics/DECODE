// File: GoalGeometry.java
package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import java.util.*;

public final class GoalGeometry {

    /**
     * Computes the incenter of a triangle defined by points a, b, and c.
     *
     * The incenter is the weighted average of the vertices using the lengths of their
     * opposite sides as the weights. This produces the point that is centered relative
     * to all three sides.
     *
     * Formula:
     *  Let A = length of side opposite vertex a (between b and c)
     *  Let B = length of side opposite vertex b (between a and c)
     *  Let C = length of side opposite vertex c (between a and b)
     *
     *  Incenter = (A*a + B*b + C*c) / (A + B + C)
     */
    static Pose incenter(Pose a, Pose b, Pose c) {
        double A = distance(b, c);
        double B = distance(a, c);
        double C = distance(a, b);

        double x = (A * a.getX() + B * b.getX() + C * c.getX()) / (A + B + C);
        double y = (A * a.getY() + B * b.getY() + C * c.getY()) / (A + B + C);

        return new Pose(x, y, 0.0);
    }

    /**
     * Distance between two poses, ignoring heading.
     */
    private static double distance(Pose p1, Pose p2) {
        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }



    public static Pose chebyshevCenter(List<Pose> vertices) {
        // Compute initial bounding box
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (Pose p : vertices) {
            minX = Math.min(minX, p.getX());
            maxX = Math.max(maxX, p.getX());
            minY = Math.min(minY, p.getY());
            maxY = Math.max(maxY, p.getY());
        }

        // Center of bounding box as initialization
        double cx = (minX + maxX) / 2.0;
        double cy = (minY + maxY) / 2.0;

        // Search parameters
        double step = Math.max(maxX - minX, maxY - minY) / 4.0;  // large coarse step
        double bestX = cx;
        double bestY = cy;

        for (int iter = 0; iter < 8; iter++) {  // 8 refinement passes
            double bestDist = -1.0;

            // Search a 7x7 grid around the current center
            for (int ix = -3; ix <= 3; ix++) {
                for (int iy = -3; iy <= 3; iy++) {
                    double x = bestX + ix * step;
                    double y = bestY + iy * step;

                    if (!pointInPolygon(x, y, vertices)) {
                        continue;
                    }

                    double d = distanceToPolygonEdges(x, y, vertices);
                    if (d > bestDist) {
                        bestDist = d;
                        cx = x;
                        cy = y;
                    }
                }
            }

            bestX = cx;
            bestY = cy;
            step /= 3.0; // refine search
        }

        return new Pose(bestX, bestY, 0.0);
    }


    // Checks if a point is in a convex polygon
    private static boolean pointInPolygon(double x, double y, List<Pose> v) {
        boolean inside = true;
        for (int i = 0; i < v.size(); i++) {
            Pose a = v.get(i);
            Pose b = v.get((i + 1) % v.size());

            double cross = (b.getX() - a.getX()) * (y - a.getY())
                    - (b.getY() - a.getY()) * (x - a.getX());

            // For a convex polygon, the sign must be consistent
            if (cross < 0) {
                inside = false;
                break;
            }
        }
        return inside;
    }


    // Minimum distance from point to any polygon edge
    private static double distanceToPolygonEdges(double x, double y, List<Pose> v) {
        double minDist = Double.POSITIVE_INFINITY;

        for (int i = 0; i < v.size(); i++) {
            Pose a = v.get(i);
            Pose b = v.get((i + 1) % v.size());

            double d = distPointToSegment(x, y, a.getX(), a.getY(), b.getX(), b.getY());
            minDist = Math.min(minDist, d);
        }

        return minDist;
    }


    // Standard point-to-segment distance
    private static double distPointToSegment(double px, double py,
                                             double ax, double ay,
                                             double bx, double by) {

        double abx = bx - ax;
        double aby = by - ay;

        double apx = px - ax;
        double apy = py - ay;

        double abLenSq = abx * abx + aby * aby;
        double t = (apx * abx + apy * aby) / abLenSq;

        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        double cx = ax + t * abx;
        double cy = ay + t * aby;

        double dx = px - cx;
        double dy = py - cy;

        return Math.sqrt(dx * dx + dy * dy);
    }

}
