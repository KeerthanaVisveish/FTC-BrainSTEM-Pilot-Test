package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Vector2d;

public class GeometryUtils {

    public static Vector2d rotateVector(Vector2d vector, double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        // cos, -sin
        // sin,  cos
        return new Vector2d(vector.x * cos - vector.y * sin, vector.x * sin + vector.y * cos);
    }

    /**
     * finds the information about the perpendicular bisector of a line given the line and a third point
     * @param A line start point [x, y]
     * @param B line end point   [x, y]
     * @param P external point  [x, y]
     * @return Result object containing distance, angle (radians), and foot point
     */
    public static double[] pointToLineDistanceAndAngle(
            double[] A, double[] B, double[] P) {

        double x1 = A[0], y1 = A[1];
        double x2 = B[0], y2 = B[1];
        double x0 = P[0], y0 = P[1];

        double dx = x2 - x1;
        double dy = y2 - y1;

        // Perpendicular distance
        double distance = Math.abs(dx * (y1 - y0) - (x1 - x0) * dy)
                / Math.hypot(dx, dy);

        // Foot of the perpendicular
        double t = ((x0 - x1) * dx + (y0 - y1) * dy)
                / (dx * dx + dy * dy);

        double fx = x1 + t * dx;
        double fy = y1 + t * dy;

        // Angle from point P to foot F (radians)
        double angle = Math.atan2(fy - y0, fx - x0);

        return new double[] {distance, angle, fx, fy};
    }

    // ensures the point is inside the field
    // if it isn't, it projects it onto the field by finding the line between the origin and the vector
    // then finding the intersection of that line and the field wall + some buffer distance
    // that intersection is the projected point
    public static Vector2d projectOntoField(Vector2d origin, Vector2d vecToProject, double extraBufferDistance) {
        final double fieldMin = -72. + extraBufferDistance;
        final double fieldMax = 72. - extraBufferDistance;

        double x0 = origin.x;
        double y0 = origin.y;

        double dx = vecToProject.x - x0;
        double dy = vecToProject.y - y0;

        // If already inside field, return as-is
        if (vecToProject.x >= fieldMin && vecToProject.x <= fieldMax &&
                vecToProject.y >= fieldMin && vecToProject.y <= fieldMax) {
            return vecToProject;
        }

        double tMin = Double.POSITIVE_INFINITY;

        // Intersect with vertical walls
        if (dx != 0) {
            double tLeft  = (fieldMin - x0) / dx;
            double tRight = (fieldMax - x0) / dx;

            if (tLeft > 0)  tMin = Math.min(tMin, tLeft);
            if (tRight > 0) tMin = Math.min(tMin, tRight);
        }

        // Intersect with horizontal walls
        if (dy != 0) {
            double tBottom = (fieldMin - y0) / dy;
            double tTop    = (fieldMax - y0) / dy;

            if (tBottom > 0) tMin = Math.min(tMin, tBottom);
            if (tTop > 0)    tMin = Math.min(tMin, tTop);
        }

        // Safety check (should not happen if origin is inside field)
        if (!Double.isFinite(tMin)) {
            return origin;
        }

        // Intersection point
        double ix = x0 + dx * tMin;
        double iy = y0 + dy * tMin;

        // Clamp for numerical safety
        ix = Math.max(fieldMin, Math.min(fieldMax, ix));
        iy = Math.max(fieldMin, Math.min(fieldMax, iy));

        return new Vector2d(ix, iy);
    }

}

