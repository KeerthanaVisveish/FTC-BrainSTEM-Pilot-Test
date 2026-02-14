package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;

public class GeometryUtils {

    public static Vector2d robotVectorToFieldVector(Vector2d robotVector, double rHeadingRad) {
        double cos = Math.cos(rHeadingRad);
        double sin = Math.sin(rHeadingRad);
        // cos, -sin
        // sin,  cos
        return new Vector2d(robotVector.x * cos - robotVector.y * sin, robotVector.x * sin + robotVector.y * cos);
    }
    public static Vector2d fieldVectorToRobotVector(Vector2d fieldVector, double rHeadingRad) {
        double cos = Math.cos(-rHeadingRad);
        double sin = Math.sin(-rHeadingRad);
        // cos, -sin
        // sin,  cos
        return new Vector2d(fieldVector.x * cos - fieldVector.y * sin, fieldVector.x * sin + fieldVector.y * cos);
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
}

