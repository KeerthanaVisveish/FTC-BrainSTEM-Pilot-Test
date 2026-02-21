package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;

public class MathTest {
    public static void main(String[] args) {

//        Vector2d correctiveVec = getCorrectiveVector(
//                new Pose2d(0, 0, Math.toRadians(45)),
//                new Vector2d(10, 10), new Vector2d(-5, 2)
//                );
//        System.out.println("corrective vector: " + correctiveVec);
    }
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
//    static double correctiveKp = 0.2;
//    static double correctiveStrength = 1;
//    private static Vector2d getCorrectiveVector(Pose2d robotPose, Vector2d prevWaypointPosition, Vector2d targetWaypointPosition) {
//        Vector2d curPos = robotPose.position;
//
//        Vector2d prevToCur = curPos.minus(prevWaypointPosition);
//        Vector2d prevToTargetDir = targetWaypointPosition.minus(prevWaypointPosition);
//        prevToTargetDir = prevToTargetDir.div(MathUtils.vecMag(prevToTargetDir));
//        Vector2d orthogonalDir = new Vector2d(-prevToTargetDir.y, 1*prevToTargetDir.x);
//
//        double projectedPerpendicularOffset = orthogonalDir.dot(prevToCur);
//        double correctiveMagnitude = projectedPerpendicularOffset * correctiveKp;
//
//        // corrective vector in the fields coordinate plane
//        Vector2d absoluteCorrectivePower = orthogonalDir.times(-correctiveMagnitude);
//
//        // rotate vector by negative robot heading to get relative corrective powers
//        Vector2d relativeCorrectivePower = GeometryUtils.rotateVector(absoluteCorrectivePower, -robotPose.heading.toDouble());
//        return relativeCorrectivePower.times(correctiveStrength); // scale relative to drive vector
//    }
}
