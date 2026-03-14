package org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class CircleTolerance implements Tolerance {
    public static class DefaultParams {
        public double distTol = 2.5;
        public double headingDegTol = 5;
        public int numToleranceCorners = 16;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    private final double distTol, headingRadTol;
    public CircleTolerance(double distTol, double headingRadTol) {
        this.distTol = distTol;
        this.headingRadTol = headingRadTol;
    }

    public CircleTolerance() {
        this(defaultParams.distTol, Math.toRadians(defaultParams.headingDegTol));
    }
    public CircleTolerance(double[] tol) {
        this(tol[0], Math.toRadians(tol[1]));
    }
    @Override
    public boolean inPositionTolerance(double xError, double yError) {
        return Math.hypot(xError, yError) <= distTol;
    }

    @Override
    public boolean inHeadingTolerance(double headingRadError) {
        return Math.abs(headingRadError) < headingRadTol;
    }
    @Override
    public ArrayList<Vector2d> getToleranceCorners(Vector2d waypointPosition) {
        double r = distTol * 0.5;
        ArrayList<Vector2d> edges = new ArrayList<>();
        double angleChange = 2 * Math.PI / defaultParams.numToleranceCorners;
        for (int i=0; i<defaultParams.numToleranceCorners; i++) {
            double angle = i * angleChange;
            edges.add(new Vector2d(Math.cos(angle) * r, Math.sin(angle) * r).plus(waypointPosition));
        }
        return edges;
    }
}
