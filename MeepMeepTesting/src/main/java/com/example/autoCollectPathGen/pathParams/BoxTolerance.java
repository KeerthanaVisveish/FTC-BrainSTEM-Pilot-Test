package com.example.autoCollectPathGen.pathParams;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;

public class BoxTolerance implements Tolerance {
    public static class DefaultParams {
        public double xTol = 2;
        public double yTol = 2;
        public double headingDegTol = 5;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    private final double xTol, yTol, headingRadTol;
    public BoxTolerance(double xTol, double yTol, double headingRadTol) {
        this.xTol = xTol;
        this.yTol = yTol;
        this.headingRadTol = headingRadTol;
    }

    public BoxTolerance(double distTol, double headingRadTol) {
        this(distTol, distTol, headingRadTol);
    }
    public BoxTolerance() {
        this(defaultParams.xTol, defaultParams.yTol, Math.toRadians(defaultParams.headingDegTol));
    }
    @Override
    public boolean inPositionTolerance(double xError, double yError) {
        return Math.abs(xError) <= xTol && Math.abs(yError) <= yTol;
    }

    @Override
    public boolean inHeadingTolerance(double headingRadError) {
        return Math.abs(headingRadError) < headingRadTol;
    }
    @Override
    public ArrayList<Vector2d> getToleranceCorners(Vector2d waypointPosition) {
        double x = xTol * 0.5;
        double y = yTol * 0.5;
        return new ArrayList<>(Arrays.asList(
                new Vector2d(-x, -y).plus(waypointPosition),
                new Vector2d( x, -y).plus(waypointPosition),
                new Vector2d( x,  y).plus(waypointPosition),
                new Vector2d(-x,  y).plus(waypointPosition)
        ));
    }
}
