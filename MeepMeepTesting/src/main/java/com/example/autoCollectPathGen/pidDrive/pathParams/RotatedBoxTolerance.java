package com.example.autoCollectPathGen.pidDrive.pathParams;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.autoCollectPathGen.GeometryUtils;

import java.util.ArrayList;
import java.util.Arrays;

public class RotatedBoxTolerance implements Tolerance {
    private final double parallelTol, perpendicularTol, headingRadTol;
    private final double axisAngleRad;
    public RotatedBoxTolerance(double parallelTol, double perpendicularTol, double axisAngleRad, double headingRadTol) {
        this.parallelTol = parallelTol;
        this.perpendicularTol = perpendicularTol;
        this.axisAngleRad = axisAngleRad;
        this.headingRadTol = headingRadTol;
    }

    @Override
    public boolean inPositionTolerance(double xError, double yError) {
        Vector2d rotatedErrors = GeometryUtils.rotateVector(new Vector2d(xError, yError), -axisAngleRad);
        return Math.abs(rotatedErrors.x) <= parallelTol && Math.abs(rotatedErrors.y) <= perpendicularTol;
    }

    @Override
    public boolean inHeadingTolerance(double headingRadError) {
        return Math.abs(headingRadError) < headingRadTol;
    }

    @Override
    public ArrayList<Vector2d> getToleranceCorners(Vector2d waypointPosition) {
        double x = parallelTol * 0.5;
        double y = perpendicularTol * 0.5;
        ArrayList<Vector2d> unRotated = new ArrayList<>(Arrays.asList(
                new Vector2d(-x, -y),
                new Vector2d( x, -y),
                new Vector2d( x,  y),
                new Vector2d(-x,  y)
        ));
        ArrayList<Vector2d> rotated = new ArrayList<>();
        for (Vector2d vec : unRotated)
            rotated.add(GeometryUtils.rotateVector(vec, axisAngleRad).plus(waypointPosition));
        return rotated;
    }
}
