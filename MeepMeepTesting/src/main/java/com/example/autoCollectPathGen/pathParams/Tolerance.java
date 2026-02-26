package com.example.autoCollectPathGen.pathParams;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public interface Tolerance {
    boolean inPositionTolerance(double xError, double yError);
    boolean inHeadingTolerance(double headingRadError);
    ArrayList<Vector2d> getToleranceCorners(Vector2d waypointPosition);
}
