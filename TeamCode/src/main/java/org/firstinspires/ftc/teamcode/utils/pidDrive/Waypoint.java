package org.firstinspires.ftc.teamcode.utils.pidDrive;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

import java.util.function.BooleanSupplier;

public class Waypoint {
    public Tolerance tolerance;
    public final Pose2d pose;
    public final PathParams params;
    private double distToNextWaypoint;
    public Waypoint(Pose2d pose) {
        this(pose, new Tolerance(Tolerance.defaultParams.xTol, Tolerance.defaultParams.yTol, Tolerance.defaultParams.headingDegTol), new PathParams());
    }
    public Waypoint(Pose2d pose, Tolerance tolerance) {
        this(pose, tolerance, new PathParams());
    }
    public Waypoint(Pose2d pose, PathParams pathParams) {
        this(pose, new Tolerance(Tolerance.defaultParams.xTol, Tolerance.defaultParams.yTol, Tolerance.defaultParams.headingDegTol), pathParams);
    }
   public Waypoint(Pose2d pose, Tolerance tolerance, PathParams pathParams) {
        this.pose = pose;
        this.tolerance = tolerance;
        this.params = pathParams;
    }
    public double x() {
        return pose.position.x;
    }
    public double y() {
        return pose.position.y;
    }
    public double headingDeg() {
        return Math.toDegrees(pose.heading.toDouble());
    }
    public double headingRad() {
       return pose.heading.toDouble();
    }

    protected void setDistToNextWaypoint(double dist) { distToNextWaypoint = dist; }
    protected double getDistToNextWaypoint() {
        return distToNextWaypoint;
    }

    public Waypoint setLateralAxialWeights(double lat, double ax) {
        params.lateralWeight = lat;
        params.axialWeight = ax;
        return this;
    }
    public Waypoint setMaxLinearPower(double maxLinearPower) {
        params.maxLinearPower = maxLinearPower;
        return this;
    }
    public Waypoint setMaxHeadingPower(double maxHeadingPower) {
        params.maxHeadingPower = maxHeadingPower;
        return this;
    }
    public Waypoint setMinLinearPower(double minLinearPower) {
        params.minLinearPower = minLinearPower;
        return this;
    }
    public Waypoint setMinHeadingPower(double minHeadingPower) {
        params.minHeadingPower = minHeadingPower;
        return this;
    }
    public Waypoint setSlowDownPercent(double percent) {
        params.slowDownPercent = percent;
        return this;
    }
    public Waypoint setPassPosition(boolean passPosition) {
        params.passPosition = passPosition;
        return this;
    }
    public Waypoint setMaxTime(double t) {
        params.maxTime = t;
        return this;
    }
    public Waypoint setCustomEndCondition(BooleanSupplier endCondition) {
        params.customEndCondition = endCondition;
        return this;
    }
    public Waypoint setUseCorrectiveKp(boolean useCorrectiveKp) {
        params.useCorrectiveKp = useCorrectiveKp;
        return this;
    }
    public Waypoint setHeadingLerp(PathParams.HeadingLerpType lerpType) {
        params.headingLerpType = lerpType;
        return this;
    }
    public Waypoint prioritizeHeadingInBeginning() {
        params.prioritizeHeadingInBeginning = true;
        return this;
    }
    public Waypoint setHeadingTangentDeactivateThreshold(double t) {
        params.tangentHeadingDeactivateThreshold = t;
        return this;
    }
    public Waypoint setControlPoint(Pose2d controlPoint, double tValueStartTime, double tValueMaxOutTime) {
        if(params.pathType == PathParams.PathType.CURVED)
            throw new IllegalStateException("path type already set to CURVED; cannot set multiple control points within one waypoint");
        params.pathType = PathParams.PathType.CURVED;
        params.controlPoint = controlPoint;
        params.tValueStartTime = tValueStartTime;
        params.tValueMaxOutTime = tValueMaxOutTime;
        return this;
    }

    @Override
    @NonNull
    public String toString() {
       return "x: " + x() + ", y: " + y() + ", heading: " + MathUtils.format2(headingDeg());
    }
}
