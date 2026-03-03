package org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams;


import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.function.BooleanSupplier;

public class Waypoint {
    public Tolerance tolerance;
    public Pose2d pose;
    public final PathParams params;
    private double distToNextWaypoint;
    public Waypoint(Pose2d pose) {
        this(pose, new CircleTolerance(), new PathParams());
    }
    public Waypoint(Pose2d pose, Tolerance tolerance) {
        this(pose, tolerance, new PathParams());
    }
    public Waypoint(Pose2d pose, PathParams pathParams) {
        this(pose, new CircleTolerance(), pathParams);
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
    public double headingRad() {
       return pose.heading.toDouble();
    }

    public void setNextWaypoint(Waypoint waypoint) {
        distToNextWaypoint = Math.hypot(waypoint.x() - x(), waypoint.y() - y());
    }
    public double getDistToNextWaypoint() {
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
    public Waypoint setMinLinearPower(double minLinearPower) {
        params.minLinearPower = minLinearPower;
        return this;
    }
    public Waypoint setFixedLinearPower(double fixedLinearPower) {
        params.maxLinearPower = fixedLinearPower;
        params.minLinearPower = fixedLinearPower;
        return this;
    }
    public Waypoint setMinHeadingPower(double minHeadingPower) {
        params.minHeadingPower = minHeadingPower;
        return this;
    }
    public Waypoint setMaxHeadingPower(double maxHeadingPower) {
        params.maxHeadingPower = maxHeadingPower;
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
    public Waypoint setCorrectiveStrength(double correctiveStrength) {
        params.correctiveStrength = correctiveStrength;
        return this;
    }
    public Waypoint setHeadingLerp(PathParams.HeadingLerpType lerpType) {
        params.headingLerpType = lerpType;
        return this;
    }
    public Waypoint setFarHeadingKP(double kP) {
        params.farHeadingKp = kP;
        return this;
    }
    public Waypoint setCloseHeadingKP(double kP) {
        params.closeHeadingKp = kP;
        return this;
    }
    public Waypoint setHeadingKPMult(double mult) {
        params.closeHeadingKp *= mult;
        params.farHeadingKp *= mult;
        return this;
    }
    public Waypoint prioritizeHeadingInBeginning() {
        params.prioritizeHeadingInBeginning = true;
        return this;
    }
    public Waypoint setHeadingTangentDeactivateThreshold(double t) {
        params.tangentHeadingDeactivateDist = t;
        return this;
    }
    public Waypoint setControlPoint(Pose2d controlPoint, double tValueStartDistError, double tValueFinishDistError) {
        if(params.pathType == PathParams.PathType.CURVED)
            throw new IllegalStateException("path type already set to CURVED; cannot set multiple control points within one waypoint");
        params.pathType = PathParams.PathType.CURVED;
        params.controlPoint = controlPoint;
        params.tValueStartDist = tValueStartDistError;
        params.tValueFinishDist = tValueFinishDistError;
        return this;
    }

    @Override
    public String toString() {
       return "x: " + x() + ", y: " + y() + ", heading: " + MathUtils.format2(headingRad());
    }
}
