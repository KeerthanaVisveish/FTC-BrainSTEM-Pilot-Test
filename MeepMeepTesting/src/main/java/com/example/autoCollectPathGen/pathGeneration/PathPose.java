package com.example.autoCollectPathGen.pathGeneration;


import com.acmerobotics.roadrunner.Pose2d;
import com.example.autoCollectPathGen.MathUtils;
import com.example.autoCollectPathGen.pathParams.Waypoint;

public class PathPose {
    public final Waypoint waypoint;
    public Types.PoseType poseType;
    public final Ball ball;
    public final Types.Approach approachType;
    public PathPose(Waypoint waypoint, Types.PoseType poseType, Ball ball, Types.Approach approachType) {
        this.waypoint = waypoint;
        this.poseType = poseType;
        this.ball = ball == null ? Ball.NULL : ball;
        this.approachType = approachType;
    }
    @Override
    public String toString() {
        return MathUtils.formatPose2(waypoint.pose);
    }
}