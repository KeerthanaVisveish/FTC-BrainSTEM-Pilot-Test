package com.example.autoCollectPathGen.pathGeneration;


import com.acmerobotics.roadrunner.Pose2d;
import com.example.autoCollectPathGen.MathUtils;

public class PathPose {
    public final Pose2d pose;
    public Types.PoseType poseType;
    public final Ball ball;
    public final Types.Approach approachType;
    public PathPose(Pose2d pose, Types.PoseType poseType, Ball ball, Types.Approach approachType) {
        this.pose = pose;
        this.poseType = poseType;
        this.ball = ball == null ? Ball.NULL : ball;
        this.approachType = approachType;
    }
    @Override
    public String toString() {
        return MathUtils.formatPose2(pose);
    }
}