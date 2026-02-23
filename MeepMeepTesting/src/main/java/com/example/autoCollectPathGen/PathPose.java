package com.example.autoCollectPathGen;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class PathPose {
    public final Pose2d pose;
    public Types.PoseType poseType;
    public final Vector2d ball;
    public final BallType ballType;
    public final Types.Approach approachType;
    public PathPose(Pose2d pose, Types.PoseType poseType, Vector2d ball, BallType ballType, Types.Approach approachType) {
        this.pose = pose;
        this.poseType = poseType;
        this.ball = ball;
        this.ballType = ballType;
        this.approachType = approachType;
    }
    @Override
    public String toString() {
        return MathUtils.formatPose2(pose);
    }
}