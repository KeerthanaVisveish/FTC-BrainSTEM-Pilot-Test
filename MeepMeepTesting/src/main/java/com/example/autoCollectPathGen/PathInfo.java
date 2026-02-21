package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class PathInfo {
    public final ArrayList<PathPose> pathPoses;
    public final ArrayList<PathPose> simplifiedPathPoses;
    public final ArrayList<ProblemBall> problemBalls;
    public final ArrayList<Vector2d> ignoredBalls;
    public PathInfo(ArrayList<PathPose> pathPoses, ArrayList<ProblemBall> problemBalls) {
        this.pathPoses = pathPoses;
        this.problemBalls = problemBalls;
        simplifiedPathPoses = new ArrayList<>();
        ignoredBalls = new ArrayList<>();
    }
    public void setSimplifiedPathPoses(ArrayList<PathPose> simplified) {
        simplifiedPathPoses.clear();
        simplifiedPathPoses.addAll(simplified);
    }
    public void setIgnoredBalls(ArrayList<Vector2d> ignoredBalls) {
        this.ignoredBalls.clear();
        this.ignoredBalls.addAll(ignoredBalls);
    }
    public ArrayList<Pose2d> getPoses() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (PathPose pathPose : pathPoses)
            poses.add(pathPose.pose);
        return poses;
    }
    public ArrayList<Pose2d> getSimplifiedPoses() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (PathPose pathPose : simplifiedPathPoses)
            poses.add(pathPose.pose);
        return poses;
    }
    @Override
    public String toString() {
        return pathPoses.toString();
    }
}