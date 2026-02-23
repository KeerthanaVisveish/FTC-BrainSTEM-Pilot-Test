package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class PathInfo {
    public enum PathType {
        EMPTY,
        LANE,
        COMPLEX
    }
    public final PathType pathType;
    public final Pose2d startPose;
    public final ArrayList<Vector2d> originalBallPath, ballPath;
    public final ArrayList<PathPose> pathPoses;
    public final ArrayList<PathPose> simplifiedPathPoses;
    public final ArrayList<ProblemBall> problemBalls;
    public final ArrayList<Vector2d> ignoredBalls;
    public PathInfo(PathType pathType, Pose2d startPose, ArrayList<Vector2d> originalBallPath, ArrayList<Vector2d> ballPath, ArrayList<PathPose> pathPoses, ArrayList<ProblemBall> problemBalls) {
        this.pathType = pathType;
        this.startPose = startPose;
        this.originalBallPath = originalBallPath;
        this.ballPath = ballPath;
        this.pathPoses = pathPoses;
        this.problemBalls = problemBalls;
        simplifiedPathPoses = new ArrayList<>();
        ignoredBalls = new ArrayList<>();
    }

    public void setIgnoredBalls(ArrayList<Vector2d> ignoredBalls) {
        this.ignoredBalls.clear();
        this.ignoredBalls.addAll(ignoredBalls);
    }
    public void setSimplifiedPathPoses(ArrayList<PathPose> simplifiedPathPoses) {
        this.simplifiedPathPoses.clear();
        this.simplifiedPathPoses.addAll(simplifiedPathPoses);
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
    public int numGoodBalls() {
        return ballPath.size() - problemBalls.size();
    }
    @Override
    public String toString() {
        return pathPoses.toString();
    }
}