package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;

public class PathInfo {
    public enum PathType {
        EMPTY,
        LANE,
        COMPLEX
    }
    public final PathType pathType;
    public final Pose2d startPose;
    public final ArrayList<Ball> ballPath;
    public final ArrayList<PathPose> pathPoses;
    public final ArrayList<PathPose> optimizedPathPoses;
    public final ArrayList<ProblemBall> problemBalls;
    public final ArrayList<Ball> ignoredBalls;
    public PathInfo(PathType pathType, Pose2d startPose, ArrayList<Ball> ballPath, ArrayList<PathPose> pathPoses, ArrayList<ProblemBall> problemBalls) {
        this.pathType = pathType;
        this.startPose = startPose;
        this.ballPath = ballPath;
        this.pathPoses = pathPoses;
        this.problemBalls = problemBalls;
        optimizedPathPoses = new ArrayList<>();
        ignoredBalls = new ArrayList<>();
    }

    public void setIgnoredBalls(ArrayList<Ball> ignoredBalls) {
        this.ignoredBalls.clear();
        this.ignoredBalls.addAll(ignoredBalls);
    }
    public void setOptimizedPathPoses(ArrayList<PathPose> optimizedPathPoses) {
        this.optimizedPathPoses.clear();
        this.optimizedPathPoses.addAll(optimizedPathPoses);
    }
    public ArrayList<Pose2d> getPoses() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (PathPose pathPose : pathPoses)
            poses.add(pathPose.waypoint.pose);
        return poses;
    }
    public ArrayList<Pose2d> getOptimizedPoses() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (PathPose pathPose : optimizedPathPoses)
            poses.add(pathPose.waypoint.pose);
        return poses;
    }
    public int numGoodBalls() {
        return ballPath.size() - problemBalls.size();
    }

    public double getTotalCost(double changeInAngleDegCost) {
        double totalCost = 0;
        for (int i=0; i<pathPoses.size()-1; i++) {
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i+1);
            double dist = next.waypoint.pose.position.minus(cur.waypoint.pose.position).norm();
            double headingChange = next.waypoint.pose.heading.minus(cur.waypoint.pose.heading);
            double combinedCost = dist + Math.abs(Math.toDegrees(headingChange)) * changeInAngleDegCost;
            totalCost += combinedCost;
        }
        return totalCost;
    }

    public boolean isUndesirable(double angleChangeDeg) {
        for (int i=0; i<optimizedPathPoses.size()-1; i++) {
            PathPose cur = optimizedPathPoses.get(i);
            PathPose next = optimizedPathPoses.get(i+1);
            double headingChange = next.waypoint.pose.heading.minus(cur.waypoint.pose.heading);
            if (Math.abs(headingChange) > Math.toRadians(angleChangeDeg))
                    return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return pathPoses.toString();
    }
}