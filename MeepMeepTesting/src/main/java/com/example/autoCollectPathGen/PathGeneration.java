package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static PathGenerationParams params = new PathGenerationParams();
    public static Pose2d pathfinderStartPose = null;
    public static PathInfo getAutoCollectPath(Pose2d robotPose, Vector2d[] ballsArray) {
        pathfinderStartPose = robotPose;
        if (ballsArray.length == 0)
            return null;

        ArrayList<Vector2d> allBalls = new ArrayList<>(Arrays.asList(ballsArray));

        Lane[] densestLanes = getDensestLanes(allBalls);
        Lane bestLane = getBestLane(robotPose.position, densestLanes);
        if (params.allowLaneCollect) {
            if (densestLanes[0].numBalls() >= 3)
                return generateLanePath(robotPose, bestLane);
            if (allBalls.size() == 2 && bestLane.numBalls() == 2)
                return generateLanePath(robotPose, bestLane);
        }

        int numAttempts = 0;
        PathInfo optimalPathInfo = null;
        int leastProblemBalls = -1;
        ArrayList<Vector2d> balls = new ArrayList<>(Arrays.asList(ballsArray));
        ArrayList<Vector2d> ignoredBalls = new ArrayList<>();
        do {
//            System.out.println("regeneration " + numAttempts);
            ArrayList<Vector2d> rawBallPath = PathFinder.findShortestPath(robotPose, balls, 3, params.changeInAngleDegCost, 0,0);
            if (rawBallPath == null)
                break;

            // generate path
            PathInfo pathInfo = generatePath(robotPose, allBalls, rawBallPath);
            pathInfo.setIgnoredBalls(ignoredBalls);
            int numProblemBalls = pathInfo.problemBalls.size();

            // artificially shift robot to the left to see if it generates a more desirable path
            Pose2d shiftedLeftRobotPose = new Pose2d(params.shiftedLeftStartX, robotPose.position.y, robotPose.heading.toDouble());
            ArrayList<Vector2d> shiftedLeftRawBallPath = PathFinder.findShortestPath(shiftedLeftRobotPose, balls, 3, params.changeInAngleDegCost, 0,0);
            if (shiftedLeftRawBallPath != null) {
                PathInfo shiftedLeftPathInfo = generatePath(robotPose, allBalls, shiftedLeftRawBallPath);
                if (shiftedLeftPathInfo.numGoodBalls() > pathInfo.numGoodBalls()) {
                    pathfinderStartPose = shiftedLeftRobotPose;
                    rawBallPath = shiftedLeftRawBallPath;
                    pathInfo = shiftedLeftPathInfo;
                }
            }
//            System.out.println("num problem: " + numProblemBalls);
//            System.out.println("num good: " + numGoodBalls);
            if (leastProblemBalls == -1 ||
                    pathInfo.numGoodBalls() > optimalPathInfo.numGoodBalls() ||
                    pathInfo.numGoodBalls() == optimalPathInfo.numGoodBalls() && numProblemBalls > leastProblemBalls) {
                leastProblemBalls = numProblemBalls;
                optimalPathInfo = pathInfo;
            }

            // optimal path
            if (numProblemBalls == 0)
                break;

            // un-optimal
            int lowestSeverityOrdinal = 100;
            for (ProblemBall problemBall : pathInfo.problemBalls) {
                System.out.println(problemBall.severity + ": " + problemBall.ballPosition);
                if (problemBall.severity.ordinal() <= lowestSeverityOrdinal)
                    lowestSeverityOrdinal = problemBall.severity.ordinal();
            }
//            System.out.println("lowest ordinal: " + lowestSeverityOrdinal);
            for (ProblemBall problemBall : pathInfo.problemBalls)
                if (problemBall.severity.ordinal() == lowestSeverityOrdinal) {
                    balls.remove(problemBall.ballPosition);
                    ignoredBalls.add(problemBall.ballPosition);
                }
//            System.out.println("considering " + balls);
//            System.out.println("ignoring " + ignoredBalls);

            numAttempts++;
        } while (!balls.isEmpty() && numAttempts <= params.maxPathRegenerationAttempts);

        if (optimalPathInfo == null) {
            return null;
        }

        int complexNumBalls = optimalPathInfo.numGoodBalls();
        int laneNumBalls = bestLane.numBalls();
        PathInfo lanePath = generateLanePath(robotPose, bestLane);
        PathInfo finalPath = complexNumBalls > laneNumBalls ? optimalPathInfo : lanePath;
        if (params.allowLaneCollect)
            return finalPath;
        return optimalPathInfo;
    }

    // use a sliding window of width laneWidth moving at a speed of increment
    // returns the list of balls in the most densely packed moment of the window
    private static Lane[] getDensestLanes(ArrayList<Vector2d> balls) {
        Vector2d[] sorted = Arrays.copyOf(balls.toArray(new Vector2d[0]), balls.size());
        Arrays.sort(sorted, Comparator.comparingDouble(a -> a.x));
        int start = (int) sorted[0].x - 1;
        int end = (int) sorted[sorted.length-1].x + 1;

        double increment = params.laneIncrement;
        int numLanes = (int) ((end - params.normalLaneWidth - start + 2) / increment);
        if (numLanes <= 0)
            numLanes = 1;
        ArrayList<Lane> lanes = new ArrayList<>();

        for (int i=0; i<numLanes; i++) {
            double left = start + i * increment;
            boolean againstBackWall = left + params.normalLaneWidth >= 72 - params.laneAgainstBackWallMaxDist;
            double laneWidth = againstBackWall ? params.againstBackWallLaneWidth : params.normalLaneWidth;
            double right = left + laneWidth;
            ArrayList<Vector2d> ballsInLane = new ArrayList<>();
            for (Vector2d vector2d : sorted)
                if (vector2d.x >= left && vector2d.x <= right)
                    ballsInLane.add(vector2d);
            lanes.add(new Lane(ballsInLane, againstBackWall));
        }
        ArrayList<Lane> validLanes = new ArrayList<>(); // list of all lanes with highest number of balls
        validLanes.add(lanes.get(0));

        for (int i=1; i<lanes.size(); i++) {
            Lane lane = lanes.get(i);
            if (lane.numBalls() >= validLanes.get(0).numBalls()) {
                // found new biggest clear out old ones
                if (lane.numBalls() > validLanes.get(0).numBalls())
                    validLanes.clear();
                validLanes.add(lane);
            }
        }
        return validLanes.toArray(new Lane[0]);
    }
    // returns the lane closest to the robot
    private static Lane getBestLane(Vector2d robotPos, Lane[] densestLanes) {
        if (densestLanes.length == 1)
            return densestLanes[0];
        Lane bestLane = null;
        double minDist = -1;
        for (int i=0; i<densestLanes.length; i++) {
            Lane lane = densestLanes[i];
            Vector2d firstPoint = new Vector2d(lane.avgX, lane.minAbsY());
            double dist = Math.hypot(firstPoint.x - robotPos.x, firstPoint.y - robotPos.y);

            if (bestLane == null || lane.closeToBackWall && !bestLane.closeToBackWall
                    || (bestLane.closeToBackWall == lane.closeToBackWall && dist < minDist)) {
                bestLane = lane;
                minDist = dist;
            }
        }
        return bestLane;
    }
    private static class Lane {
        private final ArrayList<Vector2d> balls;
        public boolean closeToBackWall;
        private final double avgX;
        public Lane(ArrayList<Vector2d> balls, boolean closeToBackWall) {
            this.balls = new ArrayList<>(balls);
            this.closeToBackWall = closeToBackWall;
            // sort by |y| value ascending (so it works for both red and blue)
            this.balls.sort(Comparator.comparingDouble(v -> Math.abs(v.y)));
            if (balls.isEmpty())
                this.avgX = 0;
            else {
                double totalX = 0;
                for (Vector2d ball : balls)
                    totalX += ball.x;
                this.avgX = totalX / balls.size();
            }
        }
        public int numBalls() {
            return balls.size();
        }
        public double minAbsY() {
            return balls.get(0).y;
        }
        public double maxAbsY() {
            return balls.get(balls.size() - 1).y;
        }
        @Override
        public String toString() {
            return "avgx: " + avgX + " | num balls: " + numBalls();
        }
    }
    private static PathInfo generateLanePath(Pose2d startPose, Lane lane) {
        double angle = Math.signum(lane.balls.get(0).y) * Math.toRadians(90);
        Vector2d ball1 = new Vector2d(lane.avgX, lane.minAbsY());
        Vector2d ball2 = new Vector2d(lane.avgX, lane.maxAbsY());

        if (lane.avgX >= 72 - params.angleLaneCollectDistFromBackWall)
            angle -= Math.signum(angle) * Math.toRadians(params.cornerCollectAngle);

        Pose2d collect1 = getCollectPose(ball1, angle, 0);
        Pose2d collect2 = getCollectPose(ball2, angle, -params.lastCollectPoseExtraDriveThrough);
        Pose2d preCollect1 = getPreCollectPose(collect1, angle, params.preCollectOffset);
        Pose2d wallSafeCollect2 = getWallSafePose(collect1);
        Pose2d wallSafeCollect3 = getWallSafePose(collect2);
        Pose2d wallSafePreCollect = getWallSafePose(preCollect1);

        PathPose pathPose1 = new PathPose(wallSafePreCollect, Types.PoseType.PRECOLLECT, ball1, BallType.NORMAL, Types.Approach.NORMAL);
        PathPose pathPose2 = new PathPose(wallSafeCollect2, Types.PoseType.COLLECT, ball1, BallType.NORMAL, Types.Approach.NORMAL);
        PathPose pathPose3 = new PathPose(wallSafeCollect3, Types.PoseType.COLLECT, ball1, BallType.NORMAL, Types.Approach.NORMAL);
        ArrayList<PathPose> pathPoses = new ArrayList<>(Arrays.asList(pathPose1, pathPose2, pathPose3));
        return new PathInfo(PathInfo.PathType.LANE, startPose, lane.balls, lane.balls, pathPoses, new ArrayList<>());
    }
    private static PathInfo generatePath(Pose2d robotPose, ArrayList<Vector2d> allBalls, ArrayList<Vector2d> originalBallPath) {
        Vector2d robotPos = robotPose.position;
        ArrayList<PathPose> pathPoses = new ArrayList<>();
        ArrayList<ProblemBall> problemBalls = new ArrayList<>();
        if (originalBallPath.isEmpty())
            return new PathInfo(PathInfo.PathType.EMPTY, robotPose, originalBallPath, originalBallPath, pathPoses, problemBalls);
        ArrayList<Vector2d> ballPath = new ArrayList<>(originalBallPath);

        ArrayList<Vector2d> allBallsInCluster = new ArrayList<>();
        ArrayList<ClusterApproach> clusterApproaches = new ArrayList<>();

        // reject balls along the wall that are too far away from each other
        // also reject balls along the wall and in the corner that are too close to each other
        if (ballPath.size() >= 2) {
            for (int i=1; i<ballPath.size(); i++) {
                Vector2d prevBall = ballPath.get(i - 1);
                Vector2d curBall = ballPath.get(i);
                BallType prevType = getBallType(prevBall);
                BallType type = getBallType(curBall);

                if (type != BallType.NORMAL && prevType != BallType.NORMAL) {
                    double horizontalDist = Math.abs(curBall.x - prevBall.x);

                    if (horizontalDist > params.rejectEdgeCaseBallsHorizontalDist) {
                        if (ballPath.size() == 2) {
                            // choose closer one
                            double curDist = MathUtils.vecDist(curBall, robotPos);
                            double prevDist = MathUtils.vecDist(prevBall, robotPos);
                            if (curDist < prevDist) {
                                ballPath.remove(i - 1);
                                break;
                            }
                            ballPath.remove(i);
                            break;
                        }
                        if (i == 1) {
                            ballPath.remove(0);
                            break;
                        }
                        ballPath.remove(2);
                        break;
                    }
//                    else if (prevType == BallType.CORNER && type == BallType.CLASSIFIER_WALL && horizontalDist < params.rejectCornerTooCloseToClassifierBallDist) {
//                        ballPath.remove(i - 1);
//                    }
                }
            }
        }

        for (int i=0; i<ballPath.size(); i++) {
            Pose2d prevPathPose = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).pose : robotPose;
            Vector2d prevPathPos = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).pose.position : robotPos;
            Vector2d prevBall = i == 0 ? robotPos : ballPath.get(i - 1);
            Vector2d curBall = ballPath.get(i);
            Vector2d nextBall = i < ballPath.size() - 1 ? ballPath.get(i + 1) : null;
            BallType ballType = getBallType(curBall);
            BallType prevBallType = getBallType(i > 0 ? ballPath.get(i - 1) : robotPos);
            BallType nextBallType = nextBall == null ? null : getBallType(nextBall);

            Pose2d wallSafePreStrafeCollectPose = null;
            Pose2d wallSafeStrafeCollectPose = null;

            // keep adding cur ball into the cluster as long as it is close enough with next ball are close enough
            if (nextBall != null) {
                Vector2d curToNextBall = nextBall.minus(curBall);
                double curToNextBallDist = Math.hypot(curToNextBall.x, curToNextBall.y);
                if (curToNextBallDist < params.clusterStrafingDist) {
                    if (ballType == BallType.NORMAL && (originalBallPath.size() == 2 || nextBallType == BallType.NORMAL)) {
                        if (!allBallsInCluster.contains(curBall))
                            allBallsInCluster.add(curBall);
                        allBallsInCluster.add(nextBall);
                        clusterApproaches.add(new ClusterApproach(prevPathPos, curBall, nextBall));
                        continue;
                    }
                }
            }
            // this only runs if current cluster has been finished merging
            // once current cluster has finished, find approach information
            if (!allBallsInCluster.isEmpty()) {
                ClusterApproach clusterApproach = null;
                if (!clusterApproaches.isEmpty()) {
                    clusterApproach = getBestClusterApproach(clusterApproaches, allBallsInCluster);
                    problemBalls.addAll(clusterApproach.problemBalls);
                    if (clusterApproach.isWallSafe) {
                        wallSafePreStrafeCollectPose = clusterApproach.preCollectPose;
                        wallSafeStrafeCollectPose = clusterApproach.collectPose;
                    }
                    else
                        clusterApproach = null;
                }
                if (clusterApproach == null) {
                    boolean closeEnoughToMerge = true;
                    Vector2d average = MathUtils.getAverage(allBallsInCluster);
                    for (Vector2d ballInCluster : allBallsInCluster) {
                        if (MathUtils.vecDist(average, ballInCluster) > params.clusterMergeDist) {
                            closeEnoughToMerge = false;
                            break;
                        }
                    }

                    // merge
                    if (closeEnoughToMerge) {
                        curBall = MathUtils.getAverage(allBallsInCluster);
                        // remove old balls in cluster
                        for (int j=0; j<allBallsInCluster.size(); j++) {
                            Vector2d clusterBall = allBallsInCluster.get(j);
                            for (int k=0; k<ballPath.size(); k++) {
                                if (ballPath.get(k).equals(clusterBall)) {
                                    ballPath.remove(k);
                                    if (k <= i)
                                        i--;
                                    break;
                                }
                            }
                        }
                        ballPath.add(curBall);
                    }
                    // choose easiest
                    else {
                        double closestToPrevDist = -1;
                        Vector2d closestToPrevBall = null;
                        for (Vector2d ballInCluster : allBallsInCluster) {
                            double dist = MathUtils.vecMag(ballInCluster.minus(prevPathPos));
                            if (closestToPrevDist == -1 || dist < closestToPrevDist) {
                                closestToPrevDist = dist;
                                closestToPrevBall = ballInCluster;
                            }
                        }
                        curBall = closestToPrevBall;
                        // add balls that were failed to merge to list of problem balls
                        for (Vector2d ballInCluster : allBallsInCluster) {
                            if (!ballInCluster.equals(curBall))
                                problemBalls.add(new ProblemBall(ProblemBall.Severity.FAILED_CLUSTER_MERGE, ballInCluster));
                        }
                    }
                }
                allBallsInCluster.clear();
                clusterApproaches.clear();
            }

            // use cluster strafe collection
            if (wallSafePreStrafeCollectPose != null) {
                pathPoses.add(new PathPose(wallSafePreStrafeCollectPose, Types.PoseType.EDGE_CASE_PRECOLLECT, curBall, ballType, Types.Approach.CLUSTER_STRAFE));
                pathPoses.add(new PathPose(wallSafeStrafeCollectPose, Types.PoseType.COLLECT, curBall, ballType, Types.Approach.CLUSTER_STRAFE));
            }
            // treat the ball as normal
            else {
                double defaultApproachAngle = Math.atan2(curBall.y - prevPathPos.y, curBall.x - prevPathPos.x);
                double preCollectExtraOffset = 0;
                double collectExtraOffset = 0;

                // build collect and pre collect poses based on the type of point
                CollectInfo collectInfo = getCollectInfo(prevBallType, ballType, nextBallType, prevPathPose, prevBall, curBall, nextBall, defaultApproachAngle);

                double totalPreCollectOffset = collectInfo.preCollectOffset + preCollectExtraOffset + collectExtraOffset;
                Pose2d collectPose = getCollectPose(curBall, collectInfo.collectAngle, -collectExtraOffset);
                collectPose = new Pose2d(collectPose.position.plus(collectInfo.collectOffset), collectPose.heading);
                Pose2d preCollectPose = getPreCollectPose(collectPose, collectInfo.preCollectToCollectAngle, totalPreCollectOffset);

                Pose2d wallSafeCollectPose = getWallSafePose(collectPose);
                Pose2d wallSafePreCollectPose = getWallSafePose(preCollectPose);
                boolean validWallSafePose;
                switch (ballType) {
                    case CLASSIFIER_WALL:
                        validWallSafePose = wallSafePreCollectPose.position.x == preCollectPose.position.x;
                        break;
                    case BACK_WALL:
                        validWallSafePose = wallSafePreCollectPose.position.y == preCollectPose.position.y;
                        break;
                    case CORNER:
                        validWallSafePose = true; break;
                    default:
                        validWallSafePose = wallSafeCollectPose.equals(collectPose);
                }
                if (!validWallSafePose) {
                    wallSafePreCollectPose = getPreCollectPose(wallSafeCollectPose, collectInfo.preCollectToCollectAngle, totalPreCollectOffset);
                    wallSafePreCollectPose = getWallSafePose(wallSafePreCollectPose);
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.OUT_OF_BOUNDS, curBall));
                }
                // add extra collect dist for last ball
                if (nextBall == null && ballType == BallType.NORMAL) {
                    collectExtraOffset += params.lastCollectPoseExtraDriveThrough;
                    collectPose = getCollectPose(curBall, collectInfo.collectAngle, -collectExtraOffset);
                    wallSafeCollectPose = getWallSafePose(collectPose);
                }

                if (ballType == BallType.CLASSIFIER_WALL && prevBallType != BallType.CLASSIFIER_WALL) {
                    Vector2d[] robotCorners = getRobotCorners(wallSafePreCollectPose); // fr, fl, bl, fr
                    ArrayList<Vector2d> polygon = new ArrayList<>(Arrays.asList(robotCorners));
                    polygon.add(1, robotPos);
                    for (Vector2d ballToCheck : allBalls) {
                        if (originalBallPath.contains(ballToCheck) || getBallType(ballToCheck) != BallType.CLASSIFIER_WALL)
                            continue;
                        boolean hitting = GeometryUtils.isCircleInsidePolygon(polygon, ballToCheck, 1);
                        if (hitting)
                            problemBalls.add(new ProblemBall(ProblemBall.Severity.OVERLAPPING_CLASSIFIER_WALL, curBall));
                    }
                }

                Types.PoseType preCollectType = Types.PoseType.PRECOLLECT;
                if (collectInfo.approachType == Types.Approach.CORNER_LENIENT)
                    preCollectType = Types.PoseType.LENIENT_CORNER_PRECOLLECT;
                else if (ballType != BallType.NORMAL)
                    preCollectType = Types.PoseType.EDGE_CASE_PRECOLLECT;
                pathPoses.add(new PathPose(wallSafePreCollectPose, preCollectType, curBall, ballType, collectInfo.approachType));
                pathPoses.add(new PathPose(wallSafeCollectPose, Types.PoseType.COLLECT, curBall, ballType, collectInfo.approachType));
            }
        }

        // strictly bind path poses to inside the field
        for (int i=0; i<pathPoses.size(); i++) {
            PathPose cur = pathPoses.get(i);
            PathPose strictPathPose = new PathPose(getStrictWallSafePose(cur.pose), cur.poseType, cur.ball, cur.ballType, cur.approachType);
            pathPoses.set(i, strictPathPose);
        }
        // if ball n and ball n + 1 are normal collects
        // if ball n has poses that collide with ball n + 1, remove ball n + 1

//        for (int i=1; i<pathPoses.size() - 1; i++) {
//            PathPose prev = pathPoses.get(i - 1);
//            PathPose cur = pathPoses.get(i);
//            PathPose next = pathPoses.get(i + 1);
//
//            Vector2d prevToCurDir = cur.pose.position.minus(prev.pose.position);
//            prevToCurDir = prevToCurDir.div(MathUtils.vecMag(prevToCurDir));
//            Vector2d curToNextDir = next.pose.position.minus(cur.pose.position);
//            curToNextDir = curToNextDir.div(MathUtils.vecMag(curToNextDir));
//            System.out.println(i + " dot prod: " + prevToCurDir.dot(curToNextDir) + " prev type " + prev.ballType + " next type " + next.ballType);
//            if (prevToCurDir.dot(curToNextDir) < params.minDotProductBetweenPoints && prev.ballType != BallType.CLASSIFIER_WALL && next.ballType != BallType.CLASSIFIER_WALL)
//                problemBalls.add(new ProblemBall(ProblemBall.Severity.BACK_TRACKING, cur.ball));
//        }

        return new PathInfo(PathInfo.PathType.COMPLEX, robotPose, originalBallPath, ballPath, pathPoses, problemBalls);
    }
    private static ClusterApproach getBestClusterApproach(ArrayList<ClusterApproach> possibleApproaches, ArrayList<Vector2d> allBallsInCluster) {
        ClusterApproach bestApproach = null;
        // first sort based on if it is wall safe or not
        // then sort by difference in angle
        ArrayList<ClusterApproach> sortedApproaches = new ArrayList<>();
        for (int i=0; i<possibleApproaches.size(); i++) {
            ClusterApproach approach = possibleApproaches.get(i);
            approach.setAllBallsInCluster(allBallsInCluster);

            if (approach.isWallSafe)
                sortedApproaches.add(0, approach);
            else
                sortedApproaches.add(approach);
        }
        for (int i=0; i<sortedApproaches.size(); i++) {
            ClusterApproach approach = sortedApproaches.get(i);
            if (bestApproach == null) {
                bestApproach = approach;
                continue;
            }

            if (bestApproach.isWallSafe && !approach.isWallSafe)
                break;

            if (Math.abs(approach.angleDiff) < Math.abs(bestApproach.angleDiff))
                bestApproach = approach;
        }
        return bestApproach;
    }
    private static class ClusterApproach {
        public final double approachAngle, collectAngle;
        public final Vector2d start, end, approxCenter;
        public ArrayList<Vector2d> allBallsInCluster;
        public ArrayList<ClusterBallInfo> includedBallsInfo;
        public final double angleDiff;
        public Vector2d exactCenter;
        public boolean isWallSafe;
        public Pose2d preCollectPose;
        public Pose2d collectPose;
        public final ArrayList<ProblemBall> problemBalls;
        public ClusterApproach(Vector2d prev, Vector2d start, Vector2d end) {
            this.start = start;
            this.end = end;
            problemBalls = new ArrayList<>();
            approxCenter = start.plus(end).times(0.5);
            approachAngle = MathUtils.v1ToV2Angle(start, end);

            Vector2d prevToStart = start.minus(prev);
            Vector2d startToEnd = end.minus(start);
            angleDiff = MathUtils.angleRadDiff(startToEnd, prevToStart);

            double maxStrafeAngleOffset = Math.min(Math.abs(angleDiff), Math.toRadians(params.strafeCollectMaxAngleOffset));
            if (angleDiff > 0)
                collectAngle = approachAngle - maxStrafeAngleOffset;
            else
                collectAngle = approachAngle + maxStrafeAngleOffset;
        }
        public void setAllBallsInCluster(ArrayList<Vector2d> allBallsInCluster) {
            this.allBallsInCluster = new ArrayList<>(allBallsInCluster);
            Vector2d approachDir = new Vector2d(Math.cos(approachAngle), Math.sin(approachAngle));
            Vector2d orthogonalApproachDir = new Vector2d(-approachDir.y, 1*approachDir.x);

            Vector2d actualStart = start;
            Vector2d actualStartRawBallPosition = start;
            Vector2d actualEnd = end;
            double minParallelOffset = 0;
            double maxParallelOffset = 0;

            includedBallsInfo = new ArrayList<>();
            for (Vector2d potentialBall : allBallsInCluster) {
                Vector2d ballToApproxCenter = potentialBall.minus(approxCenter);

                double parallelOffset = ballToApproxCenter.dot(approachDir);
                double perpendicularOffset = ballToApproxCenter.dot(orthogonalApproachDir);
                Vector2d projected = approxCenter.plus(approachDir.times(parallelOffset));
                if (Math.abs(perpendicularOffset) < params.strafeCollectMaxPerpendicularDistance)
                    includedBallsInfo.add(new ClusterBallInfo(potentialBall, parallelOffset, perpendicularOffset, projected));
                else
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.FAILED_CLUSTER_APPROACH, potentialBall));

                if (parallelOffset < minParallelOffset) {
                    minParallelOffset = parallelOffset;
                    actualStart = projected;
                    actualStartRawBallPosition = potentialBall;
                }
                else if (parallelOffset > maxParallelOffset) {
                    maxParallelOffset = parallelOffset;
                    actualEnd = projected;
                }
            }

            double totalX = 0;
            double totalY = 0;
            for (ClusterBallInfo ballInfo : includedBallsInfo) {
                totalX += ballInfo.position.x;
                totalY += ballInfo.position.y;
            }
            exactCenter = new Vector2d(totalX, totalY).div(includedBallsInfo.size()); // not used right now but might use later if we get wider intake

            collectPose = getWallSafePose(getCollectPose(actualEnd, collectAngle, 0)); // not great but its fine
            Pose2d startCollectPose = getCollectPose(actualStart, collectAngle, 0);
            preCollectPose = getPreCollectPose(startCollectPose, approachAngle, params.preCollectOffset);
            Pose2d wallSafePreCollectPose = getWallSafePose(preCollectPose);
            isWallSafe = wallSafePreCollectPose.equals(preCollectPose);
            if (!isWallSafe) {
                problemBalls.add(new ProblemBall(ProblemBall.Severity.OUT_OF_BOUNDS, actualStartRawBallPosition));
                startCollectPose = getCollectPose(start, collectAngle, 0);
                preCollectPose = getPreCollectPose(startCollectPose, approachAngle, params.preCollectOffset);
                wallSafePreCollectPose = getWallSafePose(preCollectPose);
                isWallSafe = wallSafePreCollectPose.equals(preCollectPose);
                if (!isWallSafe)
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.OUT_OF_BOUNDS, start));
            }
        }
    }
    private static class ClusterBallInfo {
        public final Vector2d position;
        public final double parallelOffset, perpendicularOffset;
        public final Vector2d projectedPosition;
        public ClusterBallInfo(Vector2d position, double parallelOffset, double perpendicularOffset, Vector2d projectedPosition) {
            this.position = position;
            this.parallelOffset = parallelOffset;
            this.perpendicularOffset = perpendicularOffset;
            this.projectedPosition = projectedPosition;
        }
    }
    private static BallType getBallType(Vector2d curBall) {
        Vector2d cornerPosition = new Vector2d(curBall.x > 0 ? 72 : -72, curBall.y > 0 ? 72 : -72);
        double distFromClassifierWall = Math.abs(cornerPosition.y - curBall.y);
        double distFromBackWall = Math.abs(cornerPosition.x - curBall.x);
        double distFromCorner = Math.hypot(distFromClassifierWall, distFromBackWall);

        BallType ballType = BallType.NORMAL;
        if (distFromCorner < params.cornerBallDistance)
            ballType = BallType.CORNER;
        else if (distFromClassifierWall < params.classifierWallDistance)
            ballType = BallType.CLASSIFIER_WALL;
        else if (distFromBackWall < params.backWallDistance)
            ballType = BallType.BACK_WALL;
        return ballType;
    }
    private static CollectInfo getCollectInfo(BallType prevBallType, BallType ballType, BallType nextBallType, Pose2d prevPathPose, Vector2d prevBall, Vector2d curBall, Vector2d nextBall, double defaultApproachAngle) {
        Vector2d prevPathPos = prevPathPose.position;
        double wallAngle = curBall.y > 0 ? Math.toRadians(90) : Math.toRadians(-90);
        Vector2d curToNextBall = nextBall == null ? null : nextBall.minus(curBall);
        double curToNextBallDist = curToNextBall == null ? Double.MAX_VALUE : MathUtils.vecMag(curToNextBall);
        double preCollectToCollectAngle = 0;
        double collectAngle = 0;
        double preCollectOffset = params.preCollectOffset;
        double collectXOffset = 0;
        double collectYOffset = 0;
        Types.Approach approachType = Types.Approach.NORMAL;
        switch (ballType) {
            case CORNER:
                approachType = Types.Approach.CORNER_CONSTRAINED;
                collectAngle = Math.signum(wallAngle) * (Math.abs(wallAngle) - Math.toRadians(params.cornerCollectAngle));
                preCollectToCollectAngle = collectAngle;
                collectYOffset = Math.signum(curBall.y) * Math.max(0, params.cornerCollectY - Math.abs(curBall.y));
                preCollectOffset += Math.abs(collectYOffset);
                if (prevBallType == BallType.CLASSIFIER_WALL || prevBallType == BallType.BACK_WALL) {
                    double dist = MathUtils.vecDist(prevBall, curBall);
                    if (dist < params.lenientCornerCollectThreshold) {
                        approachType = Types.Approach.CORNER_LENIENT;
                        collectAngle = MathUtils.averageAngle(collectAngle, prevPathPose.heading.toDouble());
                        preCollectToCollectAngle = collectAngle * 0.5;
                        preCollectOffset = 0;
                        collectYOffset = 0;
                    }
                }
                break;
            case CLASSIFIER_WALL:
                double angleD = Math.abs(MathUtils.angleNormDeltaRad(defaultApproachAngle - wallAngle));
                boolean useStrafeApproach = angleD > Math.toRadians(params.wallStrafeCollectMinApproachAngle)
                        || nextBallType == BallType.CORNER || nextBallType == BallType.CLASSIFIER_WALL
                        || prevBallType == BallType.CORNER || prevBallType == BallType.CLASSIFIER_WALL
                        || (nextBallType == BallType.BACK_WALL && curToNextBallDist < params.distToNextPointForceClassifierStrafe);
                if (useStrafeApproach) {
                    approachType = Types.Approach.CLASSIFIER_WALL_STRAFE;
                    double dx = curToNextBall == null ? curBall.x - prevPathPos.x : curToNextBall.x;
                    collectXOffset = Math.signum(dx) * params.strafeCollectDriveThroughDist;
                    preCollectOffset += Math.abs(collectXOffset);
                    if (Math.signum(dx) == 1)
                        collectAngle = Math.toRadians(params.wallCollectAngle);
                    else
                        collectAngle = Math.PI - Math.toRadians(params.wallCollectAngle);
                    collectAngle *= Math.signum(wallAngle);
                    preCollectToCollectAngle = Math.signum(dx) == 1 ? 0 : Math.PI;

                    Pose2d collectPose = getCollectPose(curBall, collectAngle, 0);
                    collectPose = new Pose2d(collectPose.position, collectPose.heading);
                    Pose2d preCollectPose = getPreCollectPose(collectPose, preCollectToCollectAngle, params.preCollectOffset);
                    if (preCollectPose.position.x != getWallSafePose(preCollectPose).position.x)
                        useStrafeApproach = false;
                }

                if (!useStrafeApproach) {
                    approachType = Types.Approach.NORMAL;
                    collectAngle = defaultApproachAngle;
                    preCollectToCollectAngle = collectAngle;
                    collectXOffset = 0;
                }
                break;
            case BACK_WALL:
                if (Math.abs(defaultApproachAngle) > Math.toRadians(params.wallStrafeCollectMinApproachAngle)) {
                    approachType = Types.Approach.BACK_WALL_STRAFE;
                    double dy = curBall.y - prevPathPos.y;
                    collectYOffset = Math.signum(dy) * params.strafeCollectDriveThroughDist;
                    preCollectOffset += Math.abs(collectYOffset);
                    collectAngle = Math.toRadians(params.wallCollectAngle) * Math.signum(dy);
                    preCollectToCollectAngle = Math.toRadians(90) * Math.signum(dy);
                }
                else {
                    approachType = Types.Approach.NORMAL;
                    collectAngle = defaultApproachAngle;
                    preCollectToCollectAngle = collectAngle;
                }
                break;
            case NORMAL:
            default:
                approachType = Types.Approach.NORMAL;
                collectAngle = defaultApproachAngle;
                preCollectToCollectAngle = collectAngle;
                break;
        }
        return new CollectInfo(approachType, preCollectOffset, preCollectToCollectAngle, collectAngle, collectXOffset, collectYOffset);
    }

    private static class CollectInfo {
        public final double preCollectOffset, preCollectToCollectAngle, collectAngle;
        public final Types.Approach approachType;
        public final Vector2d collectOffset;
        public CollectInfo(Types.Approach approachType, double preCollectOffset, double preCollectToCollectAngle, double collectAngle, double collectXOffset, double collectYOffset) {
            this.approachType = approachType;
            this.preCollectOffset = preCollectOffset;
            this.preCollectToCollectAngle = preCollectToCollectAngle;
            this.collectAngle = collectAngle;
            this.collectOffset = new Vector2d(collectXOffset, collectYOffset);
        }
    }
    protected static ArrayList<PathPose> simplifyPathPoses(Pose2d startPose, ArrayList<PathPose> pathPoses) {
        ArrayList<PathPose> simplified = new ArrayList<>();
        if (pathPoses.isEmpty())
            return simplified;
        if (pathPoses.size() == 1) {
            simplified.add(pathPoses.get(0));
            return simplified;
        }

        for (int i=0; i<pathPoses.size()-1; i++) {
            PathPose prev = !simplified.isEmpty() ? simplified.get(simplified.size() - 1) : null;
            Pose2d prevPose = prev != null ? prev.pose : startPose;
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i + 1);

            // skip any poses between classifier or back wall strafes
            if (prev != null) {
                if (prev.approachType == Types.Approach.CLASSIFIER_WALL_STRAFE && next.approachType == Types.Approach.CLASSIFIER_WALL_STRAFE)
                    continue;
                if (prev.approachType == Types.Approach.BACK_WALL_STRAFE && next.approachType == Types.Approach.BACK_WALL_STRAFE)
                    continue;
            }
            if (cur.approachType == Types.Approach.CLASSIFIER_WALL_STRAFE &&
                    (next.approachType == Types.Approach.CORNER_CONSTRAINED || next.approachType == Types.Approach.CORNER_LENIENT)) {
                simplified.add(pathPoses.get(i));
                continue;
            }

            Vector2d prevToCur = cur.pose.position.minus(prevPose.position);
            Vector2d curToNext = next.pose.position.minus(cur.pose.position);

            double prevToCurHeadingDiff = MathUtils.angleNormDeltaRad(cur.pose.heading.toDouble() - prevPose.heading.toDouble());
            double curToNextHeadingDiff = MathUtils.angleNormDeltaRad(next.pose.heading.toDouble() - cur.pose.heading.toDouble());
            double approachAngleDiff = MathUtils.angleRadDiff(curToNext, prevToCur);

            double prevToCurDist = MathUtils.vecMag(prevToCur);
            double curToNextDist = MathUtils.vecMag(curToNext);
            double prevToCurHeadingSimplification = Math.toRadians(params.maxCollectHeadingDifference.applyAsDouble(prevToCurDist));
            double curToNextHeadingSimplification = Math.toRadians(params.maxCollectHeadingDifference.applyAsDouble(curToNextDist));
//            System.out.println(i + ": prev to cur heading diff: " + Math.toDegrees(prevToCurHeadingDiff) + " | cur to next heading diff: " + Math.toDegrees(curToNextHeadingDiff));
//            System.out.println("prev to cur heading simpli: " + Math.toDegrees(prevToCurHeadingSimplification) + " | cur to next heading simpli: " + Math.toDegrees(curToNextHeadingSimplification));
//            System.out.println(i + ": " + Math.toDegrees(approachAngleDiff));
            double approachAngleSimplification = Math.toRadians(params.maxCollectApproachDifference);
            boolean approachAnglesCloseEnough = Math.abs(approachAngleDiff) < approachAngleSimplification;
            boolean headingCloseEnough = Math.abs(prevToCurHeadingDiff) < prevToCurHeadingSimplification && Math.abs(curToNextHeadingDiff) < curToNextHeadingSimplification;
            if (headingCloseEnough && approachAnglesCloseEnough)
                continue;


            simplified.add(pathPoses.get(i));
        }
        simplified.add(pathPoses.get(pathPoses.size() - 1));
        return simplified;
    }

    // returns [fr, fl, bl, br] (counter clockwise)
    private static Vector2d[] getRobotCorners(Pose2d robotPose) {
        Vector2d position = robotPose.position;
        double halfW = params.robotWidth * 0.5;
        double halfL = params.robotLength * 0.5;

        Vector2d[] positions = new Vector2d[] {
                new Vector2d(halfL, -halfW), // fr
                new Vector2d(halfL, halfW), // fl
                new Vector2d(-halfL, halfW), // bl
                new Vector2d(-halfL, -halfW), // br
        };
        for (int i=0; i<positions.length; i++) {
            Vector2d pos = GeometryUtils.rotateVector(positions[i], robotPose.heading.toDouble()).plus(position);
            positions[i] = pos;
        }
        return positions;
    }
    private static Pose2d getWallSafePose(Pose2d robotPose, double wallBuffer) {
        Vector2d position = robotPose.position;
        double halfW = params.robotWidth * 0.5;
        double halfL = params.robotLength * 0.5;

        double d = Math.hypot(halfW, halfL) - wallBuffer;
        if (position.x > -72 + d && position.x < 72 - d && position.y > -72 + d && position.y < 72 - d)
            return robotPose;

        Vector2d[] corners = getRobotCorners(robotPose);
        Vector2d[] requiredOffsets = new Vector2d[corners.length];
        for (int i=0; i<corners.length; i++) {
            Vector2d corner = corners[i];
            double wallY = Math.signum(corner.y) * (72 + wallBuffer);

            double dx = (72 + wallBuffer) - corner.x;
            double dy = wallY - corner.y;
            dx = Math.min(0, dx);
            if (wallY > 0)
                dy = Math.min(0, dy);
            else
                dy = Math.max(0, dy);
            requiredOffsets[i] = new Vector2d(dx, dy);
        }
        double requiredDx = requiredOffsets[0].x;
        double requiredDy = requiredOffsets[0].y;
        for (Vector2d offset : requiredOffsets) {
            requiredDx = Math.min(offset.x, requiredDx);
            if (Math.abs(offset.y) > Math.abs(requiredDy))
                requiredDy = offset.y;
        }

        return new Pose2d(position.x + requiredDx, position.y + requiredDy, robotPose.heading.toDouble());
    }
    public static Pose2d getStrictWallSafePose(Pose2d robotPose) {
        return getWallSafePose(robotPose, 0);
    }
    public static Pose2d getWallSafePose(Pose2d robotPose) {
        return getWallSafePose(robotPose, params.wallPoseBuffer);
    }

    private static Pose2d getCollectPose(Vector2d ballPosition, double angle, double extraOffset) {
        double r = params.collectPoseOffsetDistance + extraOffset;
        double dx = Math.cos(angle) * r;
        double dy = Math.sin(angle) * r;
        return new Pose2d(ballPosition.x - dx, ballPosition.y - dy, angle);
    }
    // assumes same heading as collect pose
    // the angle from pre collect to collect is not constrained to collect pose heading though
    private static Pose2d getPreCollectPose(Pose2d collectPose, double angleToCollectPose, double offsetDistance) {
        double dx = Math.cos(angleToCollectPose) * offsetDistance;
        double dy = Math.sin(angleToCollectPose) * offsetDistance;
        return new Pose2d(collectPose.position.x - dx, collectPose.position.y - dy, collectPose.heading.toDouble());
    }
}
