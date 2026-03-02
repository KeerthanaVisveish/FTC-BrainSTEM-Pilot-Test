package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.CircleTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.RotatedBoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static PathGenerationParams pathGenParams = new PathGenerationParams();
    public static PathDriveParams driveParams = new PathDriveParams();
    public static Pose2d pathfinderStartPose = null;
    public static PathInfo generateSimplifiedAutoCollectPath(Pose2d robotPose, ArrayList<Vector2d> ballsArray) {
        PathInfo path = generateAutoCollectPath(robotPose, ballsArray);
        if (path == null)
            return null;

        path.setSimplifiedPathPoses(PathGeneration.simplifyPathPoses(robotPose, path.pathPoses));
        return path;
    }
    private static PathInfo generateAutoCollectPath(Pose2d robotPose, ArrayList<Vector2d> ballPositionsArray) {
        pathfinderStartPose = robotPose;
        if (ballPositionsArray.isEmpty())
            return null;

        ArrayList<Ball> allBalls = Ball.toBallList(new ArrayList<>(ballPositionsArray));

        ArrayList<Lane> densestLanes = getDensestLanes(allBalls);
        Lane bestLane = getBestLane(robotPose.position, densestLanes);
        if (pathGenParams.generationMode != PathGenerationParams.GenerationMode.COMPLEX_ONLY) {
            boolean shouldUseSimple = pathGenParams.generationMode == PathGenerationParams.GenerationMode.SIMPLE_ONLY ||
                    densestLanes.get(0).numBalls() >= pathGenParams.minBallsToUseSimpleGeneration ||
                    allBalls.size() == 2 && bestLane.numBalls() == 2;
            if (shouldUseSimple)
                return generateLanePath(robotPose, bestLane);
        }

        int numAttempts = 0;
        PathInfo optimalPathInfo = null;
        int leastProblemBalls = -1;
        ArrayList<Ball> balls = new ArrayList<>(allBalls);
        ArrayList<Ball> ignoredBalls = new ArrayList<>();
        do {
            System.out.println("regeneration " + numAttempts);
            ArrayList<Vector2d> ballPositions = Ball.toVecList(balls);
            ArrayList<Vector2d> rawPath = PathFinder.findShortestPath(robotPose, ballPositions, 3, pathGenParams.changeInAngleDegCost, 0,0);
            if (rawPath == null)
                break;
            ArrayList<Ball> rawBallPath = Ball.toBallList(rawPath);

            // generate path
            System.out.println("normal path generation========");
            PathInfo pathInfo = generatePath(robotPose, allBalls, rawBallPath);
            pathInfo.setIgnoredBalls(ignoredBalls);
            int numProblemBalls = pathInfo.problemBalls.size();

            // artificially shift robot to the left to see if it generates a more desirable path
            Pose2d shiftedLeftRobotPose = new Pose2d(pathGenParams.shiftedLeftStartX, robotPose.position.y, robotPose.heading.toDouble());
            ArrayList<Vector2d> shiftedLeftRawPath = PathFinder.findShortestPath(shiftedLeftRobotPose, ballPositions, 3, pathGenParams.changeInAngleDegCost, 0,0);
            if (shiftedLeftRawPath != null) {
                ArrayList<Ball> shiftedLeftRawBallPath = Ball.toBallList(shiftedLeftRawPath);
                System.out.println("shifted left path generation========");
                PathInfo shiftedLeftPathInfo = generatePath(robotPose, allBalls, shiftedLeftRawBallPath);
                if (shiftedLeftPathInfo.numGoodBalls() > pathInfo.numGoodBalls()) {
                    pathfinderStartPose = shiftedLeftRobotPose;
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
//                System.out.println(problemBall.severity + ": " + problemBall.ballPosition);
                if (problemBall.severity.ordinal() <= lowestSeverityOrdinal)
                    lowestSeverityOrdinal = problemBall.severity.ordinal();
            }
//            System.out.println("lowest ordinal: " + lowestSeverityOrdinal);
            for (ProblemBall problemBall : pathInfo.problemBalls)
                if (problemBall.severity.ordinal() == lowestSeverityOrdinal) {
                    balls.remove(problemBall);
                    ignoredBalls.add(problemBall);
                }

            numAttempts++;
        } while (!balls.isEmpty() && numAttempts <= pathGenParams.maxPathRegenerationAttempts);

        if (optimalPathInfo == null) {
            return null;
        }

        if (pathGenParams.generationMode != PathGenerationParams.GenerationMode.COMPLEX_ONLY) {
            int complexNumBalls = optimalPathInfo.numGoodBalls();
            int laneNumBalls = bestLane.numBalls();
            if (complexNumBalls == laneNumBalls)
                return complexNumBalls == 1 ? optimalPathInfo : generateLanePath(robotPose, bestLane);
            return complexNumBalls > laneNumBalls ? optimalPathInfo : generateLanePath(robotPose, bestLane);
        }
        return optimalPathInfo;
    }

    // use a sliding window of width laneWidth moving at a speed of increment
    // returns the list of balls in the most densely packed moment of the window
    private static ArrayList<Lane> getDensestLanes(ArrayList<Ball> balls) {
        Ball[] sorted = Arrays.copyOf(balls.toArray(new Ball[0]), balls.size());
        Arrays.sort(sorted, Comparator.comparingDouble(a -> a.pos.x));
        int start = (int) sorted[0].pos.x - 1;
        int end = (int) sorted[sorted.length-1].pos.x + 1;

        double increment = pathGenParams.laneIncrement;
        int numLanes = (int) ((end - pathGenParams.normalLaneWidth - start + 2) / increment);
        if (numLanes <= 0)
            numLanes = 1;
        ArrayList<Lane> lanes = new ArrayList<>();

        for (int i=0; i<numLanes; i++) {
            double left = start + i * increment;
            boolean againstBackWall = left + pathGenParams.normalLaneWidth >= 72 - pathGenParams.laneAgainstBackWallMaxDist;
            double laneWidth = againstBackWall ? pathGenParams.againstBackWallLaneWidth : pathGenParams.normalLaneWidth;
            double right = left + laneWidth;
            ArrayList<Ball> ballsInLane = new ArrayList<>();
            for (Ball ball : sorted)
                if (ball.pos.x >= left && ball.pos.x <= right)
                    ballsInLane.add(ball);
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
        return validLanes;
    }
    // returns the lane closest to the robot
    private static Lane getBestLane(Vector2d robotPos, ArrayList<Lane> densestLanes) {
        if (densestLanes.size() == 1)
            return densestLanes.get(0);
        Lane bestLane = null;
        double minDist = -1;
        ArrayList<Lane> sortedLanes = new ArrayList<>(densestLanes);
        sortedLanes.sort(Comparator.comparingDouble(l -> l.optimizedWidth));
        if (sortedLanes.get(0).optimizedWidth <= pathGenParams.ignoreOptimizedLaneWidthSortingWidth) {
            while (sortedLanes.get(sortedLanes.size() - 1).optimizedWidth > pathGenParams.ignoreOptimizedLaneWidthSortingWidth)
                sortedLanes.remove(sortedLanes.size() - 1);
        }
        for (int i=0; i<sortedLanes.size(); i++) {
            Lane lane = sortedLanes.get(i);
            Vector2d firstPoint = new Vector2d(lane.avgX, lane.minAbsY());
            double dist = Math.hypot(firstPoint.x - robotPos.x, firstPoint.y - robotPos.y);

            if (bestLane == null || lane.closeToBackWall && !bestLane.closeToBackWall
                    || (bestLane.closeToBackWall == lane.closeToBackWall && dist < minDist)) {
                minDist = dist;
                bestLane = lane;
            }
        }
        return bestLane;
    }
    private static class Lane {
        private final ArrayList<Ball> balls;
        public boolean closeToBackWall;
        private final double avgX;
        private final double optimizedWidth;
        public Lane(ArrayList<Ball> balls, boolean closeToBackWall) {
            this.balls = new ArrayList<>(balls);
            this.closeToBackWall = closeToBackWall;
            // sort by |y| value ascending (so it works for both red and blue)
            this.balls.sort(Comparator.comparingDouble(v -> Math.abs(v.pos.y)));
            if (balls.isEmpty()) {
                this.avgX = 0;
                this.optimizedWidth = 0;
            }
            else {
                double minX = balls.get(0).pos.x;
                double maxX = balls.get(0).pos.x;
                double totalX = 0;
                for (Ball ball : balls) {
                    totalX += ball.pos.x;
                    minX = Math.min(ball.pos.x, minX);
                    maxX = Math.max(ball.pos.x, maxX);
                }
                this.avgX = totalX / balls.size();
                this.optimizedWidth = maxX - minX;
            }
        }
        public int numBalls() {
            return balls.size();
        }
        public double minAbsY() {
            return balls.get(0).pos.y;
        }
        public double maxAbsY() {
            return balls.get(balls.size() - 1).pos.y;
        }
        @Override
        public String toString() {
            return "avgx: " + avgX + " | num balls: " + numBalls();
        }
    }
    private static PathInfo generateLanePath(Pose2d startPose, Lane lane) {
        double angle = Math.signum(lane.balls.get(0).pos.y) * Math.toRadians(90);
        Ball ball1 = new Ball(new Vector2d(lane.avgX, lane.minAbsY()));
        Ball ball2 = new Ball(new Vector2d(lane.avgX, lane.maxAbsY()));

        if (lane.avgX >= 72 - pathGenParams.angleLaneCollectDistFromBackWall)
            angle -= Math.signum(angle) * Math.toRadians(pathGenParams.cornerCollectAngle);

        Pose2d collect1 = getCollectPose(ball1.pos, angle, 0);
        Pose2d collect2 = getCollectPose(ball2.pos, angle, -pathGenParams.lastCollectPoseExtraDriveThrough);
        Pose2d preCollect1 = getPreCollectPose(collect1, angle, pathGenParams.preCollectOffset);
        Pose2d wallSafeCollect2 = getStrictWallSafePose(collect1);
        Pose2d wallSafeCollect3 = getStrictWallSafePose(collect2);
        Pose2d wallSafePreCollect = getStrictWallSafePose(preCollect1);

        Waypoint waypoint1 = new Waypoint(wallSafePreCollect, new CircleTolerance());
        Waypoint waypoint2 = new Waypoint(wallSafeCollect2, new CircleTolerance()).setPassPosition(true).setMinLinearPower(driveParams.laneCollectMinLinearPower);
        Waypoint waypoint3 = new Waypoint(wallSafeCollect3, new CircleTolerance()).setPassPosition(true).setMinLinearPower(driveParams.laneCollectMinLinearPower);
        PathPose pathPose1 = new PathPose(waypoint1, Types.PoseType.PRECOLLECT, ball1, Types.Approach.NORMAL);
        PathPose pathPose2 = new PathPose(waypoint2, Types.PoseType.COLLECT, ball1, Types.Approach.NORMAL);
        PathPose pathPose3 = new PathPose(waypoint3, Types.PoseType.COLLECT, ball2, Types.Approach.NORMAL);
        ArrayList<PathPose> pathPoses = new ArrayList<>(Arrays.asList(pathPose1, pathPose2, pathPose3));
        return new PathInfo(PathInfo.PathType.LANE, startPose, lane.balls, pathPoses, new ArrayList<>());
    }
    private static PathInfo generatePath(Pose2d robotPose, ArrayList<Ball> allBalls, ArrayList<Ball> originalBallPath) {
        ArrayList<Ball> ballPath = new ArrayList<>(originalBallPath);
        Vector2d robotPos = robotPose.position;
        ArrayList<PathPose> pathPoses = new ArrayList<>();
        ArrayList<ProblemBall> problemBalls = new ArrayList<>();
        if (ballPath.isEmpty())
            return new PathInfo(PathInfo.PathType.EMPTY, robotPose, ballPath, pathPoses, problemBalls);

        // reject balls along the wall that are too far away from each other
        // also reject balls along the wall and in the corner that are too close to each other
        if (ballPath.size() >= 2) {
            for (int i=1; i<ballPath.size(); i++) {
                Ball prevBall = ballPath.get(i - 1);
                Ball curBall = ballPath.get(i);

                if (curBall.type != Ball.BallType.NORMAL && prevBall.type != Ball.BallType.NORMAL) {
                    double horizontalDist = Math.abs(curBall.pos.x - prevBall.pos.x);

                    if (horizontalDist > pathGenParams.rejectEdgeCaseBallsHorizontalDist) {
                        if (ballPath.size() == 2) {
                            // choose closer one
                            double curDist = MathUtils.vecDist(curBall.pos, robotPos);
                            double prevDist = MathUtils.vecDist(prevBall.pos, robotPos);
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
                }
            }
        }

        // set the path indexes of the ball
        for (int i=0; i<ballPath.size(); i++)
            ballPath.get(i).pathIndex = i;
        System.out.println("BALL PATH AT BEGINNING OF PATH GENERATION: ");
        System.out.println(ballPath);

        ArrayList<Ball> allBallsInCluster = new ArrayList<>();

        ArrayList<ClusterApproachPath> validClusterApproachPaths = new ArrayList<>();
        for (int i=0; i<ballPath.size(); i++) {
            Ball prev = i == 0 ? new Ball(robotPos) : ballPath.get(i - 1);
            Ball cur = ballPath.get(i);
            Ball next = i < ballPath.size() - 1 ? ballPath.get(i + 1) : null;
            // keep adding cur ball into the cluster as long as it is close enough with next ball are close enough
            if (next != null) {
                Vector2d curToNextBall = next.pos.minus(cur.pos);
                double curToNextBallDist = Math.hypot(curToNextBall.x, curToNextBall.y);
                if (curToNextBallDist < pathGenParams.clusterStrafingDist) {
                    if (cur.type == Ball.BallType.NORMAL && (ballPath.size() == 2 || next.type == Ball.BallType.NORMAL)) {
                        if (!allBallsInCluster.contains(cur)) {
                            allBallsInCluster.add(cur);
                        }
                        allBallsInCluster.add(next);
                        continue;
                    }
                }
            }
            // this only runs if current cluster has been finished merging
            // once current cluster has finished, find approach information
            if (!allBallsInCluster.isEmpty()) {
                ArrayList<ClusterApproach> clusterApproaches = new ArrayList<>();
                for (int j=0; j<allBallsInCluster.size() - 1; j++) {
                    Ball curClusterBall = allBallsInCluster.get(j);
                    Ball prevClusterBall = curClusterBall.pathIndex == 0 ? new Ball(robotPos) : ballPath.get(curClusterBall.pathIndex - 1);
                    Ball nextClusterBall = ballPath.get(curClusterBall.pathIndex + 1);
                    clusterApproaches.add(new ClusterApproach(prevClusterBall, curClusterBall, nextClusterBall, allBallsInCluster));
                }
                System.out.println("all cluster approaches: " + clusterApproaches);
                ClusterApproach clusterApproach = getBestClusterApproach(clusterApproaches);
                problemBalls.addAll(clusterApproach.problemBalls);
                if (clusterApproach.isWallSafe) {
                    ClusterApproachPath approachPath = new ClusterApproachPath(clusterApproach.start, clusterApproach.end, clusterApproach.pathPoses);
                    validClusterApproachPaths.add(approachPath);
                }
                else {
                    boolean closeEnoughToMerge = true;
                    Vector2d average = MathUtils.getAverage(Ball.toVecList(allBallsInCluster));
                    for (Ball ballInCluster : allBallsInCluster)
                        if (MathUtils.vecDist(average, ballInCluster.pos) > pathGenParams.clusterMergeDist) {
                            closeEnoughToMerge = false;
                            break;
                        }

                    // merge
                    if (closeEnoughToMerge) {
                        cur = new Ball(MathUtils.getAverage(Ball.toVecList(allBallsInCluster)));
                    }
                    // choose easiest
                    else {
                        double closestToPrevDist = -1;
                        Ball closestToPrevBall = null;
                        for (Ball ballInCluster : allBallsInCluster) {
                            double dist = MathUtils.vecMag(ballInCluster.pos.minus(prev.pos));
                            if (closestToPrevDist == -1 || dist < closestToPrevDist) {
                                closestToPrevDist = dist;
                                closestToPrevBall = ballInCluster;
                            }
                        }
                        cur = closestToPrevBall;
                        // add balls that were failed to merge to list of problem balls
                        for (Ball ballInCluster : allBallsInCluster) {
                            if (!ballInCluster.equals(cur)) {
                                problemBalls.add(new ProblemBall(ProblemBall.Severity.FAILED_CLUSTER_MERGE, ballInCluster.pos));
                            }
                        }
                    }
                }
                allBallsInCluster.clear();
                clusterApproaches.clear();
            }
        }

        for (int i=0; i<ballPath.size(); i++) {
            Pose2d prevPathPose = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).waypoint.pose : robotPose;
            Vector2d prevPathPos = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).waypoint.pose.position : robotPos;
            Ball prev = i == 0 ? new Ball(robotPos) : ballPath.get(i - 1);
            Ball cur = ballPath.get(i);
            Ball next = i < ballPath.size() - 1 ? ballPath.get(i + 1) : null;


            boolean clusterAccountsForBall = false;
            for (ClusterApproachPath clusterApproachPath : validClusterApproachPaths) {
                // only create poses first time
                if (clusterApproachPath.clusterStartBall.pathIndex == i) {
                    pathPoses.addAll(clusterApproachPath.pathPoses);
                    clusterAccountsForBall = true;
                    break;
                }
                if (clusterApproachPath.accountsForBallAtIndex(i)) {
                    clusterAccountsForBall = true;
                    break;
                }
            }

            // treat the ball as normal
            if (!clusterAccountsForBall) {
                double defaultApproachAngle = Math.atan2(cur.pos.y - prevPathPos.y, cur.pos.x - prevPathPos.x);
                double preCollectExtraOffset = 0;
                double collectExtraOffset = 0;

                // build collect and pre collect poses based on the type of point
                CollectInfo collectInfo = getCollectInfo(prevPathPose, prev, cur, next, defaultApproachAngle);

                double totalPreCollectOffset = collectInfo.preCollectOffset + preCollectExtraOffset + collectExtraOffset;
                Pose2d collectPose = getCollectPose(cur.pos, collectInfo.collectAngle, -collectExtraOffset);
                collectPose = new Pose2d(collectPose.position.plus(collectInfo.collectOffset), collectPose.heading);
                Pose2d preCollectPose = getPreCollectPose(collectPose, collectInfo.preCollectToCollectAngle, totalPreCollectOffset);

                Pose2d wallSafeCollectPose = getWallSafePose(collectPose);
                Pose2d wallSafePreCollectPose = getWallSafePose(preCollectPose);
                boolean validWallSafePose;
                switch (cur.type) {
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
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.OUT_OF_BOUNDS, cur.pos));
                }
                // add extra collect dist for last ball
                if (next == null && cur.type == Ball.BallType.NORMAL) {
                    collectExtraOffset += pathGenParams.lastCollectPoseExtraDriveThrough;
                    collectPose = getCollectPose(cur.pos, collectInfo.collectAngle, -collectExtraOffset);
                    wallSafeCollectPose = getWallSafePose(collectPose);
                }

                if (cur.type == Ball.BallType.CLASSIFIER_WALL && prev.type != Ball.BallType.CLASSIFIER_WALL) {
                    Vector2d[] robotCorners = getRobotCorners(wallSafePreCollectPose); // fr, fl, bl, fr
                    ArrayList<Vector2d> polygon = new ArrayList<>(Arrays.asList(robotCorners));
                    polygon.add(1, robotPos);
                    for (Ball ballToCheck : allBalls) {
                        if (ballPath.contains(ballToCheck) || ballToCheck.type != Ball.BallType.CLASSIFIER_WALL)
                            continue;
                        boolean hitting = GeometryUtils.isCircleInsidePolygon(polygon, ballToCheck.pos, 1);
                        if (hitting)
                            problemBalls.add(new ProblemBall(ProblemBall.Severity.OVERLAPPING_CLASSIFIER_WALL, cur.pos));
                    }
                }

                Types.PoseType preCollectType = Types.PoseType.PRECOLLECT;
                if (collectInfo.approachType == Types.Approach.CORNER_LENIENT)
                    preCollectType = Types.PoseType.LENIENT_CORNER_PRECOLLECT;
                else if (cur.type != Ball.BallType.NORMAL)
                    preCollectType = Types.PoseType.EDGE_CASE_PRECOLLECT;
                Waypoint w1 = new Waypoint(wallSafePreCollectPose, new CircleTolerance());
                Waypoint w2 = new Waypoint(wallSafeCollectPose, new CircleTolerance()).setPassPosition(true).setMinLinearPower(driveParams.collectDriveMinLinearPower);
                pathPoses.add(new PathPose(w1, preCollectType, cur, collectInfo.approachType));
                pathPoses.add(new PathPose(w2, Types.PoseType.COLLECT, cur, collectInfo.approachType));
            }
        }

        // strictly bind path poses to inside the field
        for (int i=0; i<pathPoses.size(); i++) {
            PathPose cur = pathPoses.get(i);
            cur.waypoint.pose = getStrictWallSafePose(cur.waypoint.pose);
        }
        // if ball n and ball n + 1 are normal collects
        // if ball n has poses that collide with ball n + 1, remove ball n + 1

        for (int i=1; i<pathPoses.size() - 1; i++) {
            PathPose prev = pathPoses.get(i - 1);
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i + 1);

            Vector2d curToNext = next.waypoint.pose.position.minus(cur.waypoint.pose.position);
            double curToNextDist = MathUtils.vecMag(curToNext);
            Vector2d prevToCurDir = cur.waypoint.pose.position.minus(prev.waypoint.pose.position);
            prevToCurDir = prevToCurDir.div(MathUtils.vecMag(prevToCurDir));
            Vector2d curToNextDir = curToNext;
            curToNextDir = curToNextDir.div(curToNextDist);
            if (prevToCurDir.dot(curToNextDir) < pathGenParams.backTrackingMinDotProductBetweenPoints
                    && curToNextDist > pathGenParams.backTrackingMaxDistBetweenPoints
                    && prev.ball.type != Ball.BallType.CLASSIFIER_WALL && next.ball.type != Ball.BallType.CLASSIFIER_WALL)
                problemBalls.add(new ProblemBall(ProblemBall.Severity.BACK_TRACKING, cur.ball.pos));
        }

        return new PathInfo(PathInfo.PathType.COMPLEX, robotPose, ballPath, pathPoses, problemBalls);
    }
    private static ClusterApproach getBestClusterApproach(ArrayList<ClusterApproach> possibleApproaches) {
        ClusterApproach bestApproach = null;
        // first sort based on if it is wall safe or not
        // then sort by difference in angle
        ArrayList<ClusterApproach> sortedApproaches = new ArrayList<>();
        for (int i=0; i<possibleApproaches.size(); i++) {
            ClusterApproach approach = possibleApproaches.get(i);

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
    private static class ClusterApproachPath {
        public final Ball clusterStartBall, clusterEndBall;
        public final ArrayList<PathPose> pathPoses;
        public ClusterApproachPath(Ball startBall, Ball endBall, ArrayList<PathPose> pathPoses) {
            this.clusterStartBall = startBall;
            this.clusterEndBall = endBall;
            this.pathPoses = pathPoses;
//            System.out.println("start: " + clusterStartIndex + " | end: " + clusterEndIndex + "| path: " + pathPoses);
        }

        public boolean accountsForBallAtIndex(int i) {
            return clusterStartBall.pathIndex <= i && i <= clusterEndBall.pathIndex;
        }
    }
    private static class ClusterApproach {
        public double approachAngle, collectAngle;
        public final Ball initialStart, initialEnd;
        public final Vector2d approxCenter;
        public Ball start, end;
        public ArrayList<Ball> allBallsInCluster;
        public final double angleDiff;
        public boolean isWallSafe;
        public final ArrayList<PathPose> pathPoses;
        public final ArrayList<ProblemBall> problemBalls;
        public ClusterApproach(Ball prevBall, Ball startBall, Ball endBall, ArrayList<Ball> allBallsInCluster) {
            this.initialStart = startBall;
            this.initialEnd = endBall;
            pathPoses = new ArrayList<>();
            problemBalls = new ArrayList<>();
            approxCenter = startBall.pos.plus(endBall.pos).times(0.5);
            approachAngle = MathUtils.v1ToV2Angle(startBall.pos, endBall.pos);

            Vector2d prevToStart = startBall.pos.minus(prevBall.pos);
            Vector2d startToEnd = endBall.pos.minus(startBall.pos);
            angleDiff = MathUtils.angleRadDiff(startToEnd, prevToStart);

            double maxStrafeAngleOffset = Math.min(Math.abs(angleDiff), Math.toRadians(pathGenParams.strafeCollectMaxAngleOffset));
            if (angleDiff > 0)
                collectAngle = approachAngle - maxStrafeAngleOffset;
            else
                collectAngle = approachAngle + maxStrafeAngleOffset;

            System.out.println("all balls in cluster: " + allBallsInCluster);
            this.allBallsInCluster = new ArrayList<>(allBallsInCluster);
            Vector2d approachDir = new Vector2d(Math.cos(approachAngle), Math.sin(approachAngle));
            Vector2d orthogonalApproachDir = new Vector2d(-approachDir.y, 1*approachDir.x);

            start = new Ball(initialStart.pos);
            end = new Ball(initialEnd.pos);
            Vector2d actualStartPos = initialStart.pos;
            Vector2d actualEndPos = initialEnd.pos;
            double minParallelOffset = 0;
            double maxParallelOffset = 0;

            ArrayList<ClusterBallInfo> includedBallsInStrafe = new ArrayList<>();
            // find balls included in strafe, and strafe start and end
            for (Ball potentialBall : allBallsInCluster) {
                Vector2d ballToApproxCenter = potentialBall.pos.minus(approxCenter);
                double parallelOffset = ballToApproxCenter.dot(approachDir);
                double perpendicularOffset = ballToApproxCenter.dot(orthogonalApproachDir);
                Vector2d projected = approxCenter.plus(approachDir.times(parallelOffset));
                if (Math.abs(perpendicularOffset) < pathGenParams.strafeCollectMaxPerpendicularDistance)
                    includedBallsInStrafe.add(new ClusterBallInfo(potentialBall.pos, parallelOffset, perpendicularOffset, projected));
                else
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.FAILED_CLUSTER_APPROACH, potentialBall.pos));

                if (parallelOffset < minParallelOffset) {
                    minParallelOffset = parallelOffset;
                    actualStartPos = projected;
                    start = potentialBall;
                }
                else if (parallelOffset > maxParallelOffset) {
                    maxParallelOffset = parallelOffset;
                    actualEndPos = projected;
                    end = potentialBall;
                }
            }

            Pose2d collectPose = getWallSafePose(getCollectPose(actualEndPos, collectAngle, 0)); // not great but its fine
            Pose2d startCollectPose = getCollectPose(actualStartPos, collectAngle, 0);
            Pose2d preCollectPose = getPreCollectPose(startCollectPose, approachAngle, pathGenParams.preCollectOffset);
            Pose2d wallSafePreCollectPose = getWallSafePose(preCollectPose);
            isWallSafe = wallSafePreCollectPose.equals(preCollectPose);
            if (isWallSafe) {
                Tolerance preCollectTolerance = new RotatedBoxTolerance(driveParams.clusterStrafeParallelTol, driveParams.clusterStrafePerpendicularTol, approachAngle, Math.toRadians(driveParams.clusterStrafeHeadingTol));
                Waypoint w1 = new Waypoint(preCollectPose, preCollectTolerance);
                Waypoint w2 = new Waypoint(collectPose).setPassPosition(true).setMinLinearPower(driveParams.collectDriveMinLinearPower);
                pathPoses.add(new PathPose(w1, Types.PoseType.EDGE_CASE_PRECOLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
                pathPoses.add(new PathPose(w2, Types.PoseType.COLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
                System.out.println("wall safe: " + start + " to " + end);
            }
            else {
                System.out.println("strafe is not wall safe: " + start + " to " + end);
                // keep trying more and more lenient angles until it either becomes wall safe or realizes its impossible
                double resolution = Math.toRadians(0.5); // check every 2.5 degrees
                double wallAngle = collectAngle > 0 ? Math.toRadians(90) : Math.toRadians(-90);
                double range = Math.abs(wallAngle - collectAngle);
                int numTries = (int) (range / resolution) + 1;
                double turnDirection = Math.signum(MathUtils.angleNormDeltaRad(wallAngle - collectAngle));
                for (int i = 0; i < numTries; i++) {
                    double changeAmount = turnDirection * resolution;
                    collectAngle += changeAmount;
                    approachAngle += changeAmount;
                    startCollectPose = getCollectPose(start.pos, collectAngle, 0);
                    preCollectPose = getPreCollectPose(startCollectPose, approachAngle, pathGenParams.preCollectOffset);
                    isWallSafe = preCollectPose.position.x == getWallSafePose(preCollectPose).position.x;
                    if (isWallSafe)
                        break;
                }
                if (isWallSafe) {
                    startCollectPose = new Pose2d(startCollectPose.position.x, startCollectPose.position.y, MathUtils.averageAngle(startCollectPose.heading.toDouble(), collectPose.heading.toDouble()));
                    Tolerance preCollectTolerance = new RotatedBoxTolerance(driveParams.clusterStrafeParallelTol, driveParams.clusterStrafePerpendicularTol, approachAngle, Math.toRadians(driveParams.clusterStrafeHeadingTol));
                    Waypoint w1 = new Waypoint(preCollectPose, preCollectTolerance);
                    Waypoint w2 = new Waypoint(startCollectPose).setPassPosition(true);
                    Waypoint w3 = new Waypoint(collectPose).setPassPosition(true).setMinLinearPower(driveParams.collectDriveMinLinearPower);
                    pathPoses.add(new PathPose(w1, Types.PoseType.EDGE_CASE_PRECOLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
                    pathPoses.add(new PathPose(w2, Types.PoseType.COLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
                    pathPoses.add(new PathPose(w3, Types.PoseType.COLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
                }
                else
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.FAILED_CLUSTER_APPROACH, start.pos));
            }
        }
        @Override
        public String toString() {
            return "from " + start + " to " + end;
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
    private static CollectInfo getCollectInfo(Pose2d prevPathPose, Ball prev, Ball cur, Ball next, double defaultApproachAngle) {
        Vector2d prevPathPos = prevPathPose.position;
        double wallAngle = cur.pos.y > 0 ? Math.toRadians(90) : Math.toRadians(-90);
        Vector2d prevToCur = cur.pos.minus(prevPathPos);
        double prevToCurAngle = MathUtils.vecAngle(prevToCur);
        Vector2d curToNextBall = next == null ? null : next.pos.minus(cur.pos);
        double curToNextBallDist = curToNextBall == null ? Double.MAX_VALUE : MathUtils.vecMag(curToNextBall);
        double preCollectToCollectAngle = 0;
        double collectAngle = 0;
        double preCollectOffset = pathGenParams.preCollectOffset;
        double collectXOffset = 0;
        double collectYOffset = 0;
        Types.Approach approachType = Types.Approach.NORMAL;
        switch (cur.type) {
            case CORNER:
                approachType = Types.Approach.CORNER_CONSTRAINED;
                collectAngle = Math.signum(wallAngle) * (Math.abs(wallAngle) - Math.toRadians(pathGenParams.cornerCollectAngle));
                preCollectToCollectAngle = collectAngle;
                collectYOffset = Math.signum(cur.pos.y) * Math.max(0, pathGenParams.cornerCollectY - Math.abs(cur.pos.y));
                preCollectOffset += Math.abs(collectYOffset);
                if (prev.type == Ball.BallType.CLASSIFIER_WALL || prev.type == Ball.BallType.BACK_WALL) {
                    double dist = MathUtils.vecDist(prev.pos, cur.pos);
                    if (dist < pathGenParams.lenientCornerCollectThreshold) {
                        approachType = Types.Approach.CORNER_LENIENT;
                        collectAngle = MathUtils.averageAngle(collectAngle, prevPathPose.heading.toDouble());
                        preCollectToCollectAngle = collectAngle * 0.5;
                        double preCollectOffsetT = Math.max(0, dist - 2.5) / pathGenParams.lenientCornerCollectThreshold;
                        preCollectOffset = MathUtils.lerp(0, pathGenParams.preCollectOffset, preCollectOffsetT);
                        collectYOffset = 0;
                    }
                }
                break;
            case CLASSIFIER_WALL:
                double angleD = Math.abs(MathUtils.angleNormDeltaRad(defaultApproachAngle - wallAngle));
                boolean useStrafeApproach =
                        angleD > Math.toRadians(pathGenParams.wallStrafeCollectMinApproachAngle)
                                || prev.type == Ball.BallType.CORNER || prev.type == Ball.BallType.CLASSIFIER_WALL
                                || (next != null && (next.type == Ball.BallType.BACK_WALL || next.type == Ball.BallType.CORNER || next.type == Ball.BallType.CLASSIFIER_WALL));

                if (useStrafeApproach) {
                    approachType = Types.Approach.CLASSIFIER_STRAFE;
                    double dx = curToNextBall == null ? cur.pos.x - prevPathPos.x : curToNextBall.x;
                    double strafeAngle = dx > 0 ? 0 : Math.PI;
                    collectXOffset = Math.signum(dx) * pathGenParams.strafeCollectDriveThroughDist;
                    if (Math.signum(dx) == 1)
                        collectAngle = Math.toRadians(pathGenParams.wallCollectAngle);
                    else
                        collectAngle = Math.PI - Math.toRadians(pathGenParams.wallCollectAngle);
                    collectAngle *= Math.signum(wallAngle);
                    double angDiff = Math.abs(MathUtils.angleNormDeltaRad(strafeAngle - prevToCurAngle));
                    if (Math.abs(angDiff) < Math.toRadians(pathGenParams.lenientClassifierStrafeAngleDiff)) {
                        approachType = Types.Approach.LENIENT_CLASSIFIER_STRAFE;
                        collectAngle = MathUtils.averageAngle(collectAngle, prevToCurAngle);
                        preCollectOffset += Math.abs(collectXOffset) * 0.25;
                    }
                    else
                        preCollectOffset += Math.abs(collectXOffset);
                    preCollectToCollectAngle = Math.signum(dx) == 1 ? 0 : Math.PI;

                    Pose2d collectPose = getCollectPose(cur.pos, collectAngle, 0);
                    Pose2d preCollectPose = getPreCollectPose(collectPose, preCollectToCollectAngle, pathGenParams.preCollectOffset);
                    boolean wallSafe = preCollectPose.position.x == getWallSafePose(preCollectPose).position.x;
                    if (!wallSafe) {
                        // if distance to next classifier ball is greater than threshold, attempt to create more lenient strafe that is inside the field
                        if (next != null && curToNextBallDist > pathGenParams.distToNextPointForceStrictClassifierStrafe) {
                            double resolution = Math.toRadians(0.5); // check every 2.5 degrees
                            double range = Math.abs(wallAngle - collectAngle);
                            int numTries = (int) (range / resolution) + 1;
                            double turnDirection = Math.signum(wallAngle - collectAngle);
                            for (int i = 0; i < numTries; i++) {
                                collectAngle += turnDirection * resolution;
                                collectPose = getCollectPose(cur.pos, collectAngle, 0);
                                preCollectPose = getPreCollectPose(collectPose, preCollectToCollectAngle, pathGenParams.preCollectOffset);
                                wallSafe = preCollectPose.position.x == getWallSafePose(preCollectPose).position.x;
                                if (wallSafe)
                                    break;
                            }
                            useStrafeApproach = wallSafe;
                            if (wallSafe)
                                approachType = Types.Approach.LENIENT_CLASSIFIER_STRAFE;
                        }
                    }
                }
                if (!useStrafeApproach) {
                    approachType = Types.Approach.NORMAL;
                    collectAngle = defaultApproachAngle;
                    preCollectToCollectAngle = collectAngle;
                    collectXOffset = 0;
                }
                break;
            case BACK_WALL:
                if (Math.abs(defaultApproachAngle) > Math.toRadians(pathGenParams.wallStrafeCollectMinApproachAngle)) {
                    approachType = Types.Approach.BACK_WALL_STRAFE;
                    double dy = cur.pos.y - prevPathPos.y;
                    collectYOffset = Math.signum(dy) * pathGenParams.strafeCollectDriveThroughDist;
                    preCollectOffset += Math.abs(collectYOffset);
                    collectAngle = Math.toRadians(pathGenParams.wallCollectAngle) * Math.signum(dy);
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
//            System.out.println("path poses at iteration " + i + ": " + simplified);
            PathPose prev = !simplified.isEmpty() ? simplified.get(simplified.size() - 1) : null;
            Pose2d prevPose = prev != null ? prev.waypoint.pose : startPose;
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i + 1);

            // skip any poses between classifier or back wall strafes
            if (prev != null) {
                if ((prev.approachType == Types.Approach.CLASSIFIER_STRAFE || prev.approachType == Types.Approach.LENIENT_CLASSIFIER_STRAFE)
                        && (next.approachType == Types.Approach.CLASSIFIER_STRAFE || next.approachType == Types.Approach.LENIENT_CLASSIFIER_STRAFE))
                    continue;
                if (prev.approachType == Types.Approach.BACK_WALL_STRAFE && next.approachType == Types.Approach.BACK_WALL_STRAFE)
                    continue;
            }
            // never skip in these situations
            if ((cur.approachType == Types.Approach.CLASSIFIER_STRAFE || cur.approachType == Types.Approach.LENIENT_CLASSIFIER_STRAFE) &&
                    (next.approachType == Types.Approach.CORNER_CONSTRAINED || next.approachType == Types.Approach.CORNER_LENIENT)) {
//                System.out.println("never skip, adding " + i);
                simplified.add(pathPoses.get(i));
                continue;
            }

            Vector2d prevToCur = cur.waypoint.pose.position.minus(prevPose.position);
            Vector2d curToNext = next.waypoint.pose.position.minus(cur.waypoint.pose.position);

            double prevToCurHeadingDiff = MathUtils.angleNormDeltaRad(cur.waypoint.pose.heading.toDouble() - prevPose.heading.toDouble());
            double curToNextHeadingDiff = MathUtils.angleNormDeltaRad(next.waypoint.pose.heading.toDouble() - cur.waypoint.pose.heading.toDouble());
            double approachAngleDiff = MathUtils.angleRadDiff(curToNext, prevToCur);

            double prevToCurDist = MathUtils.vecMag(prevToCur);
            double curToNextDist = MathUtils.vecMag(curToNext);
            double prevToCurHeadingSimplification = Math.toRadians(pathGenParams.maxCollectHeadingDifference.applyAsDouble(prevToCurDist));
            double curToNextHeadingSimplification = Math.toRadians(pathGenParams.maxCollectHeadingDifference.applyAsDouble(curToNextDist));
            if (cur.approachType == Types.Approach.NORMAL && next.approachType != Types.Approach.NORMAL)
                curToNextHeadingSimplification *= 0.5;
//            System.out.println(i + ": prev to cur heading diff: " + Math.toDegrees(prevToCurHeadingDiff) + " | cur to next heading diff: " + Math.toDegrees(curToNextHeadingDiff));
//            System.out.println("    prev to cur heading simpli: " + Math.toDegrees(prevToCurHeadingSimplification) + " | cur to next heading simpli: " + Math.toDegrees(curToNextHeadingSimplification));
//            System.out.println("    approach angle diff: " + Math.toDegrees(approachAngleDiff));
            double approachAngleSimplification = Math.toRadians(pathGenParams.maxCollectApproachDifference);
            boolean approachAnglesCloseEnough = Math.abs(approachAngleDiff) < approachAngleSimplification;
            boolean approachAnglesOppositeDirCloseEnough = Math.PI - Math.abs(approachAngleDiff) < approachAngleSimplification
                    && prevToCurDist <= pathGenParams.backTrackingMaxDistBetweenPoints;
            boolean headingCloseEnough = Math.abs(prevToCurHeadingDiff) < prevToCurHeadingSimplification && Math.abs(curToNextHeadingDiff) < curToNextHeadingSimplification;
//            System.out.println("heading close: " + headingCloseEnough + " | approach close: " + approachAnglesCloseEnough + " | opposite approach close: " + approachAnglesOppositeDirCloseEnough);
            if (headingCloseEnough && (approachAnglesCloseEnough || approachAnglesOppositeDirCloseEnough)) {
                double ignoreCollectHeadingDiff = pathGenParams.completelyIgnoreCollectPoseHeadingDiff;
                double ignoreCollectApproachAngleDiff = pathGenParams.completelyIgnoreCollectPoseApproachAngleDiff;
                if (Math.abs(prevToCurHeadingDiff) < ignoreCollectHeadingDiff && Math.abs(curToNextHeadingDiff) < ignoreCollectHeadingDiff
                        && Math.abs(approachAngleDiff) < ignoreCollectApproachAngleDiff)
                    continue;

                if (cur.poseType == Types.PoseType.COLLECT && cur.approachType == Types.Approach.NORMAL) {
                    // project pose onto line created by prev to cur
                    Vector2d prevToNext = next.waypoint.pose.position.minus(prevPose.position);
                    Vector2d prevToNextDir = prevToNext.div(MathUtils.vecMag(prevToNext));
                    double distAlongLine = prevToNextDir.dot(prevToCur);
                    Vector2d projectedPosition = prevPose.position.plus(prevToNextDir.times(distAlongLine));
                    cur.waypoint.pose = new Pose2d(projectedPosition, cur.waypoint.pose.heading);
                    simplified.add(new PathPose(cur.waypoint, cur.poseType, cur.ball, cur.approachType));
                }
//                System.out.println("meets conditions, skipping");
                continue;
            }

//            System.out.println("adding " + i);
            simplified.add(pathPoses.get(i));
        }
//        System.out.println("adding " + (pathPoses.size() - 1));
        simplified.add(pathPoses.get(pathPoses.size() - 1));
//        System.out.println("simplified inside function: " + simplified);
        return simplified;
    }

    // returns [fr, fl, bl, br] (counter clockwise)
    private static Vector2d[] getRobotCorners(Pose2d robotPose) {
        Vector2d position = robotPose.position;
        double halfW = pathGenParams.robotWidth * 0.5;
        double halfL = pathGenParams.robotLength * 0.5;

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
        double halfW = pathGenParams.robotWidth * 0.5;
        double halfL = pathGenParams.robotLength * 0.5;

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
        return getWallSafePose(robotPose, pathGenParams.wallPoseBuffer);
    }

    private static Pose2d getCollectPose(Vector2d ballPosition, double angle, double extraOffset) {
        double r = pathGenParams.collectPoseOffsetDistance + extraOffset;
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
