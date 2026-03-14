package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createVec;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.CircleTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.RotatedBoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static PathGenerationParams.General generalParams = new PathGenerationParams.General();
    public static PathGenerationParams.LaneCollect laneCollectParams = new PathGenerationParams.LaneCollect();
    public static PathGenerationParams.Regeneration regenerationParams = new PathGenerationParams.Regeneration();
    public static PathGenerationParams.ClusterStrafe clusterStrafeParams = new PathGenerationParams.ClusterStrafe();
    public static PathGenerationParams.WallStrafe wallStrafeParams = new PathGenerationParams.WallStrafe();
    public static PathGenerationParams.Corner cornerParams = new PathGenerationParams.Corner();
    public static PathGenerationParams.Optimization optimizationParams = new PathGenerationParams.Optimization();
    public static PathGenerationParams.Misc miscParams = new PathGenerationParams.Misc();
    public static PathDriveParams driveParams = new PathDriveParams();
    public static Pose2d pathfinderStartPose = null;
    public static PathInfo generateSimplifiedAutoCollectPath(Pose2d robotPose, ArrayList<Vector2d> ballsArray) {
        PathInfo path = generateAutoCollectPath(robotPose, ballsArray);
        if (path == null)
            return null;

        path.setOptimizedPathPoses(PathGeneration.optimizePathPoses(robotPose, path.pathPoses));
        return path;
    }
    private static PathInfo generateAutoCollectPath(Pose2d robotPose, ArrayList<Vector2d> ballPositionsArray) {
        pathfinderStartPose = robotPose;
        if (ballPositionsArray.isEmpty())
            return null;

        ArrayList<Ball> allBalls = Ball.toBallList(new ArrayList<>(ballPositionsArray));

        ArrayList<Lane> densestLanes = getDensestLanes(allBalls);
        Lane bestLane = getBestLane(robotPose.position, densestLanes);
        if (generalParams.allowLaneCollect) {
            if (bestLane.numBalls() >= laneCollectParams.alwaysUseLaneCollectNumBalls)
                return generateLanePath(robotPose, bestLane);
            if (allBalls.size() == 2 && bestLane.numBalls() == 2)
                return generateLanePath(robotPose, bestLane);
        }

        int numAttempts = 0;
        PathInfo optimalPathInfo = null;
        int leastProblemBalls = -1;
        ArrayList<Ball> balls = new ArrayList<>(allBalls);
        ArrayList<Ball> ignoredBalls = new ArrayList<>();
        do {
//            System.out.println("regeneration " + numAttempts);
            ArrayList<Vector2d> ballPositions = Ball.toVecList(balls);
            ArrayList<Vector2d> rawPath = PathFinder.findShortestPath(robotPose, ballPositions, 3, miscParams.changeInAngleDegCost, 0,0);
            if (rawPath == null)
                break;
            ArrayList<Ball> rawBallPath = Ball.toBallList(rawPath);

            // generate path
//            System.out.println("normal path generation========");
            PathInfo pathInfo = generateComplexPath(robotPose, allBalls, rawBallPath);
            pathInfo.setIgnoredBalls(ignoredBalls);
            int numProblemBalls = pathInfo.problemBalls.size();

            // artificially shift robot to the left to see if it generates a more desirable path
            Pose2d shiftedLeftRobotPose = new Pose2d(regenerationParams.shiftedLeftStartX, robotPose.position.y, robotPose.heading.toDouble());
            ArrayList<Vector2d> shiftedLeftRawPath = PathFinder.findShortestPath(shiftedLeftRobotPose, ballPositions, 3, miscParams.changeInAngleDegCost, 0,0);
            if (shiftedLeftRawPath != null) {
                ArrayList<Ball> shiftedLeftRawBallPath = Ball.toBallList(shiftedLeftRawPath);
//                System.out.println("shifted left path `========");
                PathInfo shiftedLeftPathInfo = generateComplexPath(robotPose, allBalls, shiftedLeftRawBallPath);
                if (shiftedLeftPathInfo.numGoodBalls() >= pathInfo.numGoodBalls()) {
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
                    if (!problemBall.shouldRemove)
                        break;
                    balls.remove(problemBall);
                    ignoredBalls.add(problemBall);
                }

            numAttempts++;
        } while (!balls.isEmpty() && numAttempts <= regenerationParams.maxPathRegenerationAttempts);

        if (optimalPathInfo == null) {
            return null;
        }

        if (generalParams.allowLaneCollect) {
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

        double increment = laneCollectParams.laneIncrement;
        int numLanes = (int) ((end - laneCollectParams.normalLaneWidth - start + 2) / increment);
        if (numLanes <= 0)
            numLanes = 1;
        ArrayList<Lane> lanes = new ArrayList<>();

        for (int i=0; i<numLanes; i++) {
            double left = start + i * increment;
            boolean againstBackWall = left + laneCollectParams.normalLaneWidth >= 72 - laneCollectParams.laneAgainstBackWallMaxDist;
            double laneWidth = againstBackWall ? laneCollectParams.againstBackWallLaneWidth : laneCollectParams.normalLaneWidth;
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
        if (sortedLanes.get(0).optimizedWidth <= laneCollectParams.ignoreOptimizedLaneWidthSortingWidth) {
            while (sortedLanes.get(sortedLanes.size() - 1).optimizedWidth > laneCollectParams.ignoreOptimizedLaneWidthSortingWidth)
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

        if (lane.avgX >= 72 - laneCollectParams.angleLaneCollectDistFromBackWall)
            angle -= Math.signum(angle) * Math.toRadians(cornerParams.cornerCollectAngle);

        Pose2d collect1 = getCollectPose(ball1.pos, angle, 0);
        Pose2d endCollect = getCollectPose(ball2.pos, angle, -100);
        if (72 - Math.abs(endCollect.position.y) <= laneCollectParams.snapLaneToWallDistFromWall)
            endCollect = new Pose2d(endCollect.position.x, Math.signum(endCollect.position.y) * 72, endCollect.heading.toDouble());

        Pose2d preCollect = getPreCollectPose(collect1.position, collect1.heading.toDouble(), angle, generalParams.preCollectOffset);
        Pose2d wallSafeCollect = getStrictWallSafePose(endCollect);
        Pose2d wallSafePreCollect = getStrictWallSafePose(preCollect);

        Waypoint waypoint1 = new Waypoint(wallSafePreCollect, new BoxTolerance(driveParams.lanePreCollectTol))
                .setPassPosition(true)
                .setMinLinearPower(driveParams.lanePreCollectMinLinearPower);

        double dirToBall = Math.signum(wallSafePreCollect.position.y - startPose.position.y);
        double controlY = wallSafePreCollect.position.y - laneCollectParams.laneCollectControlYOffset * dirToBall;
        double dirToControlY = Math.signum(controlY - (startPose.position.y + laneCollectParams.laneCollectControlMinYOffsetFromRobot * dirToBall));
        if (dirToControlY != dirToBall)
            controlY = startPose.position.y + dirToBall * laneCollectParams.laneCollectControlMinYOffsetFromRobot;
        Pose2d controlPose = new Pose2d(wallSafePreCollect.position.x, controlY, wallSafePreCollect.heading.toDouble());
        Vector2d startToControl = controlPose.position.minus(startPose.position);
        Vector2d halfwayBetweenStartAndControl = startPose.position.plus(startToControl.times(0.65));
        Vector2d halfwayToPreCollect = wallSafePreCollect.position.minus(halfwayBetweenStartAndControl);
        double startLerpDist = MathUtils.vecMag(halfwayToPreCollect);
        double endLerpDist = Math.abs(wallSafePreCollect.position.y - controlY);
        waypoint1.setControlPoint(controlPose, startLerpDist, endLerpDist);

        Waypoint waypoint3 = new Waypoint(wallSafeCollect, new CircleTolerance())
                .setPassPosition(true)
                .setMinLinearPower(driveParams.laneCollectMinLinearPower);
        PathPose pathPose1 = new PathPose(waypoint1, Types.PoseType.PRECOLLECT, ball1, Types.Approach.NORMAL);
        PathPose pathPose3 = new PathPose(waypoint3, Types.PoseType.COLLECT, ball2, Types.Approach.NORMAL);
        ArrayList<PathPose> pathPoses = new ArrayList<>(Arrays.asList(pathPose1, pathPose3));
        return new PathInfo(PathInfo.PathType.LANE, startPose, lane.balls, pathPoses, new ArrayList<>());
    }
    private static PathInfo generateComplexPath(Pose2d robotPose, ArrayList<Ball> allBalls, ArrayList<Ball> originalBallPath) {
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

                    if (horizontalDist > miscParams.rejectEdgeCaseBallsHorizontalDist) {
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

        ArrayList<Ball> allBallsInCluster = new ArrayList<>();

        ArrayList<ClusterApproachPath> validClusterApproachPaths = new ArrayList<>();
        for (int i=0; i<ballPath.size(); i++) {
            Ball prev = i == 0 ? new Ball(robotPos) : ballPath.get(i - 1);
            Ball cur = ballPath.get(i);
            Ball next = i < ballPath.size() - 1 ? ballPath.get(i + 1) : null;
            // keep adding cur ball into the cluster as long as it is close enough with next ball are close enough
            if (next != null && next.type != Ball.BallType.CORNER) {
                Vector2d curToNextBall = next.pos.minus(cur.pos);
                double curToNextBallDist = Math.hypot(curToNextBall.x, curToNextBall.y);
                if (curToNextBallDist < clusterStrafeParams.clusterStrafingDist) {
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
                        if (MathUtils.vecDist(average, ballInCluster.pos) > clusterStrafeParams.clusterMergeDist) {
                            closeEnoughToMerge = false;
                            break;
                        }

                    // choose easiest
                    if (!closeEnoughToMerge) {
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

        // set path indexes of new path after clustering
        for (int i=0; i<ballPath.size(); i++)
            ballPath.get(i).pathIndex = i;

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

                // build collect and pre collect poses based on the type of point
                CollectInfo collectInfo = getCollectInfo(prevPathPose, prev, cur, next, defaultApproachAngle);

                Pose2d collectPose = getCollectPose(cur.pos, collectInfo.collectAngle, 0);
                collectPose = new Pose2d(collectPose.position.plus(collectInfo.collectOffset), collectPose.heading);
                Pose2d preCollectPose = getPreCollectPose(collectPose.position, collectInfo.preCollectAngle, collectInfo.preCollectToCollectAngle, collectInfo.preCollectOffset);

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
                    wallSafePreCollectPose = getPreCollectPose(wallSafeCollectPose.position, collectInfo.preCollectAngle, collectInfo.preCollectToCollectAngle, collectInfo.preCollectOffset);
                    wallSafePreCollectPose = getWallSafePose(wallSafePreCollectPose);
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.OUT_OF_BOUNDS, cur.pos));
                }
                else if (collectInfo.ballProblemSeverity != null)
                    problemBalls.add(new ProblemBall(collectInfo.ballProblemSeverity, cur.pos));

                // add extra collect dist for last ball
                if (next == null && collectInfo.approachType == Types.Approach.NORMAL) {
                    collectPose = getCollectPose(cur.pos, collectInfo.collectAngle, -generalParams.lastCollectPoseExtraDriveThrough);
                    wallSafeCollectPose = getWallSafePose(collectPose);
                }

                // check if robot is heading
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
                if (cur.type != Ball.BallType.NORMAL)
                    preCollectType = Types.PoseType.EDGE_CASE_PRECOLLECT;
                Tolerance collectTolerance = new CircleTolerance();
                Tolerance preCollectTolerance = new CircleTolerance();
                double collectMinLinearPower = driveParams.collectNormalMinLinearPower;
                double preCollectMinLinearPower = driveParams.preCollectNormalMinLinearPower;
                boolean preCollectPassPosition = false;
                Vector2d collectCustomForceVector = new Vector2d(0, 0);
                switch (cur.type) {
                    case CLASSIFIER_WALL:
                        if (collectInfo.approachType == Types.Approach.CLASSIFIER_STRAFE) {
                            collectTolerance = new BoxTolerance(driveParams.classifierWallStrafeTol);
                            preCollectTolerance = collectTolerance;
                            collectMinLinearPower = driveParams.collectWallStrafeMinLinearPower;
                            collectCustomForceVector = createVec(driveParams.classifierStrafeCustomForce).times(Math.signum(cur.pos.y));
                        }
                        break;
                    case BACK_WALL:
                        if (collectInfo.approachType == Types.Approach.BACK_WALL_STRAFE) {
                            collectTolerance = new BoxTolerance(driveParams.backWallStrafeTol);
                            preCollectTolerance = collectTolerance;
                            collectMinLinearPower = driveParams.collectWallStrafeMinLinearPower;
                        }
                        break;
                    case CORNER:
                        collectTolerance = new CircleTolerance(driveParams.collectCornerDistTol, Math.toRadians(driveParams.collectCornerHeadingTol));
                        preCollectTolerance = new CircleTolerance(driveParams.preCollectCornerDistTol, Math.toRadians(driveParams.preCollectCornerHeadingTol));
                        collectMinLinearPower = driveParams.collectCornerMinLinearPower;
                        preCollectMinLinearPower = driveParams.collectCornerMinLinearPower;
                        preCollectPassPosition = true;
                        break;
                }
                Waypoint w1 = null;
                if (collectInfo.preCollectOffset != 0) {
                    w1 = new Waypoint(wallSafePreCollectPose, preCollectTolerance)
                            .setPassPosition(preCollectPassPosition)
                            .setMinLinearPower(preCollectMinLinearPower);
                    if (collectInfo.preCollectControlPose != null) {
                        Pose2d controlPose = new Pose2d(
                                wallSafePreCollectPose.position.plus(collectInfo.preCollectControlPose.position),
                                collectInfo.preCollectControlPose.heading.toDouble()
                        );
                        w1.setControlPoint(controlPose, collectInfo.preCollectControlLerpStart, collectInfo.preCollectControlLerpEnd);
                    }
                }

                Waypoint w2 = new Waypoint(wallSafeCollectPose, collectTolerance)
                        .setPassPosition(true)
                        .setMinLinearPower(collectMinLinearPower)
                        .setCustomForceVector(collectCustomForceVector);
                if (collectInfo.collectControlPose != null) {
                    Pose2d controlPose = new Pose2d(wallSafeCollectPose.position.plus(collectInfo.collectControlPose.position), collectInfo.collectControlPose.heading);
                    w2.setControlPoint(controlPose, collectInfo.collectControlLerpStart, collectInfo.collectControlLerpEnd);
                }

                if (w1 != null)
                    pathPoses.add(new PathPose(w1, preCollectType, cur, collectInfo.approachType));
                pathPoses.add(new PathPose(w2, Types.PoseType.COLLECT, cur, collectInfo.approachType));
            }
        }

        // make all classifier strafes end in the corner
        if (!pathPoses.isEmpty()) {
            PathPose last = pathPoses.get(pathPoses.size() - 1);
            if (last.approachType == Types.Approach.CLASSIFIER_STRAFE && pathPoses.size() >= 2) {
                PathPose secondLast = pathPoses.get(pathPoses.size() - 2);
                double dx = last.waypoint.pose.position.x - secondLast.waypoint.pose.position.x;
                if (Math.signum(dx) == 1) {
                    Vector2d newPosition = last.waypoint.pose.position.plus(new Vector2d(50, 0));
                    last.waypoint.pose = new Pose2d(newPosition, last.waypoint.pose.heading);
                    ;
                }
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
            if (prevToCurDir.dot(curToNextDir) < optimizationParams.backTrackingMinDotProductBetweenPoints
                    && curToNextDist > optimizationParams.backTrackingMaxDistBetweenPoints
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
//        System.out.println("possible approaches: " + possibleApproaches);
//        System.out.println("best approach: " + bestApproach);
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
            angleDiff = MathUtils.angleRadDiff(prevToStart, startToEnd);

            double maxStrafeAngleOffset = Math.min(Math.abs(angleDiff), Math.toRadians(clusterStrafeParams.clusterStrafeCollectMaxAngleOffset));
            System.out.println(startBall + " | " + endBall);
            if (Math.signum(startToEnd.x) > 0)
                collectAngle = approachAngle + maxStrafeAngleOffset;
            else
                collectAngle = approachAngle - maxStrafeAngleOffset;
//            System.out.println("sign of start to end: " + Math.signum(startToEnd.x));
//            System.out.println("collect angle: " + Math.toDegrees(collectAngle));

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
                if (Math.abs(perpendicularOffset) < clusterStrafeParams.clusterStrafeCollectMaxPerpendicularDistance)
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
            Pose2d preCollectPose = getPreCollectPose(startCollectPose.position, startCollectPose.heading.toDouble(), approachAngle, generalParams.preCollectOffset);
            Pose2d wallSafePreCollectPose = getWallSafePose(preCollectPose);
            isWallSafe = wallSafePreCollectPose.equals(preCollectPose);
            if (isWallSafe) {
                Tolerance preCollectTolerance = new RotatedBoxTolerance(driveParams.clusterStrafeParallelTol, driveParams.clusterStrafePerpendicularTol, approachAngle, Math.toRadians(driveParams.clusterStrafeHeadingTol));
                Waypoint w1 = new Waypoint(preCollectPose, preCollectTolerance);
                Pose2d controlPoint = getPreCollectPose(preCollectPose.position, preCollectPose.heading.toDouble(), approachAngle, driveParams.strafeCollectControlMaxOffset);
                controlPoint = getWallSafePose(controlPoint);
                w1.setControlPoint(controlPoint, driveParams.strafeCollectControlStartError, driveParams.strafeCollectControlEndError);

                Ball actualEndBall = new Ball(actualEndPos);
                double minLinearPower = actualEndBall.type == Ball.BallType.CORNER ? driveParams.collectCornerMinLinearPower : driveParams.collectNormalMinLinearPower;
                Waypoint w2 = new Waypoint(collectPose)
                        .setPassPosition(true)
                        .setMinLinearPower(minLinearPower);
                pathPoses.add(new PathPose(w1, Types.PoseType.EDGE_CASE_PRECOLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
                pathPoses.add(new PathPose(w2, Types.PoseType.COLLECT, Ball.NULL, Types.Approach.CLUSTER_STRAFE));
            }
            else {
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
                    preCollectPose = getPreCollectPose(startCollectPose.position, startCollectPose.heading.toDouble(), approachAngle, generalParams.preCollectOffset);
                    isWallSafe = preCollectPose.position.x == getWallSafePose(preCollectPose).position.x;
                    if (isWallSafe)
                        break;
                }
                if (isWallSafe) {
                    startCollectPose = new Pose2d(startCollectPose.position.x, startCollectPose.position.y, MathUtils.averageAngle(startCollectPose.heading.toDouble(), collectPose.heading.toDouble()));
                    Tolerance preCollectTolerance = new RotatedBoxTolerance(driveParams.clusterStrafeParallelTol, driveParams.clusterStrafePerpendicularTol, approachAngle, Math.toRadians(driveParams.clusterStrafeHeadingTol));
                    Waypoint w1 = new Waypoint(preCollectPose, preCollectTolerance);
                    Waypoint w2 = new Waypoint(startCollectPose).setPassPosition(true);
                    Waypoint w3 = new Waypoint(collectPose).setPassPosition(true).setMinLinearPower(driveParams.collectNormalMinLinearPower);
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
        double prevToCurDist = MathUtils.vecMag(prevToCur);
        Vector2d curToNextBall = next == null ? null : next.pos.minus(cur.pos);
        double curToNextBallDist = curToNextBall == null ? Double.MAX_VALUE : MathUtils.vecMag(curToNextBall);
        double preCollectToCollectAngle = 0;
        double collectAngle = 0;
        double preCollectAngle = 0;
        double preCollectOffset = generalParams.preCollectOffset;
        double collectXOffset = 0;
        double collectYOffset = 0;
        Types.Approach approachType = null;
        ProblemBall.Severity problemBallSeverity = null;
        Pose2d collectControlPoseOffset = null;
        double collectControlLerpStart = 0;
        double collectControlLerpEnd = 0;
        Pose2d preCollectControlPoseOffset = null;
        double preCollectControlLerpStart = 0;
        double preCollectControlLerpEnd = 0;
        switch (cur.type) {
            case CORNER:
                approachType = Types.Approach.CORNER;
                problemBallSeverity = ProblemBall.Severity.CORNER;

                boolean useYApproach = prev.type != Ball.BallType.CLASSIFIER_WALL;
                collectXOffset = 24;
                collectYOffset = Math.signum(wallAngle) * 24;

                if (useYApproach) {
                    collectAngle = Math.signum(wallAngle) * (Math.abs(wallAngle) - Math.toRadians(cornerParams.cornerCollectAngle));

                    // poses too close, create precollect pose
                    if (prevToCurDist < cornerParams.createCornerYApproachPreCollectDist) {
                        preCollectAngle = Math.signum(wallAngle) * Math.toRadians(cornerParams.cornerYApproachPreCollectAngle);
                        preCollectOffset = cornerParams.cornerYApproachPreCollectOffset + 24;
                        preCollectToCollectAngle = Math.signum(wallAngle) * Math.toRadians(90);
                    }
                    // poses are far enough away, create control point
                    else {
                        preCollectOffset = 0;
                        double dy = Math.abs(cur.pos.minus(prevPathPos).y);
                        dy = Math.max(0, dy - driveParams.cornerControlMinOffsetFromPrev);
                        double controlOffset = Math.min(driveParams.cornerControlOffset, dy);
                        collectControlPoseOffset = new Pose2d(0, -Math.signum(wallAngle) * controlOffset, collectAngle);
                        collectControlLerpStart = driveParams.cornerControlLerpStart;
                        collectControlLerpEnd = driveParams.cornerControlLerpEnd;
                    }
                }
                // always create precollect pose for x approach
                else {
                    collectAngle = Math.signum(wallAngle) * Math.toRadians(cornerParams.cornerCollectAngle);
                    preCollectAngle = collectAngle;
                    preCollectToCollectAngle = 0;
                    preCollectOffset += 24;
                }
                break;
            case CLASSIFIER_WALL:
                double angleD = Math.abs(MathUtils.angleNormDeltaRad(defaultApproachAngle - wallAngle));
                boolean useStrafeApproach =
                        angleD > Math.toRadians(wallStrafeParams.wallStrafeCollectMinApproachAngle)
                                || prev.type == Ball.BallType.CORNER || prev.type == Ball.BallType.CLASSIFIER_WALL
                                || (next != null && (next.type == Ball.BallType.BACK_WALL || next.type == Ball.BallType.CORNER || next.type == Ball.BallType.CLASSIFIER_WALL))
                                || (next != null && MathUtils.vecDist(cur.pos, next.pos) < wallStrafeParams.wallStrafeAlwaysDistToNext);

                if (useStrafeApproach) {
                    approachType = Types.Approach.CLASSIFIER_STRAFE;
                    double dx = curToNextBall == null ? cur.pos.x - prevPathPos.x : curToNextBall.x;
                    double strafeAngle = dx > 0 ? 0 : Math.PI;
                    collectXOffset = Math.signum(dx) * wallStrafeParams.wallStrafeCollectDriveThroughDist;
                    if (Math.signum(dx) == 1)
                        collectAngle = Math.toRadians(wallStrafeParams.wallStrafeCollectAngle);
                    else
                        collectAngle = Math.PI - Math.toRadians(wallStrafeParams.wallStrafeCollectAngle);
                    collectAngle *= Math.signum(wallAngle);
                    double angDiff = Math.abs(MathUtils.angleNormDeltaRad(strafeAngle - prevToCurAngle));
                    if (Math.abs(angDiff) < Math.toRadians(wallStrafeParams.lenientClassifierStrafeAngleDiff)) {
                        approachType = Types.Approach.LENIENT_CLASSIFIER_STRAFE;
                        collectAngle = MathUtils.averageAngle(collectAngle, prevToCurAngle);
                        preCollectOffset += Math.abs(wallStrafeParams.wallStrafeOffsetFromStart) * 0.25;
                    }
                    else
                        preCollectOffset += Math.abs(wallStrafeParams.wallStrafeOffsetFromStart);

                    preCollectToCollectAngle = Math.signum(dx) == 1 ? 0 : Math.PI;

                    Pose2d collectPose = getCollectPose(cur.pos, collectAngle, 0);
                    Pose2d preCollectPose = getPreCollectPose(collectPose.position, collectPose.heading.toDouble(), preCollectToCollectAngle, generalParams.preCollectOffset);
                    boolean wallSafe = preCollectPose.position.x == getWallSafePose(preCollectPose).position.x;
                    if (!wallSafe) {
                        problemBallSeverity = ProblemBall.Severity.UNIDEAL_APPROACH;
                        // if distance to next classifier ball is greater than threshold, attempt to create more lenient strafe that is inside the field
                        if (next != null && curToNextBallDist > wallStrafeParams.distToNextPointForceStrictClassifierStrafe) {
                            double resolution = Math.toRadians(0.5); // check every 2.5 degrees
                            double range = Math.abs(wallAngle - collectAngle);
                            int numTries = (int) (range / resolution) + 1;
                            double turnDirection = Math.signum(wallAngle - collectAngle);
                            for (int i = 0; i < numTries; i++) {
                                collectAngle += turnDirection * resolution;
                                collectPose = getCollectPose(cur.pos, collectAngle, 0);
                                preCollectPose = getPreCollectPose(collectPose.position, collectAngle, preCollectToCollectAngle, generalParams.preCollectOffset);
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
                preCollectAngle = collectAngle;

                if (approachType == Types.Approach.CLASSIFIER_STRAFE) {
                    double strafeDx = cur.pos.minus(prev.pos).x;
                    if (Math.signum(strafeDx) == 1) {
                        double dyFromPrevPose = Math.abs(cur.pos.y - prevPathPose.position.y);
                        double controlYOffset = Math.min(dyFromPrevPose - 8, wallStrafeParams.classifierStrafeControlYOffset);
                        preCollectControlPoseOffset = new Pose2d(0, Math.signum(wallAngle) * -controlYOffset, preCollectAngle);
                        preCollectControlLerpStart = wallStrafeParams.classifierStrafeControlLerpStart;
                        preCollectControlLerpEnd = wallStrafeParams.classifierStrafeControlLerpEnd;
                    }
                }
                break;
            case BACK_WALL:
                if (Math.abs(defaultApproachAngle) > Math.toRadians(wallStrafeParams.wallStrafeCollectMinApproachAngle)) {
                    approachType = Types.Approach.BACK_WALL_STRAFE;
                    double dy = cur.pos.y - prevPathPos.y;
                    collectYOffset = Math.signum(dy) * wallStrafeParams.wallStrafeCollectDriveThroughDist;
                    preCollectOffset += Math.abs(collectYOffset);
                    collectAngle = Math.toRadians(wallStrafeParams.wallStrafeCollectAngle) * Math.signum(dy);
                    preCollectToCollectAngle = Math.toRadians(90) * Math.signum(dy);
                }
                else {
                    approachType = Types.Approach.NORMAL;
                    collectAngle = defaultApproachAngle;
                    preCollectToCollectAngle = collectAngle;
                }
                preCollectAngle = collectAngle;
                break;
            case NORMAL:
            default:
                approachType = Types.Approach.NORMAL;
                collectAngle = defaultApproachAngle;
                preCollectAngle = collectAngle;
                preCollectToCollectAngle = collectAngle;
                break;
        }
        return new CollectInfo(problemBallSeverity, approachType, preCollectOffset, preCollectAngle, preCollectToCollectAngle, collectAngle, collectXOffset, collectYOffset, preCollectControlPoseOffset, preCollectControlLerpStart, preCollectControlLerpEnd, collectControlPoseOffset, collectControlLerpStart, collectControlLerpEnd);
    }

    private static class CollectInfo {
        public final ProblemBall.Severity ballProblemSeverity;
        public final Types.Approach approachType;
        public final double preCollectOffset, preCollectToCollectAngle, preCollectAngle, collectAngle;
        public final Vector2d collectOffset;
        public final Pose2d preCollectControlPose, collectControlPose;
        public final double preCollectControlLerpStart, preCollectControlLerpEnd, collectControlLerpStart, collectControlLerpEnd;
        public CollectInfo(ProblemBall.Severity ballProblemSeverity, Types.Approach approachType,
                           double preCollectOffset, double preCollectAngle, double preCollectToCollectAngle,
                           double collectAngle, double collectXOffset, double collectYOffset,
                           Pose2d preCollectControlPose, double preCollectControlLerpStart, double preCollectControlLerpEnd,
                           Pose2d collectControlPose, double collectControlLerpStart, double collectControlLerpEnd) {
            this.ballProblemSeverity = ballProblemSeverity;
            this.approachType = approachType;
            this.preCollectOffset = preCollectOffset;
            this.preCollectAngle = preCollectAngle;
            this.preCollectToCollectAngle = preCollectToCollectAngle;
            this.collectAngle = collectAngle;
            this.collectOffset = new Vector2d(collectXOffset, collectYOffset);
            this.preCollectControlPose = preCollectControlPose;
            this.preCollectControlLerpStart = preCollectControlLerpStart;
            this.preCollectControlLerpEnd = preCollectControlLerpEnd;
            this.collectControlPose = collectControlPose;
            this.collectControlLerpStart = collectControlLerpStart;
            this.collectControlLerpEnd = collectControlLerpEnd;
        }
    }
    private static ArrayList<PathPose> optimizePathPoses(Pose2d startPose, ArrayList<PathPose> pathPoses) {
        ArrayList<PathPose> simplified = new ArrayList<>();
        if (pathPoses.isEmpty())
            return simplified;
        if (pathPoses.size() == 1) {
            simplified.add(pathPoses.get(0));
            return simplified;
        }

        // remove unnecessary poses
        for (int i=0; i<pathPoses.size()-1; i++) {
            PathPose prev = !simplified.isEmpty() ? simplified.get(simplified.size() - 1) : null;
            Pose2d prevPose = prev != null ? prev.waypoint.pose : startPose;
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i + 1);

            // never skip in these situations
            if (((cur.approachType == Types.Approach.CLASSIFIER_STRAFE || cur.approachType == Types.Approach.LENIENT_CLASSIFIER_STRAFE)) &&
                    (next.approachType == Types.Approach.CORNER)) {
                simplified.add(pathPoses.get(i));
                continue;
            }

            // skip any poses between classifier or back wall strafes
            if (prev != null) {
                if ((prev.approachType == Types.Approach.CLASSIFIER_STRAFE || prev.approachType == Types.Approach.LENIENT_CLASSIFIER_STRAFE)
                        && (next.approachType == Types.Approach.CLASSIFIER_STRAFE || next.approachType == Types.Approach.LENIENT_CLASSIFIER_STRAFE))
                    continue;
                if (prev.approachType == Types.Approach.BACK_WALL_STRAFE && next.approachType == Types.Approach.BACK_WALL_STRAFE)
                    continue;
            }

            Vector2d prevToCur = cur.waypoint.pose.position.minus(prevPose.position);
            Vector2d curToNext = next.waypoint.pose.position.minus(cur.waypoint.pose.position);

            double prevToCurHeadingDiff = MathUtils.angleNormDeltaRad(cur.waypoint.pose.heading.toDouble() - prevPose.heading.toDouble());
            double curToNextHeadingDiff = MathUtils.angleNormDeltaRad(next.waypoint.pose.heading.toDouble() - cur.waypoint.pose.heading.toDouble());
            double approachAngleDiff = MathUtils.angleRadDiff(curToNext, prevToCur);

            double prevToCurDist = MathUtils.vecMag(prevToCur);
            double curToNextDist = MathUtils.vecMag(curToNext);
            double prevToCurHeadingSimplification = Math.toRadians(optimizationParams.maxCollectHeadingDifference.applyAsDouble(prevToCurDist));
            double curToNextHeadingSimplification = Math.toRadians(optimizationParams.maxCollectHeadingDifference.applyAsDouble(curToNextDist));
            if (cur.approachType == Types.Approach.NORMAL && next.approachType != Types.Approach.NORMAL)
                curToNextHeadingSimplification *= 0.5;
            double approachAngleSimplification = Math.toRadians(optimizationParams.maxCollectApproachDifference);
            boolean approachAnglesCloseEnough = Math.abs(approachAngleDiff) < approachAngleSimplification;
            boolean approachAnglesOppositeDirCloseEnough = Math.PI - Math.abs(approachAngleDiff) < approachAngleSimplification
                    && prevToCurDist <= optimizationParams.backTrackingMaxDistBetweenPoints;
            boolean headingCloseEnough = Math.abs(prevToCurHeadingDiff) < prevToCurHeadingSimplification && Math.abs(curToNextHeadingDiff) < curToNextHeadingSimplification;
            if (headingCloseEnough && (approachAnglesCloseEnough || approachAnglesOppositeDirCloseEnough)) {
                double ignoreCollectHeadingDiff = optimizationParams.completelyIgnoreCollectPoseHeadingDiff;
                double ignoreCollectApproachAngleDiff = optimizationParams.completelyIgnoreCollectPoseApproachAngleDiff;
                if (Math.abs(prevToCurHeadingDiff) < ignoreCollectHeadingDiff && Math.abs(curToNextHeadingDiff) < ignoreCollectHeadingDiff
                        && Math.abs(approachAngleDiff) < ignoreCollectApproachAngleDiff) {
                    if (cur.waypoint.params.pathType == PathParams.PathType.CURVED) {
                        double startLerpDist = cur.waypoint.params.tValueStartDist + curToNextDist;
                        double endLerpDist = cur.waypoint.params.tValueFinishDist + curToNextDist;
                        next.waypoint.setControlPoint(cur.waypoint.params.controlPoint, startLerpDist, endLerpDist);
                    }
                    continue;
                }

                if (cur.poseType == Types.PoseType.COLLECT && cur.approachType == Types.Approach.NORMAL) {
                    // project pose onto line created by prev to cur
                    Vector2d prevToNext = next.waypoint.pose.position.minus(prevPose.position);
                    Vector2d prevToNextDir = prevToNext.div(MathUtils.vecMag(prevToNext));
                    double distAlongLine = prevToNextDir.dot(prevToCur);
                    Vector2d projectedPosition = prevPose.position.plus(prevToNextDir.times(distAlongLine));
                    cur.waypoint.pose = new Pose2d(projectedPosition, cur.waypoint.pose.heading);
                    simplified.add(new PathPose(cur.waypoint, cur.poseType, cur.ball, cur.approachType));
                }
                continue;
            }

            simplified.add(pathPoses.get(i));
        }
        simplified.add(pathPoses.get(pathPoses.size() - 1));

        // optimize heading
        for (int i=0; i<simplified.size(); i++) {
            PathPose prev = i == 0 ? null : simplified.get(i - 1);
            Pose2d prevPose = i == 0 ? startPose : prev.waypoint.pose;
            PathPose cur = simplified.get(i);

            boolean prevIsWeird = prev != null && prev.approachType != Types.Approach.NORMAL;
            boolean curIsWeird = cur.approachType != Types.Approach.NORMAL;
            if (prevIsWeird && curIsWeird)
                continue;

            if (MathUtils.vecDist(prevPose.position, cur.waypoint.pose.position) >= driveParams.setHeadingTangentDistBetweenPoses) {
                cur.waypoint.setHeadingLerp(PathParams.HeadingLerpType.TANGENT);
                cur.waypoint.setHeadingTangentDeactivateThreshold(driveParams.correctHeadingBackFromTangentDist);
            }
        }

        return simplified;
    }

    // returns [fr, fl, bl, br] (counter clockwise)
    private static Vector2d[] getRobotCorners(Pose2d robotPose) {
        Vector2d position = robotPose.position;
        double halfW = generalParams.robotWidth * 0.5;
        double halfL = generalParams.robotLength * 0.5;

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
        double halfW = generalParams.robotWidth * 0.5;
        double halfL = generalParams.robotLength * 0.5;

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
        return getWallSafePose(robotPose, miscParams.wallPoseBuffer);
    }

    private static Pose2d getCollectPose(Vector2d ballPosition, double angle, double extraOffset) {
        double r = generalParams.collectPoseOffsetDistance + extraOffset;
        double dx = Math.cos(angle) * r;
        double dy = Math.sin(angle) * r;
        return new Pose2d(ballPosition.x - dx, ballPosition.y - dy, angle);
    }
    // assumes same heading as collect pose
    // the angle from pre collect to collect is not constrained to collect pose heading though
    private static Pose2d getPreCollectPose(Vector2d collectPosition, double preCollectHeading, double angleToCollectPose, double offsetDistance) {
        double dx = Math.cos(angleToCollectPose) * offsetDistance;
        double dy = Math.sin(angleToCollectPose) * offsetDistance;
        return new Pose2d(collectPosition.x - dx, collectPosition.y - dy, preCollectHeading);
    }
}
