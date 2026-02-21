package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static PathGenerationParams params = new PathGenerationParams();
    public static Vector2d[] ballsUsed = null; // used to draw balls on FTC dashboard
    public static PathInfo getAutoCollectPath(Pose2d startPose, Vector2d[] ballsArray) {
        if (ballsArray.length == 0) {
            ballsUsed = null;
            return null;
        }

        ArrayList<Vector2d> allBalls = new ArrayList<>(Arrays.asList(ballsArray));

        Lane[] densestLanes = getDensestLanes(allBalls);
        if (densestLanes[0].numBalls() >= 3) {
            Lane bestLane = getBestLane(startPose.position, densestLanes);
            return generateLanePath(bestLane);
        }

        int numPoints;
        int numCombinations;
        int numAttempts = 0;
        PathInfo optimalPathInfo = null;
        int leastProblemBalls = -1;
        ArrayList<Vector2d> balls = new ArrayList<>(Arrays.asList(ballsArray));
        ArrayList<ProblemBall> prevGenerationProblemBalls = new ArrayList<>();
        do {
//            System.out.println("re attempt " + numAttempts);
            // remove most severe problem balls
            int lowestSeverityOrdinal = 100;
//            System.out.println("previous problem balls: ");
            for (ProblemBall problemBall : prevGenerationProblemBalls) {
//                System.out.println(problemBall.severity + ": " + problemBall.ballPosition);
                if (problemBall.severity.ordinal() <= lowestSeverityOrdinal)
                    lowestSeverityOrdinal = problemBall.severity.ordinal();
            }
//            System.out.println("lowest ordinal: " + lowestSeverityOrdinal);
            ArrayList<Vector2d> ignoredBalls = new ArrayList<>();
            for (ProblemBall problemBall : prevGenerationProblemBalls)
                if (problemBall.severity.ordinal() == lowestSeverityOrdinal) {
                    balls.remove(problemBall.ballPosition);
                    ignoredBalls.add(problemBall.ballPosition);
                }
//            System.out.println("considering " + balls);
//            System.out.println("ignoring " + ignoredBalls);
            numPoints = Math.min(3, balls.size());
            numCombinations = PathFinder.getCombinations(balls, numPoints).size();
            if (numCombinations == 0)
                break;

            double desiredAngle = Math.signum(balls.get(0).y) * Math.toRadians(params.desiredAngle);
            ArrayList<Vector2d> rawBallPath = PathFinder.findShortestPath(startPose, balls, 3, params.changeInAngleDegCost, desiredAngle, params.changeInDesiredAngleDegCost);
            if (rawBallPath == null)
                return null;

            // generate path
            PathInfo pathInfo = generatePath(startPose.position, allBalls, rawBallPath);
            pathInfo.setIgnoredBalls(ignoredBalls);
            int numProblemBalls = pathInfo.problemBalls.size();
            if (leastProblemBalls == -1 || numProblemBalls < leastProblemBalls) {
                leastProblemBalls = numProblemBalls;
                optimalPathInfo = pathInfo;
            }

            // optimal path
            if (numProblemBalls == 0)
                break;

            // un-optimal
            prevGenerationProblemBalls = pathInfo.problemBalls;

            numAttempts++;
        } while (numAttempts <= params.maxPathRegenerationAttempts);

        optimalPathInfo.setSimplifiedPathPoses(simplifyPathPoses(startPose, optimalPathInfo.pathPoses)); // never actually going to be null
//        System.out.println(optimalPathInfo.problemBalls);
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
        private final Vector2d[] balls;
        public boolean closeToBackWall;
        private final double avgX;
        public Lane(ArrayList<Vector2d> balls, boolean closeToBackWall) {
            this.balls = balls.toArray(new Vector2d[0]);
            this.closeToBackWall = closeToBackWall;
            // sort by |y| value ascending (so it works for both red and blue)
            Arrays.sort(this.balls, (v1, v2) -> (int) (1000000 * (Math.abs(v1.y) - Math.abs(v2.y))));
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
            return balls.length;
        }
        public double minAbsY() {
            return balls[0].y;
        }
        public double maxAbsY() {
            return balls[balls.length - 1].y;
        }
        @Override
        public String toString() {
            return "avgx: " + avgX + " | num balls: " + numBalls();
        }
    }
    private static PathInfo generateLanePath(Lane lane) {
        double angle = Math.signum(lane.balls[0].y) * Math.toRadians(90);
        Vector2d ball1 = new Vector2d(lane.avgX, lane.minAbsY());
        Vector2d ball2 = new Vector2d(lane.avgX, lane.maxAbsY());
        ballsUsed = new Vector2d[] { ball1, ball2 };

        Pose2d collect1 = getCollectPose(ball1, angle, 0);
        Pose2d collect2 = getCollectPose(ball2, angle, 0);
        Pose2d preCollect1 = getPreCollectPose(collect1, angle, params.preCollectOffset);
        Pose2d wallSafeCollect2 = getWallSafePose(collect2);
        Pose2d wallSafePreCollect = getWallSafePose(preCollect1);

        PathPose pathPose1 = new PathPose(wallSafePreCollect, Types.PoseType.PRECOLLECT, ball1, BallType.NORMAL, Types.Approach.NORMAL);
        PathPose pathPose2 = new PathPose(wallSafeCollect2, Types.PoseType.COLLECT, ball1, BallType.NORMAL, Types.Approach.NORMAL);
        ArrayList<PathPose> pathPoses = new ArrayList<>(Arrays.asList(pathPose1, pathPose2));
        PathInfo pathInfo = new PathInfo(pathPoses, new ArrayList<>());
        pathInfo.setSimplifiedPathPoses(pathPoses);
        return pathInfo;
    }
    private static PathInfo generatePath(Vector2d robotPos, ArrayList<Vector2d> allBalls, ArrayList<Vector2d> originalPath) {
        ArrayList<PathPose> pathPoses = new ArrayList<>();
        ArrayList<ProblemBall> problemBalls = new ArrayList<>();
        if (originalPath.isEmpty())
            return new PathInfo(pathPoses, problemBalls);
        ArrayList<Vector2d> path = new ArrayList<>(originalPath);

        ArrayList<Vector2d> allBallsInCluster = new ArrayList<>();
        ArrayList<ClusterApproach> clusterApproaches = new ArrayList<>();

        for (int i=0; i<path.size(); i++) {
            Vector2d prevPathPoint = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).pose.position : robotPos;
            Vector2d curBall = path.get(i);
            Vector2d nextBall = i < path.size() - 1 ? path.get(i + 1) : null;
            BallType ballType = getBallType(curBall);
            BallType prevBallType = getBallType(i > 0 ? path.get(i - 1) : robotPos);
            BallType nextBallType = nextBall == null ? null : getBallType(nextBall);
            if (ballType == BallType.CORNER)
                problemBalls.add(new ProblemBall(ProblemBall.Severity.CORNER, curBall));

            Pose2d wallSafePreStrafeCollectPose = null;
            Pose2d wallSafeStrafeCollectPose = null;

            // keep adding cur ball into the cluster as long as it is close enough with next ball are close enough
            if (nextBall != null) {
                Vector2d curToNextBall = nextBall.minus(curBall);
                double curToNextBallDist = Math.hypot(curToNextBall.x, curToNextBall.y);
                if (curToNextBallDist < params.clusterStrafingDist) {
                    // && nextPointType == PointType.NORMAL
                    if (ballType == BallType.NORMAL) {
                        if (!allBallsInCluster.contains(curBall))
                            allBallsInCluster.add(curBall);
                        allBallsInCluster.add(nextBall);
                        clusterApproaches.add(new ClusterApproach(prevPathPoint, curBall, nextBall));
                        continue;
                    }
                }
            }
            // this only runs if current cluster has been finished merging
            // once current cluster has finished, find approach information
            if (!allBallsInCluster.isEmpty()) {
                ClusterApproach clusterApproach = getBestClusterApproach(clusterApproaches, allBallsInCluster);
                if (clusterApproach.isWallSafe) {
                    wallSafePreStrafeCollectPose = clusterApproach.preCollectPose;
                    wallSafeStrafeCollectPose = clusterApproach.collectPose;
                }
                else {
                    boolean closeEnoughToMerge = true;
                    for (Vector2d ballInCluster : clusterApproach.allBallsInCluster) {
                        if (MathUtils.vecMag(curBall.minus(ballInCluster)) > params.failedClusterGroupAsOneDist) {
                            closeEnoughToMerge = false;
                            break;
                        }
                    }
                    if (closeEnoughToMerge)
                        curBall = MathUtils.getAverage(clusterApproach.allBallsInCluster);
                    else {
                        double closestToPrevDist = -1;
                        Vector2d closestToPrevBall = null;
                        for (Vector2d ballInCluster : clusterApproach.allBallsInCluster) {
                            double dist = MathUtils.vecMag(ballInCluster.minus(prevPathPoint));
                            if (closestToPrevDist == -1 || dist < closestToPrevDist) {
                                closestToPrevDist = dist;
                                closestToPrevBall = ballInCluster;
                            }
                        }
                        curBall = closestToPrevBall;
                        for (Vector2d ballInCluster : clusterApproach.allBallsInCluster) {
                            if (!ballInCluster.equals(closestToPrevBall))
                                problemBalls.add(new ProblemBall(ProblemBall.Severity.FAILED_CLUSTER_MERGE, ballInCluster));
                        }
                    }
                }
                problemBalls.addAll(clusterApproach.problemBalls);
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
                double defaultApproachAngle = Math.atan2(curBall.y - prevPathPoint.y, curBall.x - prevPathPoint.x);
                double preCollectExtraOffset = 0;
                double collectExtraOffset = 0;

                // build collect and pre collect poses based on the type of point
                CollectInfo collectInfo = getCollectInfo(ballType, nextBallType, prevPathPoint, curBall, nextBall, defaultApproachAngle);
                if (nextBall == null && ballType == BallType.NORMAL)
                    collectExtraOffset += params.lastCollectPoseExtraDriveThrough;

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
                    default:
                        validWallSafePose = wallSafeCollectPose.equals(collectPose);
                }
                if (!validWallSafePose) {
                    wallSafePreCollectPose = getPreCollectPose(wallSafeCollectPose, collectInfo.preCollectToCollectAngle, totalPreCollectOffset);
                    wallSafePreCollectPose = getWallSafePose(wallSafePreCollectPose);
                    problemBalls.add(new ProblemBall(ProblemBall.Severity.OUT_OF_BOUNDS, curBall));
                }

                if (ballType == BallType.CLASSIFIER_WALL && prevBallType != BallType.CLASSIFIER_WALL) {
                    Vector2d[] robotCorners = getRobotCorners(wallSafePreCollectPose); // fr, fl, bl, fr
                    ArrayList<Vector2d> polygon = new ArrayList<>(Arrays.asList(robotCorners));
                    polygon.add(1, robotPos);
                    for (Vector2d ballToCheck : allBalls) {
                        if (originalPath.contains(ballToCheck) || getBallType(ballToCheck) != BallType.CLASSIFIER_WALL)
                            continue;
                        boolean hitting = GeometryUtils.isCircleInsidePolygon(polygon, ballToCheck, 1);
                        if (hitting)
                            problemBalls.add(new ProblemBall(ProblemBall.Severity.OVERLAPPING_CLASSIFIER_WALL, curBall));
                    }
                }

                pathPoses.add(new PathPose(wallSafePreCollectPose, ballType == BallType.NORMAL ? Types.PoseType.PRECOLLECT : Types.PoseType.EDGE_CASE_PRECOLLECT, curBall, ballType, collectInfo.approachType));
                pathPoses.add(new PathPose(wallSafeCollectPose, Types.PoseType.COLLECT, curBall, ballType, collectInfo.approachType));
            }
        }

        for (int i=1; i<pathPoses.size() - 1; i++) {
            PathPose prev = pathPoses.get(i - 1);
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i + 1);
            if (cur.poseType != Types.PoseType.PRECOLLECT && cur.poseType != Types.PoseType.EDGE_CASE_PRECOLLECT)
                continue;

            Vector2d prevToCur = cur.pose.position.minus(prev.pose.position);
            Vector2d curToNext = next.pose.position.minus(cur.pose.position);
            if (prevToCur.dot(curToNext) < 0 && prev.ballType != BallType.CLASSIFIER_WALL && next.ballType != BallType.CLASSIFIER_WALL)
                problemBalls.add(new ProblemBall(ProblemBall.Severity.BACK_TRACKING, cur.ball));
        }

        ballsUsed = path.toArray(new Vector2d[0]);
        return new PathInfo(pathPoses, problemBalls);
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
    private static CollectInfo getCollectInfo(BallType ballType, BallType nextBallType, Vector2d prevPathPoint, Vector2d curBall, Vector2d nextBall, double defaultApproachAngle) {
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
                break;
            case CLASSIFIER_WALL:
                double angleD = Math.abs(MathUtils.angleNormDeltaRad(defaultApproachAngle - wallAngle));
                boolean useStrafeApproach = angleD > Math.toRadians(params.wallStrafeCollectMinApproachAngle)
                        || nextBallType == BallType.CORNER || nextBallType == BallType.CLASSIFIER_WALL
                        || (nextBallType == BallType.BACK_WALL && curToNextBallDist < params.distToNextPointForceClassifierStrafe);
                if (useStrafeApproach) {
                    approachType = Types.Approach.CLASSIFIER_WALL_STRAFE;
                    double dx = curToNextBall == null ? curBall.x - prevPathPoint.x : curToNextBall.x;
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
                    double dy = curBall.y - prevPathPoint.y;
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
    private static ArrayList<PathPose> simplifyPathPoses(Pose2d startPose, ArrayList<PathPose> pathPoses) {
        ArrayList<PathPose> simplified = new ArrayList<>();
        if (pathPoses.isEmpty())
            return simplified;
        if (pathPoses.size() == 1) {
            simplified.add(pathPoses.get(0));
            return simplified;
        }
        for (int i=0; i<pathPoses.size()-1; i++) {
            Pose2d prevPose = i > 0 ? pathPoses.get(i - 1).pose : startPose;
            PathPose prev = i > 0 ? pathPoses.get(i - 1) : null;
            PathPose cur = pathPoses.get(i);
            PathPose next = pathPoses.get(i + 1);

            // skip any poses between classifier or back wall strafes
            if (prev != null) {
                if (prev.approachType == Types.Approach.CLASSIFIER_WALL_STRAFE && next.approachType == Types.Approach.CLASSIFIER_WALL_STRAFE)
                    continue;
                if (prev.approachType == Types.Approach.BACK_WALL_STRAFE && next.approachType == Types.Approach.BACK_WALL_STRAFE)
                    continue;
            }

            Vector2d prevToCur = cur.pose.position.minus(prevPose.position);
            Vector2d curToNext = next.pose.position.minus(cur.pose.position);
            double headingDiff = MathUtils.angleNormDeltaRad(cur.pose.heading.toDouble() - prevPose.heading.toDouble());
            double approachAngleDiff = MathUtils.angleRadDiff(curToNext, prevToCur);

            double headingSimplification = Math.toRadians(params.maxPreCollectHeadingDifference.applyAsDouble(MathUtils.vecMag(prevToCur)));
            double approachAngleSimplification = Math.toRadians(params.maxCollectApproachDifference);
            boolean approachAnglesCloseEnough = Math.abs(approachAngleDiff) < approachAngleSimplification;
            boolean headingCloseEnough = Math.abs(headingDiff) < headingSimplification;
            if (cur.poseType == Types.PoseType.PRECOLLECT) {
//                    System.out.println(i + " | PRE COLLECT: " + MathUtils.formatRad2(headingDiff) + " | < " + MathUtils.formatRad2(headingSimplification));
                if (headingCloseEnough)
                    continue;
            }
            else if (cur.poseType == Types.PoseType.COLLECT || cur.poseType == Types.PoseType.EDGE_CASE_PRECOLLECT) {
//                    System.out.println(i + " | " + curType + ": " + MathUtils.formatRad2(approachAngleDiff) + " | < " + params.maxCollectApproachDifference);
                if (headingCloseEnough && approachAnglesCloseEnough)
                    continue;
            }


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
    public static Pose2d getWallSafePose(Pose2d robotPose) {
        Vector2d position = robotPose.position;
        double halfW = params.robotWidth * 0.5;
        double halfL = params.robotLength * 0.5;

        double d = Math.hypot(halfW, halfL) - params.wallPoseBuffer;
        if (position.x > -72 + d && position.x < 72 - d && position.y > -72 + d && position.y < 72 - d)
            return robotPose;

        Vector2d[] corners = getRobotCorners(robotPose);
        Vector2d[] requiredOffsets = new Vector2d[corners.length];
        for (int i=0; i<corners.length; i++) {
            Vector2d corner = corners[i];
            double wallY = Math.signum(corner.y) * (72 + params.wallPoseBuffer);

            double dx = (72 + params.wallPoseBuffer) - corner.x;
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
