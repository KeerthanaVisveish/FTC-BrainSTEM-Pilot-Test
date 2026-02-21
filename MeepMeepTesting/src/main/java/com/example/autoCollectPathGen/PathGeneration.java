package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;

public class PathGeneration {
    public static AutoCollectParams params = new AutoCollectParams();
    public static Vector2d[] ballsUsed = null; // used to draw balls on FTC dashboard

    public enum PoseType {
        COLLECT,
        STRICT_PRECOLLECT,
        PRECOLLECT
    }
    public static class PathPose {
        public final Pose2d pose;
        public final BallType ballType;
        public PoseType poseType;
        public PathPose(Pose2d pose, PoseType poseType, BallType ballType) {
            this.pose = pose;
            this.poseType = poseType;
            this.ballType = ballType;
        }
    }
    public static ArrayList<Pose2d> getAutoCollectPoses(boolean simplifyPath, Pose2d startPose, Vector2d[] ballsArray) {
        if (ballsArray.length == 0) {
            ballsUsed = null;
            return null;
        }

        ArrayList<Vector2d> allBalls = new ArrayList<>(Arrays.asList(ballsArray));
        ArrayList<Vector2d> balls = new ArrayList<>(Arrays.asList(ballsArray));

        int numPoints;
        int numCombinations;
        int numAttempts = 0;
        Path optimalPath = null;
        int leastProblemBalls = -1;
        int leastUndesirableBalls = -1;
        do {
            double desiredAngle = Math.signum(balls.get(0).y) * Math.toRadians(params.desiredAngle);
            ArrayList<Vector2d> rawBallPath = PathFinder.findShortestPath(startPose, balls, 3, params.changeInAngleDegCost, desiredAngle, params.changeInDesiredAngleDegCost);
            if (rawBallPath == null)
                return null;

            // generate path
            Path path = generatePath(startPose.position, allBalls, rawBallPath);
            int numProblemBalls = path.problemBalls.size();
            int numUndesirableBalls = path.undesirableBalls.size();
            if (leastProblemBalls == -1 || numProblemBalls < leastProblemBalls || (numProblemBalls == leastProblemBalls && numUndesirableBalls < leastUndesirableBalls)) {
                leastProblemBalls = numProblemBalls;
                leastUndesirableBalls = numUndesirableBalls;
                optimalPath = path;
            }

            // optimal path
            if (numProblemBalls == 0 && numUndesirableBalls == 0)
                break;

            // un-optimal
            for (Vector2d problemBall : path.problemBalls)
                balls.remove(problemBall);
            numAttempts++;
            numPoints = Math.min(3, balls.size());
            numCombinations = PathFinder.getCombinations(balls, numPoints).size();
        } while (balls.size() >= 3 && numCombinations > 0 && numAttempts <= params.maxPathRegenerationAttempts);

        ArrayList<PathPose> pathPoses = simplifyPath ? simplifyPathPoses(startPose, optimalPath.pathPoses) : optimalPath.pathPoses;
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (PathPose pathPose : pathPoses)
            poses.add(pathPose.pose);
        return poses;
    }

    // use a sliding window of width laneWidth moving at a speed of increment
    // returns the list of balls in the most densely packed moment of the window
//    private static Lane[] getDensestLanes(Vector2d[] points, double increment, double laneWidth) {
//        if (points.length == 0)
//            return null;
//
//        Vector2d[] sorted = Arrays.copyOf(points, points.length);
//        Arrays.sort(sorted, Comparator.comparingDouble(a -> a.x));
//        int start = (int) sorted[0].x - 1;
//        int end = (int) sorted[sorted.length-1].x + 1;
//
//        int numLanes = (int) ((end - laneWidth - start + 2) / increment);
//        if (numLanes <= 0)
//            numLanes = 1;
//        ArrayList<ArrayList<Vector2d>> ballsInLanes = new ArrayList<>();
//
//        for (int i=0; i<numLanes; i++) {
//            double left = start + i * increment;
//            double right = left + laneWidth;
//            ArrayList<Vector2d> ballsInLane = new ArrayList<>();
//            for (Vector2d vector2d : sorted)
//                if (vector2d.x >= left && vector2d.x <= right)
//                    ballsInLane.add(vector2d);
//            ballsInLanes.add(ballsInLane);
//        }
//        ArrayList<Lane> validLanes = new ArrayList<>(); // list of all lanes with highest number of balls
//        validLanes.add(new Lane(ballsInLanes.get(0)));
//
//        for (int i=1; i<ballsInLanes.size(); i++) {
//            ArrayList<Vector2d> lane = ballsInLanes.get(i);
//            if (lane.size() >= validLanes.get(0).numBalls()) {
//                // found new biggest clear out old ones
//                if (lane.size() > validLanes.get(0).numBalls())
//                    validLanes.clear();
//                validLanes.add(new Lane(lane));
//            }
//        }
//        return validLanes.toArray(new Lane[0]);
//    }
//    // returns the lane closest to the robot
//    private static Lane getBestLane(Vector2d robotPos, Lane[] densestLanes) {
//        if (densestLanes.length == 1)
//            return densestLanes[0];
//        Lane bestLane = null;
//        double minDist = -1;
//        for (int i=0; i<densestLanes.length; i++) {
//            Lane lane = densestLanes[i];
//            Vector2d firstPoint = new Vector2d(lane.avgX, lane.minAbsY());
//            double dist = Math.hypot(firstPoint.x - robotPos.x, firstPoint.y - robotPos.y);
//
//            if (bestLane == null || dist < minDist) {
//                bestLane = lane;
//                minDist = dist;
//            }
//        }
//        return bestLane;
//    }
//    private static class Lane {
//        private final Vector2d[] balls;
//        private final double avgX;
//        public Lane(ArrayList<Vector2d> balls) {
//            this.balls = balls.toArray(new Vector2d[0]);
//            // sort by |y| value ascending (so it works for both red and blue)
//            Arrays.sort(this.balls, (v1, v2) -> (int) (1000000 * (Math.abs(v1.y) - Math.abs(v2.y))));
//            if (balls.isEmpty())
//                this.avgX = 0;
//            else {
//                double totalX = 0;
//                for (Vector2d ball : balls)
//                    totalX += ball.x;
//                this.avgX = totalX / balls.size();
//            }
//        }
//        public int numBalls() {
//            return balls.length;
//        }
//        public double minAbsY() {
//            return balls[0].y;
//        }
//        public double maxAbsY() {
//            return balls[balls.length - 1].y;
//        }
//        @Override
//        public String toString() {
//            return "avgx: " + avgX + " | num balls: " + numBalls();
//        }
//    }
//    private static ArrayList<Pose2d> generateSimplePathPoses(boolean onRedAlliance, Lane lane) {
//        double angle = onRedAlliance ? Math.toRadians(90) : -Math.toRadians(90);
//        Vector2d ball1 = new Vector2d(lane.avgX, lane.minAbsY());
//        Vector2d ball2 = new Vector2d(lane.avgX, lane.maxAbsY());
//        ballsUsed = new Vector2d[] { ball1, ball2 };
//
//        Pose2d collect1 = getCollectPose(ball1, angle, 0);
//        Pose2d collect2 = getCollectPose(ball2, angle, 0);
//        Pose2d preCollect1 = getPreCollectPose(collect1, angle, params.preCollectOffset);
//        return new ArrayList<>(Arrays.asList(preCollect1, collect1, collect2));
//    }

    public enum BallType {
        NORMAL,
        CORNER,
        CLASSIFIER_WALL,
        BACK_WALL
    }
    private static class Path {
        public final ArrayList<PathPose> pathPoses;
        public final ArrayList<Vector2d> problemBalls, undesirableBalls;
        public Path( ArrayList<PathPose> pathPoses, ArrayList<Vector2d> problemBalls, ArrayList<Vector2d> undesirableBalls) {
            this.pathPoses = pathPoses;
            this.problemBalls = problemBalls;
            this.undesirableBalls = undesirableBalls;
        }
    }
    private static Path generatePath(Vector2d robotPos, ArrayList<Vector2d> allBalls, ArrayList<Vector2d> originalPath) {
        ArrayList<PathPose> pathPoses = new ArrayList<>();
        ArrayList<Vector2d> problemBalls = new ArrayList<>();
        ArrayList<Vector2d> undesirableBalls = new ArrayList<>();
        if (originalPath.isEmpty())
            return new Path(pathPoses, problemBalls, undesirableBalls);
        ArrayList<Vector2d> path = new ArrayList<>(originalPath);

        ArrayList<Vector2d> allBallsInCluster = new ArrayList<>();
        ArrayList<ClusterApproach> clusterApproaches = new ArrayList<>();

        for (int i=0; i<path.size(); i++) {
            Vector2d prevPathPoint = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).pose.position : robotPos;
            Vector2d curBall = path.get(i);
            Vector2d nextBall = i < path.size() - 1 ? path.get(i + 1) : null;
            BallType ballType = getPointType(curBall);
            BallType nextBallType = nextBall == null ? null : getPointType(nextBall);
            if (ballType == BallType.CORNER)
                problemBalls.add(curBall);

            Pose2d wallSafePreStrafeCollectPose = null;
            ArrayList<Pose2d> wallSafeStrafeCollectPoses = new ArrayList<>();

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
                    wallSafeStrafeCollectPoses.addAll(clusterApproach.collectPoses);
                }
                else
                    curBall = MathUtils.getAverage(clusterApproach.allBallsInCluster);
                problemBalls.addAll(clusterApproach.problemBalls);
                allBallsInCluster.clear();
                clusterApproaches.clear();
            }

            // use cluster strafe collection
            if (wallSafePreStrafeCollectPose != null) {
                pathPoses.add(new PathPose(wallSafePreStrafeCollectPose, PoseType.STRICT_PRECOLLECT, ballType));
                for (Pose2d collectPose : wallSafeStrafeCollectPoses)
                    pathPoses.add(new PathPose(collectPose, PoseType.COLLECT, ballType));
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
                if (!wallSafeCollectPose.equals(collectPose)) {
                    wallSafePreCollectPose = getPreCollectPose(wallSafeCollectPose, collectInfo.preCollectToCollectAngle, totalPreCollectOffset);
                    wallSafePreCollectPose = getWallSafePose(wallSafePreCollectPose);
                }
                pathPoses.add(new PathPose(wallSafePreCollectPose, ballType == BallType.NORMAL ? PoseType.PRECOLLECT : PoseType.STRICT_PRECOLLECT, ballType));
                pathPoses.add(new PathPose(wallSafeCollectPose, PoseType.COLLECT, ballType));
            }
        }

//        for (Vector2d ball : allBalls) {
//            PointType type = getPointType(ball);
//            if (type == PointType.NORMAL || originalPath.contains(ball) && !problemBalls.contains(ball))
//                continue;
//            // if ball is inside robot
//        }

        ballsUsed = path.toArray(new Vector2d[0]);
        return new Path(pathPoses, problemBalls, undesirableBalls);
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
        public final ArrayList<Pose2d> collectPoses;
        public final ArrayList<Vector2d> problemBalls;
        public ClusterApproach(Vector2d prev, Vector2d start, Vector2d end) {
            this.start = start;
            this.end = end;
            collectPoses = new ArrayList<>();
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
            this.allBallsInCluster = allBallsInCluster;
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
                    problemBalls.add(potentialBall);

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

            collectPoses.add(getCollectPose(actualEnd, collectAngle, 0));
            Pose2d startCollectPose = getCollectPose(actualStart, collectAngle, 0);
            preCollectPose = getPreCollectPose(startCollectPose, approachAngle, params.preCollectOffset);
            Pose2d wallSafePreCollectPose = getWallSafePose(preCollectPose);
            isWallSafe = wallSafePreCollectPose.equals(preCollectPose);
            if (!isWallSafe) {
//                System.out.println("not wall safe, adding " + actualStartRawBallPosition);
                problemBalls.add(actualStartRawBallPosition);
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
    private static BallType getPointType(Vector2d curBall) {
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
        double preCollectToCollectAngle;
        double collectAngle;
        double preCollectOffset = params.preCollectOffset;
        double collectXOffset = 0;
        double collectYOffset = 0;
        switch (ballType) {
            case CORNER:
                collectAngle = Math.signum(wallAngle) * (Math.abs(wallAngle) - Math.toRadians(params.cornerCollectAngle));
                preCollectToCollectAngle = collectAngle;
                collectYOffset = Math.signum(curBall.y) * Math.max(0, params.cornerCollectY - Math.abs(curBall.y));
                break;
            case CLASSIFIER_WALL:
                double angleD = Math.abs(MathUtils.angleNormDeltaRad(defaultApproachAngle - wallAngle));
                if (angleD > Math.toRadians(params.wallStrafeCollectMinApproachAngle)
                        || nextBallType == BallType.CORNER || nextBallType == BallType.CLASSIFIER_WALL
                        || (nextBallType == BallType.BACK_WALL && curToNextBallDist < params.distToNextPointForceClassifierStrafe)) {
                    double dx = curToNextBall == null ? curBall.x - prevPathPoint.x : curToNextBall.x;
                    collectXOffset = Math.signum(dx) * params.strafeCollectDriveThroughDist;
                    preCollectOffset += Math.abs(collectXOffset);
                    if (Math.signum(dx) == 1)
                        collectAngle = Math.toRadians(params.wallCollectAngle);
                    else
                        collectAngle = Math.PI - Math.toRadians(params.wallCollectAngle);
                    collectAngle *= Math.signum(wallAngle);
                    preCollectToCollectAngle = Math.signum(dx) == 1 ? 0 : Math.PI;
                }
                else {
                    collectAngle = defaultApproachAngle;
                    preCollectToCollectAngle = collectAngle;
                }
                break;
            case BACK_WALL:
                if (Math.abs(defaultApproachAngle) > Math.toRadians(params.wallStrafeCollectMinApproachAngle)) {
                    double dy = curBall.y - prevPathPoint.y;
                    collectYOffset = Math.signum(dy) * params.strafeCollectDriveThroughDist;
                    preCollectOffset += Math.abs(collectYOffset);
                    collectAngle = Math.toRadians(params.wallCollectAngle) * Math.signum(dy);
                    preCollectToCollectAngle = Math.toRadians(90) * Math.signum(dy);
                }
                else {
                    collectAngle = defaultApproachAngle;
                    preCollectToCollectAngle = collectAngle;
                }
                break;
            case NORMAL:
            default:
                collectAngle = defaultApproachAngle;
                preCollectToCollectAngle = collectAngle;
                break;
        }
        return new CollectInfo(preCollectOffset, preCollectToCollectAngle, collectAngle, collectXOffset, collectYOffset);
    }

    private static class CollectInfo {
        public final double preCollectOffset, preCollectToCollectAngle, collectAngle;
        public final Vector2d collectOffset;
        public CollectInfo(double preCollectOffset, double preCollectToCollectAngle, double collectAngle, double collectXOffset, double collectYOffset) {
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

            // skip any poses between classifier wall strafes
            if (prev != null)
                if (prev.ballType == BallType.CLASSIFIER_WALL && next.ballType == BallType.CLASSIFIER_WALL &&
                        cur.poseType == PoseType.STRICT_PRECOLLECT)
                    continue;

            Vector2d prevToCur = cur.pose.position.minus(prevPose.position);
            Vector2d curToNext = next.pose.position.minus(cur.pose.position);
            double headingDiff = MathUtils.angleNormDeltaRad(cur.pose.heading.toDouble() - prevPose.heading.toDouble());
            double approachAngleDiff = MathUtils.angleRadDiff(curToNext, prevToCur);

            double headingSimplification = Math.toRadians(params.maxPreCollectHeadingDifference.applyAsDouble(MathUtils.vecMag(prevToCur)));
            double approachAngleSimplification = Math.toRadians(params.maxCollectApproachDifference);
            boolean approachAnglesCloseEnough = Math.abs(approachAngleDiff) < approachAngleSimplification || Math.abs(approachAngleDiff) > Math.PI - approachAngleSimplification;
            boolean headingCloseEnough = Math.abs(headingDiff) < headingSimplification;
            if (cur.poseType == PoseType.PRECOLLECT) {
//                    System.out.println(i + " | PRE COLLECT: " + MathUtils.formatRad2(headingDiff) + " | < " + MathUtils.formatRad2(headingSimplification));
                if (headingCloseEnough)
                    continue;
            }
            else if (cur.poseType == PoseType.COLLECT || cur.poseType == PoseType.STRICT_PRECOLLECT) {
//                    System.out.println(i + " | " + curType + ": " + MathUtils.formatRad2(approachAngleDiff) + " | < " + params.maxCollectApproachDifference);
                if (headingCloseEnough && approachAnglesCloseEnough)
                    continue;
            }


            simplified.add(pathPoses.get(i));
        }
        simplified.add(pathPoses.get(pathPoses.size() - 1));
        return simplified;
    }
    public static Pose2d getWallSafePose(Pose2d pose) {
        Vector2d position = pose.position;
        double halfW = params.robotWidth * 0.5;
        double halfL = params.robotLength * 0.5;

        Vector2d[] positionsToCheck = new Vector2d[] {
                new Vector2d(halfL, -halfW), // fr
                new Vector2d(halfL, halfW), // fl
                new Vector2d(-halfL, -halfW), // br
                new Vector2d(-halfL, halfW) // bl
        };
        double d = Math.hypot(halfW, halfL) - params.wallPoseBuffer;
        if (position.x > -72 + d && position.x < 72 - d && position.y > -72 + d && position.y < 72 - d)
            return pose;

        Vector2d[] requiredOffsets = new Vector2d[positionsToCheck.length];
        for (int j=0; j<positionsToCheck.length; j++) {
            Vector2d pos = GeometryUtils.rotateVector(positionsToCheck[j], pose.heading.toDouble()).plus(position);
            double wallY = Math.signum(pos.y) * (72 + params.wallPoseBuffer);

            double dx = (72 + params.wallPoseBuffer) - pos.x;
            double dy = wallY - pos.y;
            dx = Math.min(0, dx);
            if (wallY > 0)
                dy = Math.min(0, dy);
            else
                dy = Math.max(0, dy);
            requiredOffsets[j] = new Vector2d(dx, dy);
        }
        double requiredDx = requiredOffsets[0].x;
        double requiredDy = requiredOffsets[0].y;
        for (Vector2d offset : requiredOffsets) {
            requiredDx = Math.min(offset.x, requiredDx);
            if (Math.abs(offset.y) > Math.abs(requiredDy))
                requiredDy = offset.y;
        }

        return new Pose2d(position.x + requiredDx, position.y + requiredDy, pose.heading.toDouble());
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
