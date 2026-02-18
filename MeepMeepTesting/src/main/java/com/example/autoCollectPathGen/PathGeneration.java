package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static class AutoCollectParams {
        public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 8 extra inches
        public double laneIncrement = 1, laneWidth = 6;
        public double groupAsClusterDist = 8;
        public double groupAsClusterAngle = 45;
        public double constrainClusterApproachAngleThreshold = 50;
        public double maxPathSimplificationEpsilon = 1; // max dist from original path to simplified path
        public double maxHeadingChangeSimplification = 5;

        public double wallPoseBuffer = 0; // the pose can be outside the field walls by this much

        public double collectPoseOffsetDistance = 7;
        public double preCollectOffset = 5;

        public double cornerBallDistance = 6.5;

        public double classifierWallDistance = 4;
        public double classifierWallMaxApproachAngleDiff = 10;

        public double backWallDistance = 5;
        public double backWallCollectYOffset = 2;
        public double backWallCollectAngle = 45;
        public double backWallStrafeCollectMinApproachAngle = 45;

    }
    public static AutoCollectParams params = new AutoCollectParams();

    public static Vector2d[] ballsUsed = null; // used to draw balls on FTC dashboard

    public static ArrayList<Pose2d> getSimplifiedAutoCollectPathPoses(boolean onRedAlliance, double robotWidth, double robotLength, Pose2d startPose, Vector2d[] points, int simplePathingThreshold, int maxPointsInPath) {
        ArrayList<Pose2d> pathPoses = getAutoCollectPathPoses(onRedAlliance, robotWidth, robotLength, startPose, points, simplePathingThreshold, maxPointsInPath);
        if (pathPoses == null)
            return null;
        return simplifyPathPoses(startPose, pathPoses);
    }
    public static ArrayList<Pose2d> getAutoCollectPathPoses(boolean onRedAlliance, double robotWidth, double robotLength, Pose2d startPose, Vector2d[] points, int simplePathingThreshold, int maxPointsInPath) {
        if (points.length == 0) {
            ballsUsed = null;
            return null;
        }

        Vector2d startPosition = startPose.position;

        // simple sliding window approach
        // find lane with densest amount of balls
        // drive in straight line through the lane
        Lane[] bestLanes = getDensestLanes(points, params.laneIncrement, params.laneWidth);
        if (bestLanes == null) {
            ballsUsed = null;
            return null;
        }
        if (bestLanes[0].numBalls() >= simplePathingThreshold) {
            // find closest lane if there are multiple lanes
            Lane bestLane = getBestLane(startPosition, bestLanes);
            return generateSimplePathPoses(onRedAlliance, bestLane);
        }

        // more complex pathfinding approach
        // when there are not enough balls for sliding window, actually find ideal combo of balls and drive path
        ArrayList<Vector2d> path = getShortestPath(startPose, points, maxPointsInPath);
        return generateComplexPathPoses(onRedAlliance, robotWidth, robotLength, startPosition, path);
    }

    // use a sliding window of width laneWidth moving at a speed of increment
    // returns the list of balls in the most densely packed moment of the window
    private static Lane[] getDensestLanes(Vector2d[] points, double increment, double laneWidth) {
        if (points.length == 0)
            return null;

        Vector2d[] sorted = Arrays.copyOf(points, points.length);
        Arrays.sort(sorted, Comparator.comparingDouble(a -> a.x));
        int start = (int) sorted[0].x - 1;
        int end = (int) sorted[sorted.length-1].x + 1;

        int numLanes = (int) ((end - laneWidth - start + 2) / increment);
        if (numLanes <= 0)
            numLanes = 1;
        ArrayList<ArrayList<Vector2d>> ballsInLanes = new ArrayList<>();

        for (int i=0; i<numLanes; i++) {
            double left = start + i * increment;
            double right = left + laneWidth;
            ArrayList<Vector2d> ballsInLane = new ArrayList<>();
            for (Vector2d vector2d : sorted)
                if (vector2d.x >= left && vector2d.x <= right)
                    ballsInLane.add(vector2d);
            ballsInLanes.add(ballsInLane);
        }
        ArrayList<Lane> validLanes = new ArrayList<>(); // list of all lanes with highest number of balls
        validLanes.add(new Lane(ballsInLanes.get(0)));

        for (int i=1; i<ballsInLanes.size(); i++) {
            ArrayList<Vector2d> lane = ballsInLanes.get(i);
            if (lane.size() >= validLanes.get(0).numBalls()) {
                // found new biggest clear out old ones
                if (lane.size() > validLanes.get(0).numBalls())
                    validLanes.clear();
                validLanes.add(new Lane(lane));
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

            if (bestLane == null || dist < minDist) {
                bestLane = lane;
                minDist = dist;
            }
        }
        return bestLane;
    }
    private static class Lane {
        private final Vector2d[] balls;
        private final double avgX;
        public Lane(ArrayList<Vector2d> balls) {
            this.balls = balls.toArray(new Vector2d[0]);
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
    private static ArrayList<Pose2d> generateSimplePathPoses(boolean onRedAlliance, Lane lane) {
        double angle = onRedAlliance ? Math.toRadians(90) : -Math.toRadians(90);
        Vector2d ball1 = new Vector2d(lane.avgX, lane.minAbsY());
        Vector2d ball2 = new Vector2d(lane.avgX, lane.maxAbsY());
        ballsUsed = new Vector2d[] { ball1, ball2 };

        Pose2d collect1 = getCollectPose(ball1, angle, 0);
        Pose2d collect2 = getCollectPose(ball2, angle, 0);
        Pose2d preCollect1 = getPreCollectPose(collect1, angle, params.preCollectOffset);
        return new ArrayList<>(Arrays.asList(preCollect1, collect1, collect2));
    }

    private static ArrayList<Vector2d> getShortestPath(Pose2d startPose, Vector2d[] nodes, int maxBlobsInPath) {
        return PathFinder.findShortestPath(startPose, nodes, maxBlobsInPath, params.changeInAngleDegCost);
    }
    private enum PointType {
        NORMAL,
        CORNER,
        CLASSIFIER_WALL,
        BACK_WALL
    }
    private static ArrayList<Pose2d> generateComplexPathPoses(boolean onRedAlliance, double robotWidth, double robotLength, Vector2d robotPos, ArrayList<Vector2d> path) {
        ArrayList<Pose2d> pathPoses = new ArrayList<>();
        Vector2d cornerPosition = new Vector2d(72, onRedAlliance ? 72 : -72);
        ArrayList<Vector2d> clusteredPositions = new ArrayList<>();
        ArrayList<double[]> clusteredCollectAngleInfo = new ArrayList<>(); // [[angle, absAngleDiff]]

        for (int i=0; i<path.size(); i++) {
            Vector2d curBall = path.get(i);
            Vector2d nextBall = i < path.size() - 1 ? path.get(i + 1) : null;
            Vector2d prevPathPoint = !pathPoses.isEmpty() ? pathPoses.get(pathPoses.size() - 1).position : robotPos;

            double defaultApproachAngle = -100;
            double preCollectExtraOffset = 0;
            double collectExtraOffset = 0;

            // keep merging balls into clusters as long as they are close enough
            if (nextBall != null) {
                Vector2d curToNextBall = nextBall.minus(curBall);
                double curToNextBallDist = Math.hypot(curToNextBall.x, curToNextBall.y);
                if (curToNextBallDist < params.groupAsClusterDist) {
                    if (!clusteredPositions.contains(curBall))
                        clusteredPositions.add(curBall);
                    clusteredPositions.add(nextBall);
                    Vector2d prevPathPointToCurBall = curBall.minus(prevPathPoint);
                    double prevToCurAngle = Math.atan2(prevPathPointToCurBall.y, prevPathPointToCurBall.x);
                    double curToNextAngle = Math.atan2(curToNextBall.y, curToNextBall.x);
                    double angleDiff = Math.abs(MathUtils.findAngleRadDiff(curToNextAngle, prevToCurAngle));

                    if (angleDiff < Math.toRadians(params.constrainClusterApproachAngleThreshold))
                        clusteredCollectAngleInfo.add(new double[]{curToNextAngle, angleDiff});

                    path.set(i + 1, getMergedBall(nextBall, clusteredPositions, angleDiff));
                    path.remove(i);
                    i--;
                    continue;
                }
            }

            // this only runs if current cluster has been finished merging
            // once current cluster has finished, find approach information
            if (!clusteredPositions.isEmpty()) {
                if (!clusteredCollectAngleInfo.isEmpty())
                    defaultApproachAngle = getBestClusterApproachAngle(clusteredCollectAngleInfo);

                // find extra pre collect offset dist
                double[] minMaxDistance = getMinMaxClusterDistancesFromCenter(curBall, defaultApproachAngle, clusteredPositions);
                preCollectExtraOffset = Math.abs(minMaxDistance[0]);
                collectExtraOffset = Math.abs(minMaxDistance[1]);
//                System.out.println(i + ": " + preCollectExtraOffset + " | " + collectExtraOffset);

                clusteredPositions.clear();
                clusteredCollectAngleInfo.clear();
            }

            if (defaultApproachAngle == -100)
                defaultApproachAngle = Math.atan2(curBall.y - prevPathPoint.y, curBall.x - prevPathPoint.x);

            // categorize point
            PointType pointType = getPointType(curBall, cornerPosition);

            // build collect and pre collect poses based on the type of point
            double[] collectInfo = getCollectInfo(pointType, curBall, prevPathPoint, defaultApproachAngle);
            double preCollectOffset = collectInfo[0];
            double preCollectToCollectAngle = collectInfo[1];
            double collectAngle = collectInfo[2];
            double collectXOffset = collectInfo[3];
            double collectYOffset = collectInfo[4];

            Pose2d collectPose = getCollectPose(curBall, collectAngle, -collectExtraOffset);
            collectPose = new Pose2d(collectPose.position.plus(new Vector2d(collectXOffset, collectYOffset)), collectPose.heading);
            Pose2d preCollectPose = getPreCollectPose(collectPose, preCollectToCollectAngle, preCollectOffset + preCollectExtraOffset + collectExtraOffset);

            Pose2d wallSafeCollectPose = getWallSafePose(robotWidth, robotLength, collectPose, collectAngle);
            Pose2d wallSafePreCollectPose = getWallSafePose(robotWidth, robotLength, preCollectPose, collectAngle);
            if (!wallSafeCollectPose.equals(collectPose))
                wallSafePreCollectPose = getPreCollectPose(wallSafeCollectPose, preCollectToCollectAngle, preCollectOffset + preCollectExtraOffset);


            pathPoses.add(wallSafePreCollectPose);
            pathPoses.add(wallSafeCollectPose);
        }

        ballsUsed = path.toArray(new Vector2d[0]);
        return pathPoses;
    }
    private static Vector2d getMergedBall(Vector2d nextBall, ArrayList<Vector2d> clusteredPositions, double angleDiff) {
        Vector2d newBall;
        if (angleDiff > Math.toRadians(params.groupAsClusterAngle)) {
            double totalX = 0;
            double totalY = 0;
            for (Vector2d ballInCluster : clusteredPositions) {
                totalX += ballInCluster.x;
                totalY += ballInCluster.y;
            }
            newBall = new Vector2d(totalX, totalY).div(clusteredPositions.size());
        }
        else
            newBall = nextBall;
        return newBall;
    }
    private static double getBestClusterApproachAngle(ArrayList<double[]> clusteredCollectAngleInfo) {
        double minDiff = clusteredCollectAngleInfo.get(0)[1];
        double bestAngle = clusteredCollectAngleInfo.get(0)[0];
        for (int j = 1; j < clusteredCollectAngleInfo.size(); j++) {
            double[] info = clusteredCollectAngleInfo.get(j);
            if (info[1] < minDiff) {
                minDiff = info[1];
                bestAngle = info[0];
            }
        }
        return bestAngle;
    }
    private static double[] getMinMaxClusterDistancesFromCenter(Vector2d clusterCenter, double approachAngle, ArrayList<Vector2d> clusterPositions) {
        Vector2d dir = new Vector2d(Math.cos(approachAngle), Math.sin(approachAngle));
        double minT = Double.POSITIVE_INFINITY;
        double maxT = Double.NEGATIVE_INFINITY;

        for (Vector2d p : clusterPositions) {
            Vector2d fromCenter = p.minus(clusterCenter);
            // Scalar projection onto the line direction
            double t = fromCenter.dot(dir);
            minT = Math.min(minT, t);
            maxT = Math.max(maxT, t);
        }
        return new double[] { minT, maxT };
    }
    private static PointType getPointType(Vector2d curBall, Vector2d cornerPosition) {
        double distFromClassifierWall = Math.abs(cornerPosition.y - curBall.y);
        double distFromBackWall = Math.abs(cornerPosition.x - curBall.x);
        double distFromCorner = Math.hypot(distFromClassifierWall, distFromBackWall);

        PointType pointType = PointType.NORMAL;
        if (distFromCorner < params.cornerBallDistance)
            pointType = PointType.CORNER;
        else if (distFromClassifierWall < params.classifierWallDistance)
            pointType = PointType.CLASSIFIER_WALL;
        else if (distFromBackWall < params.backWallDistance)
            pointType = PointType.BACK_WALL;
        return pointType;
    }
    // pre collect offset, pre collect, collect
    private static double[] getCollectInfo(PointType pointType, Vector2d curBall, Vector2d prevPathPoint, double defaultApproachAngle) {
        double wallAngle = curBall.y > 0 ? Math.toRadians(90) : Math.toRadians(-90);
        double preCollectToCollectAngle;
        double collectAngle;
        double preCollectOffset = params.preCollectOffset;
        double collectXOffset = 0;
        double collectYOffset = 0;
        switch (pointType) {
            case CORNER:
                collectAngle = wallAngle;
                preCollectToCollectAngle = collectAngle;
                break;
            case CLASSIFIER_WALL:
                double angleDiff = defaultApproachAngle - wallAngle;
                double maxAngle = Math.toRadians(params.classifierWallMaxApproachAngleDiff);
                if (Math.abs(angleDiff) > maxAngle)
                    angleDiff = Math.signum(angleDiff) * maxAngle;
                collectAngle = wallAngle + angleDiff;
                preCollectToCollectAngle = collectAngle;
                break;
            case BACK_WALL:
                if (Math.abs(defaultApproachAngle) > Math.toRadians(params.backWallStrafeCollectMinApproachAngle)) {
                    double dy = curBall.y - prevPathPoint.y;
                    collectYOffset = Math.signum(dy) + params.backWallCollectYOffset;
                    preCollectOffset += collectYOffset;
                    collectAngle = Math.toRadians(params.backWallCollectAngle) * Math.signum(dy);
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
        return new double[] { preCollectOffset, preCollectToCollectAngle, collectAngle, collectXOffset, collectYOffset };
    }
    private static ArrayList<Pose2d> simplifyPathPoses(Pose2d startPose, ArrayList<Pose2d> pathPoses) {
        ArrayList<Pose2d> simplified = new ArrayList<>();
//        System.out.println("path poses");
//        for (Pose2d pose : pathPoses)
//            System.out.println(MathUtils.formatPose(pose));
        if (pathPoses.isEmpty())
            return simplified;
        if (pathPoses.size() == 1) {
            simplified.add(pathPoses.get(0));
            return simplified;
        }
        for (int i=0; i<pathPoses.size()-1; i++) {
            Pose2d prev = getCollectorPose(i > 0 ? pathPoses.get(i - 1) : startPose);
            Pose2d cur = getCollectorPose(pathPoses.get(i));
            Pose2d next = getCollectorPose(pathPoses.get(i + 1));

            Vector2d prevToCur = cur.position.minus(prev.position);
//            double distFromPrevToCur = Math.hypot(prevToCur.x, prevToCur.y);

            double angleDiff1 = MathUtils.findAngleRadDiff(cur.heading.toDouble(), prev.heading.toDouble());
            double angleDiff2 = MathUtils.findAngleRadDiff(next.heading.toDouble(), cur.heading.toDouble());
//            System.out.println(i + ": " + Math.toDegrees(cur.heading.toDouble()));
//            System.out.println(i + ": " + Math.toDegrees(angleDiff1) + " | " + Math.toDegrees(angleDiff2));
            double angleSimplification = Math.toRadians(params.maxHeadingChangeSimplification);
            if (Math.abs(angleDiff1) > angleSimplification || Math.abs(angleDiff2) > angleSimplification) {
                simplified.add(pathPoses.get(i));
                continue;
            }

            Vector2d prevToNext = next.position.minus(prev.position);

            Vector2d simplifiedDir = prevToNext.div(Math.hypot(prevToNext.x, prevToNext.y));
            double projectedCurDistFromPrev = prevToCur.dot(simplifiedDir);
            Vector2d projectedCur = prev.position.plus(simplifiedDir.times(projectedCurDistFromPrev));
            Vector2d curToProjectedCur = projectedCur.minus(cur.position);
            double projectedDist = Math.hypot(curToProjectedCur.x, curToProjectedCur.y);

            if (projectedDist > params.maxPathSimplificationEpsilon)
                simplified.add(pathPoses.get(i));
        }
        simplified.add(pathPoses.get(pathPoses.size() - 1));
        return simplified;
    }
    public static Pose2d getWallSafePose(double robotWidth, double robotLength, Pose2d pose, double desiredCollectAngle) {
        Vector2d position = pose.position;
        double halfW = robotWidth * 0.5;
        double halfL = robotLength * 0.5;

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
            Vector2d pos = GeometryUtils.robotVectorToFieldVector(positionsToCheck[j], pose.heading.toDouble()).plus(position);
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
    private static Pose2d getCollectorPose(Pose2d robotPose) {
        double dx = robotPose.heading.component1() * params.collectPoseOffsetDistance;
        double dy = robotPose.heading.component2() * params.collectPoseOffsetDistance;
        return new Pose2d(robotPose.position.x + dx, robotPose.position.y + dy, robotPose.heading.toDouble());
    }
    // assumes same heading as collect pose
    // the angle from pre collect to collect is not constrained to collect pose heading though
    private static Pose2d getPreCollectPose(Pose2d collectPose, double angleToCollectPose, double offsetDistance) {
        double dx = Math.cos(angleToCollectPose) * offsetDistance;
        double dy = Math.sin(angleToCollectPose) * offsetDistance;
        return new Pose2d(collectPose.position.x - dx, collectPose.position.y - dy, collectPose.heading.toDouble());
    }
}
