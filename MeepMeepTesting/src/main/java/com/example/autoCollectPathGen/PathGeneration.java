package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static class AutoCollectParams {
        public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 10 extra inches
        public double laneIncrement = 1, laneWidth = 6;

        public double collectPoseOffsetDistance = 8;
        public double normalPreCollectOffset = 4;

        public double cornerBallDistance = 10;
        public double cornerBallPreCollectOffset = 8;

        public double classifierWallDistance = 10;
        public double classifierWallBackupDistance = 8;
        public double classifierWallMaxSidewaysDist = 5;

        public double backWallDistance = 10;
        public double backWallCollectAngle = 45;
        public double backWallPreCollectOffset = 8;

    }
    public static AutoCollectParams params = new AutoCollectParams();

    public static Vector2d[] ballsUsed = null; // used to draw balls on FTC dashboard

    public static ArrayList<Pose2d> getAutoCollectPathPoses(boolean onRedAlliance, Pose2d startPose, Vector2d[] points, int simplePathingThreshold, int maxPointsInPath) {
        if (points.length == 0)
            return null;

        Vector2d robotPos = startPose.position;

        // simple sliding window approach
        // find lane with densest amount of balls
        // drive in straight line through the lane
        Lane[] bestLanes = getDensestLanes(points, params.laneIncrement, params.laneWidth);
        if (bestLanes == null)
            return null;
        if (bestLanes[0].numBalls() >= simplePathingThreshold) {
            // find closest lane if there are multiple lanes
            Lane bestLane = getBestLane(robotPos, bestLanes);
            return generateSimplePathPoses(onRedAlliance, bestLane);
        }

        // more complex pathfinding approach
        // when there are not enough balls for sliding window, actually find ideal combo of balls and drive path
        Vector2d[] path = getShortestPath(robotPos, points, maxPointsInPath);
        return generateComplexPathPoses(onRedAlliance, robotPos, path);
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
        if (numLanes < 0)
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
    public static Vector2d[] getShortestPath(Vector2d startPosition, Vector2d[] nodes, int maxBlobsInPath) {
        return PathFinder.findShortestPath(startPosition, nodes, maxBlobsInPath, params.changeInAngleDegCost);
    }
    private static ArrayList<Pose2d> generateSimplePathPoses(boolean onRedAlliance, Lane lane) {
        double angle = onRedAlliance ? Math.toRadians(90) : -Math.toRadians(90);
        Vector2d ball1 = new Vector2d(lane.avgX, lane.minAbsY());
        Vector2d ball2 = new Vector2d(lane.avgX, lane.maxAbsY());
        ballsUsed = new Vector2d[] { ball1, ball2 };

        Pose2d collect1 = getCollectPose(ball1, angle);
        Pose2d collect2 = getCollectPose(ball2, angle);
        Pose2d preCollect1 = getPreCollectPose(collect1, angle, params.normalPreCollectOffset);
        return new ArrayList<>(Arrays.asList(preCollect1, collect1, collect2));
    }
    private enum PointType {
        NORMAL,
        CORNER,
        CLASSIFIER_WALL,
        BACK_WALL
    }
    public static ArrayList<Pose2d> generateComplexPathPoses(boolean onRedAlliance, Vector2d robotPos, Vector2d[] path) {
        ballsUsed = path;

        ArrayList<Pose2d> pathPoses = new ArrayList<>();
        Vector2d cornerPosition = new Vector2d(72, onRedAlliance ? 72 : -72);

        for (int i=0; i<path.length; i++) {
            Vector2d point = path[i];
            Vector2d prev = i > 0 ? pathPoses.get(pathPoses.size() - 1).position : robotPos;

            // categorize point
            double distFromClassifierWall = Math.abs(cornerPosition.y - point.y);
            double distFromBackWall = Math.abs(cornerPosition.x - point.x);
            double distFromCorner = Math.hypot(distFromClassifierWall, distFromBackWall);

            PointType pointType = PointType.NORMAL;
            if (distFromCorner < params.cornerBallDistance)
                pointType = PointType.CORNER;
            else if (distFromClassifierWall < params.classifierWallDistance)
                pointType = PointType.CLASSIFIER_WALL;
            else if (distFromBackWall < params.backWallDistance)
                pointType = PointType.BACK_WALL;

            // build collect and pre collect poses based on the type of point
            Pose2d preCollectPose;
            Pose2d collectPose;
            double angle;
            switch (pointType) {
                case CORNER:
                    angle = point.y > 0 ? Math.toRadians(90) : Math.toRadians(-90);
                    collectPose = getCollectPose(point, angle);
                    preCollectPose = getPreCollectPose(collectPose, angle, params.cornerBallPreCollectOffset);
                    break;
                case CLASSIFIER_WALL:
                    double dx = point.x - prev.x;
                    if (Math.abs(dx) > params.classifierWallMaxSidewaysDist)
                        dx = Math.signum(dx) * params.classifierWallMaxSidewaysDist;
                    else
                        dx *= 0.5;
                    double dy = params.classifierWallBackupDistance;
                    angle = Math.atan2(dy, dx);
                    collectPose = getCollectPose(point, angle);
                    preCollectPose = new Pose2d(collectPose.position.x - dx, collectPose.position.y - dy, angle);
                    break;
                case BACK_WALL:
                    angle = Math.toRadians(params.backWallCollectAngle) * (point.y > 0 ? 1 : -1);
                    double preCollectToCollectAngle = Math.toRadians(90) * (point.y > 0 ? 1 : -1);
                    collectPose = getCollectPose(point, angle);
                    preCollectPose = getPreCollectPose(collectPose, preCollectToCollectAngle, params.backWallPreCollectOffset);
                    break;
                case NORMAL:
                default:
                    angle = Math.atan2(point.y - prev.y, point.x - prev.x);
                    collectPose = getCollectPose(point, angle);
                    preCollectPose = getPreCollectPose(collectPose, angle, params.normalPreCollectOffset);
                    break;
            }

            pathPoses.add(preCollectPose);
            pathPoses.add(collectPose);
        }
        return pathPoses;
    }
    private static Pose2d getCollectPose(Vector2d ballPosition, double angle) {
        double dx = Math.cos(angle) * params.collectPoseOffsetDistance;
        double dy = Math.sin(angle) * params.collectPoseOffsetDistance;
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
