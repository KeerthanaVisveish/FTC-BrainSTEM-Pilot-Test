package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.math.PathFinder;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class PathGeneration {
    public static class AutoCollectParams {
        public double cornerBallDistance = 10;
        public double minLinearPower = 0.1;
        public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 10 extra inches
        public double laneIncrement = 1, laneWidth = 6;
        public double collectPoseOffsetDistance = 8;
        public double preCollectPoseOffsetDistance = 4;
    }
    public static AutoCollectParams params = new AutoCollectParams();

    public static Vector2d[] ballsUsed = null; // used to draw balls on FTC dashboard

    public static DrivePath getAutoCollectPath(MecanumDrive drive, Vector2d[] points, int simplePathingThreshold, int maxPointsInPath, boolean shouldUpdatePose) {
        if (points.length == 0)
            return null;

        Vector2d robotPos = drive.localizer.getPose().position;

        // simple sliding window approach
        // find lane with densest amount of balls
        // drive in straight line through the lane
        Lane[] bestLanes = getDensestLanes(points, params.laneIncrement, params.laneWidth);
        if (bestLanes == null)
            return null;
        if (bestLanes[0].numBalls() >= simplePathingThreshold) {
            // find closest lane if there are multiple lanes
            Lane bestLane = getBestLane(robotPos, bestLanes);
            return generateSimpleDrivePath(drive, bestLane, shouldUpdatePose);
        }

        // more complex pathfinding approach
        // when there are not enough balls for sliding window, actually find ideal combo of balls and drive path
        Vector2d[] path = getShortestPath(robotPos, points, maxPointsInPath);
        return generateComplexDrivePath(robotPos, drive, path, shouldUpdatePose);
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
        @NonNull
        @Override
        public String toString() {
            return "avgx: " + avgX + " | num balls: " + numBalls();
        }
    }
    public static Vector2d[] getShortestPath(Vector2d startPosition, Vector2d[] nodes, int maxBlobsInPath) {
        return PathFinder.findShortestPath(startPosition, nodes, maxBlobsInPath, params.changeInAngleDegCost);
    }
    private static DrivePath generateSimpleDrivePath(MecanumDrive drive, Lane lane, boolean shouldUpdatePose) {
        double angle = BrainSTEMRobot.alliance == Alliance.RED ? Math.toRadians(90) : -Math.toRadians(90);
        Vector2d ball1 = new Vector2d(lane.avgX, lane.minAbsY());
        Vector2d ball2 = new Vector2d(lane.avgX, lane.maxAbsY());
        ballsUsed = new Vector2d[] { ball1, ball2 };

        Pose2d collect1 = getCollectPose(ball1, angle);
        Pose2d collect2 = getCollectPose(ball2, angle);
        Pose2d preCollect1 = getPreCollectPose(collect1);
        DrivePath drivePath = new DrivePath(drive, new Waypoint(preCollect1), new Waypoint(collect1), new Waypoint(collect2));
        drivePath.setShouldUpdatePose(shouldUpdatePose);
        return drivePath;
    }
    public static DrivePath generateComplexDrivePath(Vector2d robotPos, MecanumDrive drive, Vector2d[] path, boolean shouldUpdatePose) {
        ballsUsed = path;

        DrivePath drivePath = new DrivePath(drive);
        drivePath.setShouldUpdatePose(shouldUpdatePose);
        Vector2d cornerPosition = new Vector2d(72, BrainSTEMRobot.alliance == Alliance.RED ? 72 : -72);

        for (int i=0; i<path.length; i++) {
            Vector2d point = path[i];
            Vector2d prev = i > 0 ? drivePath.getWaypoint(i - 1).pose.position : robotPos;

            // constrain the robot to enter parallel to the field wall if the ball is in the corner
            double distFromCorner = Math.hypot(cornerPosition.x - point.x, cornerPosition.y - point.y);

            if (distFromCorner < params.cornerBallDistance) {
                // figure out which way to approach the ball
                Vector2d p1 = new Vector2d(point.x - 10, point.y);
                Vector2d p2 = new Vector2d(point.x, point.y + (BrainSTEMRobot.alliance == Alliance.RED ? -10 : 10));
                double d1 = Math.hypot(point.x - p1.x, point.y - p1.y) + Math.hypot(p1.x - prev.x, p1.y - prev.y);
                double d2 = Math.hypot(point.x - p2.x, point.y - p2.y) + Math.hypot(p2.x - prev.x, p2.y - prev.y);
                Vector2d setupPoint = d1 < d2 ? p1 : p2;
                double angle = Math.atan2(setupPoint.y - prev.y, setupPoint.x - prev.x);
                Waypoint w = new Waypoint(new Pose2d(setupPoint.x, setupPoint.y, angle))
                        .prioritizeHeadingInBeginning()
                        .setMinLinearPower(params.minLinearPower);
                drivePath.addWaypoint(w);
                prev = setupPoint;
            }

            double angle = Math.atan2(point.y - prev.y, point.x - prev.x);
            Waypoint waypoint = new Waypoint(getCollectPose(point, angle))
                    .prioritizeHeadingInBeginning()
                    .setMinLinearPower(params.minLinearPower);
            drivePath.addWaypoint(waypoint);
        }
        return drivePath;
    }
    private static Pose2d getCollectPose(Vector2d ballPosition, double angle) {
        double dx = Math.cos(angle) * params.collectPoseOffsetDistance;
        double dy = Math.sin(angle) * params.collectPoseOffsetDistance;
        return new Pose2d(ballPosition.x - dx, ballPosition.y - dy, angle);
    }
    private static Pose2d getPreCollectPose(Pose2d collectPose) {
        double angle = collectPose.heading.toDouble();
        double dx = Math.cos(angle) * params.preCollectPoseOffsetDistance;
        double dy = Math.sin(angle) * params.preCollectPoseOffsetDistance;
        return new Pose2d(collectPose.position.x - dx, collectPose.position.y - dy, angle);
    }
}
