package com.example;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class Test {
    public static void main(String[] args) {
        Lane[] bestLanes = getDensestLanes(
                new Vector2d[] {
                        new Vector2d(0, 0),
                        new Vector2d(1, 0),
                        new Vector2d(8, 0),
                        new Vector2d(9, 0),
                },
                1, 5
        );
        System.out.println("all best lanes");
        if (bestLanes != null)
            for (Lane lane : bestLanes)
                System.out.println(lane);
        System.out.println("final best lane");
        if (bestLanes != null) {
            Lane bestLane = getBestLane(new Vector2d(10, 0), bestLanes);
            System.out.println(bestLane);
        }
        else
            System.out.println("null");
    }
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
        @Override
        public String toString() {
            return "avgx: " + avgX + " | num balls: " + numBalls();
        }
    }
}
