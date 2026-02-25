package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.ArrayList;
import java.util.List;

public class PathFinder {
    // extra distance of path = total change in angle * changeInAngleDegCost
    public static ArrayList<Vector2d> findShortestPath(Pose2d startPose, ArrayList<Vector2d> nodes, int maxNodesInPath, double extraDistPerChangeInAngleDeg, double desiredAngleRad, double desiredAngleDifferenceCost) {
        // get all of the possible combinations of paths
        int combinationLength = Math.min(maxNodesInPath, nodes.size());
        List<List<Vector2d>> combinations = getCombinations(nodes, combinationLength);
        if (combinations.isEmpty() || combinations.get(0).isEmpty())
            return null;

        // find the shortest path for all of the possible combinations
        List<PathResult> results = new ArrayList<>(); // array of path results, parallel with combinations
        for (int i=0; i<combinations.size(); i++) {

            Vector2d[] combo = combinations.get(i).toArray(new Vector2d[0]);
            results.add(PathFinder.findShortestPath(startPose, combo, extraDistPerChangeInAngleDeg, desiredAngleRad, desiredAngleDifferenceCost));
        }

        // find the shortest path out of all of the different combinations
        PathResult shortestResult = results.get(0);
        List<Vector2d> shortestCombo = combinations.get(0);
        for (int i=1; i<results.size(); i++) {
            if (results.get(i).distance() < shortestResult.distance()) {
                shortestResult = results.get(i);
                shortestCombo = combinations.get(i);
            }
        }

        // convert the shortest path into a useful format
        int[] pathIndexes = shortestResult.path();
        ArrayList<Vector2d> shortestPath = new ArrayList<>();
        for (int i=0; i<pathIndexes.length; i++) {
            int index = pathIndexes[i];
            shortestPath.add(shortestCombo.get(index));
        }
        return shortestPath;
    }

    // finding all possible combinations of length L given a list of items.size()
    public static <T> List<List<T>> getCombinations(List<T> items, int L) {
        List<List<T>> result = new ArrayList<>();
        backtrack(items, L, 0, new ArrayList<>(), result);
        return result;
    }

    private static <T> void backtrack(
            List<T> items,
            int k,
            int start,
            List<T> current,
            List<List<T>> result
    ) {
        if (current.size() == k) {
            result.add(new ArrayList<>(current));
            return;
        }

        for (int i = start; i < items.size(); i++) {
            current.add(items.get(i));
            backtrack(items, k, i + 1, current, result);
            current.remove(current.size() - 1);
        }
    }

    // finding the shortest path given a list of nodes of length N
    public static PathResult findShortestPath(Pose2d start, Vector2d[] nodes, double extraDistPerChangeInAngleDeg, double desiredAngle, double desiredAngleDiffCost) {
        int n = nodes.length;
        int[] indices = new int[n];
        for (int i = 0; i < n; i++) {
            indices[i] = i;
        }

        PathResult best = new PathResult(Double.POSITIVE_INFINITY, 0, null);

        permute(indices, 0, start, nodes, best, extraDistPerChangeInAngleDeg, desiredAngle, desiredAngleDiffCost);

        return best;
    }

    private static void permute(int[] arr, int startIdx, Pose2d start, Vector2d[] nodes, PathResult best, double extraDistPerChangeInAngleDeg, double desiredAngle, double desiredAngleDiffCost) {
        if (startIdx == arr.length) {
            double dist = pathLengthFromStart(start.position, arr, nodes);
            double[] angles = getAngles(start, arr, nodes);
            double totalAngleChange = 0;
            double totalAngleDiffFromDesiredAngle = 0;
            for (int i=0; i<angles.length; i++) {
                double prevAngle = i == 0 ? start.heading.toDouble() : angles[i - 1];
                double angle = angles[i];
                double angleDiff = MathUtils.angleNormDeltaRad(angle - prevAngle);
                totalAngleChange += Math.abs(angleDiff);
                double desiredAngleDiff = MathUtils.angleNormDeltaRad(desiredAngle - angle);
                totalAngleDiffFromDesiredAngle += Math.abs(desiredAngleDiff);
            }
            double changeInAngleCost = Math.toDegrees(totalAngleChange) * extraDistPerChangeInAngleDeg;
            double diffFromDesiredAngleCost = Math.toDegrees(totalAngleDiffFromDesiredAngle) * desiredAngleDiffCost;
            dist += changeInAngleCost;
            dist += diffFromDesiredAngleCost;
            if (dist < best.distance) {
                best.distance = dist;
                best.path = arr.clone();
            }
            return;
        }

        for (int i = startIdx; i < arr.length; i++) {
            swap(arr, startIdx, i);
            permute(arr, startIdx + 1, start, nodes, best, extraDistPerChangeInAngleDeg, desiredAngle, desiredAngleDiffCost);
            swap(arr, startIdx, i);
        }
    }

    private static double pathLengthFromStart(Vector2d start, int[] order, Vector2d[] nodes) {
        double sum = 0.0;

        // Start → first node
        Vector2d first = nodes[order[0]];
        sum += Math.hypot(first.x - start.x, first.y - start.y);

        // Between nodes
        for (int i = 0; i < order.length - 1; i++) {
            Vector2d n1 = nodes[order[i]];
            Vector2d n2 = nodes[order[i + 1]];
            sum += Math.hypot(n2.x - n1.x, n2.y - n1.y);
        }

        return sum;
    }
    public static double[] getAngles(Pose2d start, int[] order, Vector2d[] nodes) {
        double[] angles = new double[order.length];
        for (int i=0; i<order.length; i++) {
            Vector2d n1 = i > 0 ? nodes[order[i - 1]] : start.position;
            Vector2d n2 = nodes[order[i]];
            double angle = MathUtils.vecAngle(n2.minus(n1));
            angles[i] = angle;
        }
        return angles;
    }

    private static void swap(int[] arr, int i, int j) {
        int tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
    }
    public static class PathResult {
        double distance;
        double totalAngleChange;
        int[] path;

        PathResult(double distance, double totalAngleChange, int[] path) {
            this.distance = distance;
            this.totalAngleChange = totalAngleChange;
            this.path = path;
        }
        public double distance() {
            return distance;
        }
        public double totalAngleChange() {
            return totalAngleChange;
        }
        public int[] path() {
            return path;
        }
    }
}

