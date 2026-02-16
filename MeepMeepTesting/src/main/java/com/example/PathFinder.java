package com.example;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathFinder {
    public static Vector2d[] findShortestPath(Vector2d startPosition, Vector2d[] nodes, int maxNodesInPath) {
        // get all of the possible combinations of paths
        int combinationLength = Math.min(maxNodesInPath, nodes.length);
        List<List<Vector2d>> combinations = getCombinations(new ArrayList<>(Arrays.asList(nodes)), combinationLength);
        if (combinations.isEmpty() || combinations.get(0).isEmpty())
            return null;

        // find the shortest path for all of the possible combinations
        List<PathFinder.PathResult> results = new ArrayList<>(); // array of path results, parallel with combinations
        for (int i=0; i<combinations.size(); i++) {

            Vector2d[] combo = combinations.get(i).toArray(new Vector2d[0]);
            results.add(PathFinder.findShortestPath(startPosition, combo));
        }

        // find the shortest path out of all of the different combinations
        PathFinder.PathResult shortestResult = results.get(0);
        List<Vector2d> shortestCombo = combinations.get(0);
        for (int i=1; i<results.size(); i++) {
            if (results.get(i).distance() < shortestResult.distance()) {
                shortestResult = results.get(i);
                shortestCombo = combinations.get(i);
            }
        }

        // convert the shortest path into a useful format
        int[] pathIndexes = shortestResult.path();
        System.out.println(Arrays.toString(pathIndexes));
        Vector2d[] shortestPath = new Vector2d[pathIndexes.length];
        for (int i=0; i<shortestPath.length; i++) {
            int index = pathIndexes[i];
            shortestPath[i] = shortestCombo.get(index);
        }
        return shortestPath;
    }

    // finding all possible combinations of length L given a list of length N
    public static <T> List<List<T>> getCombinations(List<T> items, int k) {
        List<List<T>> result = new ArrayList<>();
        backtrack(items, k, 0, new ArrayList<>(), result);
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
    public static PathResult findShortestPath(Vector2d start, Vector2d[] nodes) {
        int n = nodes.length;
        int[] indices = new int[n];
        for (int i = 0; i < n; i++) {
            indices[i] = i;
        }

        PathResult best = new PathResult(Double.POSITIVE_INFINITY, null);

        permute(indices, 0, start, nodes, best);

        return best;
    }

    private static void permute(int[] arr, int startIdx, Vector2d start, Vector2d[] nodes, PathResult best) {
        if (startIdx == arr.length) {
            double dist = pathLengthFromStart(start, arr, nodes);
            if (dist < best.bestDistance) {
                best.bestDistance = dist;
                best.bestPath = arr.clone();
            }
            return;
        }

        for (int i = startIdx; i < arr.length; i++) {
            swap(arr, startIdx, i);
            permute(arr, startIdx + 1, start, nodes, best);
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

    private static void swap(int[] arr, int i, int j) {
        int tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
    }
    public static class PathResult {
        double bestDistance;
        int[] bestPath;

        PathResult(double d, int[] p) {
            bestDistance = d;
            bestPath = p;
        }
        public double distance() {
            return bestDistance;
        }
        public int[] path() {
            return bestPath;
        }
    }
}

