package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.roadrunner.Vector2d;

public class PathFinder {

    public static PathResult findShortestPath(Vector2d[] nodes) {
        int n = nodes.length;
        int[] indices = new int[n];
        for (int i = 0; i < n; i++) {
            indices[i] = i;
        }

        PathResult best = new PathResult(Double.POSITIVE_INFINITY, null);

        permute(indices, 0, nodes, best);

        return best;
    }

    // ================= helpers =================

    private static void permute(int[] arr, int start, Vector2d[] nodes, PathResult best) {
        if (start == arr.length) {
            double dist = pathLength(arr, nodes);
            if (dist < best.bestDistance) {
                best.bestDistance = dist;
                best.bestPath = arr.clone();
            }
            return;
        }

        for (int i = start; i < arr.length; i++) {
            swap(arr, start, i);
            permute(arr, start + 1, nodes, best);
            swap(arr, start, i);
        }
    }

    private static double pathLength(int[] order, Vector2d[] nodes) {
        double sum = 0.0;
        for (int i = 0; i < order.length - 1; i++) {
            sum += distance(nodes[order[i]], nodes[order[i + 1]]);
        }
        return sum;
    }

    private static double distance(Vector2d a, Vector2d b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        return Math.hypot(dx, dy);
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

