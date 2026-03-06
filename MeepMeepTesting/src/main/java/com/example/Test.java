package com.example;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.autoCollectPathGen.MathUtils;

import java.util.ArrayList;
import java.util.Arrays;

public class Test {
    public static void main(String[] args) {
        System.out.println(getCombinedBlobPositions());
    }
    private static class Blob {
        private Vector2d pos;
        public Blob(double x, double y) {
            this.pos = new Vector2d(x, y);
        }
        public Vector2d pos() {
            return pos;
        }
    }
    private static ArrayList<ArrayList<Blob>> previousSnapshots = new ArrayList<>(Arrays.asList(
            new ArrayList<>(Arrays.asList(new Blob(0, 0), new Blob(5, 0))),
            new ArrayList<>(Arrays.asList(new Blob(0.1, 0), new Blob(5.1, 0)))
    ));
    static double maxDistToCombineSnapshotBlobs = 2;
    public static ArrayList<Vector2d> getCombinedBlobPositions() {
        ArrayList<Vector2d> allPositions = new ArrayList<>();
        for (ArrayList<Blob> snapshotBlobs : previousSnapshots)
            for (Blob snapshotBlob : snapshotBlobs)
                allPositions.add(snapshotBlob.pos());

        ArrayList<Vector2d> combined = new ArrayList<>();
        ArrayList<Integer> indexesToSkip = new ArrayList<>();
        for (int i = 0; i < allPositions.size(); i++) {
            if (indexesToSkip.contains(i))
                continue;

            Vector2d pos = allPositions.get(i);
            boolean merge = false;
            for (int j = 0; j < allPositions.size(); j++) {
                if (i == j)
                    continue;

                Vector2d check = allPositions.get(j);
                if (MathUtils.vecDist(pos, check) <= maxDistToCombineSnapshotBlobs) {
                    combined.add(pos.plus(check).times(0.5));
                    indexesToSkip.add(j);
                    merge = true;
                    break;
                }
            }
            if (!merge) {
                combined.add(pos);
            }
        }
        return combined;
    }
}
