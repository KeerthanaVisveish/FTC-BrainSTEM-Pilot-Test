package com.example;

import com.acmerobotics.roadrunner.Vector2d;
import com.example.autoCollectPathGen.MathUtils;

import java.util.ArrayList;
import java.util.Collections;

public class Test {
    public static void main(String[] args) {
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
    private static ArrayList<ArrayList<Blob>> previousSnapshots = new ArrayList<>(
            new ArrayList<>()
    );
    static double maxDistToCombineSnapshotBlobs = 2;
    public ArrayList<Vector2d> getCombinedBlobPositions() {
        ArrayList<ArrayList<Vector2d>> combinedRaw = new ArrayList<>();
        for (ArrayList<Blob> snapshotBlobs : previousSnapshots) {
            for (Blob snapshotBlob : snapshotBlobs) {
                double minDist = -1;
                ArrayList<Vector2d> closestList = null;
                for (ArrayList<Vector2d> existingList : combinedRaw) {
                    Vector2d currentAverage = MathUtils.getAverage(existingList);
                    double dist = MathUtils.vecMag(currentAverage.minus(snapshotBlob.pos()));
                    if (dist <= minDist) {
                        minDist = dist;
                        closestList = existingList;
                    }
                }
                if (minDist != -1 && minDist < maxDistToCombineSnapshotBlobs) {
                    closestList.add(snapshotBlob.pos());
                }
                else
                    combinedRaw.add(new ArrayList<>(Collections.singletonList(snapshotBlob.pos())));
            }
        }
        ArrayList<Vector2d> combined = new ArrayList<>();
        for (ArrayList<Vector2d> readings : combinedRaw)
            combined.add(MathUtils.getAverage(readings));
        return combined;
    }
}
