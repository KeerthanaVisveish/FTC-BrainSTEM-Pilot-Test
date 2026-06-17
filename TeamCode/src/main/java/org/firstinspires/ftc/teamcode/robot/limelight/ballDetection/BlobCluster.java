package org.firstinspires.ftc.teamcode.robot.limelight.ballDetection;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.ArrayList;
import java.util.HashSet;

class BlobCluster {
    ArrayList<Vector2d> points = new ArrayList<>();
    HashSet<Integer> snapshotIds = new HashSet<>();

    Vector2d getCenter() {
        return MathUtils.getAverage(points);
    }
}