package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Vector2d;

public class ProblemBall {

    public enum Severity {
        // most severe
        OUT_OF_BOUNDS,
        BACK_TRACKING,
        FAILED_CLUSTER_MERGE, // failed to generate cluster approach AND failed to merge balls into one cluster
        FAILED_CLUSTER_APPROACH, // only failed to generate cluster approach, but successfully merged into one cluster
        OVERLAPPING_CLASSIFIER_WALL,
        CORNER,
        // least severe (most preferable)
    }
    public final Severity severity;
    public final Vector2d ballPosition;
    public ProblemBall(Severity severity, Vector2d ballPosition) {
        this.severity = severity;
        this.ballPosition = ballPosition;
    }
    @Override
    public String toString() {
        return severity + " " + MathUtils.formatVec2(ballPosition);
    }
}