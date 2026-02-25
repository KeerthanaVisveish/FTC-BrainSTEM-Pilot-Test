package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

public class Types {
    public enum Approach {
        NORMAL,
        CLUSTER_STRAFE,
        BACK_WALL_STRAFE,
        CLASSIFIER_STRAFE,
        LENIENT_CLASSIFIER_STRAFE,
        CORNER_CONSTRAINED,
        CORNER_LENIENT
    }

    public enum PoseType {
        COLLECT,
        EDGE_CASE_PRECOLLECT,
        LENIENT_CORNER_PRECOLLECT,
        PRECOLLECT
    }
}
