package com.example.autoCollectPathGen;

public class Types {
    public enum Approach {
        NORMAL,
        CLUSTER_STRAFE,
        BACK_WALL_STRAFE,
        CLASSIFIER_WALL_STRAFE,
        CORNER_CONSTRAINED
    }

    public enum PoseType {
        COLLECT,
        EDGE_CASE_PRECOLLECT,
        PRECOLLECT
    }
}
