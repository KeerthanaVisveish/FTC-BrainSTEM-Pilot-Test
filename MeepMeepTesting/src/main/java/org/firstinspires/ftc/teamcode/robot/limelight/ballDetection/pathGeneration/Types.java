package org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration;

public class Types {
    public enum Approach {
        NORMAL,
        CLUSTER_STRAFE,
        BACK_WALL_STRAFE,
        CLASSIFIER_STRAFE,
        LENIENT_CLASSIFIER_STRAFE,
        CORNER
    }

    public enum PoseType {
        COLLECT,
        EDGE_CASE_PRECOLLECT,
        PRECOLLECT
    }
}
