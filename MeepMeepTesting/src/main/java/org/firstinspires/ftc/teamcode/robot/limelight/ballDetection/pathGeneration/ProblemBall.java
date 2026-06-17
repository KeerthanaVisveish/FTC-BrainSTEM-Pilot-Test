package org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

public class ProblemBall extends Ball {

    public enum Severity {
        // most severe
        OUT_OF_BOUNDS,
        BACK_TRACKING,
        FAILED_CLUSTER_MERGE, // failed to generate cluster approach AND failed to merge balls into one cluster
        FAILED_CLUSTER_APPROACH, // only failed to generate cluster approach, but successfully merged into one cluster
        OVERLAPPING_CLASSIFIER_WALL,
        UNIDEAL_APPROACH,
        CORNER,
        // least severe (most preferable)
    }
    public final Severity severity;
    public final boolean shouldRemove;
    public ProblemBall(Severity severity, Vector2d pos) {
        super(pos);
        this.severity = severity;
        switch (severity) {
            case OUT_OF_BOUNDS:
            case BACK_TRACKING:
            case FAILED_CLUSTER_MERGE:
            case FAILED_CLUSTER_APPROACH:
            case OVERLAPPING_CLASSIFIER_WALL:
                shouldRemove = true; break;

            case UNIDEAL_APPROACH:
            case CORNER:
            default:
                shouldRemove = false; break;
        }
    }
    @Override
    public String toString() {
        return severity + " " + MathUtils.formatVec2(pos);
    }
}