package org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration;

import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

public class PathPose {
    public final Waypoint waypoint;
    public Types.PoseType poseType;
    public final Ball ball;
    public final Types.Approach approachType;
    public PathPose(Waypoint waypoint, Types.PoseType poseType, Ball ball, Types.Approach approachType) {
        this.waypoint = waypoint;
        this.poseType = poseType;
        this.ball = ball == null ? Ball.NULL : ball;
        this.approachType = approachType;
    }
    @Override
    public String toString() {
        return MathUtils.formatPose1(waypoint.pose);
    }
}