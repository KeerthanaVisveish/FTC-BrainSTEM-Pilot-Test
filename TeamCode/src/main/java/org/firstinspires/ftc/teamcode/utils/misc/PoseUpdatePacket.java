package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;

public class PoseUpdatePacket {
    public static ArrayList<PoseUpdatePacket> poseUpdatePackets = new ArrayList<>();
    public enum UpdateType {
        CORNER,
        LIMELIGHT,
        ODOMETRY
    }
    public final UpdateType updateType;
    public final Pose2d pose;
    public PoseUpdatePacket(UpdateType updateType, Pose2d pose) {
        this.updateType = updateType;
        this.pose = pose;
    }
}
