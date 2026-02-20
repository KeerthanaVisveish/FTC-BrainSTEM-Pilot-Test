package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection;

import com.acmerobotics.dashboard.config.Config;

import java.util.function.DoubleUnaryOperator;

@Config
public class AutoCollectParams {
    public double robotWidth = 13.5;
    public double robotLength = 16;
    public double maxPathRegenerationAttempts = 5;
    public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 8 extra inches

    public double collectPoseOffsetDistance = 7;
    public double preCollectOffset = 5;
    public double lastCollectPoseExtraDriveThrough = 4;

    public double clusterGroupingDist = 6, clusterStrafingDist = 10; // groupAsClusterDist MUST always be smaller than strafeThroughClusterDist
    public double clusterUseAverageMinAngle = 20;
    public double strafeCollectMaxAngleOffset = 30;
    public double strafeCollectDriveThroughDist = 3;
    public double strafeCollectMaxPerpendicularDistance = 2;
    public double maxCollectApproachDifference = 5;
    public double[] preCollectSimplification = new double[]{ 20, 75, 15, 0.3, 36 }; // low, range, center, steepness, ignore completely distance
    public DoubleUnaryOperator maxPreCollectHeadingDifference = x -> preCollectSimplification[1] / (1 + Math.pow(Math.E, -preCollectSimplification[3] * (x - preCollectSimplification[2]))) + preCollectSimplification[0];

    public double wallPoseBuffer = 0; // the pose can be outside the field walls by this much

    public double cornerBallDistance = 7.5;
    public double cornerCollectY = 73;
    public double wallStrafeCollectMinApproachAngle = 45;
    public double wallCollectAngle = 45;
    public double classifierWallDistance = 5;
    public double backWallDistance = 6;
}

