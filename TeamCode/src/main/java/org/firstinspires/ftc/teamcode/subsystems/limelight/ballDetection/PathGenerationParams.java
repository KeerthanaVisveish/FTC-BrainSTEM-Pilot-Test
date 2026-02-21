package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection;

import com.acmerobotics.dashboard.config.Config;

import java.util.function.DoubleUnaryOperator;

/*
problems:
tolerances for drive paths are bad
what happens if a back wall ball is right next to a corner ball
corner collect angle cannot be wall angle - needs slight offset b/c collector isn't wide enough
*/
@Config
public class PathGenerationParams {
    public static double robotWidth = 13.5;
    public static double robotLength = 16;
    public static double maxPathRegenerationAttempts = 5;
    public static double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 8 extra inches

    public static double collectPoseOffsetDistance = 7;
    public static double preCollectOffset = 5;
    public static double lastCollectPoseExtraDriveThrough = 4;

    public static double clusterGroupingDist = 6, clusterStrafingDist = 10; // groupAsClusterDist MUST always be smaller than strafeThroughClusterDist
    public static double clusterUseAverageMinAngle = 20; // not used right now
    public static double strafeCollectMaxAngleOffset = 15;
    public static double strafeCollectDriveThroughDist = 3;
    public static double strafeCollectMaxPerpendicularDistance = 2;
    public static double maxCollectApproachDifference = 5;
    public static double[] preCollectSimplification = new double[]{ 20, 75, 15, 0.3, 36 }; // low, range, center, steepness, ignore completely distance
    public DoubleUnaryOperator maxPreCollectHeadingDifference = x -> preCollectSimplification[1] / (1 + Math.pow(Math.E, -preCollectSimplification[3] * (x - preCollectSimplification[2]))) + preCollectSimplification[0];

    public static double wallPoseBuffer = 0; // the pose can be outside the field walls by this much

    public static double cornerBallDistance = 7.5;
    public static double cornerCollectY = 73;
    public static double wallStrafeCollectMinApproachAngle = 45;
    public static double wallCollectAngle = 45;
    public static double classifierWallDistance = 5;
    public static double backWallDistance = 6;
}

