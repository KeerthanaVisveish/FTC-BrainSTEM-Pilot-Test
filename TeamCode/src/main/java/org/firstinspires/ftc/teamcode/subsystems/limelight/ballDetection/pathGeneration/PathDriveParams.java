package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

public class PathDriveParams {
    public double isStuckMaxLinearVel = 15;
    public double isStuckMaxHeadingVel = 10;
    public double isStuckConfirmationTime = 0.15;
    public double startCheckingIsStuckTime = 0.5;

    public double setHeadingTangentDistBetweenPoses = 44;
    public double correctHeadingBackFromTangentDist = 24;
    public double clusterStrafeParallelTol = 2.5;
    public double clusterStrafePerpendicularTol = 1.5;
    public double clusterStrafeHeadingTol = 3;
    public double[] classifierStrafeCustomForce = new double[] { 0, 0.2, };
    public double preCollectClusterStrafeMinLinearPower = 0.45;
    public double collectNormalMinLinearPower = 0.8, preCollectNormalMinLinearPower = 0.7;
    public double collectWallStrafeMinLinearPower = 0.7;
    public double collectCornerMinLinearPower = 0.55;
    public double cornerControlOffset = 32, cornerControlLerpStart = 28, cornerControlLerpEnd = 23, cornerControlMinOffsetFromPrev = 9;
    public double lanePreCollectMinLinearPower = 0.7, laneCollectMinLinearPower = 0.99;
    public double[] lanePreCollectTol = new double[] { 3, 4, 5 };
    public double[] classifierWallStrafeTol = new double[] { 3, 1, 3 };
    public double[] backWallStrafeTol = new double[] { 3, 2, 4 };
    public double collectCornerDistTol = 1, collectCornerHeadingTol = 5;
    public double preCollectCornerDistTol = 2, preCollectCornerHeadingTol = 5;
}
