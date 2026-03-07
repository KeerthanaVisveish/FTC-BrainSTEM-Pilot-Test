package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

public class PathDriveParams {
    public double isStuckMaxLinearVel = 10;
    public double isStuckMaxHeadingVel = 10;
    public double isStuckConfirmationTime = 0.3;
    public double startCheckingIsStuckTime = 0.5;

    public double setHeadingTangentDistBetweenPoses = 44;
    public double correctHeadingBackFromTangentDist = 24;
    public double clusterStrafeParallelTol = 2;
    public double clusterStrafePerpendicularTol = 1;
    public double clusterStrafeHeadingTol = 3;
    public double strafeCollectControlMaxOffset = 10, strafeCollectControlStartError = 20, strafeCollectControlEndError = 15;
    public double collectNormalMinLinearPower = 0.2, preCollectNormalMinLinearPower = 0.2;
    public double collectWallStrafeMinLinearPower = 0.6;
    public double collectCornerMinLinearPower = 0.4;
    public double cornerControlOffset = 32, cornerControlLerpStart = 28, cornerControlLerpEnd = 23, cornerControlMinOffsetFromPrev = 9;
    public double lanePreCollectMinLinearPower = 0.7, laneCollectMinLinearPower = 0.5;
    public double[] lanePreCollectTol = new double[] { 3, 4, 5 };
    public double[] classifierWallStrafeTol = new double[] { 3, 1, 3 };
    public double[] backWallStrafeTol = new double[] { 3, 1, 3 };
    public double collectCornerDistTol = 0.5, collectCornerHeadingTol = 5;
    public double preCollectCornerDistTol = 2, preCollectCornerHeadingTol = 5;
}
