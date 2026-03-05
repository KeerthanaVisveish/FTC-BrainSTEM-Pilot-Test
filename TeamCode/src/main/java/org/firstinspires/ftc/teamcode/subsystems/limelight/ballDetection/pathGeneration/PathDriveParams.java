package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration;

public class PathDriveParams {
    public double isStuckMaxLinearVel = 6;
    public double isStuckMaxHeadingVel = 10;
    public double isStuckConfirmationTime = 0.3;
    public double startCheckingIsStuckTime = 0.5;

    public double setHeadingTangentDistBetweenPoses = 44;
    public double correctHeadingBackFromTangentDist = 24;
    public double clusterStrafeParallelTol = 2;
    public double clusterStrafePerpendicularTol = 1;
    public double clusterStrafeHeadingTol = 3;
    public double strafeCollectControlMaxOffset = 10, strafeCollectControlStartError = 20, strafeCollectControlEndError = 15;
    public double collectNormalMinLinearPower = 0.2;
    public double collectWallStrafeMinLinearPower = 0.6;
    public double collectCornerMinLinearPower = 0.4;
    public double laneCollectMinLinearPower = 0.5;
    public double[] classifierWallStrafeTol = new double[] { 3, 1, 3 };
    public double[] backWallStrafeTol = new double[] { 3, 1, 3 };
    public double collectCornerDistTol = 0.5, collectCornerHeadingTol = 5;
}
