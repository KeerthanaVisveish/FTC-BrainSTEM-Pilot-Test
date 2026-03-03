package com.example.autoCollectPathGen.pathGeneration;

import java.util.function.DoubleUnaryOperator;

public class PathGenerationParams {
        public double robotWidth = 13.5;
        public double robotLength = 16;
        public double shiftedLeftStartX = 40;

        public boolean allowLaneCollect = true;
        public double normalLaneWidth = 6;
        public double againstBackWallLaneWidth = 9;
        public double laneAgainstBackWallMaxDist = 8;
        public double laneIncrement = 1;
        public double angleLaneCollectDistFromBackWall = 3.5;
        public double ignoreOptimizedLaneWidthSortingWidth = 3;
        public double laneCollectControlYOffset = 15;
        public double laneCollectControlMinYOffsetFromRobot = 12;

        public double maxPathRegenerationAttempts = 5;
        public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 10 extra inches
        public double backTrackingMinDotProductBetweenPoints = -0.5; // for back tracking problem balls
        public double backTrackingMaxDistBetweenPoints = 2;


        public double collectPoseOffsetDistance = 7;
        public double preCollectOffset = 5;
        public double lastCollectPoseExtraDriveThrough = 4;

        public double clusterMergeDist = 5.5;
        public double clusterStrafingDist = 24;
        public double strafeCollectMaxAngleOffset = 30;
        public double strafeCollectDriveThroughDist = 3;
        public double strafeCollectMaxPerpendicularDistance = 2;
        public double maxCollectApproachDifference = 10;
        public double[] collectSimplificationConstants = new double[]{ 20, 75, 15, 0.3, 36 }; // low, range, center, steepness, ignore completely distance
        public DoubleUnaryOperator maxCollectHeadingDifference = x -> collectSimplificationConstants[1] / (1 + Math.pow(Math.E, -collectSimplificationConstants[3] * (x - collectSimplificationConstants[2]))) + collectSimplificationConstants[0];
        public double completelyIgnoreCollectPoseApproachAngleDiff = 3;
        public double completelyIgnoreCollectPoseHeadingDiff = 5;
        public double wallPoseBuffer = 0; // the pose can be outside the field walls by this much

        public double cornerBallDistance = 7;
        public double cornerCollectY = 73;
        public double cornerCollectAngle = 10;
        public double lenientCornerCollectThreshold = 12;

        public double wallStrafeCollectMinApproachAngle = 35;
        public double wallCollectAngle = 45;
        public double classifierWallDistance = 7;
        public double rejectEdgeCaseBallsHorizontalDist = 28;
        public double lenientClassifierStrafeAngleDiff = 70;
        public double distToNextPointForceStrictClassifierStrafe = 15;
        public double backWallDistance = 6;
}