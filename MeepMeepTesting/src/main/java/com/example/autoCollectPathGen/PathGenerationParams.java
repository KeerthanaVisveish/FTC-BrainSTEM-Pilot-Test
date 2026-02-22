package com.example.autoCollectPathGen;

import java.util.function.DoubleUnaryOperator;

public class PathGenerationParams {
        public double robotWidth = 13.5;
        public double robotLength = 16;
        public double shiftedLeftStartX = 40, useShiftedLeftStartXBallXThreshold = 50;

        public double normalLaneWidth = 5;
        public double againstBackWallLaneWidth = 9;
        public double laneAgainstBackWallMaxDist = 8;
        public double laneIncrement = 1;

        public double maxPathRegenerationAttempts = 5;
        public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 10 extra inches

        public double collectPoseOffsetDistance = 7;
        public double preCollectOffset = 5;
        public double lastCollectPoseExtraDriveThrough = 4;

        public double failedClusterGroupAsOneDist = 5.5;
        public double clusterStrafingDist = 12;
        public double strafeCollectMaxAngleOffset = 30;
        public double strafeCollectDriveThroughDist = 3;
        public double strafeCollectMaxPerpendicularDistance = 2;
        public double maxCollectApproachDifference = 5;
        public double[] collectSimplificationConstants = new double[]{ 20, 75, 15, 0.3, 36 }; // low, range, center, steepness, ignore completely distance
        public DoubleUnaryOperator maxCollectHeadingDifference = x -> collectSimplificationConstants[1] / (1 + Math.pow(Math.E, -collectSimplificationConstants[3] * (x - collectSimplificationConstants[2]))) + collectSimplificationConstants[0];

        public double wallPoseBuffer = 0; // the pose can be outside the field walls by this much

        public double cornerBallDistance = 7;
        public double cornerCollectY = 73;
        public double cornerCollectAngle = 10;

        public double wallStrafeCollectMinApproachAngle = 45;
        public double wallCollectAngle = 45;
        public double classifierWallDistance = 7;
        public double rejectTwoClassifierBallsDist = 24;
        public double rejectCornerTooCloseToClassifierBallDist = 14;
        public double distToNextPointForceClassifierStrafe = 18;
        public double backWallDistance = 6;
    }