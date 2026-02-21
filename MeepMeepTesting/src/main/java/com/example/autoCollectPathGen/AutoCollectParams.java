package com.example.autoCollectPathGen;

import java.util.function.DoubleUnaryOperator;

public class AutoCollectParams {
        public double robotWidth = 13.5;
        public double robotLength = 16;
        public double maxPathRegenerationAttempts = 5;
        public double changeInAngleDegCost = 10 / 90.; // 90 degrees -> 10 extra inches
        public double desiredAngle = 45;
        public double changeInDesiredAngleDegCost = 8. / 90.; // 90 degrees -> 8 extra inches

        public double collectPoseOffsetDistance = 7;
        public double preCollectOffset = 5;
        public double lastCollectPoseExtraDriveThrough = 4;

        public double clusterStrafingDist = 10;
        public double strafeCollectMaxAngleOffset = 30;
        public double strafeCollectDriveThroughDist = 3;
        public double strafeCollectMaxPerpendicularDistance = 2;
        public double maxCollectApproachDifference = 5;
        public double[] preCollectSimplification = new double[]{ 20, 75, 15, 0.3, 36 }; // low, range, center, steepness, ignore completely distance
        public DoubleUnaryOperator maxPreCollectHeadingDifference = x -> preCollectSimplification[1] / (1 + Math.pow(Math.E, -preCollectSimplification[3] * (x - preCollectSimplification[2]))) + preCollectSimplification[0];

        public double wallPoseBuffer = 0; // the pose can be outside the field walls by this much

        public double cornerBallDistance = 5;
        public double cornerCollectY = 73;
        public double cornerCollectAngle = 10;

        public double wallStrafeCollectMinApproachAngle = 45;
        public double wallCollectAngle = 45;
        public double classifierWallDistance = 7;
        public double distToNextPointForceClassifierStrafe = 18;
        public double backWallDistance = 6;
    }