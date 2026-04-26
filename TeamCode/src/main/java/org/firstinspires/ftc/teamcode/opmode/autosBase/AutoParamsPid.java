package org.firstinspires.ftc.teamcode.opmode.autosBase;

public class AutoParamsPid {
    public static class Collect {
        public double hitGateVelThreshold = 15;
        public double hitBallVelThreshold = 15;
        public double collectDrivePower = 0.85, firstCollectDrivePower = .6, secondCollectDrivePower = .5, thirdCollectDrivePower = .5;
        public double gateOpenDrive1MinPower = .8, gateOpenDrive2MinPower = .2, gateOpenDrive2MaxPower = .4;
        public double firstCorrectiveStrength = .3;
        public double loadingSlowDrivePower = .35, loadingMinHeadingPower = .45;
        public double loadingNormDrivePower = .5;

        public double[] first = {-11.5, 48, 90};
        public double[] second = {12, 48, 90};
        public double[] secondIfOpenGate = {12, 45.5, 90};
        public double[] third = {36, 50, 90};

        public double[] firstControlPointNear = { -10, 31, 90 };
        public double firstNearT1 = 24, firstNearT2 = 19;

        public double[] firstControlPointFar = {-6, 31, 150};
        public double firstFarT1 = 30, firstFarT2 = 20;

        public double[] secondNearWaypoint = { 5.5, 25.5, 55 };
        public double[] secondNearWaypointTol = { 3, 3, 5 };
        public double[] secondNearControlPoint = { 12, 27, 70};
        public double secondNearT1 = 26, secondNearT2 = 18;
        public double secondNearCloseHeadingKP = .003;

        public double[] thirdNearWaypoint = { 20, 26.5, 35 };
        public double[] thirdNearWaypointTol = { 6, 3, 5 };
        public double[] thirdNearControlPoint = { 34, 29, 60};
        public double thirdNearT1 = 23, thirdNearT2 = 18;
        public double thirdCloseHeadingKp = .008;

        public double[] thirdFarWaypoint = { 40, 25, 110 };
        public double[] thirdFarWaypointTol = { 3, 3, 5 };
        public double[] thirdFarControlPoint = { 39, 31, 100 };
        public double thirdFarT1 = 23, thirdFarT2 = 18;

        public double[] gateNearWaypoint = { 2, 26, 75 };
        public double gateTapNearWaypointX = 1.5;
        public double[] gateNearWaypointTol = { 2, 4, 5 };
        public double[] gateNearWaypoint2 = { 9, 41, 88 };
        public double gateTapNearWaypoint2X = 5.5;
        public double[] gateFarControlPoint = { 13, 35, 135 };
        public double gateCollectOpenFarTStartError = 25, gateCollectOpenFarTFinishError = 15;
        public double[] gateCollectOpen = { 10, 58, 110 };
        public double postGateTapSetupGateLinearPower = .5, postGateTapSetupGateHeadingPower = .3;
        public double gatePropertiesRedXOffset = 0;
        public double[] gateOpenTol = { 1, 1, 1 };
        public double[] gateCollect = { 16.5, 63, 130 };
        public double[] gateTapBackup = { 8, 56, 120 };
        public double[] gateTap = { 6.5, 58, 80 };

        public double[] preLoadingWaypoint = { 52, 48, 90 };
        public double[] preLoadingWaypointTol = { 3, 5 };
        public double[] preLoading = { 50, 59.5, 55 }; // (53.024, 59.406, 60.84)
        public double[] loadingWaypoint = { 62, 61, 60 };
        public double[] postLoading = { 63.5, 61, 95 }; //  (61.434, 61.613, 11.484)
        public double[] postLoading2 = { 63, 64, 90 };
        public double[] loadingCorner = { 64, 63, 90 };
        public double loadingCornerBackup = 4;
        public double[] loadingCornerControlPoint = { 64, 30, 90 };
        public double loadingCornerT1 = 37, loadingCornerT2 = 34;
        public double[] loadingGateWait = { 43, 62.5, 150 };
        public double[] loadingGateWaitWaypoint = { 47, 50, 135 };
        public double[] loadingGateWaitWaypointTol = { 3, 3, 5 };

        public double[] limelightScanPos1 = { 48, 60 };
        public double[] limelightScanPos2 = { 12, 60 };
    }
    public static class Shoot {
        public double maxNearShooterVoltage = 9.5;
        public double earlyEngageClutchDist = 10;
        public double nearMinDrivePower2Dist = 20;
        public double nearMinDrivePower1 = .99, minDrivePower2 = .5;
        public double farMinDrivePower = .2;

        // shooting positions
        public double[] nearPreload = {-16, 19, 50};
        public double[] nearPreloadMotif = { -16, 19, 90 };
        public double[] near1 = {-9, 23, 80};
        public double[] near1Last = {-32, 28, 45};
        public double[] near2 = {-6, 26.5, 55};
        public double[] nearGate = { -5, 24, 55 };
        public double[] near3 = {-7, 25.5, 50};
        public double[] near3Last = {-20, 23, 38};

        public double[] farSpike = {52, 17, 145};
        public double[] farPreloadLoading = { 52, 16, 100 };
        public double[] farLoading = { 52, 18, 90 };
        public double[] farLoadingOptimized = { 56.5, 28, 90 };
        public double[] farLoadingControlPoint = { 55, 40, 90 };
        public double farLoadingT1 = 35, farLoadingT2 = 20;
        public double loadingShootCloseHeadingKP = .005;

        // shooting path waypoints to not hit other balls
        public double[] gateNearControlPoint = { 11, 36, 50 };
        public double gateNearShootTStartError = 35, gateNearShootTFinishError = 18;

        public double[] gateFarControlPoint = { 9, 30, 135 };
        public double gateFarTStartError = 20, gateFarTFinishError = 10;

        public double[] far1ControlPoint = { 0, 24, 140 };
        public double firstShootFarT1 = 20, firstShootFarT2 = 10;

        public double[] far2ControlPoint = { 22, 38, 130 };
        public double secondShootFarT1 = 20, secondShootFarT2 = 10;
        public double limelightHeadingTangentDeactivateDist = 28;
    }

    public static class Misc {
        public double[] startNearRed = { -60.3, 39.05, 0 };
        public double[] startNearBlue = { -60.5, -39.05, 0 };
        // RED:
        // -60.614, 39.928, -.491
        // -60.093, 40.032, -0.341
        // BLUE:
        // -61.77, -38.69, .654
        // -61.98, -38.35, .282
        public double[] startFarBlue = { 61.62, -17.9, 180 };
        public double[] startFarRed = { 61.6, 17.5, 180 };
        // BLUE:
        // 61.685, -18.084, 179.189
        // (61.575, -17.677, 179.744)
        // RED:
        // (62.043, 15.995, -179.94)
        // (61.841, 15.906, -179.922)
        //(61.965, 16.458, -179.724)
        public double motifScanTurretRelAngle = Math.PI * .5;
        public double[] gate1 = { -5, 55, 90 };
        public double[] gate2 = { 4.5, 55.5, 90 };
        public double gate1BackupDist = 3;
        public double gatePrepMinPower = .7, gateMinPower = .5;

        public double smartParkNearDist = 25;
        public double smartParkFarDist = 20;
        public double[] parkFar = { 52, 24, 90 };
        public double minParkFarPower = .5;
        public double[] gatePark = { -3, 45, 90 };
    }
    public static class TimeConstraints {
        public double maxMotifScanTime = 3;

        public double shoot1FirstTime = .25, shoot2FirstTime = .4;
        public double shoot1SecondTime = .15;
        public double shootThirdTime = .4;

        public double missBallAdjustTime = .3;
        public double maxShootTime = 1.2;
        public double spikeGateOpeningWait = .5, gateCollectOpenWait = .12, gateTapWait = 1.2;
        public double gateCollectMaxTime = .5;
        public double farParkTime = 28.9, nearParkStopTime = 29, stopAllTime = 31;
        public double postIntakeTime = 0.8, loadingSlowIntakeTime = 1;
        public double farPreloadDriveDelay = .5;
        public double maxLimelightWait = 1;
        public double shooterInterlockMaxWait = .1, colorSortShooterInterlockMaxWait = 1;
        public double nearPostFlickerShootTime = .2, farPostFlickerShootTime = .2;

    }
}

