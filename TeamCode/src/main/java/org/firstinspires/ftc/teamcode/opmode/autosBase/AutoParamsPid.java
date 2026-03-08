package org.firstinspires.ftc.teamcode.opmode.autosBase;

public class AutoParamsPid {
    public static class Collect {
        public double hitGateVelThreshold = 15;
        public double collectDrivePower = 0.85, firstCollectDrivePower = .6, secondCollectDrivePower = .5, thirdCollectDrivePower = .5;
        public double gateOpenDrive1MinPower = .8, gateOpenDrive2MinPower = .5, gateOpenDrive2MaxPower = .8;
        public double firstCorrectiveStrength = .3;
        public double loadingDrivePower = 0.4, loadingHeadingPower = .3;

        public double[] first = {-11.5, 47.5, 90};
        public double[] second = {12, 48, 90};
        public double[] secondIfOpenGate = {12, 44, 90};
        public double[] third = {36, 50, 90};

        public double[] firstControlPointNear = { -10, 31, 90 };
        public double firstNearT1 = 24, firstNearT2 = 19;

        public double[] firstControlPointFar = {-6, 31, 150};
        public double firstFarT1 = 30, firstFarT2 = 20;

        public double[] secondNearControlPointRed = { 7, 25, 65};
        public double[] secondNearControlPointBlue = { 7, -25, -65};
        public double secondNearT1 = 29.5, secondNearT2 = 20;

        public double[] secondFarControlPoint = {19, 26.5, 135};
        public double secondFarT1 = 30, secondFarT2 = 20;

        public double[] thirdNearWaypoint = { 20, 26.5, 35 };
        public double[] thirdNearWaypointTol = { 6, 3, 5 };
        public double[] thirdNearControlPoint = { 34, 29, 60};
        public double thirdNearT1 = 23, thirdNearT2 = 18;
        public double thirdCloseHeadingKp = .008;

        public double[] thirdFarWaypoint = { 40, 25, 110 };
        public double[] thirdFarWaypointTol = { 3, 3, 5 };
//        public double[] thirdFarControlPoint = {40, 27, 110};
//        public double thirdFarT1 = 26, thirdFarT2 = 20;

//        public double preLoadingXRed = 48.5, preLoadingYRed = 60.5, preLoadingARed = Math.toRadians(60);
//        public double preLoadingXBlue = 48.5, preLoadingYBlue = -60.5, preLoadingABlue = Math.toRadians(-60);
        public double[] preLoading = { 46.5, 61, 40 };
        public double[] preLoadingTol = { 2, 1, 2 };
        public double[] loadingWaypoint = { 58, 64, 40 };
        public double[] postLoading = { 59.5, 64, 18 };
        public double[] postLoadingTol = { 1.5, 1, 3 };

        public double[] gateOpen = { 11.5, 57.5, 119 };
        // 10.4, 57.6, 118.7

        public double[] gateCollect = { 17, 64, 135 };
        public double[] gateNearWaypoint = { 2, 36, 70 };
        public double[] gateNearWaypointTol = { 1.5, 4, 3 };
        public double[] gateOpenTol = { 1, 2, 3 };
        public double[] gateFarControlPoint = { 13, 35, 135 };
        public double gateCollectOpenFarTStartError = 25, gateCollectOpenFarTFinishError = 15;

        public double[] limelightScanPos1 = { 48, 60 };
        public double[] limelightScanPos2 = { 12, 60 };
    }
    public static class Shoot {
        public double earlyEngageClutchDist = 10;
        public double nearMinDrivePower2Dist = 20;
        public double nearMinDrivePower1 = .99, minDrivePower2 = .5;
        public double farMinDrivePower = .2;

        // shooting positions
        public double[] nearPreload = {-16, 19, 50};
        public double[] near1 = {-9, 23, 80};
        public double[] near1Last = {-35, 25, 45};
        public double[] near2 = {-7, 25.5, 55};
        public double[] nearGate = { -5.5, 24, 55 };
        public double[] near3 = {-7, 25.5, 50};
        public double[] near3Last = {-20, 21, 35};

        public double[] farSpike = {52, 16, 145};
        public double[] farPreloadLoading = { 52, 16, 100 };
        public double[] farLoading = { 52, 16, 90 };

        // shooting path waypoints to not hit other balls
        public double[] gateNearControlPoint = { 11, 36, 50 };
        public double gateNearShootTStartError = 35, gateNearShootTFinishError = 18;

        public double[] gateFarControlPoint = { 9, 30, 135 };
        public double gateFarTStartError = 20, gateFarTFinishError = 10;

        public double[] far1ControlPoint = { 0, 24, 140 };
        public double firstShootFarT1 = 20, firstShootFarT2 = 10;

        public double[] far2ControlPoint = { 22, 38, 130 };
        public double secondShootFarT1 = 20, secondShootFarT2 = 10;
    }

    public static class Misc {
        public double[] startNearRed = { -60.3, 39.8, 0 };
        public double[] startNearBlue = { -60.5, -38.52, 0 };
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
        public double[] gate1 = {-3, 55, 90};
        public double[] gate2 = {5, 55, 90};
        public double gateBackupDist = 3;
        public double gatePrepMinPower = 0.7, gateMinPower = 0.5;

        public double[] parkFar = { 52, 21, 90 };
        public double minParkFarPower = .8;
    }
    public static class TimeConstraints {
        public double maxShootTime = 1.5;
        public double lastShootExtraTime = 2;
        public double gateOpeningWait = 1, gateCollectOpenWait = .05;
        public double gateCollectMaxTime = .1;
        public double cornerCollectMaxTime = 1.9;
        public double autoEndTime = 29.5;
        public double stopEverythingTime = 35;
        public double postIntakeTime = 0.8, loadingSlowIntakeTime = 1;
        public double secondGaitWait = .9, thirdGateWait = 0;
        public double farPreloadDriveDelay = .5;
        public double maxLimelightWaitTime = 1;

    }
}

