package org.firstinspires.ftc.teamcode.opmode.autosBase;

public class AutoParamsPid {
    public static class Collect {
        public double hitGateVelThreshold = 15;
        public double hitBallVelThreshold = 15;
        public double collectDrivePower = 0.85, firstCollectDrivePower = .6, secondCollectDrivePower = .5, thirdCollectDrivePower = .5;
        public double gateOpenDrive1MinPower = .8, gateOpenDrive2MinPower = .1, gateOpenDrive2MaxPower = .4;
        public double firstCorrectiveStrength = .3;
        public double loadingDrivePower = 0.35, loadingMinHeadingPower = .5;

        public double[] first = {-11.5, 47.5, 90};
        public double[] second = {12, 48, 90};
        public double[] secondIfOpenGate = {12, 44, 90};
        public double[] third = {36, 50, 90};

        public double[] firstControlPointNear = { -10, 31, 90 };
        public double firstNearT1 = 24, firstNearT2 = 19;

        public double[] firstControlPointFar = {-6, 31, 150};
        public double firstFarT1 = 30, firstFarT2 = 20;

        public double[] secondNearWaypoint = { 5.5, 25.5, 55 };
        public double[] secondNearWaypointTol = { 3, 3, 5 };
        public double[] secondNearControlPoint = { 12, 27, 70};
        public double secondNearT1 = 26, secondNearT2 = 18;
        public double secondNearCloseHeadingKP = .008;

        public double[] thirdNearWaypoint = { 20, 26.5, 35 };
        public double[] thirdNearWaypointTol = { 6, 3, 5 };
        public double[] thirdNearControlPoint = { 34, 29, 60};
        public double thirdNearT1 = 23, thirdNearT2 = 18;
        public double thirdCloseHeadingKp = .008;

        public double[] thirdFarWaypoint = { 40, 25, 110 };
        public double[] thirdFarWaypointTol = { 3, 3, 5 };
        public double[] thirdFarControlPoint = { 38, 31, 100 };
        public double thirdFarT1 = 23, thirdFarT2 = 18;

        public double[] gateNearWaypoint = { 2, 32, 75 };
        public double[] gateNearWaypointTol = { 1.5, 4, 4 };
        public double[] gateNearControlPoint = { 11, 40, 90 };
        public double gateNearT1 = 23, gateNearT2 = 20;
        public double[] gateFarControlPoint = { 13, 35, 135 };
        public double gateCollectOpenFarTStartError = 25, gateCollectOpenFarTFinishError = 15;
        public double[] gateOpen = { 10.5, 60.5, 115 };
        public double[] gateOpenTol = { 1, 1, 1 };
        public double[] gateOpenHold = { 10.5, 60.5, 115 };
        public double[] gateCollect = { 16, 63, 135 };
        public double[] gateTapBackup = { 10, 56, 120 };
        public double[] gateTap = { 7, 58.5, 80 };

        public double[] preLoadingControlPoint = { 49, 56, 70 };
        public double preLoadingT1 = 14, preLoadingT2 = 10;
        public double[] preLoading = { 51, 59.5, 55 }; // (53.024, 59.406, 60.84)
        public double[] preLoadingTol = { 2, 2, 5 };
        public double[] loadingWaypoint = { 62, 61, 60 };
        public double[] postLoading = { 62.5, 63, 70 }; //  (61.434, 61.613, 11.484)
        public double[] postLoadingTol = { 1, 1, 3 };
        public double[] loadingCorner = { 62, 63, 90 };
        public double loadingCornerBackup = 5;
        public double[] loadingCornerWaypoint = { 62, 30, 90 };
        public double[] loadingCornerWaypointTol = { 3, 2, 5, };
        public double[] loadingGateWait = { 43, 59, 135 };
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
        public double[] near1 = {-9, 23, 80};
        public double[] near1Last = {-35, 25, 45};
        public double[] near2 = {-7, 25.5, 55};
        public double[] nearGate = { -4, 24, 55 };
        public double[] near3 = {-7, 25.5, 50};
        public double[] near3Last = {-20, 21, 38};

        public double[] farSpike = {52, 16, 145};
        public double[] farPreloadLoading = { 52, 16, 100 };
        public double[] farLoading = { 52, 16, 90 };
        public double[] farLoadingControlPoint = { 55, 40, 90 };
        public double farLoadingT1 = 35, farLoadingT2 = 20;

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
        public double[] gate1 = {-5, 55, 90};
        public double[] gate2 = {7, 55, 90};
        public double gate1BackupDist = 3, gate2BackupDist = 5;
        public double gatePrepMinPower = .7, gateMinPower = .5;

        public double smartParkNearDist = 25;
        public double smartParkFarDist = 25;
        public double[] parkFar = { 52, 21, 90 };
        public double minParkFarPower = .8;
    }
    public static class TimeConstraints {
        public double maxShootTime = 1.4;
        public double lastShootExtraTime = 2;
        public double gateOpeningWait = .9, gateCollectOpenWait = .12, gateTapWait = .5;
        public double gateCollectMaxTime = .5;
        public double farParkTime = 29.1, nearParkStopTime = 29, farParkStopTime = 28;
        public double postIntakeTime = 0.8, loadingSlowIntakeTime = 1;
        public double farPreloadDriveDelay = .5;
        public double maxLimelightWaitTime = 1;

    }
}

