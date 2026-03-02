package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Tolerance;

public class AutoParamsPid {
    public static class Collect {
        public double hitGateVelThreshold = 15, gateCollectHitWallThreshold = 15;
        public double collectDrivePower = 0.85, firstCollectDrivePower = .6, secondCollectDrivePower = .45, thirdCollectDrivePower = .5;
        public double gateOpenDrive1Power = .8, gateOpenDrive2Power = .45;
        public double firstCorrectiveStrength = .3;
        public double loadingZoneCollectDrivePower = 0.3;

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
        public double secondFarT1 = 30, secondFarT2r = 20;

        public double[] thirdNearWaypoint = { 20, 26.5, 35 };
        public double thirdNearWaypointXTol = 6, thirdNearWaypointYTol = 3, thirdNearWaypointHeadingTol = 5;
        public double[] thirdNearControlPoint = { 34, 28, 60};
        public double thirdNearT1Red = 21, thirdNearT1Blue = 21, thirdNearT2 = 18;
        public double thirdCloseHeadingKp = .008;

        public double[] thirdFarControlPoint = {39, 40, 135};
        public double thirdFarT1 = 30, thirdFarT2 = 20;

        public double preLoadingXRed = 48.5, preLoadingYRed = 60.5, preLoadingARed = Math.toRadians(60), preLoadingXBlue = 48.5, preLoadingYBlue = -60.5, preLoadingABlue = Math.toRadians(-60);
        public double postLoadingXRed = 62, postLoadingYRed = 62, postLoadingARed = Math.toRadians(65), postLoadingXBlue = 62, postLoadingYBlue = -62, postLoadingABlue = Math.toRadians(-65);
        public double cornerCollectXRed = 66, cornerCollectYRed = 68, cornerCollectARed = Math.toRadians(90);
        public double cornerCollectXBlue = 66, cornerCollectYBlue = -68, cornerCollectABlue = Math.toRadians(-90);
        public double cornerCollectRetryX = 55, cornerCollectRetryYRed = 46, cornerCollectRetryYBlue = -46;

        public double gateOpenKPMult = 1.2;
        public double[] gateOpen = { 7.8, 53, 90 };
        public double[] postGateOpenWaypoint = {16, 58, 130 };

        public double[] gateCollect = { 17, 64, 135 };
        public double[] gateNearControlPoint = { 9, 36, 90 };
        public double gateNearT1 = 27, gateNearT2 = 22;
        public double[] gateFarControlPoint = { 13, 35, 135 };
        public double gateCollectOpenFarTStartError = 25, gateCollectOpenFarTFinishError = 15;
    }
    public static class Shoot {
        public double earlyEngageClutchDist = 15;
        public double minPower2Dist = 20;
        public double minDrivePower1 = .99, minDrivePower2 = .5;
        public PathParams.HeadingLerpType preloadHeadingLerp = PathParams.HeadingLerpType.TANGENT;
        public double waypointTolX = 3, waypointTolY = 3, waypointTolA = Math.toRadians(5);
        public Tolerance waypointTol = new BoxTolerance(waypointTolX, waypointTolY, Math.toRadians(waypointTolA));
        public double waypointSlowDown = 0.3;

        // shooting positions
        public double[] nearPreload = new double[] {-16, 19, 35};
        public double[] near1 = new double[] {-9, 23, 80};
        public double[] near1Last = new double[] {-35, 25, 45};
        public double[] near2 = new double[] {-7, 25.5, 55};
        public double[] near3 = new double[] {-7, 25.5, 50};
        public double[] near3Last = new double[] {-20, 22, 40};

        public double[] far = new double[] {54, 16};
        public double farSetup1A = 180, farSetup2A = 170, farSetup3A = 150, farSetupLoadingA = 95;

        // shooting path waypoints to not hit other balls
        public double[] gateNearControlPoint = { 11, 36, 50 };
        public double gateNearShootTStartError = 35, gateNearShootTFinishError = 18;

        public double[] gateShootFarControlPoint = { 9, 30, 135 };
        public double gateFarTStartError = 20, gateFarTFinishError = 10;

        public double[] far1ControlPoint = { 0, 24, 140 };
        public double firstShootFarTStartError = 20, firstShootFarTFinishError = 10;

        public double[] far2ControlPoint = { 22, 38, 130 };
        public double secondShootFarTStartError = 20, secondShootFarTFinishError = 10;
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
        public double[] startFar = { 63, 16.6868, 180 };
        public double[] gate1 = {-3, 55, 90};
        public double[] gate2 = {5, 55, 90};
        public double gateBackupDist = 3;
        public double gatePrepMinPower = 0.7, gateMinPower = 0.5;
    }
    public static class TimeConstraints {
        public double maxShootTime = 1.5;
        public double lastShootExtraTime = 2;
        public double gateOpeningWait = 0.2, gateCollectOpenWait = .05;
        public double gateCollectMaxTime = .95;
        public double cornerCollectMaxTime = 1.9;
        public double autoEndTime = 29.5;
        public double stopEverythingTime = 35;
        public double postIntakeTime = 0.8, loadingSlowIntakeTime = 1;
        public double secondGaitWait = .9, thirdGateWait = 0;

    }
}

