package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Tolerance;

public class AutoParamsPid {
    public static class Collect {
        public double hitGateVelThreshold = 10;
        public double curvedCollect2NearA = Math.toRadians(50);
        public double curvedCollect2FarA = Math.toRadians(135);
        public double collectDrivePower = 0.8, gateOpenDrivePower = .7;
        public double gateHitDrivePower = .5, gateHitActivateDist = 5;
        public double secondTValueStartError = 30.5, secondTValueFinishError = 20;
        public double thirdTValueStartError = 31, thirdTValueFinishError = 20;
        public double loadingZoneCollectDrivePower = 0.3;
        public double waypointSlowDown = 0.4;
        public double collectWaypointTolX = 0.75, collectWaypointTolY = 2, collectWaypointTolHeading = 7;
        public Tolerance waypointTol = new BoxTolerance(collectWaypointTolX, collectWaypointTolY, collectWaypointTolHeading);
        public double lineARed = Math.toRadians(90), lineABlue = Math.toRadians(-90);
        public double firstX = -12.5, postFirstY = 46;
        public double secondX = 12, postSecondY = 47, postSecondYOffsetIfOpenGate = 3;
        public double preSecondY = 26.5, preSecondXNear = 7, preSecondXFar = 19;
        public double thirdX = 36, preThirdY = 27.5, postThirdY = 46;
        public double preThirdXNear = 30, preThirdXFar = 40;
        public double preCollect3NearA = Math.toRadians(40);
        public double preCollect3FarA = Math.toRadians(140);

        public double preLoadingXRed = 48.5, preLoadingYRed = 60.5, preLoadingARed = Math.toRadians(60), preLoadingXBlue = 48.5, preLoadingYBlue = -60.5, preLoadingABlue = Math.toRadians(-60);
        public double postLoadingXRed = 62, postLoadingYRed = 62, postLoadingARed = Math.toRadians(65), postLoadingXBlue = 62, postLoadingYBlue = -62, postLoadingABlue = Math.toRadians(-65);
        public double cornerCollectXRed = 66, cornerCollectYRed = 68, cornerCollectARed = Math.toRadians(90);
        public double cornerCollectXBlue = 66, cornerCollectYBlue = -68, cornerCollectABlue = Math.toRadians(-90);
        public double cornerCollectRetryX = 55, cornerCollectRetryYRed = 46, cornerCollectRetryYBlue = -46;
        public double gateCollectDistTol = 1, gateCollectHeadingTol = 5;
        public Tolerance gateCollectOpenTol = new BoxTolerance(gateCollectDistTol, gateCollectHeadingTol);
        public double gateCollectMinPower = 0.4;
        public double[] gateCollectOpenFarRed = { 8, 61, 116 };
        public double[] gateCollectOpenFarBlue = { 8, -61, -116 };
        public double[] gateCollectOpenNear = { 7.5, 53, 100 };

        public double[] gateCollect = { 14, 61, 135 };
        public double[] gateCollectOpenControlPoint = { 13, 36, 50 };
        public double gateCollectOpenTStartError = 40, gateCollectOpenTFinishError = 18;

        public double[] firstCollectControlPointNear = { -12, 31, 90 };
        public double firstCollectTStartError = 28, firstCollectTFinishError = 20;
    }
    public static class Shoot {
        public double earlyEngageClutchDist = 15;
        public double minShootDrivePower = .7;
        public double[] gateShootControlPoint = { 11, 36, 50 };
        public double gateShootTStartError = 35, gateShootTFinishError = 18;
        public PathParams.HeadingLerpType preloadHeadingLerp = PathParams.HeadingLerpType.TANGENT;
        public double waypointTolX = 3, waypointTolY = 3, waypointTolA = Math.toRadians(5);
        public Tolerance waypointTol = new BoxTolerance(waypointTolX, waypointTolY, waypointTolA);
        public double waypointSlowDown = 0.3;

        // shooting positions
        public double[] preload = new double[] {-14, 21, 25.5};
        public double[] near1 = new double[] {-6.5, 24, 80};
        public double[] firstLast = new double[] {-35, 25, 45};
        public double[] near2 = new double[] {-7, 25.5, 55};
        public double[] near3 = new double[] {-7, 25.5, 50};
        public double[] thirdLast = new double[] {-25, 22, 30};
        public double shootFarXRed = 54, shootFarYRed = 16, shootFarXBlue = 54, shootFarYBlue = -16;
        public double shootFarSetup1ARed = Math.toRadians(180), shootFarSetup2ARed = Math.toRadians(170), shootFarSetup3ARed = Math.toRadians(150), shootFarSetupLoadingARed = Math.toRadians(95);
        public double shootFarSetup1ABlue = Math.toRadians(-180), shootFarSetup2ABlue = Math.toRadians(-170), shootFarSetup3ABlue = Math.toRadians(-150), shootFarSetupLoadingABlue = Math.toRadians(-95);

        // shooting path waypoints to not hit other balls
        public double shootFar1WaypointXRed = 0, shootFar1WaypointYRed = 24, shootFar1WaypointARed = Math.toRadians(140);
        public double shootFar2WaypointXRed = 22, shootFar2WaypointYRed = 38, shootFar2WaypointARed = Math.toRadians(130);

        public double shootFar1WaypointXBlue = 0, shootFar1WaypointYBlue = -24, shootFar1WaypointABlue = Math.toRadians(-140);
        public double shootFar2WaypointXBlue = 22, shootFar2WaypointYBlue = -38, shootFar2WaypointABlue = Math.toRadians(-130);
    }

    public static class Misc {
        public double[] startNear = { -61.5, 41.1, 0 };
        public double[] startFar = { 63, 17.625, 180 };
        public double startFarXRed = 63, startFarYRed = 17.625, startFarARed = Math.toRadians(180),
                startFarXBlue = 63, startFarYBlue = -17.625, startFarABlue = Math.toRadians(180);

        public double gate1X = -3, gate2X = 5, gateY = 55;
        public double gateA = Math.toRadians(90);
        public double gateBackupDist = 3;
        public double gatePrepMinPower = 0.7, gateMinPower = 0.5;

        public double[] gateFarWaypointRed = { 16, 36, 135 };
        public double[] gateFarWaypointBlue = { 16, -36, -135 };

        public double parkFarXRed = 55, parkFarYRed = 23, parkFarARed = Math.toRadians(135);
        public double parkFarXBlue = 55, parkFarYBlue = -23, parkFarABlue = Math.toRadians(-135);
    }
    public static class TimeConstraints {
        public double maxShootTime = 1.5;
        public double gateOpeningWait = 0.2, gateCollectOpenWait = .2;
        public double gateCollectMaxTime = .85;
        public double cornerCollectMaxTime = 1.9;
        public double autoEndTime = 29.5;
        public double stopEverythingTime = 35;
        public double postIntakeTime = 0.5, loadingSlowIntakeTime = 1;

    }
}

