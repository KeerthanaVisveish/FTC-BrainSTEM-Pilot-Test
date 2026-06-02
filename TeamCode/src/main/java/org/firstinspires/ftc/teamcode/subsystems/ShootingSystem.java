package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.Vector3dOld;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCacher;
import org.firstinspires.ftc.teamcode.utils.misc.ServoImplCacher;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt1;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt2;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.util.Arrays;
import java.util.function.ToDoubleFunction;

@Config
public class ShootingSystem {
    public static class TestingParams {
        public boolean useNewShooting = false;
        public boolean powerTurret = true;
        public boolean powerLowShooter = true, powerHighShooter = true;
        public boolean drawShootingRings = false;
        public boolean dynamicHood = true;
    }
    public static class GoalParams {
        public Vector2d redNearPerpLaunchLine = new Vector2d(1, 1).div(Math.sqrt(2));
        public Vector2d blueNearPerpLaunchLine = new Vector2d(1, -1).div(Math.sqrt(2));
        public Vector2d redFarPerpLaunchLine = new Vector2d(0, 1);
        public Vector2d blueFarPerpLaunchLine = new Vector2d(0, -1);
        public double startShootingWhileMovingNearDist = 6, startShootingWhileMovingFarDist = 26;
        // when shooting from really close
        public double closeRedX = -65, closeRedY = 63;
        public double closeBlueX = -65, closeBlueY = -62;
        public double closeHeight = 38;
        public double closeImpactAng = Math.toRadians(-20);

        // when cycling from gate
        public double gateRedX = -66, gateRedY = 64.5;
        public double gateBlueX = -66, gateBlueY = -63;
        public double gateHeight = 39.5;
        public double gateImpactAng = -.31;

        // when shooting from opposing goal area
        public double oppositeRedX = -61, oppositeRedY = 66;
        public double oppositeBlueX = -62, oppositeBlueY = -66;
        public double oppositeHeight = 41.5;
        public double oppositeImpactAng = -.31;


        // when shooting in far zone
        public double farRedX = -65, farRedY = 67;
        public double farBlueX = -66, farBlueY = -62;
        public double farHeight = 41;
        public double farImpactAng = Math.toRadians(-30);

        public double missExitAngle = Math.toRadians(35);

        public double reallyCloseRadius = 45;
        public double gateLocationYThreshold = -12, gateLocationXThreshold = -50;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
        public double resolution = 0.009;
        public double robotVelThresholdToSetHood = 2;
    }
    public static class GeneralParams {
        public double estVoltage = 13.4, voltageSensorTrust = .65;
        public double shootingZoneRadius = 31;
        public double firstShootToleranceMps = .3, closeShootToleranceMps = .5, farShootToleranceMps = .43;
        public double lookAheadTime = .2; // time to look ahead for pose prediction
        public double shooterTau = 0.1;
        public double axisOfRotationTrust = .25;
        public int numApproximations = 4;
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
        public double efficiencyCoefWeight = .4;
        public double maxShootWhileMovingSpeed = 1;

        // estimated accel thresholds: position: 20, heading: 5
    }
    public static class FarParams {
        public double autoFarVelOffset = .25;
        public double farVelOffset = .33;
        public double far2SwitchY = 3;

        public double far1ExitAng = .715585;
        public double far1SecondBallOffset = Math.toRadians(1);
        public double far1ThirdBallOffset = Math.toRadians(2);

        public double far2ExitAng = .683225;
        public double far2SecondBallOffset = 0;
        public double far2ThirdBallOffset = 0;

        public double robotVelNoiseThreshold = .1;
        public double maxShootingDist = 175;
    }
    public static TestingParams testingParams = new TestingParams();
    public static GoalParams goalParams = new GoalParams();
    public static HoodParams hoodParams = new HoodParams();
    public static GeneralParams generalParams = new GeneralParams();
    public static FarParams farParams = new FarParams();

    public enum Location {
        GATE_CYCLE, OPPOSITE_SIDE, REALLY_CLOSE, FAR
    }
    public Location locationState;
    public boolean usingHighArc;
    public Vector3dOld closeGoalPos, gateGoalPos, oppositeGoalPos, farGoalPos;
    private boolean shouldScore;
    public Vector2d corner;
    private final HardwareMap hardwareMap;
    private final BrainSTEMRobot robot;
    private MotorCacher turretMotor, shooterLowMotor, shooterHighMotor;
    private ServoImplCacher hoodLeftServo, hoodRightServo;
    public Vector3dOld goalPosIn;
    public Vector2d futureTurretPosRelativeToGoal;
    public double impactAngleRad;
    public Pose2d futureRobotPose;

    public Vector2d robotVelAtTurretIps, lookAheadRobotVelAtTurretIps;
    public double robotSpeedAtTurretIps, lookAheadRobotSpeedAtTurretIps;

    public Vector3dOld lookAheadTargetExitVelMps;
    public double lookAheadTurretTargetAngleRad, currentTurretTargetAngleRad;
    public double lookAheadTargetExitSpeedMps, currentTargetExitSpeedMps;
    public double idealBallExitAng;
    public double ballTargetExitSpeedMps;
    public double efficiencyCoef;

    private double filteredShooterSpeedTps, prevFilteredShooterSpeedTps, rawShooterSpeedTps;
    public double curExitSpeedMps;
    public double ballExitAngleRad, hoodExitAngleRad;
    public double[] physicsExitAngleRads;
    public OdoInfo robotVelCm;
    public double turretPosGoalDistIn, futureTurretPosGoalDistIn;
    public Pose2d turretPose;
    public Vector2d futureTurretPos;
    public double desiredBallDir;

    private double curTimeMs;
    public double dt;
    public ShooterLookup lookupTable;
    public double relGoalHeightM;
    public boolean checkShootingWhileMoving, currentlyShootingWhileMoving;
    private final ShootingMath shootingMathNew;
    private AnswerKeyPt1 lookAheadAnswerKeyPt1, currentAnswerKeyPt1;
    private AnswerKeyPt2 lookAheadAnswerKeyPt2, currentAnswerKeyPt2;
    private final Telemetry telemetry;
    public ShootingSystem(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;

        shootingMathNew = new ShootingMath();

        setShouldScore(true);
        initTurret();
        initShooter();
        initHood();
        curTimeMs = System.currentTimeMillis();
        lookupTable = new ShooterLookup();
        efficiencyCoef = 0.39;


        locationState = Location.GATE_CYCLE;

        physicsExitAngleRads = new double[generalParams.numApproximations];
        updateGoalPoses();
    }
    public void updateGoalPoses() {
        if(BrainSTEMRobot.alliance == Alliance.BLUE) {
            corner = new Vector2d(-72, -72);
            closeGoalPos = new Vector3dOld(goalParams.closeBlueX, goalParams.closeHeight, goalParams.closeBlueY);
            gateGoalPos = new Vector3dOld(goalParams.gateBlueX, goalParams.gateHeight, goalParams.gateBlueY);
            oppositeGoalPos = new Vector3dOld(goalParams.oppositeBlueX, goalParams.oppositeHeight, goalParams.oppositeBlueY);
            farGoalPos = new Vector3dOld(goalParams.farBlueX, goalParams.farHeight, goalParams.farBlueY);
        }
        else {
            corner = new Vector2d(-72, 72);
            closeGoalPos = new Vector3dOld(goalParams.closeRedX, goalParams.closeHeight, goalParams.closeRedY);
            gateGoalPos = new Vector3dOld(goalParams.gateRedX, goalParams.gateHeight, goalParams.gateRedY);
            oppositeGoalPos = new Vector3dOld(goalParams.oppositeRedX, goalParams.oppositeHeight, goalParams.oppositeRedY);
            farGoalPos = new Vector3dOld(goalParams.farRedX, goalParams.farHeight, goalParams.farRedY);
        }
    }
    private void initTurret() {
        DcMotorEx rawTurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        rawTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawTurretMotor.setPower(0);
        this.turretMotor = new MotorCacher(rawTurretMotor);
    }
    private void initShooter() {
        DcMotorEx rawShooterMotorLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        rawShooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawShooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawShooterMotorLow.setDirection(DcMotorSimple.Direction.FORWARD);
        rawShooterMotorLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterLowMotor = new MotorCacher(rawShooterMotorLow);

        DcMotorEx rawShooterMotorHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        rawShooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawShooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawShooterMotorHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        rawShooterMotorHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterHighMotor = new MotorCacher(rawShooterMotorHigh);
    }
    private void initHood() {
        ServoImplEx rawHoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        rawHoodLeftServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));
        this.hoodLeftServo = new ServoImplCacher(rawHoodLeftServo);

        ServoImplEx rawHoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        rawHoodRightServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));
        this.hoodRightServo = new ServoImplCacher(rawHoodRightServo);
    }
    public void updateProperties() {
        double prevTimeMs = curTimeMs;
        curTimeMs = System.currentTimeMillis();
        dt = (curTimeMs - prevTimeMs) / 1000;

        hoodLeftServo.updateInfo();
        hoodRightServo.updateInfo();

        Pose2d robotPose = robot.drive.localizer.getPose();
        futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(generalParams.lookAheadTime);

        updateGoalPoses();
        updateGoalPos(robotPose.position);

        updateTurretProperties(robotPose);
        shooterHighMotor.updateInfo();
        shooterLowMotor.updateInfo();

        rawShooterSpeedTps = (shooterHighMotor.getVelTps() + shooterLowMotor.getVelTps()) * 0.5;
        prevFilteredShooterSpeedTps = filteredShooterSpeedTps;
        if(filteredShooterSpeedTps == 0)
            filteredShooterSpeedTps = rawShooterSpeedTps;
        else {
            double a_s = generalParams.shooterTau == 0 ? 0 : Math.exp(-dt / generalParams.shooterTau);
            filteredShooterSpeedTps = filteredShooterSpeedTps * a_s + rawShooterSpeedTps * (1 - a_s);
        }

        desiredBallDir = Math.atan2(goalPosIn.z - futureTurretPos.y, goalPosIn.x - futureTurretPos.x);

        Vector2d robotVelAtTurretMps = robotVelAtTurretIps.times(.0254);
        Vector2d lookAheadRobotVelAtTurretMps = lookAheadRobotVelAtTurretIps.times(.0254);
        if(testingParams.useNewShooting)
            updatePhysicsPropertiesNew();
        else {
            Vector2d perpLaunchLine;
            if(locationState == Location.FAR)
                perpLaunchLine = BrainSTEMRobot.alliance == Alliance.RED ? goalParams.redFarPerpLaunchLine : goalParams.blueFarPerpLaunchLine;
            else
                perpLaunchLine = BrainSTEMRobot.alliance == Alliance.RED ? goalParams.redNearPerpLaunchLine : goalParams.blueNearPerpLaunchLine;

            double threshold = locationState == Location.FAR ? goalParams.startShootingWhileMovingFarDist : goalParams.startShootingWhileMovingNearDist;
            if(robot.drive.pinpoint().getPose().position.dot(perpLaunchLine) > threshold)
                lookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
            updatePhysicsProperties(desiredBallDir, robotVelAtTurretMps, lookAheadRobotVelAtTurretMps);
        }
    }

    public void updatePhysicsPropertiesNew() {
        Pose2d robotPose = robot.drive.pinpoint().getPose();
        Vector3d exitPosM = new Vector3d(turretPose.position.x, turretPose.position.y, ShootingMathOld.approximateExitHeightM(locationState == Location.GATE_CYCLE)).times(.0254);
        Vector3d robotPosM = new Vector3d(robotPose.position.x, robotPose.position.y, 0).times(.0254);
        OdoInfo robotVel = robot.drive.pinpoint().getVelocity();
        Vector3d robotVelCm = new Vector3d(robotVel.x, robotVel.y, 0).times(.0254);
        double robotAngularVel = robot.drive.pinpoint().getVelocity().headingRad;
        Vector3d goalPosM = new Vector3d(goalPosIn.x, goalPosIn.y, goalPosIn.z).times(.0254);
        relGoalHeightM = goalPosM.z - exitPosM.z;
        ToDoubleFunction<Double> shooterConversion = exitAngle -> {
            double e = calcEfficiencyCoef(exitAngle);
            return ShootingMathOld.ticksPerSecToExitSpeedMps(1, e);
        };

        telemetry.addData("exit pos m", exitPosM);
        telemetry.addData("robot pos m", robotPosM);
        telemetry.addData("robot vel cm", robotVelCm);
        telemetry.addData("robot angular vel", robotAngularVel);
        telemetry.addData("goal pos m", goalPosM);

        // TODO: change robot pos argument to center of rotation of robot instead of odometry readings
        lookAheadAnswerKeyPt1 = shootingMathNew.godSolvePart1(exitPosM, robotPosM, robotVelCm, robotAngularVel, goalPosM, impactAngleRad, generalParams.lookAheadTime);
        lookAheadAnswerKeyPt2 = shootingMathNew.godSolvePart2(lookAheadAnswerKeyPt1, goalPosM, impactAngleRad, filteredShooterSpeedTps, shooterConversion);

        currentAnswerKeyPt1 = shootingMathNew.godSolvePart1(exitPosM, robotPosM, robotVelCm, robotAngularVel, goalPosM, impactAngleRad, 0);
        currentAnswerKeyPt2 = shootingMathNew.godSolvePart2(currentAnswerKeyPt1, goalPosM, impactAngleRad, filteredShooterSpeedTps, shooterConversion);

        futureRobotPose = new Pose2d(
                robotPose.position.plus(new Vector2d(robotVel.x, robotVel.y).times(generalParams.lookAheadTime)),
                robotPose.heading.toDouble() + robotAngularVel * generalParams.lookAheadTime);
        if(lookAheadAnswerKeyPt1.solutionExists) {
            lookAheadTargetExitSpeedMps = lookAheadAnswerKeyPt1.launchData.speed;
            if(robot.collector.getClutchState() == Collector.ClutchState.ENGAGED && robot.collector.getCollectionState() == Collector.CollectionState.INTAKE) {
                if(lookAheadAnswerKeyPt2.solutionExists) {
                    lookAheadTurretTargetAngleRad = lookAheadAnswerKeyPt2.launchData.turretAng;
                    hoodExitAngleRad = lookAheadAnswerKeyPt2.launchData.exitAng;
                }
            }
            else {
                lookAheadTurretTargetAngleRad = lookAheadAnswerKeyPt1.launchData.turretAng;
                hoodExitAngleRad = lookAheadAnswerKeyPt1.launchData.exitAng;
            }
        }
        if(currentAnswerKeyPt1.solutionExists) {
            currentTargetExitSpeedMps = lookAheadAnswerKeyPt1.launchData.speed;
            if(currentAnswerKeyPt2.solutionExists)
                currentTurretTargetAngleRad = currentAnswerKeyPt2.launchData.turretAng;
        }
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird
    private void updatePhysicsProperties(double desiredBallDir, Vector2d noLookAheadRobotVelAtTurretMps, Vector2d lookAheadRobotVelAtTurretMps) {
        if(testingParams.dynamicHood)
            checkShootingWhileMoving = locationState != Location.FAR;
        else
            checkShootingWhileMoving = true;
        // get delta y of projectory (need approximate exit height of the ball)
        double exitHeightM = ShootingMathOld.approximateExitHeightM(locationState == Location.REALLY_CLOSE);
        relGoalHeightM = (goalPosIn.y * 0.0254 - exitHeightM);
        double futureDist = futureTurretPosGoalDistIn * 0.0254;

        double[] launchVector;

        if(checkShootingWhileMoving) {
            double sign = BrainSTEMRobot.alliance == Alliance.RED ? -1 : 1;
            boolean inFar2 = robot.drive.pinpoint().getPose().position.y * sign > farParams.far2SwitchY;
            if(locationState == Location.FAR) {
                double exitAng =  inFar2 ? farParams.far2ExitAng : farParams.far1ExitAng;
                launchVector = new double[] {
                        ShootingMathOld.calculateLaunchVelocityWithExitAngle(futureDist, relGoalHeightM, exitAng),
                        exitAng
                };
            }
            else
                launchVector = ShootingMathOld.calculateLaunchVectorWithImpactAngle(futureDist, relGoalHeightM, impactAngleRad);

            ballTargetExitSpeedMps = launchVector[0];
            ballExitAngleRad = launchVector[1];

            currentlyShootingWhileMoving = robot.collector.getClutchState() == Collector.ClutchState.ENGAGED;
            if(!currentlyShootingWhileMoving) {
                noLookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
                lookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
            }
            lookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(desiredBallDir, launchVector[1], launchVector[0], lookAheadRobotVelAtTurretMps);
            double baseLength = Math.hypot(lookAheadTargetExitVelMps.x, lookAheadTargetExitVelMps.z);

            // setting hood angle
            hoodExitAngleRad = Range.clip(Math.atan2(lookAheadTargetExitVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
            if(locationState == Location.FAR) {
                if(robot.shooter.getNumBallsShot() == 1)
                    hoodExitAngleRad += inFar2 ? farParams.far2SecondBallOffset : farParams.far1SecondBallOffset;
                else if(robot.shooter.getNumBallsShot() == 2)
                    hoodExitAngleRad += inFar2 ? farParams.far2ThirdBallOffset : farParams.far1ThirdBallOffset;
            }

            // setting turret angle
            lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);

            // setting shooter speed
            efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
            curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
            lookAheadTargetExitSpeedMps = Math.hypot( baseLength, lookAheadTargetExitVelMps.y );
            if(checkShootingWhileMoving && locationState == Location.FAR)
                lookAheadTargetExitSpeedMps += farParams.farVelOffset;

            // setting no lookahead values
            double noLookAheadDist = turretPosGoalDistIn * 0.0254;
            double[] noLookAheadLaunchVector = ShootingMathOld.calculateLaunchVectorWithImpactAngle(noLookAheadDist, relGoalHeightM, impactAngleRad);
            double noLookAheadBallDir = Math.atan2(goalPosIn.z - turretPose.position.y, goalPosIn.x - turretPose.position.x);
            Vector3dOld noLookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(noLookAheadBallDir, noLookAheadLaunchVector[1], noLookAheadLaunchVector[0], noLookAheadRobotVelAtTurretMps);
            currentTurretTargetAngleRad = Math.atan2(noLookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
            currentTargetExitSpeedMps = Math.hypot( Math.hypot(noLookAheadTargetExitVelMps.x, noLookAheadTargetExitVelMps.y), noLookAheadTargetExitVelMps.z );
        }
        else {
            futureDist = Math.min(futureDist, farParams.maxShootingDist);
            int sign = BrainSTEMRobot.alliance == Alliance.RED ? -1 : 1;
            double farExitAng = robot.drive.localizer.getPose().position.y * sign > farParams.far2SwitchY ? farParams.far2ExitAng : farParams.far1ExitAng;
            launchVector = new double[] {ShootingMathOld.calculateLaunchVelocityWithExitAngle(futureDist, relGoalHeightM, farExitAng), hoodParams.minExitAngRad};
            ballTargetExitSpeedMps = launchVector[0];
            idealBallExitAng = launchVector[1];
            ballExitAngleRad = idealBallExitAng;

            currentlyShootingWhileMoving = false;
            efficiencyCoef = calcEfficiencyCoef(launchVector[1]); // initial guess for efficiency coefficient
            curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
            curExitSpeedMps = curExitSpeedMps * generalParams.efficiencyCoefWeight + ballTargetExitSpeedMps + (1 - generalParams.efficiencyCoefWeight);

            // determining whether to use high arc or low arc
            double highArcExitAng = ShootingMathOld.calculateBallExitAngleRad(true, relGoalHeightM, futureDist, curExitSpeedMps);
            if (highArcExitAng != -1) {
                double lowArcExitAng = ShootingMathOld.calculateBallExitAngleRad(false, relGoalHeightM, futureDist, curExitSpeedMps);
                double highArcImpactAng = ShootingMathOld.calculateImpactAngle(futureDist, relGoalHeightM, curExitSpeedMps, highArcExitAng);
                double lowArcImpactAng = ShootingMathOld.calculateImpactAngle(futureDist, relGoalHeightM, curExitSpeedMps, lowArcExitAng);
                usingHighArc = Math.abs(highArcImpactAng - impactAngleRad) < Math.abs(lowArcImpactAng - impactAngleRad);

                // estimating hood ang and current shooter speed
                physicsExitAngleRads[0] = usingHighArc ? highArcExitAng : lowArcExitAng;
                ballExitAngleRad = physicsExitAngleRads[0];
                efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef) - robot.shooter.getCurVelocityAdjustment();

                for (int i = 1; i < generalParams.numApproximations; i++) {
                    physicsExitAngleRads[i] = ShootingMathOld.calculateBallExitAngleRad(usingHighArc, relGoalHeightM, futureDist, curExitSpeedMps);
                    if (physicsExitAngleRads[i] == -1) {
                        for (int j = i + 1; j < generalParams.numApproximations; j++)
                            physicsExitAngleRads[j] = -1;
                        break;
                    }
                    ballExitAngleRad = physicsExitAngleRads[i];
                    efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                    curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef) - robot.shooter.getCurVelocityAdjustment();
                }
            } else
                Arrays.fill(physicsExitAngleRads, -1);


            lookAheadTargetExitSpeedMps = ballTargetExitSpeedMps + (robot.collector.inAuto() ? farParams.autoFarVelOffset : farParams.farVelOffset);
            hoodExitAngleRad = ballExitAngleRad;

            // basic estimation of turret angle to try account for shooting while moving
            if(robotSpeedAtTurretIps < farParams.robotVelNoiseThreshold || robot.shooter.getShooterState() == Shooter.ShooterState.OFF) {
                lookAheadTurretTargetAngleRad = desiredBallDir;
                currentTurretTargetAngleRad = desiredBallDir;
            }
            else {
                lookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, curExitSpeedMps, lookAheadRobotVelAtTurretMps);
                lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
                Vector3dOld noLookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, curExitSpeedMps, noLookAheadRobotVelAtTurretMps);
                currentTurretTargetAngleRad = Math.atan2(noLookAheadTargetExitVelMps.z, noLookAheadTargetExitVelMps.x);
            }
            currentTargetExitSpeedMps = ballTargetExitSpeedMps;
        }

        if(!shouldScore)
            hoodExitAngleRad = goalParams.missExitAngle;
    }

    // pro: easy to tune
    // con: no velocity-based hood adjustment
//    private void updateLookupProperties(double desiredBallDir, Vector2d robotVel) {
//        // getting lookup properties
//        double lookupDist = Range.clip(futureTurretPosGoalDistIn, lookupDistsI[0] + 0.01, lookupDistsI[lookupDistsI.length-1] - 0.01);
//        ballExitAngleRad = lookupTable.lookupExitAngleRad(lookupDist);
//        ballTargetExitSpeedMps = lookupTable.lookupVelocityMetersPerSec(lookupDist);
//
//        // allows for shooting while moving
//        lookAheadTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, ballTargetExitSpeedMps, robotVel);
//        double baseLength = Math.hypot(lookAheadTargetExitVelMps.x, lookAheadTargetExitVelMps.z);
//        hoodExitAngleRad = Range.clip(Math.atan2(lookAheadTargetExitVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
//        lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
//
//        efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
//        curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
//        lookAheadTargetExitSpeedMps = Math.hypot( baseLength, lookAheadTargetExitVelMps.y );
//    }
    public void sendHardwareInfo() {
        turretMotor.sendInfo();
        shooterHighMotor.sendInfo();
        shooterLowMotor.sendInfo();
        if(Math.abs(hoodLeftServo.getPosition() - hoodLeftServo.getTargetPosition()) > hoodParams.resolution) {
            hoodLeftServo.sendInfo();
            hoodRightServo.sendInfo();
        }
    }
    private void updateTurretProperties(Pose2d robotPose) {
        turretMotor.updateInfo();
        robot.turret.currentEncoder = getTurretEncoder();

        turretPose = ShootingMathOld.getTurretPose(robotPose, robot.turret.curRelAngleRad);
        futureTurretPos = ShootingMathOld.getTurretPose(futureRobotPose, 0).position;

        robotVelCm = robot.drive.pinpoint().getVelocity();
        Vector2d robotVelCm = new Vector2d(this.robotVelCm.x, this.robotVelCm.y);

        Vector2d relativeTurretPos = turretPose.position.minus(robotPose.position).times(generalParams.axisOfRotationTrust);
        Vector2d robotTanVel = new Vector2d(-relativeTurretPos.y, relativeTurretPos.x*1).times(this.robotVelCm.headingRad); // v = r * w
        robotVelAtTurretIps = robotVelCm.plus(robotTanVel);
        robotSpeedAtTurretIps = Math.hypot(robotVelAtTurretIps.x, robotVelAtTurretIps.y);

        Vector2d lookAheadRelativeTurretPos = futureTurretPos.minus(futureRobotPose.position);
        Vector2d lookAheadRobotTanVel = new Vector2d(-lookAheadRelativeTurretPos.y, lookAheadRelativeTurretPos.x*1).times(this.robotVelCm.headingRad);
        lookAheadRobotVelAtTurretIps = robotVelCm.plus(lookAheadRobotTanVel);
        lookAheadRobotSpeedAtTurretIps = Math.hypot(lookAheadRobotVelAtTurretIps.x, lookAheadRobotVelAtTurretIps.y);

        futureTurretPosRelativeToGoal = new Vector2d(futureTurretPos.x - goalPosIn.x, futureTurretPos.y - goalPosIn.z);
        futureTurretPosGoalDistIn = Math.hypot(futureTurretPosRelativeToGoal.x, futureTurretPosRelativeToGoal.y);
        turretPosGoalDistIn = Math.hypot(turretPose.position.x - goalPosIn.x, turretPose.position.y - goalPosIn.z);
    }
    private void updateGoalPos(Vector2d robotPos) {
        double sign = BrainSTEMRobot.alliance == Alliance.RED ? 1 : -1;
        double distFromCorner = Math.hypot(corner.x - robotPos.x, corner.y - robotPos.y);
        if(robotPos.x > 24) {
            locationState = Location.FAR;
            goalPosIn = farGoalPos;
            impactAngleRad = goalParams.farImpactAng;
        }
        else if(distFromCorner < goalParams.reallyCloseRadius) {
            locationState = Location.REALLY_CLOSE;
            goalPosIn = closeGoalPos;
            impactAngleRad = goalParams.closeImpactAng;
        }
        else if(robotPos.y * sign < goalParams.gateLocationYThreshold || robotPos.x < goalParams.gateLocationXThreshold) {
            locationState = Location.OPPOSITE_SIDE;
            goalPosIn = oppositeGoalPos;
            impactAngleRad = goalParams.oppositeImpactAng;
        }
        else {
            locationState = Location.GATE_CYCLE;
            goalPosIn = gateGoalPos;
            impactAngleRad = goalParams.gateImpactAng;
        }
    }
    public void setShouldScore(boolean shouldScore) {
        this.shouldScore = shouldScore;
    }

    public void printInfo(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("SHOOTING SYSTEM-------");
        telemetry.addData("dist state", locationState);
        telemetry.addData("efficiency coef", efficiencyCoef);
        telemetry.addData("turret absolute target deg", Math.toDegrees(lookAheadTurretTargetAngleRad));
        telemetry.addData("shooter target speed mps", lookAheadTargetExitSpeedMps);
        telemetry.addData("hood exit angle deg", MathUtils.format(Math.toDegrees(hoodExitAngleRad), 3));
        telemetry.addData("ball exit angle deg", MathUtils.format(Math.toDegrees(ballExitAngleRad), 3));
        telemetry.addData("cur exit speed mps", curExitSpeedMps);
        telemetry.addData("shooter norm good", shooterNormGood());
        telemetry.addData("turret on target", robot.turret.onTarget());
        telemetry.addData("safety interlocks met", meetsSafetyInterlocks());
        telemetry.addData("shooter norm good num", shooterNormGood() ? 1 : 0);
        telemetry.addData("turret on target num", robot.turret.onTarget() ? 2: 0);
        telemetry.addData("meets safety interlocks num", meetsSafetyInterlocks() ? 3 : 0);
        telemetry.addData("shooter low current", shooterLowMotor.getCurrent());
        telemetry.addData("shooter high current", shooterHighMotor.getCurrent());
        telemetry.addLine();
//        telemetry.addData("rel height to target meters", relGoalHeightM);
//        telemetry.addData("dist state", distState);
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }
    public int getTurretEncoderRaw() {
        return turretMotor.getCurrentPositionRaw();
    }
    public void resetTurretEncoder() {
        turretMotor.resetEncoders();
    }
    public void setTurretVoltage(double voltage) {
        if (testingParams.powerTurret) {
            double power = voltage / robot.getFilteredVoltage();
            turretMotor.setPower(power);
        }
    }
    public double getTurretPower() {
        return turretMotor.getPower();
    }
    public double getTurretVelTps() {
        return turretMotor.getVelTps();
    }
    private void setShooterPower(double p) {
        p = Range.clip(p, -.99, .99);
        if(testingParams.powerHighShooter)
            shooterHighMotor.setPower(p);
        if(testingParams.powerLowShooter)
            shooterLowMotor.setPower(p);
    }
    public void setShooterVoltage(double voltage) {
        double estVoltage = robot.getFilteredVoltage() * generalParams.voltageSensorTrust + generalParams.estVoltage * (1 - generalParams.voltageSensorTrust);
        double power = voltage / estVoltage;
        setShooterPower(power);
    }
    public double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = generalParams.efficiencyCoefM * ballExitAngleRad + generalParams.efficiencyCoefB;
        return Range.clip(generalParams.minEfficiencyCoef, rawE, generalParams.maxEfficiencyCoef);
    }
    public boolean shooterNormGood() {
        double shooterError = Math.abs(currentTargetExitSpeedMps - curExitSpeedMps);
//        if(distState == Dist.FAR) {
//            boolean hoodValid = turretPosGoalDistIn < generalParams.enableHoodCheckDist || Math.abs(hoodExitAngleRad - idealBallExitAng) < generalParams.maxDynamicHoodError;
//            return robot.shootingSystem.physicsExitAngleRads[0] != -1 && hoodValid && shooterError < generalParams.farShotTolerance;
//        }
        return shooterError < (locationState == Location.FAR ? generalParams.farShootToleranceMps : generalParams.closeShootToleranceMps);
    }
    public boolean inShootingZone() {
        Vector2d robotPos = robot.drive.pinpoint().getPose().position;
        return Math.hypot(robotPos.x - corner.x, robotPos.y - corner.y) > generalParams.shootingZoneRadius;
    }
    public boolean shooterFirstGood() {
        double shooterError = Math.abs(currentTargetExitSpeedMps - curExitSpeedMps);
        boolean shooterInRange = shooterError < generalParams.firstShootToleranceMps;
//        if(distState == Dist.NEAR || distState == Dist.MID)
        return shooterInRange;
//        return robot.shootingSystem.physicsExitAngleRads[0] != -1 && shooterInRange;
    }
    public boolean meetsSafetyInterlocks() {
        return robot.turret.onTarget() && shooterNormGood() && inShootingZone();
    }
    public double getShooterHighPower() {
        return shooterHighMotor.getPower();
    }
    public double getShooterLowPower() {
        return shooterLowMotor.getPower();
    }
    public double getShooterHighRawVelTps() {
        return shooterHighMotor.getVelTps();
    }
    public double getShooterLowRawVelTps() {
        return shooterLowMotor.getVelTps();
    }
    public double getPrevFilteredShooterSpeedTps() {
        return prevFilteredShooterSpeedTps;
    }
    public double getFilteredShooterSpeedTps() {
        return filteredShooterSpeedTps;
    }
    public double getRawShooterSpeedTps() {
        return rawShooterSpeedTps;
    }

    public void setHoodPosition(double p) {
        hoodLeftServo.setPosition(p);
        hoodRightServo.setPosition(p);
    }
    public double getHoodPosition() {
        return hoodLeftServo.getPosition();
    }
    public void drawShootingInfo(Canvas fieldOverlay) {
        // draw goal and shooting rings
        fieldOverlay.setStroke("yellow");
        fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.z, 3);
        if (testingParams.drawShootingRings) {
            fieldOverlay.setAlpha(0.4);
            fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.z, goalParams.gateLocationYThreshold); // end of near range, start of far range
            fieldOverlay.setAlpha(1);
        }

        fieldOverlay.setStroke("red");
        Drawing.drawCirclePose(fieldOverlay, turretPose, 5);

        fieldOverlay.setStroke("purple");
        double dist = 300;
        fieldOverlay.strokeLine(
                turretPose.position.x,
                turretPose.position.y,
                turretPose.position.x + dist * Math.cos(robot.turret.currentAbsoluteAngleRad),
                turretPose.position.y + dist * Math.sin(robot.turret.currentAbsoluteAngleRad)
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                turretPose.position.x,
                turretPose.position.y,
                turretPose.position.x + dist * Math.cos(lookAheadTurretTargetAngleRad),
                turretPose.position.y + dist * Math.sin(lookAheadTurretTargetAngleRad)
        );
        fieldOverlay.setStroke("red");
        double a = robot.drive.pinpoint().getPose().heading.toDouble() + robot.turret.targetEncoder / Turret.turretParams.ticksPerRad;
        fieldOverlay.strokeLine(
                turretPose.position.x,
                turretPose.position.y,
                turretPose.position.x + dist * Math.cos(a),
                turretPose.position.y + dist * Math.sin(a)
        );

        if(robot.turret.perpVelVec != null) {
            fieldOverlay.setStroke("blue");
            fieldOverlay.strokeLine(
                    turretPose.position.x,
                    turretPose.position.y,
                    turretPose.position.x + robotVelAtTurretIps.x,
                    turretPose.position.y + robotVelAtTurretIps.y
            );
        }
    }
}