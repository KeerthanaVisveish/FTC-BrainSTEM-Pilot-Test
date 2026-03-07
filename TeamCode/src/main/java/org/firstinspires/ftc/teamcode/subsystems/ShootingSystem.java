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
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.Vector3d;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCacher;
import org.firstinspires.ftc.teamcode.utils.misc.ServoImplCacher;

import java.util.Arrays;

@Config
public class ShootingSystem {
    public static class TestingParams {
        public boolean powerTurret = true;
        public boolean powerLowShooter = true, powerHighShooter = true;
        public boolean drawShootingRings = true;
    }
    public static class GoalParams {
        public double nearRedX = -64, nearRedY = 66, nearHeight = 39;
        public double nearBlueX = -64, nearBlueY = -64;
        public double midRedX = -64, midRedY = 68, midHeight = 38;
        public double midBlueX = -65.25, midBlueY = -64;
        public double farRedX = -65, farRedY = 66, farHeight = 42;
        public double farBlueX = -65.75, farBlueY = -64;
        public double nearImpactAng = Math.toRadians(-25), midImpactAng = Math.toRadians(-24), farImpactAng = Math.toRadians(-24);
        public double nearStateThreshold = 58;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
        public double resolution = 0.005;
        public double robotVelThresholdToSetHood = 2;
    }
    public static class GeneralParams {
        public double robotVelNoiseThreshold = .1;
        public double far1ExitAng = Math.toRadians(38);
        public double far2SwitchY = 8;
        public double far2ExitAng = Math.toRadians(36);
        public double maxShootingDist = 180;
        public double maxDynamicHoodError = Math.toRadians(12), enableHoodCheckDist = 146;
        public double firstShootToleranceMps = 0.1, normShootToleranceMps = 0.3;
        public double lookAheadTime = 0.15; // time to look ahead for pose prediction
        public double shooterTau = 0.1;
        public int numApproximations = 4;
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
        public double maxShootWhileMovingSpeed = 0.6;

        // estimated accel thresholds: position: 20, heading: 5
    }
    public static TestingParams testingParams = new TestingParams();
    public static GoalParams goalParams = new GoalParams();
    public static HoodParams hoodParams = new HoodParams();
    public static GeneralParams generalParams = new GeneralParams();

    public enum Dist {
        NEAR, MID, FAR
    }
    public Dist distState;
    public boolean usingHighArc;
    public Vector3d nearGoalPos, midGoalPos, farGoalPos;
    public Vector2d corner;
    private final HardwareMap hardwareMap;
    private final BrainSTEMRobot robot;
    private MotorCacher turretMotor, shooterLowMotor, shooterHighMotor;
    private ServoImplCacher hoodLeftServo, hoodRightServo;
    public Vector3d goalPosIn;
    public Vector2d futureTurretPosRelativeToGoal;
    public double impactAngleRad;
    public Pose2d futureRobotPose;

    public Vector2d robotVelAtTurretIps, lookAheadRobotVelAtTurretIps;
    public double robotSpeedAtTurretIps, lookAheadRobotSpeedAtTurretIps;

    public Vector3d lookAheadTargetExitVelMps;
    public double lookAheadTurretTargetAngleRad, currentTurretTargetAngleRad;
    public double lookAheadTargetExitSpeedMps, currentTargetExitSpeedMps;
    public double ballTargetExitSpeedMps, idealBallExitAng;
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
    public ShootingSystem(HardwareMap hardwareMap, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

//        if(robot != null)
//            ballExitPos = ShootingMath.getExitPositionInches(ShootingMath.getTurretPose(robot.drive.localizer.getPose(), 0), ballExitAngleRad);

        initTurret();
        initShooter();
        initHood();
        curTimeMs = System.currentTimeMillis();
        lookupTable = new ShooterLookup();
        efficiencyCoef = 0.39;


        distState = Dist.NEAR;

        physicsExitAngleRads = new double[generalParams.numApproximations];
        updateGoalPoses();
    }
    public void updateGoalPoses() {
        if(BrainSTEMRobot.alliance == Alliance.BLUE) {
            corner = new Vector2d(-72, -72);
            nearGoalPos = new Vector3d(goalParams.nearBlueX, goalParams.nearHeight, goalParams.nearBlueY);
            midGoalPos = new Vector3d(goalParams.midBlueX, goalParams.midHeight, goalParams.midBlueY);
            farGoalPos = new Vector3d(goalParams.farBlueX, goalParams.farHeight, goalParams.farBlueY);
        }
        else {
            corner = new Vector2d(-72, 72);
            nearGoalPos = new Vector3d(goalParams.nearRedX, goalParams.nearHeight, goalParams.nearRedY);
            midGoalPos = new Vector3d(goalParams.midRedX, goalParams.midHeight, goalParams.midRedY);
            farGoalPos = new Vector3d(goalParams.farRedX, goalParams.farHeight, goalParams.farRedY);
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
    public void updateInfo(boolean useTurretLookAhead) {
        updateGoalPoses();

        double prevTimeMs = curTimeMs;
        curTimeMs = System.currentTimeMillis();
        dt = (curTimeMs - prevTimeMs) / 1000;

        hoodLeftServo.updateProperties();
        hoodRightServo.updateProperties();

        Pose2d robotPose = robot.drive.localizer.getPose();
        futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(generalParams.lookAheadTime);

        updateGoalProperties(robotPose.position);

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

//        desiredBallDir = Math.atan2(goalPosIn.z - futureBallExitPos.y, goalPosIn.x - futureBallExitPos.x);
        desiredBallDir = Math.atan2(goalPosIn.z - futureTurretPos.y, goalPosIn.x - futureTurretPos.x);

        Vector2d robotVelAtTurretMps = robotVelAtTurretIps.times(.0254);
        Vector2d lookAheadRobotVelAtTurretMps = lookAheadRobotVelAtTurretIps.times(.0254);
//        if(testingParams.usingLookup)
//            updateLookupProperties(desiredBallDir, robotVelAtTurretMps);
//        else
            updatePhysicsProperties(desiredBallDir, robotVelAtTurretMps, lookAheadRobotVelAtTurretMps);
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird
    private void updatePhysicsProperties(double desiredBallDir, Vector2d noLookAheadRobotVelAtTurretMps, Vector2d lookAheadRobotVelAtTurretMps) {
        checkShootingWhileMoving = distState == Dist.NEAR || distState == Dist.MID;
        // get delta y of projectory (need approximate exit height of the ball)
        double exitHeightM = ShootingMath.approximateExitHeightM(distState == Dist.NEAR);
        relGoalHeightM = (goalPosIn.y * 0.0254 - exitHeightM);
        double futureDist = futureTurretPosGoalDistIn * 0.0254;

        double[] launchVector;

        if(checkShootingWhileMoving) {
            launchVector = ShootingMath.calculateLaunchVectorWithImpactAngle(futureDist, relGoalHeightM, impactAngleRad);
            ballTargetExitSpeedMps = launchVector[0];
            ballExitAngleRad = launchVector[1];

            currentlyShootingWhileMoving = robot.collection.getClutchState() == Collection.ClutchState.ENGAGED;
            if(!currentlyShootingWhileMoving) {
                noLookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
                lookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
            }
            lookAheadTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, launchVector[1], launchVector[0], lookAheadRobotVelAtTurretMps);
            double baseLength = Math.hypot(lookAheadTargetExitVelMps.x, lookAheadTargetExitVelMps.z);
            hoodExitAngleRad = Range.clip(Math.atan2(lookAheadTargetExitVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
            lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);

            efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
            curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
            lookAheadTargetExitSpeedMps = Math.hypot( baseLength, lookAheadTargetExitVelMps.y );

            double noLookAheadDist = turretPosGoalDistIn * 0.0254;
            double[] noLookAheadLaunchVector = ShootingMath.calculateLaunchVectorWithImpactAngle(noLookAheadDist, relGoalHeightM, impactAngleRad);
            double noLookAheadBallDir = Math.atan2(goalPosIn.z - turretPose.position.y, goalPosIn.x - turretPose.position.x);
            Vector3d noLookAheadTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(noLookAheadBallDir, noLookAheadLaunchVector[1], noLookAheadLaunchVector[0], noLookAheadRobotVelAtTurretMps);
            currentTurretTargetAngleRad = Math.atan2(noLookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
            currentTargetExitSpeedMps = Math.hypot( Math.hypot(noLookAheadTargetExitVelMps.x, noLookAheadTargetExitVelMps.y), noLookAheadTargetExitVelMps.z );
        }
        else {
            futureDist = Math.min(futureDist, generalParams.maxShootingDist);
            int sign = BrainSTEMRobot.alliance == Alliance.RED ? -1 : 1;
            double farExitAng = robot.drive.localizer.getPose().position.y * sign > generalParams.far2SwitchY ? generalParams.far2ExitAng : generalParams.far1ExitAng;
            launchVector = new double[] {ShootingMath.calculateLaunchVelocityWithExitAngle(futureDist, relGoalHeightM, farExitAng), hoodParams.minExitAngRad};
            ballTargetExitSpeedMps = launchVector[0];
            idealBallExitAng = launchVector[1];
            ballExitAngleRad = idealBallExitAng;

            currentlyShootingWhileMoving = false;
            efficiencyCoef = calcEfficiencyCoef(launchVector[1]); // initial guess for efficiency coefficient
            curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef) - robot.shooter.getCurVelocityAdjustment(); // initial guess for current shooter speed

            // determining whether to use high arc or low arc
            double highArcExitAng = ShootingMath.calculateBallExitAngleRad(true, relGoalHeightM, futureDist, curExitSpeedMps);
            if (highArcExitAng != -1) {
                double lowArcExitAng = ShootingMath.calculateBallExitAngleRad(false, relGoalHeightM, futureDist, curExitSpeedMps);
                double highArcImpactAng = ShootingMath.calculateImpactAngle(futureDist, relGoalHeightM, curExitSpeedMps, highArcExitAng);
                double lowArcImpactAng = ShootingMath.calculateImpactAngle(futureDist, relGoalHeightM, curExitSpeedMps, lowArcExitAng);
                usingHighArc = Math.abs(highArcImpactAng - impactAngleRad) < Math.abs(lowArcImpactAng - impactAngleRad);

                // estimating hood ang and current shooter speed
                physicsExitAngleRads[0] = usingHighArc ? highArcExitAng : lowArcExitAng;
                ballExitAngleRad = physicsExitAngleRads[0];
                efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef) - robot.shooter.getCurVelocityAdjustment();

                for (int i = 1; i < generalParams.numApproximations; i++) {
                    physicsExitAngleRads[i] = ShootingMath.calculateBallExitAngleRad(usingHighArc, relGoalHeightM, futureDist, curExitSpeedMps);
                    if (physicsExitAngleRads[i] == -1) {
                        for (int j = i + 1; j < generalParams.numApproximations; j++)
                            physicsExitAngleRads[j] = -1;
                        break;
                    }
                    ballExitAngleRad = physicsExitAngleRads[i];
                    efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                    curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef) - robot.shooter.getCurVelocityAdjustment();
                }
            } else
                Arrays.fill(physicsExitAngleRads, -1);


            lookAheadTargetExitSpeedMps = ballTargetExitSpeedMps;
            hoodExitAngleRad = ballExitAngleRad;

            // basic estimation of turret angle to try account for shooting while moving
            if(robotSpeedAtTurretIps < generalParams.robotVelNoiseThreshold || robot.shooter.getShooterState() == Shooter.ShooterState.OFF) {
                lookAheadTurretTargetAngleRad = desiredBallDir;
                currentTurretTargetAngleRad = desiredBallDir;
            }
            else {
                lookAheadTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, curExitSpeedMps, lookAheadRobotVelAtTurretMps);
                lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
                Vector3d noLookAheadTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, curExitSpeedMps, noLookAheadRobotVelAtTurretMps);
                currentTurretTargetAngleRad = Math.atan2(noLookAheadTargetExitVelMps.z, noLookAheadTargetExitVelMps.x);
            }
            currentTargetExitSpeedMps = ballTargetExitSpeedMps;
        }
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

        turretPose = ShootingMath.getTurretPose(robotPose, robot.turret.curRelAngleRad);
        futureTurretPos = ShootingMath.getTurretPose(futureRobotPose, 0).position;

//        double approxBallExitAng = distState == Dist.FAR ? generalParams.approxFarExitAngRad : generalParams.approxNearExitAngRad;
//        ballExitPos = ShootingMath.getExitPositionInches(idealTurretPose, approxBallExitAng);
//        futureBallExitPos = ShootingMath.getExitPositionInches(futureTurretPose, approxBallExitAng);

        robotVelCm = robot.drive.pinpoint().getVelocity();
        Vector2d robotVelCm = new Vector2d(this.robotVelCm.x, this.robotVelCm.y);

        Vector2d relativeTurretPos = turretPose.position.minus(robotPose.position);
        Vector2d robotTanVel = new Vector2d(-relativeTurretPos.y, relativeTurretPos.x*1).times(this.robotVelCm.headingRad); // v = r * w
        robotVelAtTurretIps = robotVelCm.plus(robotTanVel);
        robotSpeedAtTurretIps = Math.hypot(robotVelAtTurretIps.x, robotVelAtTurretIps.y);

        Vector2d lookAheadRelativeTurretPos = futureTurretPos.minus(futureRobotPose.position);
        Vector2d lookAheadRobotTanVel = new Vector2d(-lookAheadRelativeTurretPos.y, lookAheadRelativeTurretPos.x*1).times(this.robotVelCm.headingRad);
        lookAheadRobotVelAtTurretIps = robotVelCm.plus(lookAheadRobotTanVel);
        lookAheadRobotSpeedAtTurretIps = Math.hypot(lookAheadRobotVelAtTurretIps.x, lookAheadRobotVelAtTurretIps.y);

//        double deltaX = goalPosIn.x - ballExitPos.x;
//        double deltaY = goalPosIn.z - ballExitPos.y;
//        exitPosGoalDistIn = Math.hypot(deltaX, deltaY);

        futureTurretPosRelativeToGoal = new Vector2d(futureTurretPos.x - goalPosIn.x, futureTurretPos.y - goalPosIn.z);
        futureTurretPosGoalDistIn = Math.hypot(futureTurretPosRelativeToGoal.x, futureTurretPosRelativeToGoal.y);
        turretPosGoalDistIn = Math.hypot(turretPose.position.x - goalPosIn.x, turretPose.position.y - goalPosIn.z);
    }
    private void updateGoalProperties(Vector2d robotPos) {
        double distToCorner = Math.hypot(corner.x - robotPos.x, corner.y - robotPos.y);
        if(robotPos.x > 24) {
            distState = Dist.FAR;
            goalPosIn = farGoalPos;
            impactAngleRad = goalParams.farImpactAng;
        }
        else if(distToCorner > goalParams.nearStateThreshold) {
            distState = Dist.MID;
            goalPosIn = midGoalPos;
            impactAngleRad = goalParams.midImpactAng;
        }
        else {
            distState = Dist.NEAR;
            goalPosIn = nearGoalPos;
            impactAngleRad = goalParams.nearImpactAng;
        }
    }

    public void printInfo(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("SHOOTING SYSTEM-------");
        telemetry.addData("robot vel center of mass", robotVelCm.toString(3));
        telemetry.addData("future turret pos relative to goal", MathUtils.formatVec3(futureTurretPosRelativeToGoal));
        telemetry.addData("robot vel at turret ips", MathUtils.formatVec3(robotVelAtTurretIps));

        telemetry.addData("checking shooting while moving", checkShootingWhileMoving);
        telemetry.addData("currently shooting while moving", currentlyShootingWhileMoving);
        telemetry.addData("efficiency coef", efficiencyCoef);
        telemetry.addData("absolute turret target rad", lookAheadTurretTargetAngleRad);
        telemetry.addData("absolute turret target deg", Math.toDegrees(lookAheadTurretTargetAngleRad));
        telemetry.addData("robot-relative target exit speed mps", lookAheadTargetExitSpeedMps);
        telemetry.addData("ball exit angle deg", Math.toDegrees(ballExitAngleRad));
        telemetry.addData("ideal ball exit angle deg", Math.toDegrees(idealBallExitAng));
        telemetry.addData("robot speed at turret", robotSpeedAtTurretIps);
        telemetry.addData("physics exit angle rad", MathUtils.format3(physicsExitAngleRads));
//        telemetry.addData("ball meters from goal", exitPosGoalDistIn * 0.0254);
//        telemetry.addData("ball inches from goal", exitPosGoalDistIn);
        telemetry.addData("future turret meters from goal", futureTurretPosGoalDistIn * 0.0254);
        telemetry.addData("future turret inches from goal", futureTurretPosGoalDistIn);
        telemetry.addData("absolute target exit speed mps", ballTargetExitSpeedMps);
        telemetry.addData("robot speed at turret", robotSpeedAtTurretIps);
        telemetry.addData("lookahead robot speed at turret", lookAheadRobotSpeedAtTurretIps);
        telemetry.addData("dt", dt);
        telemetry.addLine();
        telemetry.addData("rel height to target meters", relGoalHeightM);
        telemetry.addData("dist state", distState);
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
        double power = voltage / robot.getFilteredVoltage();
        setShooterPower(power);
    }
    public double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = generalParams.efficiencyCoefM * ballExitAngleRad + generalParams.efficiencyCoefB;
        return Range.clip(generalParams.minEfficiencyCoef, rawE, generalParams.maxEfficiencyCoef);
    }
    public boolean shooterNormGood() {
        if(distState == Dist.FAR) {
            boolean hoodValid = turretPosGoalDistIn < generalParams.enableHoodCheckDist || Math.abs(hoodExitAngleRad - idealBallExitAng) < generalParams.maxDynamicHoodError;
            return robot.shootingSystem.physicsExitAngleRads[0] != -1 && hoodValid;
        }
        double shooterError = Math.abs(currentTargetExitSpeedMps - (curExitSpeedMps - robot.shooter.getCurVelocityAdjustment()));
        return shooterError < generalParams.normShootToleranceMps;
    }
    public boolean shooterFirstGood() {
        double shooterError = Math.abs(currentTargetExitSpeedMps - (curExitSpeedMps - robot.shooter.getCurVelocityAdjustment()));
        boolean shooterInRange = shooterError < generalParams.firstShootToleranceMps;
        if(distState == Dist.NEAR || distState == Dist.MID)
            return shooterInRange;
        return robot.shootingSystem.physicsExitAngleRads[0] != -1 && shooterInRange;
    }
    public boolean meetsSafetyInterlocks() {
        return robot.turret.inRange() && robot.turret.onTarget() && robot.shootingSystem.shooterNormGood();
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
        fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.y, 3);
        if (testingParams.drawShootingRings) {
            fieldOverlay.setAlpha(0.4);
            fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.y, goalParams.nearStateThreshold); // end of near range, start of far range
            fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.y, generalParams.enableHoodCheckDist); // start of "hella far" range
            fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.y, generalParams.maxShootingDist); // max shooting distance
            fieldOverlay.setAlpha(1);
        }

        fieldOverlay.setStroke("green");
        Drawing.drawRobotSimple(fieldOverlay, turretPose, 5);

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
        fieldOverlay.strokeLine(
                turretPose.position.x,
                turretPose.position.y,
                turretPose.position.x + dist * Math.cos(desiredBallDir),
                turretPose.position.y + dist * Math.sin(desiredBallDir)
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