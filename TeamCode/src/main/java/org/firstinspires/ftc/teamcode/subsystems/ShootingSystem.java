package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.lookupDistsI;

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
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.Vector3d;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCacher;
import org.firstinspires.ftc.teamcode.utils.misc.ServoImplCacher;

import java.util.Arrays;

@Config
public class ShootingSystem {
    public static class TestingParams {
        public boolean usingLookup = false;
    }
    public static class GoalParams {
        public double nearRedX = -63, nearRedY = 63;
        public double midRedX = -65, midRedY = 65;
        public double farRedX = -66, farRedY = 65;
        public double nearBlueX = -66, nearBlueY = -65;
        public double midBlueX = -66, midBlueY = -65;
        public double farBlueX = -67, farBlueY = -63;
        public double nearHeight = 38, midHeight = 39, farHeight = 42;
        public double nearImpactAng = Math.toRadians(-30), midImpactAng = Math.toRadians(-25), farImpactAng = Math.toRadians(-24);
        public double nearStateThreshold = 58;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
        public double resolution = 0.005;
        public double robotVelThresholdToSetHood = 2;
    }
    public static class GeneralParams {
        public double firstShootTolerance = 0.1, physicsShootTolerance = 0.1;
        public double approxNearExitAngRad = Math.toRadians(53), approxFarExitAngRad = Math.toRadians(37);
        public int lookAheadAvgNum = 5;
        public double rawLookAheadTime = 0.25; // time to look ahead for pose prediction
        public double shooterTau = 0.2;
        public int numApproximations = 4;
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
        public double maxShootWhileMovingSpeed = 0.6;

        // estimated accel thresholds: position: 20, heading: 5
    }
    public static class JoystickVelParams {
        // theory: velocity will stabilize with constant joystick input over time
        public double xM = 0,  xB = 0;
        public double yM = 0, yB = 0;
        public double aM = 0, aB = 0;
    }
    public static TestingParams testingParams = new TestingParams();
    public static GoalParams goalParams = new GoalParams();
    public static HoodParams hoodParams = new HoodParams();
    public static GeneralParams generalParams = new GeneralParams();
    public static JoystickVelParams jvParams = new JoystickVelParams();

    public enum Dist {
        NEAR, MID, FAR
    }
    public Dist distState;
    public boolean usingHighArc;
    public final Vector3d nearGoalPos, midGoalPos, farGoalPos;

    private final HardwareMap hardwareMap;
    private final BrainSTEMRobot robot;
    private MotorCacher turretMotor, shooterLowMotor, shooterHighMotor;
    private ServoImplCacher hoodLeftServo, hoodRightServo;
    public final Vector2d corner;
    public Vector3d goalPosIn;
    public Vector2d futureExitPosRelativeToGoal;
    public double impactAngleRad;
    public Pose2d futureRobotPose;

    public double currentLookAhead;
    public double[] prevLookAheads;

    public Vector2d robotVelAtExitPosIps;
    public double robotSpeedAtExitPosIps;

    public Vector3d actualTargetExitVelMps;
    public double actualTurretTargetAngleRad;
    public double actualTargetExitSpeedMps;
    public double ballTargetExitSpeedMps;
    public double efficiencyCoef;

    public double filteredShooterSpeedTps, rawShooterSpeedTps;
    public double curExitSpeedMps;
    public double ballExitAngleRad, hoodExitAngleRad;
    public double[] physicsExitAngleRads;
    public Vector2d ballExitPos, futureBallExitPos;
    public double exitPosGoalDistIn, futureExitPosGoalDistIn;
    public Pose2d turretPose, futureTurretPose;

    private double curTimeMs;
    public double dt;
    public OdoInfo odoVel;
    public ShooterLookup lookupTable;
    public double relGoalHeightM;
    public boolean shootingWhileMoving;
    public ShootingSystem(HardwareMap hardwareMap, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        if(robot != null)
            ballExitPos = ShootingMath.getExitPositionInches(ShootingMath.getTurretPose(robot.drive.localizer.getPose(), 0), ballExitAngleRad);

        initTurret();
        initShooter();
        initHood();
        initMisc();
        curTimeMs = System.currentTimeMillis();
        lookupTable = new ShooterLookup();
        efficiencyCoef = 0.39;


        distState = Dist.NEAR;
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

        physicsExitAngleRads = new double[generalParams.numApproximations];
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
    private void initMisc() {
        prevLookAheads = new double[generalParams.lookAheadAvgNum];
        for(int i = 0; i < generalParams.lookAheadAvgNum; i++)
            prevLookAheads[i] = 0;
    }
    public void updateInfo(boolean useTurretLookAhead) {
        updateLookAheadTime(useTurretLookAhead);

        double prevTimeMs = curTimeMs;
        curTimeMs = System.currentTimeMillis();
        dt = (curTimeMs - prevTimeMs) / 1000;

        hoodLeftServo.updateProperties();
        hoodRightServo.updateProperties();

        Pose2d robotPose = robot.drive.localizer.getPose();
        futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(currentLookAhead);

        updateGoalProperties(robotPose.position);

        updateTurretProperties(robotPose);
        shooterHighMotor.updateInfo();
        shooterLowMotor.updateInfo();

        rawShooterSpeedTps = (shooterHighMotor.getVelTps() + shooterLowMotor.getVelTps()) * 0.5;
        if(filteredShooterSpeedTps == 0)
            filteredShooterSpeedTps = rawShooterSpeedTps;
        else {
            double a_s = generalParams.shooterTau == 0 ? 0 : Math.exp(-dt / generalParams.shooterTau);
            filteredShooterSpeedTps = filteredShooterSpeedTps * a_s + rawShooterSpeedTps * (1 - a_s);
        }

        double desiredBallDir = Math.atan2(goalPosIn.z - futureBallExitPos.y, goalPosIn.x - futureBallExitPos.x);

        if(testingParams.usingLookup)
            updateLookupProperties(desiredBallDir, robotVelAtExitPosIps);
        else
            updatePhysicsProperties(desiredBallDir, robotVelAtExitPosIps);
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird
    private void updatePhysicsProperties(double desiredBallDir, Vector2d robotExitPosVel) {
        shootingWhileMoving = distState == Dist.NEAR || distState == Dist.MID;
        // get delta y of projectory (need approximate exit height of the ball)
        double exitHeightM = ShootingMath.approximateExitHeightM(distState == Dist.NEAR);
        relGoalHeightM = (goalPosIn.y * 0.0254 - exitHeightM);
        double futureDist = futureExitPosGoalDistIn * 0.0254;

        double[] launchVector = ShootingMath.calculateLaunchVector(futureDist, relGoalHeightM, impactAngleRad);

        ballTargetExitSpeedMps = launchVector[0];
        ballExitAngleRad = launchVector[1];
        if(shootingWhileMoving) {
            if(robot.collection.getClutchState() == Collection.ClutchState.UNENGAGED)
                robotExitPosVel = new Vector2d(0, 0);
            actualTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, launchVector[1], launchVector[0], robotExitPosVel);
            double baseLength = Math.hypot(actualTargetExitVelMps.x, actualTargetExitVelMps.z);
            hoodExitAngleRad = Range.clip(Math.atan2(actualTargetExitVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
            actualTurretTargetAngleRad = Math.atan2(actualTargetExitVelMps.z, actualTargetExitVelMps.x);

            efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
            curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
            actualTargetExitSpeedMps = Math.hypot( baseLength, actualTargetExitVelMps.y );
        }
        else {
            efficiencyCoef = calcEfficiencyCoef(launchVector[1]); // initial guess for efficiency coefficient
            curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef); // initial guess for current shooter speed

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
                curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

                for (int i = 1; i < generalParams.numApproximations; i++) {
                    physicsExitAngleRads[i] = ShootingMath.calculateBallExitAngleRad(usingHighArc, relGoalHeightM, futureDist, curExitSpeedMps);
                    if (physicsExitAngleRads[i] == -1) {
                        for (int j = i + 1; j < generalParams.numApproximations; j++)
                            physicsExitAngleRads[j] = -1;
                        break;
                    }
                    ballExitAngleRad = physicsExitAngleRads[i];
                    efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                    curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
                }
            } else
                Arrays.fill(physicsExitAngleRads, -1);


            // old code with no flexibility for shooting while moving
            actualTargetExitSpeedMps = ballTargetExitSpeedMps;
            hoodExitAngleRad = ballExitAngleRad;
            actualTurretTargetAngleRad = desiredBallDir;
        }
    }

    // pro: easy to tune
    // con: no velocity-based hood adjustment
    private void updateLookupProperties(double desiredBallDir, Vector2d robotVel) {
        // getting lookup properties
        double lookupDist = Range.clip(exitPosGoalDistIn, lookupDistsI[0] + 0.01, lookupDistsI[lookupDistsI.length-1] - 0.01);
        ballExitAngleRad = lookupTable.lookupExitAngleRad(lookupDist);
        ballTargetExitSpeedMps = lookupTable.lookupVelocityMetersPerSec(lookupDist);

        // allows for shooting while moving
        actualTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, ballTargetExitSpeedMps, robotVel);
        double baseLength = Math.hypot(actualTargetExitVelMps.x, actualTargetExitVelMps.z);
        hoodExitAngleRad = Range.clip(Math.atan2(actualTargetExitVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
        actualTurretTargetAngleRad = Math.atan2(actualTargetExitVelMps.z, actualTargetExitVelMps.x);

        efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
        curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
        actualTargetExitSpeedMps = Math.hypot( baseLength, actualTargetExitVelMps.y );
    }
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

        turretPose = ShootingMath.getTurretPose(robotPose, robot.turret.curRelAngleRad);
        futureTurretPose = new Pose2d(ShootingMath.getTurretPose(futureRobotPose, 0).position, Math.atan2(goalPosIn.z, goalPosIn.x));

        double approxBallExitAng = distState == Dist.FAR ? generalParams.approxFarExitAngRad : generalParams.approxNearExitAngRad;
        ballExitPos = ShootingMath.getExitPositionInches(turretPose, approxBallExitAng);
        futureBallExitPos = ShootingMath.getExitPositionInches(futureTurretPose, approxBallExitAng);

        odoVel = robot.drive.pinpoint().getMostRecentVelocity();
        Vector2d robotVelCm = new Vector2d(odoVel.x, odoVel.y);
        Vector2d relativeExitPos = ballExitPos.minus(robotPose.position);
        Vector2d robotTanVel = new Vector2d(-relativeExitPos.y, relativeExitPos.x*1).times(odoVel.headingRad); // v = r * w
        robotVelAtExitPosIps = robotVelCm.plus(robotTanVel);
        robotSpeedAtExitPosIps = Math.hypot(robotVelAtExitPosIps.x, robotVelAtExitPosIps.y);
        robotSpeedAtExitPosIps = Math.hypot(robotVelAtExitPosIps.x, robotVelAtExitPosIps.y);

        double deltaX = goalPosIn.x - ballExitPos.x;
        double deltaY = goalPosIn.z - ballExitPos.y;
        exitPosGoalDistIn = Math.hypot(deltaX, deltaY);

        futureExitPosRelativeToGoal = new Vector2d(futureBallExitPos.x - goalPosIn.x, futureBallExitPos.y - goalPosIn.z);
        futureExitPosGoalDistIn = Math.hypot(futureExitPosRelativeToGoal.x, futureExitPosRelativeToGoal.y);
    }

    private void updateLookAheadTime(boolean useLookAhead) {
        double rawLookAhead = useLookAhead ? generalParams.rawLookAheadTime : 0;
        for(int i = prevLookAheads.length-1; i > 0; i--)
            prevLookAheads[i] = prevLookAheads[i-1];
        prevLookAheads[0] = rawLookAhead;
        currentLookAhead = getAvgLookAheads();
    }
    private double getAvgLookAheads() {
        double sum = 0;
        for (double prevLookAhead : prevLookAheads) sum += prevLookAhead;
        return sum / prevLookAheads.length;
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
        telemetry.addData("----raw x vel", robot.drive.pinpoint().getMostRecentVelocity().x);
        telemetry.addData("----raw y vel", robot.drive.pinpoint().getMostRecentVelocity().y);
        telemetry.addData("----raw heading vel", robot.drive.pinpoint().getMostRecentVelocity().headingRad);
        telemetry.addData("-----raw x accel", robot.drive.pinpoint().getMostRecentAcceleration().x);
        telemetry.addData("-----raw y accel", robot.drive.pinpoint().getMostRecentAcceleration().y);
        telemetry.addData("-----raw heading accel", robot.drive.pinpoint().getMostRecentAcceleration().headingRad);

        telemetry.addData("shooting while moving", shootingWhileMoving);
        telemetry.addData("efficiency coef", efficiencyCoef);
        telemetry.addData("absolute turret target rad", actualTurretTargetAngleRad);
        telemetry.addData("absolute turret target deg", Math.toDegrees(actualTurretTargetAngleRad));
        telemetry.addData("robot-relative target exit speed mps", actualTargetExitSpeedMps);
        telemetry.addData("ball exit angle rad", ballExitAngleRad);
        telemetry.addData("robot speed at exit pos", robotSpeedAtExitPosIps);
        telemetry.addData("physics exit angle rad", MathUtils.format3(physicsExitAngleRads));
        telemetry.addData("ball meters from goal", exitPosGoalDistIn * 0.0254);
        telemetry.addData("ball inches from goal", exitPosGoalDistIn);
        telemetry.addData("future ball meters from goal", futureExitPosGoalDistIn * 0.0254);
        telemetry.addData("future ball inches from goal", futureExitPosGoalDistIn);
        telemetry.addData("absolute target exit speed mps", ballTargetExitSpeedMps);
        telemetry.addData("dt", dt);
        telemetry.addData("--------pinpoint dt", robot.drive.pinpoint().dt);
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
        double power = voltage / robot.getFilteredVoltage();
        turretMotor.setPower(power);
    }
    public double getTurretPower() {
        return turretMotor.getPower();
    }
    public double getTurretVelTps() {
        return turretMotor.getVelTps();
    }
    public void setShooterPower(double p) {
        shooterHighMotor.setPower(p);
        shooterLowMotor.setPower(p);
    }
    public double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = generalParams.efficiencyCoefM * ballExitAngleRad + generalParams.efficiencyCoefB;
        return Range.clip(generalParams.minEfficiencyCoef, rawE, generalParams.maxEfficiencyCoef);
    }
    public double getShooterErrorMps() {
        return actualTargetExitSpeedMps - curExitSpeedMps;
    }
    public boolean shooterGood() {
        double shooterError = getShooterErrorMps();
        return (ShootingSystem.testingParams.usingLookup ? shooterError <= generalParams.firstShootTolerance : robot.shootingSystem.physicsExitAngleRads[0] != -1 || shooterError < generalParams.physicsShootTolerance);
    }

    public void setShooterPowerRaw(double p) {
        shooterHighMotor.setPowerRaw(p);
        shooterLowMotor.setPowerRaw(p);
    }
    public double getShooterPower() {
        return shooterHighMotor.getPower();
    }
    public double getShooterHighVelTps() {
        return shooterHighMotor.getVelTps();
    }
    public double getShooterLowVelTps() {
        return shooterLowMotor.getVelTps();
    }
    public double getPrevShooterVelTps() {
        return (shooterHighMotor.getPrevVelTps() + shooterLowMotor.getPrevVelTps()) * 0.5;
    }

    public void setHoodPosition(double p) {
        hoodLeftServo.setPosition(p);
        hoodRightServo.setPosition(p);
    }
    public double getHoodPosition() {
        return hoodLeftServo.getPosition();
    }
}