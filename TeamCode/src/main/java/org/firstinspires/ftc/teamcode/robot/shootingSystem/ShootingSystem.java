package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt1;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt2;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.util.function.DoubleSupplier;
import java.util.function.ToDoubleFunction;

@Config
public class ShootingSystem extends Component {
    public static class GoalParams {
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

        public double reallyCloseRadius = 45;
        public double gateLocationYThreshold = -12, gateLocationXThreshold = -50;
    }
    public static class GeneralParams {
        public double firstShootToleranceTps = 40, closeShootToleranceTps = 80, farShootToleranceTps = 40;
        public double farTurretTol = Math.toRadians(3), nearTurretTol = Math.toRadians(10);
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
        public double shooterLookAhead = 0;
    }
    public static GoalParams goalParams = new GoalParams();
    public static GeneralParams generalParams = new GeneralParams();
    public enum Location {
        GATE_CYCLE, NEAR_WALL, REALLY_CLOSE, FAR
    }
    private Location locationState;
    private Vector3d goalPosIn;
    private double impactAngleRad;

    private double turretAngleAdjustment, shooterSpeedAdjustment;

    private double turretTargetAngle, turretGoalTargetAngle, turretGoalLockOnVelocity;
    private double lookAheadTargetExitSpeedTps, currentTargetExitSpeedTps;


    private double hoodExitAngleRad;
    private Pose2d turretPoseIn;
    private Pose2d clippedRobotPoseIn, clippedTurretPoseIn;
    public final Vector2d nearTriangleTip = new Vector2d(RobotProperties.length * Math.sqrt(2), 0);
    public final Vector2d farTriangleTip = new Vector2d(48 - RobotProperties.length * Math.sqrt(2), 0);
    private final ShootingMath shootingMathNew;

    public enum TurretState {
        GOAL_TARGETING, ANGLE_TARGETING
    }
    private TurretState turretState;

    // data for angle targeting mode
    private double angleTargetingMinVoltage = 0;
    private boolean angleTargetingPassPosition = false;
    private double angleTargetingInitialDir;

    public enum ShooterState {
        OFF, GOAL_TARGETING, CUSTOM_VOLTAGE;
    }
    private ShooterState shooterState;

    private double customShooterVoltage = 0;

    public enum HoodState {
        GOAL_TARGETING, CUSTOM_ANGLE;
    }
    private HoodState hoodState;

    private double customHoodAngle = Math.toRadians(45); // default angle

    public final Shooter shooter;
    public final Hood hood;
    public final Turret turret;
    public ShootingSystem(HardwareMap hardwareMap, Telemetry telemetry, Pose2d robotPose, Alliance alliance) {
        super(hardwareMap, telemetry);

        shooter = new Shooter(hardwareMap, telemetry);
        hood = new Hood(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);

        setTurretToCenter();
        setShooterOff();
        setHoodToGoalTargeting();

        shootingMathNew = new ShootingMath();

        locationState = Location.GATE_CYCLE;

        updateGoalPoses(robotPose.position, alliance);
    }


    private void updateGoalPoses(Vector2d robotPos, Alliance alliance) {
        Vector2d corner;
        Vector3d closeGoalPos;
        Vector3d farGoalPos;
        Vector3d gateGoalPos;
        Vector3d oppositeGoalPos;
        if(alliance == Alliance.BLUE) {
            corner = new Vector2d(-72, -72);
            closeGoalPos = new Vector3d(goalParams.closeBlueX, goalParams.closeBlueY, goalParams.closeHeight);
            gateGoalPos = new Vector3d(goalParams.gateBlueX, goalParams.gateBlueY, goalParams.gateHeight);
            oppositeGoalPos = new Vector3d(goalParams.oppositeBlueX, goalParams.oppositeBlueY, goalParams.oppositeHeight);
            farGoalPos = new Vector3d(goalParams.farBlueX, goalParams.farBlueY, goalParams.farHeight);
        }
        else {
            corner = new Vector2d(-72, 72);
            closeGoalPos = new Vector3d(goalParams.closeRedX, goalParams.closeRedY, goalParams.closeHeight);
            gateGoalPos = new Vector3d(goalParams.gateRedX, goalParams.gateRedY, goalParams.gateHeight);
            oppositeGoalPos = new Vector3d(goalParams.oppositeRedX, goalParams.oppositeRedY, goalParams.oppositeHeight);
            farGoalPos = new Vector3d(goalParams.farRedX, goalParams.farRedY, goalParams.farHeight);
        }

        double sign = alliance == Alliance.RED ? 1 : -1;
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
            locationState = Location.NEAR_WALL;
            goalPosIn = oppositeGoalPos;
            impactAngleRad = goalParams.oppositeImpactAng;
        }
        else {
            locationState = Location.GATE_CYCLE;
            goalPosIn = gateGoalPos;
            impactAngleRad = goalParams.gateImpactAng;
        }
    }

    private Vector2d clipVectorInsideLaunchLine(Vector2d pos, Vector2d perpLaunchLine, Vector2d triangleTip) {
        pos = pos.minus(triangleTip);
        return pos.minus(perpLaunchLine.times(Math.max(0, pos.dot(perpLaunchLine))));
    }
    private Vector2d[] getClippedPositions(Vector2d robotPosIn, Vector2d turretPosIn) {
        Vector2d clippedTurretPosIn, clippedRobotPosIn;
        if((locationState == Location.NEAR_WALL || locationState == Location.GATE_CYCLE) && Math.abs(robotPosIn.y) > 1e-6) {
            Vector2d perpLaunchLine = new Vector2d(1, Math.signum(robotPosIn.y)).div(Math.sqrt(2));
            clippedTurretPosIn = clipVectorInsideLaunchLine(turretPosIn, perpLaunchLine, nearTriangleTip);
            clippedRobotPosIn = clippedTurretPosIn.plus(robotPosIn.minus(turretPosIn));
        }
        else if(locationState == Location.FAR) {
            Vector2d perpLaunchLine = new Vector2d(-1, Math.signum(robotPosIn.y)).div(Math.sqrt(2));
            clippedTurretPosIn = clipVectorInsideLaunchLine(turretPosIn, perpLaunchLine, farTriangleTip);
            clippedRobotPosIn = clippedTurretPosIn.plus(robotPosIn.minus(turretPosIn));
        }
        else {
            clippedRobotPosIn = robotPosIn;
            clippedTurretPosIn = turretPosIn;
        }
        return new Vector2d[] {clippedRobotPosIn, clippedTurretPosIn};
    }
    private double calculateTurretGoalLockOnVelocity(Vector2d robotPosIn, Vector2d robotVel, double currentTargetGoalAngle) {
        double timeStep = .01;
        Vector2d futureDisp = robotVel.times(timeStep);
        Vector2d[] futureClippedPositions = getClippedPositions(robotPosIn.plus(futureDisp), turretPoseIn.position.plus(futureDisp));
        double[] futureLaunchData = calculateLaunchTrajectory(futureClippedPositions[0], futureClippedPositions[1], goalPosIn, robotVel);

        if(futureLaunchData != null && turretInRange())
            return (futureLaunchData[2] - currentTargetGoalAngle) / timeStep;
        else
            return 0;
    }
    public void updateSubsystems(Pose2d robotPoseIn, OdoInfo robotVelIn, OdoInfo robotAccel, double batteryVoltage, double dt, Alliance alliance) {
        updateGoalPoses(robotPoseIn.position, alliance);
        Vector2d robotVel = new Vector2d(robotVelIn.x, robotVelIn.y);

        turret.updateProperties(dt);
        turretPoseIn = ShootingMathOld.calcTurretPose(robotPoseIn, turret.getCurAngleRad());

        shooter.updateProperties();

        Vector2d[] clippedPoses = getClippedPositions(robotPoseIn.position, turretPoseIn.position);
        clippedRobotPoseIn = new Pose2d(clippedPoses[0], robotPoseIn.heading.toDouble());
        clippedTurretPoseIn = new Pose2d(clippedPoses[1], turretPoseIn.heading.toDouble());
        double[] launchData = calculateLaunchTrajectory(clippedRobotPoseIn.position, clippedTurretPoseIn.position, goalPosIn, robotVel);
        if(launchData != null) {
            lookAheadTargetExitSpeedTps = launchData[0];
            currentTargetExitSpeedTps = launchData[1];
            turretGoalTargetAngle = launchData[2];
            hoodExitAngleRad = launchData[3];
        }
        else
            throw new RuntimeException("launch data is null for some reason");

        double turretTargetVel;
        switch(turretState) {
            case GOAL_TARGETING:
                turretTargetAngle = MathUtils.angleNormDeltaRad(turretGoalTargetAngle - robotPoseIn.heading.toDouble() + turretAngleAdjustment);
                turretGoalLockOnVelocity = calculateTurretGoalLockOnVelocity(robotPoseIn.position, robotVel, turretGoalTargetAngle);
                turretTargetVel = turretGoalLockOnVelocity + turret.calculateMotionProfile(turretTargetAngle);
                turret.controlTurretToTarget(turretTargetAngle, turretTargetVel, 0, 0, turretInRange() ? Turret.powerTuning.maxVoltage : Turret.powerTuning.outOfRangeMaxVoltage, robotPoseIn.heading.toDouble(), robotAccel, batteryVoltage);
                break;
            case ANGLE_TARGETING:
                turretTargetVel = turret.calculateMotionProfile(turretTargetAngle);
                if(angleTargetingPassPosition && Math.signum(turretTargetAngle - turret.getCurAngleRad()) != angleTargetingInitialDir)
                    turret.setTurretPower(0);
                else
                    turret.controlTurretToTarget(turretTargetAngle, turretTargetVel, 0, angleTargetingMinVoltage, turretInRange() ? Turret.powerTuning.maxVoltage : Turret.powerTuning.outOfRangeMaxVoltage, robotPoseIn.heading.toDouble(), robotAccel, batteryVoltage);
                break;
        }


        switch (shooterState) {
            case OFF:
                shooter.setShooterVoltage(0, batteryVoltage);
                break;

            case GOAL_TARGETING:
                shooter.setShooterVelocityPID(lookAheadTargetExitSpeedTps + shooterSpeedAdjustment, shooter.getVelTps(), batteryVoltage);
                break;
            case CUSTOM_VOLTAGE:
                shooter.setShooterVoltage(customShooterVoltage, batteryVoltage);
        }

        switch (hoodState) {
            case GOAL_TARGETING:
                hood.setExitAngle(hoodExitAngleRad);
                break;
            case CUSTOM_ANGLE:
                hood.setExitAngle(customHoodAngle);
                break;

        }
    }

    private double[] calculateLaunchTrajectory(Vector2d robotPosIn, Vector2d turretPosIn, Vector3d goalPosIn, Vector2d robotVelocityIps) {
        Vector3d exitPosM = new Vector3d(turretPosIn.x, turretPosIn.y, ShootingMathOld.approximateExitHeightM(locationState == Location.GATE_CYCLE)).times(.0254);
        Vector3d robotPosM = new Vector3d(robotPosIn.x, robotPosIn.y, 0).times(.0254);
        Vector3d goalPosM = new Vector3d(goalPosIn.x, goalPosIn.y, goalPosIn.z).times(.0254);
        ToDoubleFunction<Double> shooterConversion = exitAngle -> {
            double e = calcEfficiencyCoef(exitAngle);
            return ShootingMathOld.ticksPerSecToExitSpeedMps(1, e);
        };
        Vector3d robotVelocityMps = new Vector3d(robotVelocityIps.x, robotVelocityIps.y, 0).times(.0254);

        AnswerKeyPt1 answerKeyPt1 = shootingMathNew.godSolvePart1(exitPosM, robotPosM, robotVelocityMps, 0, goalPosM, impactAngleRad, 0);
        AnswerKeyPt2 answerKeyPt2 = shootingMathNew.godSolvePart2(answerKeyPt1, goalPosM, impactAngleRad, shooter.getVelTps(), shooterConversion);

        if(answerKeyPt1.solutionExists) {
            double currentTargetExitSpeedTps = answerKeyPt1.launchData.speed;
            double lookAheadTargetExitSpeedTps = currentTargetExitSpeedTps + robotVelocityIps.dot(turretPosIn.minus(new Vector2d(goalPosIn.x, goalPosIn.y))) * generalParams.shooterLookAhead;

            double turretGoalTargetAngle, hoodExitAngleRad;
            if(answerKeyPt2.solutionExists) {
                turretGoalTargetAngle = answerKeyPt2.launchData.turretAng;
                hoodExitAngleRad = answerKeyPt2.launchData.exitAng;
            }
            else {
                turretGoalTargetAngle = answerKeyPt1.launchData.turretAng;
                hoodExitAngleRad = answerKeyPt1.launchData.exitAng;
            }
            return new double[] { lookAheadTargetExitSpeedTps, currentTargetExitSpeedTps, turretGoalTargetAngle, hoodExitAngleRad };
        }
        return null;
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird
//    private void updatePhysicsProperties(double desiredBallDir, Vector2d noLookAheadRobotVelAtTurretMps, Vector2d lookAheadRobotVelAtTurretMps) {
//        if(testingParams.dynamicHood)
//            checkShootingWhileMoving = locationState != Location.FAR;
//        else
//            checkShootingWhileMoving = true;
//        // get delta y of projectory (need approximate exit height of the ball)
//        double exitHeightM = ShootingMathOld.approximateExitHeightM(locationState == Location.REALLY_CLOSE);
//        relGoalHeightM = (goalPosIn.y * 0.0254 - exitHeightM);
//        double futureDist = futureTurretPosGoalDistIn * 0.0254;
//
//        double[] launchVector;
//
//        if(checkShootingWhileMoving) {
//            double sign = BrainSTEMRobot.alliance == Alliance.RED ? -1 : 1;
//            boolean inFar2 = robot.drive.pinpoint().getPose().position.y * sign > farParams.far2SwitchY;
//            if(locationState == Location.FAR) {
//                double exitAng =  inFar2 ? farParams.far2ExitAng : farParams.far1ExitAng;
//                launchVector = new double[] {
//                        ShootingMathOld.calculateLaunchVelocityWithExitAngle(futureDist, relGoalHeightM, exitAng),
//                        exitAng
//                };
//            }
//            else
//                launchVector = ShootingMathOld.calculateLaunchVectorWithImpactAngle(futureDist, relGoalHeightM, impactAngleRad);
//
//            ballTargetExitSpeedMps = launchVector[0];
//            ballExitAngleRad = launchVector[1];
//
//            currentlyShootingWhileMoving = robot.collector.getClutchState() == Collector.ClutchState.ENGAGED;
//            if(!currentlyShootingWhileMoving) {
//                noLookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
//                lookAheadRobotVelAtTurretMps = new Vector2d(0, 0);
//            }
//            lookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(desiredBallDir, launchVector[1], launchVector[0], lookAheadRobotVelAtTurretMps);
//            double baseLength = Math.hypot(lookAheadTargetExitVelMps.x, lookAheadTargetExitVelMps.z);
//
//            // setting hood angle
//            hoodExitAngleRad = Range.clip(Math.atan2(lookAheadTargetExitVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
//
//            // setting turret angle
//            lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
//
//            // setting shooter speed
//            efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
//            curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(shooter.getVelTps(), efficiencyCoef);
//            lookAheadTargetExitSpeedMps = Math.hypot( baseLength, lookAheadTargetExitVelMps.y );
//            if(checkShootingWhileMoving && locationState == Location.FAR)
//                lookAheadTargetExitSpeedMps += farParams.farVelOffset;
//
//            // setting no lookahead values
//            double noLookAheadDist = turretToGoalDist * 0.0254;
//            double[] noLookAheadLaunchVector = ShootingMathOld.calculateLaunchVectorWithImpactAngle(noLookAheadDist, relGoalHeightM, impactAngleRad);
//            double noLookAheadBallDir = Math.atan2(goalPosIn.z - turretPose.position.y, goalPosIn.x - turretPose.position.x);
//            Vector3dOld noLookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(noLookAheadBallDir, noLookAheadLaunchVector[1], noLookAheadLaunchVector[0], noLookAheadRobotVelAtTurretMps);
//            turretTargetAngleRad = Math.atan2(noLookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
//            currentTargetExitSpeedMps = Math.hypot( Math.hypot(noLookAheadTargetExitVelMps.x, noLookAheadTargetExitVelMps.y), noLookAheadTargetExitVelMps.z );
//        }
//        else {
//            futureDist = Math.min(futureDist, farParams.maxShootingDist);
//            int sign = BrainSTEMRobot.alliance == Alliance.RED ? -1 : 1;
//            double farExitAng = robot.drive.localizer.getPose().position.y * sign > farParams.far2SwitchY ? farParams.far2ExitAng : farParams.far1ExitAng;
//            launchVector = new double[] {ShootingMathOld.calculateLaunchVelocityWithExitAngle(futureDist, relGoalHeightM, farExitAng), hoodParams.minExitAngRad};
//            ballTargetExitSpeedMps = launchVector[0];
//            idealBallExitAng = launchVector[1];
//            ballExitAngleRad = idealBallExitAng;
//
//            currentlyShootingWhileMoving = false;
//            efficiencyCoef = calcEfficiencyCoef(launchVector[1]); // initial guess for efficiency coefficient
//            curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(shooter.getVelTps(), efficiencyCoef);
//            curExitSpeedMps = curExitSpeedMps * generalParams.efficiencyCoefWeight + ballTargetExitSpeedMps + (1 - generalParams.efficiencyCoefWeight);
//
//            // determining whether to use high arc or low arc
//            double highArcExitAng = ShootingMathOld.calculateBallExitAngleRad(true, relGoalHeightM, futureDist, curExitSpeedMps);
//            if (highArcExitAng != -1) {
//                double lowArcExitAng = ShootingMathOld.calculateBallExitAngleRad(false, relGoalHeightM, futureDist, curExitSpeedMps);
//                double highArcImpactAng = ShootingMathOld.calculateImpactAngle(futureDist, relGoalHeightM, curExitSpeedMps, highArcExitAng);
//                double lowArcImpactAng = ShootingMathOld.calculateImpactAngle(futureDist, relGoalHeightM, curExitSpeedMps, lowArcExitAng);
//                usingHighArc = Math.abs(highArcImpactAng - impactAngleRad) < Math.abs(lowArcImpactAng - impactAngleRad);
//
//                // estimating hood ang and current shooter speed
//                physicsExitAngleRads[0] = usingHighArc ? highArcExitAng : lowArcExitAng;
//                ballExitAngleRad = physicsExitAngleRads[0];
//                efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
//                double velAdjustment = locationState == Location.FAR ? shooter.getFarVelAdjustment() : shooter.getNearVelAdjustment();
//                curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(shooter.getVelTps() - velAdjustment, efficiencyCoef);
//
//                for (int i = 1; i < generalParams.numApproximations; i++) {
//                    physicsExitAngleRads[i] = ShootingMathOld.calculateBallExitAngleRad(usingHighArc, relGoalHeightM, futureDist, curExitSpeedMps);
//                    if (physicsExitAngleRads[i] == -1) {
//                        for (int j = i + 1; j < generalParams.numApproximations; j++)
//                            physicsExitAngleRads[j] = -1;
//                        break;
//                    }
//                    ballExitAngleRad = physicsExitAngleRads[i];
//                    efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
//                    curExitSpeedMps = ShootingMathOld.ticksPerSecToExitSpeedMps(shooter.getVelTps() - velAdjustment, efficiencyCoef);
//                }
//            } else
//                Arrays.fill(physicsExitAngleRads, -1);
//
//
//            lookAheadTargetExitSpeedMps = ballTargetExitSpeedMps + farParams.farVelOffset;
//            hoodExitAngleRad = ballExitAngleRad;
//
//            // basic estimation of turret angle to try account for shooting while moving
//            if(robotSpeedAtTurretIps < farParams.robotVelNoiseThreshold || robot.shooter.getShooterState() == Shooter.ShooterState.OFF) {
//                lookAheadTurretTargetAngleRad = desiredBallDir;
//                turretTargetAngleRad = desiredBallDir;
//            }
//            else {
//                lookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, curExitSpeedMps, lookAheadRobotVelAtTurretMps);
//                lookAheadTurretTargetAngleRad = Math.atan2(lookAheadTargetExitVelMps.z, lookAheadTargetExitVelMps.x);
//                Vector3dOld noLookAheadTargetExitVelMps = ShootingMathOld.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, curExitSpeedMps, noLookAheadRobotVelAtTurretMps);
//                turretTargetAngleRad = Math.atan2(noLookAheadTargetExitVelMps.z, noLookAheadTargetExitVelMps.x);
//            }
//            currentTargetExitSpeedMps = ballTargetExitSpeedMps;
//        }
//
//        if(!shouldScore)
//            hoodExitAngleRad = goalParams.missExitAngle;
//    }

    public Pose2d getTurretPose() {
        return turretPoseIn;
    }
    public void changeTurretAngleAdjustment(double adjustment) {
        turretAngleAdjustment += adjustment;
    }
    public void changeShooterSpeedAdjustment(double adjustment) {
        shooterSpeedAdjustment += adjustment;
    }
    public void resetAdjustments() {
        turretAngleAdjustment = 0;
        shooterSpeedAdjustment = 0;
    }

    public double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = generalParams.efficiencyCoefM * ballExitAngleRad + generalParams.efficiencyCoefB;
        return Range.clip(rawE, generalParams.minEfficiencyCoef, generalParams.maxEfficiencyCoef);
    }
    public boolean shooterNormGood() {
        double shooterError = Math.abs(currentTargetExitSpeedTps - shooter.getVelTps());
        return shooterError < (locationState == Location.FAR ? generalParams.farShootToleranceTps : generalParams.closeShootToleranceTps);
    }
    public boolean shooterFirstGood() {
        double shooterError = Math.abs(currentTargetExitSpeedTps - shooter.getVelTps());
        return shooterError < generalParams.firstShootToleranceTps;
    }
    public boolean turretOnTarget() {
        double error = Math.abs(turretTargetAngle - turret.getCurAngleRad());
        return error < (locationState == Location.FAR ? generalParams.farTurretTol : generalParams.nearTurretTol);
    }
    public boolean turretInRange() {
        return Math.abs(turretTargetAngle) < Turret.turretParams.maxAngle;
    }
    public boolean meetsSafetyInterlocks() {
        return turretOnTarget() && shooterNormGood();
    }
    public Location getLocationState() {
        return locationState;
    }

    public void setHoodToGoalTargeting() {
        hoodState = HoodState.GOAL_TARGETING;
    }
    public void setHoodToCustomExitAngle(double e) {
        hoodState = HoodState.CUSTOM_ANGLE;
        customHoodAngle = e;
    }

    public void setShooterToGoalTargeting() {
        shooterState = ShooterState.GOAL_TARGETING;
    }
    public void setShooterToCustomVoltage(double v) {
        shooterState = ShooterState.CUSTOM_VOLTAGE;
        customShooterVoltage = v;
    }
    public void setShooterOff() {
        shooterState = ShooterState.OFF;
    }
    public boolean shooterTargetingGoal() {
        return shooterState == ShooterState.GOAL_TARGETING;
    }
    public boolean turretCentered() {
        return turretState == TurretState.ANGLE_TARGETING && Math.abs(turretTargetAngle) < 1e-6;
    }
    public boolean turretTargetingGoal() {
        return turretState == TurretState.GOAL_TARGETING;
    }
    public void setTurretToGoalTargeting() {
        turretState = TurretState.GOAL_TARGETING;
    }
    public void setTurretToCenter() {
        setTurretToAngleTargeting(0, 0, false);
    }
    public void setTurretToCustomAngle(double targetRelAngle, double minVoltage, boolean passPosition) {
        setTurretToAngleTargeting(targetRelAngle, minVoltage, passPosition);
    }
    private void setTurretToAngleTargeting(double targetRelAngle, double minVoltage, boolean passPosition) {
        turretState = TurretState.ANGLE_TARGETING;
        turretTargetAngle = MathUtils.angleNormDeltaRad(targetRelAngle);
        angleTargetingMinVoltage = minVoltage;
        angleTargetingPassPosition = passPosition;
        angleTargetingInitialDir = Math.signum(targetRelAngle - turret.getCurAngleRad());
    }
    public Action rotateTurretToCustomAngle(DoubleSupplier relativeTargetAngleSup) {
        return new SequentialAction(
                new InstantAction(() -> {
                    double targetRelAngle = relativeTargetAngleSup.getAsDouble();
                    setTurretToAngleTargeting(targetRelAngle, 0, false);
                }),
                new TimedAction(new CustomEndAction(this::turretOnTarget), 1)
        );
    }

    public void drawClippedPoses(Canvas fieldOverlay) {
        fieldOverlay.setStroke("gray");
        Drawing.drawRobot(fieldOverlay, clippedRobotPoseIn);
        Drawing.drawCirclePose(fieldOverlay, clippedTurretPoseIn, 5);
    }
    public void drawShootingInfo(Canvas fieldOverlay) {
        // draw goal and shooting rings
        fieldOverlay.setStroke("yellow");
        fieldOverlay.strokeCircle(goalPosIn.x, goalPosIn.y, 3);

        fieldOverlay.setStroke("red");
        Drawing.drawCirclePose(fieldOverlay, turretPoseIn, 5);

        fieldOverlay.setStroke("purple");
        double dist = 300;
        fieldOverlay.strokeLine(
                turretPoseIn.position.x,
                turretPoseIn.position.y,
                turretPoseIn.position.x + dist * Math.cos(turretPoseIn.heading.toDouble()),
                turretPoseIn.position.y + dist * Math.sin(turretPoseIn.heading.toDouble())
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                turretPoseIn.position.x,
                turretPoseIn.position.y,
                turretPoseIn.position.x + dist * Math.cos(turretGoalTargetAngle),
                turretPoseIn.position.y + dist * Math.sin(turretGoalTargetAngle)
        );
    }

    @Override
    public void printInfo() {
        telemetry.addLine();
        telemetry.addLine("SHOOTING SYSTEM-------");
        telemetry.addData("dist state", locationState);
        telemetry.addData("shooter norm good", shooterNormGood());
        telemetry.addData("safety interlocks met", meetsSafetyInterlocks());
        telemetry.addData("shooter norm good num", shooterNormGood() ? 1 : 0);
        telemetry.addData("turret on target num", turretOnTarget() ? 2: 0);
        telemetry.addData("meets safety interlocks num", meetsSafetyInterlocks() ? 3 : 0);
        telemetry.addLine("");
        telemetry.addData("turret relative target deg", Math.toDegrees(turretGoalTargetAngle));
        telemetry.addData("turret lock on velocity", turretGoalLockOnVelocity);
        telemetry.addLine("");
        telemetry.addData("shooter lookahead target speed tps", lookAheadTargetExitSpeedTps);
        telemetry.addData("shooter current target speed tps", currentTargetExitSpeedTps);
        telemetry.addLine("");
        telemetry.addData("hood exit angle deg", MathUtils.format(Math.toDegrees(hoodExitAngleRad), 3));
        telemetry.addLine("");
    }
}