package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean enableTestingKF = false;
        public double kfTestingVoltage = 0;
        public boolean enableTestingControl = false;
        public double testingTargetPos = 300;
        public double testingTargetVel = 0;
        public double alpha = .5;
        public boolean enableMotionProfiling = true;
    }
    public static class Params {
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public int minBound = -300;
        public int maxBound = 300;
        public double maxClutchEngageError = 20; // if the turret error is greater than this, do not allow the intake to spin while the clutch is engaged
    }
    public static class PowerTuning {
        public double rotKV = .0001, transKV = .01;
        public double ignoreProfilingHeadingVelThreshold = 1.5;
        public double profileAccel = 2000, profileMaxVel = 200, profileBufferDist = 20;
        public double ignoreAngularVelocityNoiseThreshold = Math.toRadians(1);
        public double ignoreAngularVelocityPositionThreshold = Math.toRadians(4) * turretParams.ticksPerRad;
        public double ignoreFBVelocityThreshold = 100; // if the velocity error is greater than this amount, i don't apply kVP
        public double A = .005, B = 0.02, x0 = 175, k = .02;
        public double smallKV = 0.008, bigKV = 0.01, kVP = .015;
        public double switchKVThreshold = 150;
        public double noPowerThreshold = 3;

        public double[] kfPosLookupData = new double[] {
                -350, .5,
                -300, .7,
                0, .4,
                30, .55,
                100, .7,
                160, .75,
                170, .8,
                190, .98,
                230, 1.1,
                300, 1.3,
                350, 1.3
        };
        public double[] kfNegLookupData = new double[] {
                -350, -.55,
                -300, -.55,
                -75, -.5,
                -25, -.5,
                0, -.5,
                5, -.6,
                110, -.6,
                170, -.6,
                350, -.6
        };
    }
    public static TestingParams testingParams = new TestingParams();
    //    public static GoalParams goalParams = new GoalParams();
    public static Params turretParams = new Params();
    public static PowerTuning powerTuning = new PowerTuning();
    public enum TurretState {
        TRACKING, CENTER
    }
    public TurretState turretState;
    private int nearEncoderAdjustment, farEncoderAdjustment;
    public double targetEncoder;
//    public double targetVelocity, prevTargetVelocity;
    public double targetVelocityFromRotation, targetVelocityFromTranslation;
    private boolean usingMotionProfiling;
    public double currentEncoder, currentVelocity, currentAcceleration;
    public double positionError, velocityError;
    private double kP;
    private double kF;
    private double pVoltage;
//    private double vFFVoltage, vFBVoltage;
    private double vRVoltage, vTVoltage;
    private final InterpLUT kFPosLookup, kfNegLookup;

    public double targetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double curRelAngleRad;
    public boolean inRange;

    private double currentTestingTarget;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);
        kFPosLookup = new InterpLUT();
        for(int i = 0; i < powerTuning.kfPosLookupData.length; i+=2)
            kFPosLookup.add(powerTuning.kfPosLookupData[i], powerTuning.kfPosLookupData[i+1]);
        kFPosLookup.createLUT();
        kfNegLookup = new InterpLUT();
        for(int i = 0; i < powerTuning.kfNegLookupData.length; i+=2)
            kfNegLookup.add(powerTuning.kfNegLookupData[i], powerTuning.kfNegLookupData[i+1]);
        kfNegLookup.createLUT();

        turretState = TurretState.CENTER;

        currentTestingTarget = testingParams.testingTargetPos;
    }

    @Override
    public void update() {
        currentEncoder = robot.shootingSystem.getTurretEncoder();
        double prevVelocity = currentVelocity;
        currentVelocity = robot.shootingSystem.getTurretVelTps();
        currentAcceleration = (currentVelocity - prevVelocity) / robot.shootingSystem.dt;

        curRelAngleRad = currentEncoder / turretParams.ticksPerRad;
        currentAbsoluteAngleRad = curRelAngleRad + robot.drive.localizer.getPose().heading.toDouble();

        if(testingParams.enableTestingControl) {
            currentTestingTarget = testingParams.testingTargetPos * Math.signum(currentTestingTarget);
            positionError = currentTestingTarget - currentEncoder;
            double errorSign = Math.signum(positionError);
            if(Math.signum(currentTestingTarget) != errorSign) {
                currentTestingTarget *= -1;
                positionError = currentTestingTarget - currentEncoder;
                errorSign = Math.signum(positionError);
            }
            targetEncoder = currentTestingTarget;
//            prevTargetVelocity = targetVelocity;
//            targetVelocity = testingParams.testingTargetVel * errorSign;
//            targetVelocity = testingParams.alpha * targetVelocity + (1 - testingParams.alpha) * prevTargetVelocity;
            robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(positionError, 0, 0));
            return;
        }

        switch (turretState) {
            case TRACKING:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretVoltage(0);
                    break;
                }
                updateTarget();
                robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(positionError, targetVelocityFromRotation, targetVelocityFromTranslation));
                break;

            case CENTER:
                inRange = true;
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretVoltage(0);
                    break;
                }
                targetEncoder = 0;
                positionError = -currentEncoder;
                robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(positionError, 0, 0));
                break;
        }
    }

    public double calculateTurretVoltage(double positionError, double targetVelocityFromRotation, double targetVelocityFromTranslation) {
        if(Math.abs(positionError) <= powerTuning.noPowerThreshold)
            return 0;
        if(testingParams.enableTestingKF)
            return testingParams.kfTestingVoltage;
        double dir = Math.signum(positionError);
        double input = Range.clip(currentEncoder, turretParams.minBound, turretParams.maxBound); // reversing input if traveling in the opposite direction
        kF = dir == 1 ? kFPosLookup.get(input) : kfNegLookup.get(input);

        kP = getLogisticKP(Math.abs(positionError));
        pVoltage = kP * positionError;

        vRVoltage = powerTuning.rotKV * targetVelocityFromRotation;
        vTVoltage = powerTuning.transKV * targetVelocityFromTranslation;
        return kF + pVoltage + vRVoltage + vTVoltage;
    }
    private double getLogisticKP(double errorMag) {
        return powerTuning.A / (1 + Math.exp(-powerTuning.k * (errorMag - powerTuning.x0)) ) + powerTuning.B;
    }
    private double getProfileTargetVelocity(double positionError) {
        // 2ad = vf^2 - vi^2
        // vi^2 = vf^2 - 2ad
        // if vf = 0:
        // vi^2 = -2ad
        if(Math.abs(positionError) < powerTuning.profileBufferDist)
            return 0;
        double dir = Math.signum(positionError);
        double targetVelMag = Math.sqrt(2 * Math.abs(powerTuning.profileAccel * positionError));
        return Range.clip(targetVelMag, 0, powerTuning.profileMaxVel) * dir;
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    private void updateTarget() {
        // updating target angle
        targetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.actualTurretTargetAngleRad - robot.shootingSystem.futureRobotPose.heading.toDouble());
        // mirrors the angle if the turret cannot reach it (visual cue)
        if (targetRelAngleRad > Math.toRadians(ShootingMath.turretSystemParams.maxAngleDeg)) {
            targetRelAngleRad = Math.PI - targetRelAngleRad;
            inRange = false;
        }
        else if (targetRelAngleRad < Math.toRadians(ShootingMath.turretSystemParams.minAngleDeg)) {
            targetRelAngleRad = -Math.PI - targetRelAngleRad;
            inRange = false;
        }
        else
            inRange = true;

        targetEncoder = (int) (targetRelAngleRad * turretParams.ticksPerRad);
        targetEncoder += robot.shootingSystem.distState != ShootingSystem.Dist.FAR ? nearEncoderAdjustment : farEncoderAdjustment;
        targetEncoder = Range.clip(targetEncoder, turretParams.minBound, turretParams.maxBound);
        positionError = targetEncoder - currentEncoder;

        // updating target angular velocity
        Vector2d perpVelVec = new Vector2d(-robot.shootingSystem.futureExitPosRelativeToGoal.y, robot.shootingSystem.futureExitPosRelativeToGoal.x *1);
        perpVelVec = perpVelVec.div(robot.shootingSystem.futureExitPosGoalDistIn);
        double dot = robot.shootingSystem.robotVelAtExitPosIps.dot(perpVelVec);
        double goalAngularVel = dot / robot.shootingSystem.futureExitPosGoalDistIn;
        if(Math.abs(goalAngularVel) < powerTuning.ignoreAngularVelocityNoiseThreshold) // to eliminate random pinpoint noise
            goalAngularVel = 0;
        targetVelocityFromTranslation = goalAngularVel * turretParams.ticksPerRad;
        targetVelocityFromRotation = -robot.shootingSystem.odoVel.headingRad * turretParams.ticksPerRad;

//        if(testingParams.enableMotionProfiling && Math.abs(robot.shootingSystem.odoVel.headingRad) < powerTuning.ignoreProfilingHeadingVelThreshold) {
//            targetVelocity += getProfileTargetVelocity(positionError);
//            usingMotionProfiling = true;
//        }
//        else
//            usingMotionProfiling = false;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addLine("-----");
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
        telemetry.addData("p Voltage", pVoltage);
        telemetry.addData("v rot Voltage", vRVoltage);
        telemetry.addData("v trans Voltage", vTVoltage);
        telemetry.addData("kP", kP);
        telemetry.addData("kf", kF);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("target velocity from rot", targetVelocityFromRotation);
        telemetry.addData("target velocity from trans", targetVelocityFromTranslation);
//        telemetry.addLine("-----");
        telemetry.addData("current encoder", currentEncoder);
        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current acceleration", currentAcceleration);
//        telemetry.addData("turret current relative angle deg", Math.toDegrees(curRelAngleRad));
//        telemetry.addData("turret target relative angle deg", Math.toDegrees(targetRelAngleRad));
//        telemetry.addLine("-----");
        telemetry.addData("angle degree error", Math.toDegrees(positionError / turretParams.ticksPerRad));
        telemetry.addData("encoder error", positionError);
        telemetry.addData("velocity error", velocityError);
        telemetry.addData("voltage", robot.getFilteredVoltage());
        telemetry.addData("voltage raw", robot.getRawVoltage());
        telemetry.addLine("------");
        telemetry.addData("using motion profiling", usingMotionProfiling ? 100 : 0);
        telemetry.addData("odo heading rad", robot.shootingSystem.odoVel.headingRad);
//        telemetry.addLine("-----");
        telemetry.addData("inRange", inRange());
    }

    public void changeEncoderAdjustment(int amount) {
        if(robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
            nearEncoderAdjustment += amount;
        else
            farEncoderAdjustment += amount;
    }
    public boolean inRange() {
        return inRange;
    }
}