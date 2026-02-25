package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
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
    }
    public static class Params {
        public double shotOutOfRangeBuffer = Math.toRadians(2);
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public double maxAngle = Math.toRadians(95);
        public double maxClutchEngageError = 2000; // if the turret error is greater than this, do not allow the intake to spin while the clutch is engaged
    }
    public static class PowerTuning {
        public double kA = .0004;
//        public double transKV = .01;
//        public double profileAccel = 2000, profileMaxVel = 200, profileBufferDist = 20;
        public double ignoreAngularVelocityNoiseThreshold = Math.toRadians(1);
        public double ignoreKPScalingErrorThreshold = 40;
        public double APos = .002, BPos = 0.02, x0Pos = 150, kPos = .03;
        public double x0kPScaler = 20, kKPScaler = .25, BKPScaler = .3;
        public double AVel = 0.1, BVel = .006, x0Vel = 30, kVel = .04;
        public double smallVelKV = .017, smallVelThreshold = 50;
        public double noPowerThreshold = 1, robotNotMovingThreshold = .5;

        public double[] kfPosLookupData = new double[] {
                -350, .75,
                -300, .75,
                -130, .75,
                0, .6,
                30, .75,
                100, .9,
                160, .95,
                170, 1,
                190, 1.18,
                230, 1.25,
                300, 1.45,
                350, 1.45
        };
        public double[] kfNegLookupData = new double[] {
                -350, -.8,
                -300, -.8,
                -130, -.8,
                -75, -.7,
                -25, -.7,
                0, -.7,
                5, -.75,
                110, -.75,
                170, -.75,
                350, -.75
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
//    private double targetVelocityFromRotation, targetVelocityFromTranslation;
    private double targetVelocity;
    private double goalAngularVel;
    private double targetAccel;
    private boolean usingMotionProfiling;
    public double currentEncoder, currentVelocity, currentAcceleration;
    public double positionError, velocityError;
    public Vector2d perpVelVec;
    private double kP, kF;
    private double kV;
//    private double rotKV;
    private double pVoltage;
//    private double vRVoltage, vTVoltage, vVoltage;
    private double vVoltage;
    private double aVoltage;
    private final InterpLUT kFPosLookup, kfNegLookup;

    public double targetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double curRelAngleRad;
    public boolean inRange, inRangeForShot;

    private double currentTestingTarget;
    private boolean smoothWhenOutOfRange;

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
        smoothWhenOutOfRange = true;
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
            targetVelocity = testingParams.testingTargetVel * errorSign;
            robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(positionError, targetVelocity, 0, 0));
            return;
        }

        switch (turretState) {
            case TRACKING:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretVoltage(0);
                    break;
                }
                updateTarget();
                robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(positionError, targetVelocity, targetAccel, robot.shootingSystem.robotSpeedAtTurretIps));
                break;

            case CENTER:
                inRange = true;
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretVoltage(0);
                    break;
                }
                targetEncoder = 0;
                positionError = -currentEncoder;
                robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(positionError, 0, 0, 0));
                break;
        }
    }

    public double calculateTurretVoltage(double positionError, double targetVelocity, double targetAccel, double robotSpeedAtTurret) {
        if(Math.abs(positionError) <= powerTuning.noPowerThreshold && robotSpeedAtTurret < powerTuning.robotNotMovingThreshold)
            return 0;
        if(testingParams.enableTestingKF)
            return testingParams.kfTestingVoltage;
        double dir = Math.signum(positionError);
        double maxBound = turretParams.maxAngle * turretParams.ticksPerRad;
        double input = Range.clip(currentEncoder, -maxBound, maxBound); // reversing input if traveling in the opposite direction
        kF = dir == 1 ? kFPosLookup.get(input) : kfNegLookup.get(input);

        kP = getLogisticErrorKP(Math.abs(positionError));
        if(Math.abs(positionError) < powerTuning.ignoreKPScalingErrorThreshold)
            kP *= getLogisticKPScaler(Math.abs(targetVelocity));
        pVoltage = kP * positionError;
        kV = Math.abs(targetVelocity) < powerTuning.smallVelThreshold ? powerTuning.smallVelKV : getLogisticKV(Math.abs(targetVelocity));
        vVoltage = kV * targetVelocity;

        aVoltage = powerTuning.kA * targetAccel;

        return kF + pVoltage + vVoltage + aVoltage;
    }
    private double getLogisticErrorKP(double errorMag) {
        return powerTuning.APos / (1 + Math.exp(powerTuning.kPos * (errorMag - powerTuning.x0Pos)) ) + powerTuning.BPos;
    }
    private double getLogisticKPScaler(double targetVelMag) {
        return (1 - powerTuning.BKPScaler) / (1 + Math.exp(powerTuning.kKPScaler * (targetVelMag - powerTuning.x0kPScaler))) + powerTuning.BKPScaler ;
    }
    private double getLogisticKV(double targetRotVelMag) {
        return powerTuning.AVel / (1 + Math.exp(powerTuning.kVel * (targetRotVelMag - powerTuning.x0Vel))) + powerTuning.BVel;
    }
    public void setSmoothWhenOutOfRange(boolean smoothWhenOutOfRange) {
        this.smoothWhenOutOfRange = smoothWhenOutOfRange;
    }
//    private double getProfileTargetVelocity(double positionError) {
//        // 2ad = vf^2 - vi^2
//        // vi^2 = vf^2 - 2ad
//        // if vf = 0:
//        // vi^2 = -2ad
//        if(Math.abs(positionError) < powerTuning.profileBufferDist)
//            return 0;
//        double dir = Math.signum(positionError);
//        double targetVelMag = Math.sqrt(2 * Math.abs(powerTuning.profileAccel * positionError));
//        return Range.clip(targetVelMag, 0, powerTuning.profileMaxVel) * dir;
//    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    private void updateTarget() {
        // updating target angle
        targetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.actualTurretTargetAngleRad - robot.shootingSystem.futureRobotPose.heading.toDouble());
        // mirrors the angle if the turret cannot reach it (visual cue)
        double maxAngleRad = turretParams.maxAngle;
        if (targetRelAngleRad > maxAngleRad) {
            if(smoothWhenOutOfRange)
                targetRelAngleRad = Math.PI - targetRelAngleRad;
            else
                targetRelAngleRad = maxAngleRad;
            inRange = false;
        }
        else if (targetRelAngleRad < -maxAngleRad) {
            if(smoothWhenOutOfRange)
                targetRelAngleRad = -Math.PI - targetRelAngleRad;
            else
                targetRelAngleRad = -maxAngleRad;
            inRange = false;
        }
        else
            inRange = true;
        inRangeForShot = Math.abs(targetRelAngleRad) < maxAngleRad + turretParams.shotOutOfRangeBuffer;

        targetEncoder = (int) (targetRelAngleRad * turretParams.ticksPerRad);
        targetEncoder += robot.shootingSystem.distState != ShootingSystem.Dist.FAR ? nearEncoderAdjustment : farEncoderAdjustment;
        double maxBound = turretParams.maxAngle * turretParams.ticksPerRad;
        targetEncoder = Range.clip(targetEncoder, -maxBound, maxBound);
        positionError = targetEncoder - currentEncoder;

        // updating target angular velocity
        perpVelVec = new Vector2d(-robot.shootingSystem.futureTurretPosRelativeToGoal.y, robot.shootingSystem.futureTurretPosRelativeToGoal.x *1);
        perpVelVec = perpVelVec.div(robot.shootingSystem.futureTurretPosGoalDistIn);
        double dot = robot.shootingSystem.robotVelAtTurretIps.dot(perpVelVec);
        telemetry.addData("DOT", dot);
        telemetry.addData("PER VEL VEC", MathUtils.formatVec3(perpVelVec));
        goalAngularVel = dot / robot.shootingSystem.futureTurretPosGoalDistIn;
        if(Math.abs(goalAngularVel) < powerTuning.ignoreAngularVelocityNoiseThreshold) // to eliminate random pinpoint noise
            goalAngularVel = 0;

        double targetAngularVelocity = goalAngularVel - robot.drive.pinpoint().driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        double prevTargetVel = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        targetAccel = (targetVelocity - prevTargetVel) / robot.shootingSystem.dt;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addLine("-----");
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
        telemetry.addData("p Voltage", pVoltage);
        telemetry.addData("vel Voltage", vVoltage);
        telemetry.addData("accel Voltage", aVoltage);
        telemetry.addData("kP", kP);
        telemetry.addData("kf", kF);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("target accel", targetAccel);
        telemetry.addData("target goal angular vel", goalAngularVel);
        telemetry.addData("target robot angular vel", -robot.drive.pinpoint().driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        telemetry.addData("desired ball dir", Math.toDegrees(robot.shootingSystem.desiredBallDir));
        telemetry.addData("actual turret target angle", Math.toDegrees(robot.shootingSystem.actualTurretTargetAngleRad));
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
    public boolean inRangeForShot() {return inRangeForShot; }
}