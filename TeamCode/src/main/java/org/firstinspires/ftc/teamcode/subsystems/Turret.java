package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.opmode.testing.TurretLogger;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean enableTestingKF = false;
        public double kfTestingVoltage = 0;
        public boolean enableTestingControl = false;
        public double testingTargetPos = 300;
        public double testingTargetVel = 0;
    }
    public static class TurretParams {
        public double shotOutOfRangeBuffer = Math.toRadians(0);
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public double maxAngle = Math.toRadians(95);
        public double maxClutchEngageError = 50; // if the turret error is greater than this, do not allow the intake to spin while the clutch is engaged
        public double maxTurretVelocityError = 200; // ticks/sec
        public double outOfRangeAngleLerpStart = Math.toRadians(120);
    }
    public static class PowerTuning {
//        public double kAYInt = .0007, kASlope = -.0000000001, minKA = .0004;
        public double kAYInt = 0, kASlope = 0, minKA = 0;
        public double goalAngularVelSign = 1;
        public double ignoreAngularVelocityNoiseThreshold = .05;
        public double ignoreKPScalingErrorThreshold = 40;
        public double APos = .005, BPos = 0.045, x0Pos = 130, kPos = .03;
        public double kD = 0.0007, kDExponent = 1;
        public double x0kPScaler = 20, kKPScaler = .2, BKPScaler = 1;
//        public double AVel = .05, BVel = .003, x0Vel = 30, kVel = .04;
        public double AVel = 0, BVel = 0, x0Vel = 0, kVel = 0;
        public double noVoltageThreshold = 1, noPowerIfOscillatingThreshold = 3, robotNotMovingThreshold = .5;
        public double maxVoltage = 7;
        public int prevEncoderStorageSize = 5, prevEncoderOscillatingSize = 3;

        public double[] kfPosLookupData = new double[] {
                -350, .75,
                -300, .75,
                -130, .75,
                0, .6,
                30, .7,
                100, .85,
                160, .9,
                170, .95,
                190, 1.1,
                230, 1.1,
                300, 1.3,
                350, 1.3
        };
        public double[] kfNegLookupData = new double[] {
                -350, -.85,
                -300, -.85,
                -156, -.85,
                -130, -.8,
                -75, -.75,
                -25, -.75,
                0, -.7,
                5, -.65,
                110, -.65,
                170, -.7,
                350, -.7
        };
    }
    public static TestingParams testingParams = new TestingParams();
    //    public static GoalParams goalParams = new GoalParams();
    public static TurretParams turretParams = new TurretParams();
    public static PowerTuning powerTuning = new PowerTuning();
    public enum TurretState {
        TRACKING, CENTER, TRACK_CUSTOM_TARGET
    }
    public TurretState turretState;
    private int nearEncoderAdjustment, farEncoderAdjustment;
    public double targetEncoder;
    private PIDController pidController;
//    private double targetVelocityFromRotation, targetVelocityFromTranslation;
    private double targetVelocity;
    private double goalAngularVel;
    private double targetAccel;
    public double currentEncoder, currentVelocity, currentAcceleration;
    public double positionError, velocityError;
    public Vector2d perpVelVec;
    private double kP, fVoltage;
    private double kV, kA;
    private final double[] prevErrors;
    private double pVoltage, dVoltage;
//    private double vRVoltage, vTVoltage, vVoltage;
    private double vVoltage;
    private double aVoltage;
    private final InterpLUT kFPosLookup, kfNegLookup;

    public double targetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double curRelAngleRad;
    private boolean inRange, inRangeForShot, onTarget;

    private double currentTestingTarget;
    private boolean smoothWhenOutOfRange;
    private boolean wasOscillating;
    public boolean addOscillationData = false;

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
        prevErrors = new double[powerTuning.prevEncoderStorageSize];
    }

    @Override
    public void update() {
        currentEncoder = robot.shootingSystem.getTurretEncoder();
        double prevPositionError = positionError;
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
            robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(currentTestingTarget, positionError, prevPositionError, targetVelocity, 0, 0));
            return;
        }

        if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
            robot.shootingSystem.setTurretVoltage(0);
            return;
        }

        double motorVoltage;
        switch (turretState) {
            case TRACKING:
                updateTargetToGoal();
                double actualTarget = MathUtils.angleNormDeltaRad(robot.shootingSystem.currentTurretTargetAngleRad) * turretParams.ticksPerRad;
                motorVoltage = calculateTurretVoltage(actualTarget, positionError, prevPositionError, targetVelocity, targetAccel, robot.shootingSystem.robotSpeedAtTurretIps);
                robot.shootingSystem.setTurretVoltage(motorVoltage);
                break;
            case CENTER:
                inRange = true;
                targetEncoder = 0;
                positionError = -currentEncoder;
                motorVoltage = calculateTurretVoltage(targetEncoder, positionError, prevPositionError, 0, 0, 0);
                robot.shootingSystem.setTurretVoltage(motorVoltage);
                break;
            case TRACK_CUSTOM_TARGET:
                inRange = true; // i clip targetEncoder so its always in range
                positionError = targetEncoder - currentEncoder;
                motorVoltage = calculateTurretVoltage(targetEncoder, positionError, prevPositionError, 0, 0, 0);
                robot.shootingSystem.setTurretVoltage(motorVoltage);
                break;
        }
    }
    private void updatePrevEncoderErrors(double error) {
        for(int i = prevErrors.length - 1; i > 0; i--)
            prevErrors[i] = prevErrors[i-1];
        prevErrors[0] = error;
    }
    private static boolean errorIsOscillating(double[] prevErrors) {
        boolean oscillating = true;
        boolean prevInBound = false;
        boolean inBoundEveryTime = true;
        for (int i = 0; i < powerTuning.prevEncoderOscillatingSize; i++) {
            boolean curInBound = Math.abs(prevErrors[i]) <= powerTuning.noVoltageThreshold;
            if(!curInBound)
                inBoundEveryTime = false;
            if(i == 0) {
                prevInBound = curInBound;
                continue;
            }
            if(prevInBound == curInBound) {
                oscillating = false;
                break;
            }
            prevInBound = curInBound;
        }
        if(oscillating)
            return true;
        if(inBoundEveryTime)
            return false;
        boolean ranAtLeastOnce = false;
        for(int i = 0; i < prevErrors.length; i++) {
            boolean curInBound = Math.abs(prevErrors[i]) <= powerTuning.noVoltageThreshold;
            if(i == 0) {
                prevInBound = curInBound;
                continue;
            }
            if (prevErrors[i] == powerTuning.noVoltageThreshold)
                continue;

            ranAtLeastOnce = true;
            if(prevInBound == curInBound)
                return false;
            prevInBound = curInBound;
        }
        if (!ranAtLeastOnce)
            return false;
        return true;
    }

    public double calculateTurretVoltage(double actualTargetEncoder, double positionError, double prevPositionError, double targetVelocity, double targetAccel, double robotSpeedAtTurret) {
        updatePrevEncoderErrors(positionError);
        boolean isOscillating = errorIsOscillating(prevErrors);
        if (addOscillationData)
            TurretLogger.addInfo(prevErrors, isOscillating);
        boolean canStopIfOscillating = Math.abs(positionError) <= powerTuning.noPowerIfOscillatingThreshold && isOscillating;
        if(canStopIfOscillating)
            wasOscillating = true;
        if(wasOscillating && Math.abs(positionError) > powerTuning.noPowerIfOscillatingThreshold)
            wasOscillating = false;
        if((Math.abs(positionError) <= powerTuning.noVoltageThreshold || wasOscillating) && robotSpeedAtTurret < powerTuning.robotNotMovingThreshold) {
            onTarget = true;
            return 0;
        }
        if(testingParams.enableTestingKF) {
            onTarget = true;
            return testingParams.kfTestingVoltage;
        }
        boolean onTargetPositionTol = Math.abs(actualTargetEncoder - currentEncoder) <= turretParams.maxClutchEngageError;
        boolean onTargetVelocityTol = Math.abs(targetVelocity - currentVelocity) <= turretParams.maxTurretVelocityError;
        onTarget = onTargetPositionTol && onTargetVelocityTol;

        double dir = Math.signum(positionError);
        double maxBound = turretParams.maxAngle * turretParams.ticksPerRad;
        double input = Range.clip(currentEncoder, -maxBound, maxBound); // reversing input if traveling in the opposite direction
        fVoltage = dir == 1 ? kFPosLookup.get(input) : kfNegLookup.get(input);

        kP = getLogisticErrorKP(Math.abs(positionError));
        kP = getLogisticErrorKP(Math.abs(positionError));
        if(Math.abs(positionError) < powerTuning.ignoreKPScalingErrorThreshold)
            kP *= getLogisticKPScaler(Math.abs(targetVelocity));
        pVoltage = kP * positionError;

        double errorSpeed = Math.abs((positionError - prevPositionError) / robot.shootingSystem.dt);
        dVoltage = powerTuning.kD * Math.pow(errorSpeed, powerTuning.kDExponent);
        dVoltage = Math.abs(dVoltage) * -Math.signum(positionError);

        kV = getLogisticKV(Math.abs(targetVelocity));
        vVoltage = kV * targetVelocity;

        kA = Math.max(Math.abs(targetAccel) * powerTuning.kASlope + powerTuning.kAYInt, powerTuning.minKA);
        aVoltage = targetVelocity == 0 ? kA * targetAccel : 0;

        double totalVoltage = fVoltage + pVoltage + dVoltage + vVoltage + aVoltage;
        totalVoltage = Range.clip(totalVoltage, -powerTuning.maxVoltage, powerTuning.maxVoltage);
        return totalVoltage;
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
    private void updateTargetToGoal() {
        // updating target angle
        targetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.lookAheadTurretTargetAngleRad - robot.shootingSystem.futureRobotPose.heading.toDouble());
        // mirrors the angle if the turret cannot reach it (visual cue)
        inRange = Math.abs(targetRelAngleRad) <= turretParams.maxAngle;
        if (!inRange) {
            double sign = Math.signum(targetRelAngleRad);
            double max = sign * turretParams.maxAngle;
            if(smoothWhenOutOfRange) {
                if (Math.abs(targetRelAngleRad) <= turretParams.outOfRangeAngleLerpStart)
                    targetRelAngleRad = max;
                else {
                    double a = Math.abs(turretParams.outOfRangeAngleLerpStart);
                    double b = Math.PI;
                    double x = Math.abs(targetRelAngleRad);
                    double outOfBoundsT = MathUtils.inverseLerp(a, b, x);
                    targetRelAngleRad = MathUtils.lerp(max, 0, outOfBoundsT);
                }
            }
            else
                targetRelAngleRad = max;
        }
        inRangeForShot = Math.abs(targetRelAngleRad) < turretParams.maxAngle + turretParams.shotOutOfRangeBuffer;

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
        double targetAngularVelocity = goalAngularVel * powerTuning.goalAngularVelSign - robot.drive.pinpoint().getVelocity().headingRad;
        if(Math.abs(targetAngularVelocity) < powerTuning.ignoreAngularVelocityNoiseThreshold) // to eliminate random pinpoint noise
            targetAngularVelocity = 0;

        double prevTargetVel = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        targetAccel = (targetVelocity - prevTargetVel) / robot.shootingSystem.dt;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addLine("-----");
        telemetry.addData("add oscillation data", addOscillationData);
        telemetry.addData("prev errors", Arrays.toString(prevErrors));
        telemetry.addData("error oscillating", errorIsOscillating(prevErrors));
        telemetry.addData("was oscillating", wasOscillating);
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
        telemetry.addData("voltage kP", pVoltage);
        telemetry.addData("voltage vel", vVoltage);
        telemetry.addData("voltage accel", aVoltage);
        telemetry.addData("voltage kd", dVoltage);
        telemetry.addData("kP", kP);
        telemetry.addData("kf", fVoltage);
        telemetry.addData("kV", kV);
        telemetry.addData("kA", kA);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("target accel", targetAccel);
        telemetry.addData("encoder error", positionError);
        telemetry.addData("angle degree error", Math.toDegrees(positionError / turretParams.ticksPerRad));
        telemetry.addData("velocity error", velocityError);
        telemetry.addData("target goal angular vel", goalAngularVel);
        telemetry.addData("target robot angular vel", -robot.drive.pinpoint().driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        telemetry.addData("desired ball dir", Math.toDegrees(robot.shootingSystem.desiredBallDir));
        telemetry.addData("actual turret target angle", Math.toDegrees(robot.shootingSystem.lookAheadTurretTargetAngleRad));
//        telemetry.addLine("-----");
        telemetry.addData("current encoder", currentEncoder);
        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current acceleration", currentAcceleration);
//        telemetry.addData("turret current relative angle deg", Math.toDegrees(curRelAngleRad));
//        telemetry.addData("turret target relative angle deg", Math.toDegrees(targetRelAngleRad));
//        telemetry.addLine("-----");
        telemetry.addData("voltage", robot.getFilteredVoltage());
        telemetry.addData("voltage raw", robot.getRawVoltage());
        telemetry.addLine("------");
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
    public boolean onTarget() {
        return onTarget;
    }
    public void setCustomTargetEncoder(double encoder) {
        if (turretState == TurretState.TRACK_CUSTOM_TARGET)
            targetEncoder = encoder;
    }
    public Action rotateToCustomTarget(DoubleSupplier targetAngle) {
        return new SequentialAction(
                new InstantAction(() -> {
                    turretState = TurretState.TRACK_CUSTOM_TARGET;
                    double clippedAngle = Range.clip(targetAngle.getAsDouble(), -turretParams.maxAngle, turretParams.maxAngle);
                    targetEncoder = clippedAngle * turretParams.ticksPerRad;
                }),
                telemetryPacket -> positionError > powerTuning.noVoltageThreshold
        );
    }
}