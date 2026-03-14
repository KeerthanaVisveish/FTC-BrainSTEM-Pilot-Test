package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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
        public double minParkRotateVoltage = 9;
        public double gateCollectYThreshold = 58, gateCollectMaxAngle = Math.toRadians(30);
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public double maxAngle = Math.toRadians(90);
        public double maxNearClutchEngageError = 25, maxFarClutchEngageError = 8; // if the turret error is greater than this, do not allow the intake to spin while the clutch is engaged
        public double outOfRangeAngleLerpStart = Math.toRadians(135);
    }
    public static class PowerTuning {
        public double ignoreGoalAngularVelThreshold = .0001;
//        public double kAYInt = .0007, kASlope = -.0000000001, minKA = .0004;
        public double kAYInt = 0.00, kASlope = 0, minKA = 0;
        public double accelVoltageSign = -1;
        public double ignoreAngularVelocityNoiseThreshold = .05;
        public double ignoreKPScalingErrorThreshold = 40;
        public double APos = .004, BPos = 0.028, x0Pos = 130, kPos = .03;
        public double kD = 0.0005, kDExponent = 1.1;
        public double x0kPScaler = 20, kKPScaler = .2, BKPScaler = 1;
//        public double AVel = .05, BVel = .003, x0Vel = 30, kVel = .04;
        public double AVel = 0, BVel = 0, x0Vel = 0, kVel = 0;
        public double AInertia = .0, BInertia = -.000, kInertia = -3, x0Inertia = 1;
        public double noVoltageThreshold = 1, noPowerIfOscillatingThreshold = 3, robotNotMovingThreshold = .5;
        public double maxVoltage = 7;
        public int prevEncoderStorageSize = 5, prevEncoderOscillatingSize = 3;

        public double[] kfPosLookupData = new double[] {
                -350, .7,
                -300, .7,
                -130, .7,
                0, .6,
                30, .7,
                100, .8,
                160, .8,
                170, .9,
                190, 1,
                230, 1,
                300, 1.1,
                350, 1.1
        };
        public double[] kfNegLookupData = new double[] {
                -350, -.75,
                -300, -.75,
                -156, -.75,
                -130, -.75,
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
    private double goalAngularVel, goalAngularAccel;
    private double targetAccel;
    public double currentEncoder, currentVelocity, currentAcceleration;
    public double positionError, velocityError;
    public Vector2d perpVelVec;
    private double kP, fVoltage;
    private double kV, kA;
    private final double[] prevErrors;
    private double prevErrorSpeed;
    private double pVoltage, dVoltage, totalVoltage;
//    private double vRVoltage, vTVoltage, vVoltage;
    private double vVoltage;
    private double aVoltage;
    private final InterpLUT kFPosLookup, kfNegLookup;

    public double lookAheadTargetRelAngleRad, noLookAheadTargetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double curRelAngleRad;
    private boolean inRange, onTarget;

    private double currentTestingTarget;
    private boolean smoothWhenOutOfRange;
    private boolean wasOscillating;
    public boolean addOscillationData = false;
    private double inertialAngleOffset;
    private double trackCustomTargetMinPower;
    private double trackCustomTargetStartEncoder;
    private boolean trackCustomTargetPassPosition, trackCustomTargetPassPositionDone;

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
            robot.shootingSystem.setTurretVoltage(calculateTurretVoltage(currentTestingTarget, positionError, prevPositionError, 0));
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
                motorVoltage = calculateTurretVoltage(noLookAheadTargetRelAngleRad * turretParams.ticksPerRad, positionError, prevPositionError, robot.shootingSystem.robotSpeedAtTurretIps);
                robot.shootingSystem.setTurretVoltage(motorVoltage);
                break;
            case CENTER:
                inRange = true;
                targetEncoder = 0;
                positionError = -currentEncoder;
                motorVoltage = calculateTurretVoltage(targetEncoder, positionError, prevPositionError, 0);
                robot.shootingSystem.setTurretVoltage(motorVoltage);
                break;
            case TRACK_CUSTOM_TARGET:
                inRange = true; // i clip targetEncoder so its always in range
                positionError = targetEncoder - currentEncoder;
                if(trackCustomTargetPassPosition &&
                (trackCustomTargetPassPositionDone || Math.signum(positionError) != Math.signum(targetEncoder - trackCustomTargetStartEncoder))) {
                    robot.shootingSystem.setTurretVoltage(0);
                    trackCustomTargetPassPositionDone = true;
                }
                else {
                    motorVoltage = calculateTurretVoltage(targetEncoder, positionError, prevPositionError, 0);
                    motorVoltage = Math.max(trackCustomTargetMinPower, Math.abs(motorVoltage)) * Math.signum(motorVoltage);
                    robot.shootingSystem.setTurretVoltage(motorVoltage);
                }
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
        return ranAtLeastOnce;
    }

    public double calculateTurretVoltage(double actualTargetEncoder, double positionError, double prevPositionError, double robotSpeedAtTurret) {
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
        double maxError = robot.shootingSystem.distState == ShootingSystem.Dist.FAR ? turretParams.maxFarClutchEngageError : turretParams.maxNearClutchEngageError;
        onTarget = Math.abs(actualTargetEncoder - currentEncoder) <= maxError;

        double dir = Math.signum(positionError);
        double maxBound = turretParams.maxAngle * turretParams.ticksPerRad;
        double input = Range.clip(currentEncoder, -maxBound, maxBound); // reversing input if traveling in the opposite direction
        fVoltage = dir == 1 ? kFPosLookup.get(input) : kfNegLookup.get(input);

        kP = getLogisticErrorKP(Math.abs(positionError));
        kP = getLogisticErrorKP(Math.abs(positionError));
        if(Math.abs(positionError) < powerTuning.ignoreKPScalingErrorThreshold)
            kP *= getLogisticKPScaler(Math.abs(targetVelocity));
        pVoltage = kP * positionError;

        double errorVel = (positionError - prevPositionError) / robot.shootingSystem.dt;
        double errorSpeed = Math.abs(errorVel);
        double errorAccelSign = Math.signum(errorSpeed - prevErrorSpeed);
        if(errorAccelSign > 0 && Math.signum(errorVel) == Math.signum(positionError))
            dVoltage = 0;
        else {
            dVoltage = powerTuning.kD * Math.pow(errorSpeed, powerTuning.kDExponent);
            dVoltage = Math.abs(dVoltage) * -Math.signum(positionError);
        }
        prevErrorSpeed = errorSpeed;

        kV = getLogisticKV(Math.abs(targetVelocity));
        vVoltage = kV * targetVelocity;

        kA = Math.max(Math.abs(targetAccel) * powerTuning.kASlope + powerTuning.kAYInt, powerTuning.minKA);
        aVoltage = targetVelocity == 0 ? kA * targetAccel * powerTuning.accelVoltageSign : 0;

        totalVoltage = fVoltage + pVoltage + dVoltage + vVoltage + aVoltage;
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
    private double getLogisticInertialDistOffset(double x) {
        return Math.max(powerTuning.AInertia / (1 + Math.exp(powerTuning.kInertia * (x - powerTuning.x0Inertia))) + powerTuning.BInertia, 0);
    }
    public void setSmoothWhenOutOfRange(boolean smoothWhenOutOfRange) {
        this.smoothWhenOutOfRange = smoothWhenOutOfRange;
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    private void updateTargetToGoal() {
        // updating target angular velocity
        perpVelVec = new Vector2d(-robot.shootingSystem.futureTurretPosRelativeToGoal.y, robot.shootingSystem.futureTurretPosRelativeToGoal.x *1);
        perpVelVec = perpVelVec.div(robot.shootingSystem.futureTurretPosGoalDistIn);
        double dot = robot.shootingSystem.robotVelAtTurretIps.dot(perpVelVec);

        double prevGoalAngularVel = goalAngularVel;
        goalAngularVel = dot / robot.shootingSystem.futureTurretPosGoalDistIn;
        if(Math.abs(goalAngularVel) < powerTuning.ignoreGoalAngularVelThreshold)
            goalAngularVel = 0;
        goalAngularAccel = (goalAngularVel - prevGoalAngularVel) / robot.shootingSystem.dt;

        double targetAngularVelocity = goalAngularVel - robot.drive.pinpoint().getVelocity().headingRad;
        if(Math.abs(targetAngularVelocity) < powerTuning.ignoreAngularVelocityNoiseThreshold) // to eliminate random pinpoint noise
            targetAngularVelocity = 0;
        // updating target angular acceleration
        double prevTargetVel = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        targetAccel = (targetVelocity - prevTargetVel) / robot.shootingSystem.dt;

        // updating target angle
        noLookAheadTargetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.currentTurretTargetAngleRad - robot.drive.localizer.getPose().heading.toDouble());
        lookAheadTargetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.lookAheadTurretTargetAngleRad - robot.shootingSystem.futureRobotPose.heading.toDouble());
        inertialAngleOffset = getLogisticInertialDistOffset(Math.abs(goalAngularAccel)) * -Math.signum(goalAngularAccel);
        lookAheadTargetRelAngleRad += inertialAngleOffset;

        Pose2d pose = robot.drive.localizer.getPose();
        boolean useSmallerRange = pose.position.x > 0 && pose.position.x < 20 && Math.abs(pose.position.y) > turretParams.gateCollectYThreshold;
        double maxAngle = useSmallerRange ? turretParams.gateCollectMaxAngle : turretParams.maxAngle;
        inRange = Math.abs(lookAheadTargetRelAngleRad) <= maxAngle;
        if (!inRange) {
            double sign = Math.signum(lookAheadTargetRelAngleRad);
            double max = sign * maxAngle;
            // clamp turret to smaller range when close to the wall
            if (useSmallerRange)
                lookAheadTargetRelAngleRad = max;
            else {
                // mirrors the angle if the turret cannot reach it (visual cue)
                if (smoothWhenOutOfRange) {
                    if (Math.abs(lookAheadTargetRelAngleRad) <= turretParams.outOfRangeAngleLerpStart)
                        lookAheadTargetRelAngleRad = max;
                    else {
                        double a = Math.abs(turretParams.outOfRangeAngleLerpStart);
                        double b = Math.PI;
                        double x = Math.abs(lookAheadTargetRelAngleRad);
                        double outOfBoundsT = MathUtils.inverseLerp(a, b, x);
                        lookAheadTargetRelAngleRad = MathUtils.lerp(max, 0, outOfBoundsT);
                    }
                }
                else
                    lookAheadTargetRelAngleRad = max;
            }
        }

        targetEncoder = (int) (lookAheadTargetRelAngleRad * turretParams.ticksPerRad);
        targetEncoder += robot.shootingSystem.distState != ShootingSystem.Dist.FAR ? nearEncoderAdjustment : farEncoderAdjustment;
        double maxBound = turretParams.maxAngle * turretParams.ticksPerRad;
        targetEncoder = Range.clip(targetEncoder, -maxBound, maxBound);
        positionError = targetEncoder - currentEncoder;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
//        telemetry.addData("INERTIAL ANGLE OFFSET DEG", MathUtils.format3(inertialAngleOffset * 180 / Math.PI));
        telemetry.addData("turret state", turretState);
        telemetry.addLine("-----");
//        telemetry.addData("dt", robot.shootingSystem.dt);

//        telemetry.addData("target rel angle deg lookahead", Math.toDegrees(lookAheadTargetRelAngleRad));
//        telemetry.addData("target rel angle deg no lookahead", Math.toDegrees(noLookAheadTargetRelAngleRad));
//        telemetry.addData("add oscillation data", addOscillationData);
//        telemetry.addData("prev errors", Arrays.toString(prevErrors));
//        telemetry.addData("error oscillating", errorIsOscillating(prevErrors));
//        telemetry.addData("was oscillating", wasOscillating);
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
        telemetry.addData("turret is pass position", trackCustomTargetPassPosition);
//        telemetry.addData("voltage kP", pVoltage);
//        telemetry.addData("voltage vel", vVoltage);
//        telemetry.addData("voltage accel", aVoltage);
//        telemetry.addData("voltage kd", dVoltage);
//        telemetry.addData("total voltage", totalVoltage);
//        telemetry.addData("kP", kP);
//        telemetry.addData("kf", fVoltage);
//        telemetry.addData("kV", kV);
//        telemetry.addData("kA", kA);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
//        telemetry.addData("target velocity", targetVelocity);
//        telemetry.addData("target accel", targetAccel);
//        telemetry.addData("encoder error", positionError);
        telemetry.addData("angle degree error", Math.toDegrees(positionError / turretParams.ticksPerRad));
//        telemetry.addData("velocity error", velocityError);
//        telemetry.addData("goal angular vel", goalAngularVel);
//        telemetry.addData("goal angular accel", goalAngularAccel);
//        telemetry.addData("target robot angular vel", -robot.drive.pinpoint().driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//        telemetry.addData("desired ball dir", Math.toDegrees(robot.shootingSystem.desiredBallDir));
//        telemetry.addData("actual turret target angle", Math.toDegrees(robot.shootingSystem.lookAheadTurretTargetAngleRad));
//        telemetry.addLine("-----");
        telemetry.addData("current encoder", currentEncoder);
//        telemetry.addData("current velocity", currentVelocity);
//        telemetry.addData("current acceleration", currentAcceleration);
//        telemetry.addLine("-----");
//        telemetry.addData("voltage", robot.getFilteredVoltage());
//        telemetry.addData("voltage raw", robot.getRawVoltage());
        telemetry.addLine("------");
//        telemetry.addLine("-----");
//        telemetry.addData("inRange", inRange());
//        telemetry.addData("onTarget", onTarget());
    }

    public void changeEncoderAdjustment(int amount) {
        if(robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
            nearEncoderAdjustment += amount;
        else
            farEncoderAdjustment += amount;
    }
    public void resetAllEncoderAdjustments() {
        nearEncoderAdjustment = 0;
        farEncoderAdjustment = 0;
    }
    public boolean inRange() {
        return inRange;
    }
    public boolean onTarget() {
        return onTarget;
    }
    public TurretState getTurretState() {
        return turretState;
    }
    public void setTurretState(TurretState turretState) {
        this.turretState = turretState;
    }

    public void rotateToRelativeCustomTarget(double targetAngle) {
        setTurretState(TurretState.TRACK_CUSTOM_TARGET);
        double clippedAngle = Range.clip(targetAngle, -turretParams.maxAngle, turretParams.maxAngle);
        targetEncoder = clippedAngle * turretParams.ticksPerRad;
        positionError = targetEncoder - currentEncoder;
        trackCustomTargetStartEncoder = currentEncoder;
    }
    public void setCustomTargetMinPower(double p) {
        trackCustomTargetMinPower = p;
    }
    public void setCustomTargetPassPosition(boolean passPosition) {
        trackCustomTargetPassPosition = passPosition;
        trackCustomTargetPassPositionDone = false;
    }
    public Action rotateToCustomTargetAction(DoubleSupplier targetFieldAngleSup) {
        return new SequentialAction(
                new InstantAction(() -> {
//                    turretState = TurretState.TRACK_CUSTOM_TARGET;
                    double targetFieldAngle = targetFieldAngleSup.getAsDouble();
                    double robotAngle = robot.drive.localizer.getPose().heading.toDouble();
                    double targetRelAngle = MathUtils.angleNormDeltaRad(targetFieldAngle - robotAngle);
                    rotateToRelativeCustomTarget(targetRelAngle);
//                    setCustomTargetPassPosition(true);
//                    double clippedAngle = Range.clip(targetRelAngle, -turretParams.maxAngle, turretParams.maxAngle);
//                    targetEncoder = clippedAngle * turretParams.ticksPerRad;
//                    positionError = targetEncoder - currentEncoder;
                    telemetry.addData("rotating to field angle", Math.toDegrees(targetFieldAngleSup.getAsDouble()));
//                    telemetry.addData("rotating to turret angle", Math.toDegrees(clippedAngle));
                }),
                new Action() {
                    ElapsedTime timer = null;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (timer == null) {
                            timer = new ElapsedTime();
                            timer.reset();
                        }
                        telemetry.addData("ROTATE TO CUSTOM TARGET TIME", timer.seconds());
                        return positionError > powerTuning.noVoltageThreshold || timer.seconds() < 0.2;
                    }
                }
        );
    }
}