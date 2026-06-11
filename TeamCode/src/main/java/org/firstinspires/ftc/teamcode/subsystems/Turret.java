package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShootingSystem.Location.FAR;

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
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.function.DoubleSupplier;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean enableKF = true;
        public boolean enableKV = true;
        public boolean enableMotionProfiling = false;
        public boolean enableKA = false;
        public boolean enablePID = true;
        public boolean enableTorqueMitigation = true;
        public boolean enablePower = true;
//        public boolean usingMotionProfiling = false;
    }
    public static class TurretParams {
        public double minParkRotateVoltage = 9;
        public double gateCollectYThreshold = 58, gateCollectMaxAngle = Math.toRadians(30);
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public double accelFilterConcavity = 2, accelFilterMax = 200;
        public double maxTeleAngle = Math.toRadians(90), maxAutoAngle = Math.toRadians(90);
        public double maxNearClutchEngageError = 25, maxFarClutchEngageError = 15; // if the turret error is greater than this, do not allow the intake to spin while the clutch is engaged
        public double maxFarTeleClutchEngageVelocityError = ticksPerRad * Math.toRadians(25);
        public double maxFarAutoClutchEngageVelocityError = ticksPerRad * Math.toRadians(10);
        public double outOfRangeAngleLerpStart = Math.toRadians(135);
    }
    public static class PowerTuning {
        public double kP = .028, kI = 0, kD = 0;
        public double maxPid = 3.5;
        public double maxIntegral = 5;
        public double motionProfileAccel = .01, motionProfileMaxVel = 1, motionProfileDeadZone = 5;
        public double maxAngularVelocity = Math.toRadians(240);
        public double ignoreTargetVelocityNoiseThreshold = .5, ignoreTargetAccelNoiseThreshold = 1;
//        public double APos = .018, BPos = 0.025, x0Pos = 60, kPos = .03;
//        public double x0kPScaler = 20, kKPScaler = .2, BKPScaler = 1;
//        public double AVel = 0, BVel = 0.007, x0Vel = 140, kVel = .05;
//        public double kA = 0;
        public double dampeningRadius = 1, dampeningFactor = .5;
        public double maxVoltageInRange = 7, maxVoltageOutOfRange = 5;

        public double kT = -.015;
        public double kV = 0;

        public double[] kfPosLookupData = new double[] {
                -350, .4,
                -300, .4,
                -130, .4,
                0, .4,
                30, .45,
                100, .5,
                120, .6,
                160, .6,
                170, .7,
                190, .8,
                230, .85,
                300, 1.05,
                350, 1.05
        };
        public double[] kfNegLookupData = new double[] {
                -350, -1,
                -300, -1,
                -156, -.9,
                -130, -.8,
                -75, -.7,
                -25, -.6,
                0, -.5,
                5, -.5,
                110, -.4,
                170, -.4,
                350, -.4
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
    private double targetVelocity, motionProfileVel;
    private double targetAccel;
    public double currentEncoder, currentVelocity, currentAcceleration;
    public double positionError, actualPositionError;
    public Vector2d perpVelVec;
    private double goalAngularVel;
    private double kP, fVoltage;
    private double kV, kA;
    private final PIDController pidController;
    private double pidVoltage, totalVoltage;
    private double vVoltage;
    private double aVoltage;
    private double tVoltage;
    private double accelMagAtTurret;
    private final InterpLUT kFPosLookup, kfNegLookup;

    public double lookAheadTargetRelAngleRad, noLookAheadTargetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double curRelAngleRad;
    private boolean inRange, onTarget;

    private double currentTestingTarget;
    private boolean smoothWhenOutOfRange;
//    private boolean wasOscillating;
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

        pidController = new PIDController(powerTuning.kP, powerTuning.kI, powerTuning.kD);
        pidController.setMaxIntegral(powerTuning.maxIntegral);

        smoothWhenOutOfRange = true;
    }

    @Override
    public void update() {
        double prevVelocity = currentVelocity;
        currentVelocity = robot.shootingSystem.getTurretVelTps();
        currentAcceleration = (currentVelocity - prevVelocity) / robot.shootingSystem.dt;

        curRelAngleRad = currentEncoder / turretParams.ticksPerRad;
        currentAbsoluteAngleRad = curRelAngleRad + robot.drive.localizer.getPose().heading.toDouble();

        double motorVoltage;
        updateTargetToGoal();
        switch (turretState) {
            case TRACKING:
                motorVoltage = calculateTurretVoltage(noLookAheadTargetRelAngleRad * turretParams.ticksPerRad, positionError);
                robot.shootingSystem.setTurretVoltage(motorVoltage);
                break;
            case CENTER:
                inRange = true;
                targetEncoder = 0;
                positionError = -currentEncoder;
                motorVoltage = calculateTurretVoltage(targetEncoder, positionError);
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
                    motorVoltage = calculateTurretVoltage(targetEncoder, positionError);
                    motorVoltage = Math.max(trackCustomTargetMinPower, Math.abs(motorVoltage)) * Math.signum(motorVoltage);
                    robot.shootingSystem.setTurretVoltage(motorVoltage);
                }
                break;
        }
    }

    public double calculateTurretVoltage(double actualTargetEncoder, double positionError) {
        double maxError = robot.shootingSystem.locationState == FAR ? turretParams.maxFarClutchEngageError : turretParams.maxNearClutchEngageError;
        onTarget = Math.abs(actualTargetEncoder - currentEncoder) <= maxError
                && (robot.shootingSystem.locationState != FAR || Math.abs(targetVelocity - currentVelocity) < (robot.collector.inAuto() ? turretParams.maxFarAutoClutchEngageVelocityError : turretParams.maxFarTeleClutchEngageVelocityError));

        if(testingParams.enableKF) {
            double dir = targetVelocity == 0 ? Math.signum(positionError) : Math.signum(targetVelocity);
            double maxAngle = robot.collector.inAuto() ? turretParams.maxAutoAngle : turretParams.maxTeleAngle;
            double maxEncoder = maxAngle * turretParams.ticksPerRad;
            double input = Range.clip(currentEncoder, -maxEncoder, maxEncoder); // reversing input if traveling in the opposite direction
            fVoltage = dir == 1 ? kFPosLookup.get(input) : kfNegLookup.get(input);
        }
        else
            fVoltage = 0;

        if(testingParams.enablePID) {
            pidController.setPIDValues(powerTuning.kP, powerTuning.kI, powerTuning.kD);
            pidVoltage = Range.clip(pidController.updateWithError(positionError), -powerTuning.maxPid, powerTuning.maxPid);
        }
        else
            pidVoltage = 0;

        if(testingParams.enableKV) {
            kV = powerTuning.kV;
            vVoltage = kV * targetVelocity;
        }
        else
            vVoltage = 0;

        if(testingParams.enableKA) {
//            kA = powerTuning.kA;
            aVoltage = kA * targetAccel;
        }
        else
            aVoltage = 0;

        OdoInfo robotAccel = robot.drive.pinpoint().getRawAccel();
        if(testingParams.enableTorqueMitigation && robotAccel != null) {
            telemetry.addData("robot accel", robotAccel.x + ", " + robotAccel.y + ", " + robotAccel.headingRad);
            double robotHeading = robot.drive.pinpoint().getPose().heading.toDouble();

            Vector2d linearRobotAccel = new Vector2d(robotAccel.x, robotAccel.y);


            // a = r * alpha
            Vector2d tangentialAccel = new Vector2d(-Math.sin(robotHeading), Math.cos(robotHeading))
                    .times(BrainSTEMRobot.turretToCenterOfMassDist)
                    .times(robotAccel.headingRad);

            Vector2d linearAccelAtTurret = linearRobotAccel.plus(tangentialAccel);

            accelMagAtTurret = Math.hypot(linearAccelAtTurret.x, linearAccelAtTurret.y);
            // filtering accel mag at turret
            accelMagAtTurret = Math.pow(Math.min(Math.abs(accelMagAtTurret) / turretParams.accelFilterMax, 1), turretParams.accelFilterConcavity) * Math.signum(accelMagAtTurret) * 250;

            double angleDif = currentEncoder / turretParams.ticksPerRad - Math.atan2(robotAccel.y, robotAccel.x);
            tVoltage = -powerTuning.kT * accelMagAtTurret * Math.sin(angleDif);
        }

        if(testingParams.enablePower) {
            if(Math.abs(positionError) < powerTuning.dampeningRadius)
                fVoltage *= powerTuning.dampeningFactor;

            totalVoltage = fVoltage + pidVoltage + vVoltage + aVoltage + tVoltage;
        }
        else
            totalVoltage = 0;

        double maxVolts = inRange ? powerTuning.maxVoltageInRange : powerTuning.maxVoltageOutOfRange;
        totalVoltage = Range.clip(totalVoltage, -maxVolts, maxVolts);
        return totalVoltage;
    }
//    private double getLogisticErrorKP(double errorMag) {
//        return powerTuning.APos / (1 + Math.exp(powerTuning.kPos * (errorMag - powerTuning.x0Pos)) ) + powerTuning.BPos;
//    }
//    private double getLogisticKPScaler(double targetVelMag) {
//        return (1 - powerTuning.BKPScaler) / (1 + Math.exp(powerTuning.kKPScaler * (targetVelMag - powerTuning.x0kPScaler))) + powerTuning.BKPScaler ;
//    }
//    private double getLogisticKV(double targetRotVelMag) {
//        return powerTuning.AVel / (1 + Math.exp(powerTuning.kVel * (targetRotVelMag - powerTuning.x0Vel))) + powerTuning.BVel;
//    }
    public void setSmoothWhenOutOfRange(boolean smoothWhenOutOfRange) {
        this.smoothWhenOutOfRange = smoothWhenOutOfRange;
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    private void updateTargetToGoal() {
        // calculating target angular velocity
        double targetAngularVelocity = 0;

        if(turretState == TurretState.TRACKING) {
            Vector2d robotPos = robot.drive.pinpoint().getPose().position;
            OdoInfo robotVelocity = robot.drive.pinpoint().getVelocity();
            double deltaX = robotPos.x - robot.shootingSystem.goalPosIn.x;
            double deltaY = robotPos.y - robot.shootingSystem.goalPosIn.z;
            double num = robotVelocity.y * deltaX - robotVelocity.x * deltaY;
            double denom = deltaX * deltaX + deltaY * deltaY;
            targetAngularVelocity = num / denom - robotVelocity.headingRad;
            targetAngularVelocity = Range.clip(targetAngularVelocity, -powerTuning.maxAngularVelocity, powerTuning.maxAngularVelocity);
        }

        // motion profiling code
        if(testingParams.enableMotionProfiling) {
            // x = (v1^2 - v0^2) / (2a)
            // v0^2 = v1^2 - 2ax
            motionProfileVel = Math.signum(positionError) * Math.min(powerTuning.motionProfileMaxVel, Math.sqrt(2 * powerTuning.motionProfileAccel * Math.max(0, Math.abs(positionError) - powerTuning.motionProfileDeadZone)));
            targetAngularVelocity += motionProfileVel;
        }


        // updating target angular acceleration
        double prevTargetVel = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        if(Math.abs(targetVelocity) < powerTuning.ignoreTargetVelocityNoiseThreshold)
            targetVelocity = 0;
        targetAccel = (targetVelocity - prevTargetVel) / robot.shootingSystem.dt;
        if(Math.abs(targetAccel) < powerTuning.ignoreTargetAccelNoiseThreshold)
            targetAccel = 0;

        // updating target angle
        if(turretState == TurretState.TRACKING) {
            noLookAheadTargetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.currentTurretTargetAngleRad - robot.drive.localizer.getPose().heading.toDouble());
            lookAheadTargetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.lookAheadTurretTargetAngleRad - robot.shootingSystem.futureRobotPose.heading.toDouble());
        }
        else if(turretState == TurretState.CENTER) {
            noLookAheadTargetRelAngleRad = 0;
            lookAheadTargetRelAngleRad = 0;
        }
        else {
            noLookAheadTargetRelAngleRad = targetEncoder;
            lookAheadTargetRelAngleRad = targetEncoder;
        }
        double absoluteMaxAngle = robot.collector.inAuto() ? turretParams.maxAutoAngle : turretParams.maxTeleAngle;
        Pose2d pose = robot.drive.localizer.getPose();
        boolean useSmallerRange = pose.position.x > 0 && pose.position.x < 20 && Math.abs(pose.position.y) > turretParams.gateCollectYThreshold;
        double maxAngle = useSmallerRange ? turretParams.gateCollectMaxAngle : absoluteMaxAngle;
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
        targetEncoder += robot.shootingSystem.locationState != FAR ? nearEncoderAdjustment : farEncoderAdjustment;
        double maxBound = absoluteMaxAngle * turretParams.ticksPerRad;
        targetEncoder = Range.clip(targetEncoder, -maxBound, maxBound);
        positionError = targetEncoder - currentEncoder;
        actualPositionError = noLookAheadTargetRelAngleRad * turretParams.ticksPerRad - currentEncoder;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
        telemetry.addData("turret state", turretState);
        telemetry.addLine("-----");
        telemetry.addData("turret raw accel at turret", accelMagAtTurret);
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
        telemetry.addData("turret is pass position", trackCustomTargetPassPosition);
        telemetry.addData("voltage kF", fVoltage);
        telemetry.addData("voltage pid", pidVoltage);
        telemetry.addData("voltage kV", vVoltage);
        telemetry.addData("voltage kA", aVoltage);
        telemetry.addData("voltage kT", tVoltage);
        telemetry.addData("total voltage", totalVoltage);
        telemetry.addData("goal angular vel", goalAngularVel);
        telemetry.addLine("-----");
        telemetry.addData("target velocity tps", targetVelocity);
        telemetry.addData("motion profile vel rad/s", motionProfileVel);
        telemetry.addData("current velocity tps", currentVelocity);
        telemetry.addData("velocity error tps", targetVelocity - currentVelocity);
//        telemetry.addData("target accel", targetAccel);
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("current encoder", currentEncoder);
        telemetry.addData("position error", positionError);
        telemetry.addData("lookahead angle error deg", MathUtils.format3(Math.toDegrees(positionError / turretParams.ticksPerRad)));
        telemetry.addData(" actual angle error deg", MathUtils.format3(Math.toDegrees(actualPositionError / turretParams.ticksPerRad)));
//        telemetry.addData("goal angular vel", goalAngularVel);
//        telemetry.addData("goal angular accel", goalAngularAccel);
//        telemetry.addData("target robot angular vel", -robot.drive.pinpoint().driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//        telemetry.addData("desired ball dir", Math.toDegrees(robot.shootingSystem.desiredBallDir));
//        telemetry.addData("actual turret target angle", Math.toDegrees(robot.shootingSystem.lookAheadTurretTargetAngleRad));
//        telemetry.addLine("-----");
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
        if(robot.shootingSystem.locationState != FAR)
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
        double clippedAngle = Range.clip(targetAngle, -turretParams.maxTeleAngle, turretParams.maxTeleAngle);
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
                        return isAccurate() || timer.seconds() < 0.2;
                    }
                }
        );
    }
    public boolean isAccurate() {
        return Math.abs(positionError) < powerTuning.dampeningRadius;
    }
}