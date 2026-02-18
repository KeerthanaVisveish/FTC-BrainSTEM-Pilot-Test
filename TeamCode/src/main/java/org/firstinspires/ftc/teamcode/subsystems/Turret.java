package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean actuallyPowerTurret = true;
        public boolean enableTestingKF = false;
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
        public double ignoreAngularVelocityNoiseThreshold = Math.toRadians(5);
        public double ignoreAngularVelocityPositionThreshold = Math.toRadians(4) * turretParams.ticksPerRad;
        public double ignoreTargetAccelThreshold = 3000;
        public double A = .0, B = .03, x0 = 175, k = .02;
        public double smallKV = 0.001, bigKV = .001, kVP = 0, accelIncreaseKA = 0.000, accelDecreaseKA = .000;
        public double switchKVThreshold = 200;
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
                -350, -.3,
                -300, -.3,
                -75, -.45,
                -25, -.5,
                0, -.5,
                5, -.6,
                110, -.6,
                170, -.6,
                350, -.6
        };
        public double kfTestingVoltage = 0;
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
    public double targetEncoder, targetVelocity, prevTargetVelocity, targetAngularVelocity, dot;
    public double targetAccel;
    private Vector2d perpVelVec;
    public double currentEncoder, currentVelocity;
    public double positionError, velocityError;
    private double kP, kF, dir;
    private double pVoltage, vFFVoltage, vFBVoltage, aVoltage;
    private final InterpLUT kFPosLookup, kfNegLookup;

    public double targetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double currentRelativeAngleRad;
    public boolean inRange;

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
    }

    @Override
    public void update() {
        currentEncoder = robot.shootingSystem.getTurretEncoder();
        currentVelocity = robot.shootingSystem.getTurretVelTps();
        switch (turretState) {
            case TRACKING:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretVoltage(0);
                    break;
                }
                updateTarget();
                targetEncoder = (int) (targetRelAngleRad * turretParams.ticksPerRad);
                targetEncoder += robot.shootingSystem.distState != ShootingSystem.Dist.FAR ? nearEncoderAdjustment : farEncoderAdjustment;
                targetEncoder = Range.clip(targetEncoder, turretParams.minBound, turretParams.maxBound);

                if(testingParams.actuallyPowerTurret)
                    robot.shootingSystem.setTurretVoltage(calculateTurretVoltageNew());
                else
                    robot.shootingSystem.setTurretVoltage(0);
                break;

            case CENTER:
                inRange = true;
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretVoltage(0);
                    break;
                }
                targetEncoder = 0;
                robot.shootingSystem.setTurretVoltage(calculateTurretVoltageNew());
                break;
        }

        currentRelativeAngleRad = currentEncoder / turretParams.ticksPerRad;
        currentAbsoluteAngleRad = currentRelativeAngleRad + robot.drive.localizer.getPose().heading.toDouble();
    }

    public double calculateTurretVoltageNew() {
        positionError = targetEncoder - currentEncoder;
        if(Math.abs(positionError) <= powerTuning.noPowerThreshold)
            return 0;
        if(testingParams.enableTestingKF)
            return powerTuning.kfTestingVoltage;
        velocityError = targetVelocity == 0 ? 0 : targetVelocity - currentVelocity;
        dir = Math.signum(positionError);

        kP = getLogisticKP(Math.abs(positionError));
        double kV = Math.abs(targetVelocity) < powerTuning.switchKVThreshold ? powerTuning.bigKV : powerTuning.smallKV;

        double input = Range.clip(currentEncoder, turretParams.minBound, turretParams.maxBound); // reversing input if traveling in the opposite direction
        kF = dir == 1 ? kFPosLookup.get(input) : kfNegLookup.get(input);

        pVoltage = kP * positionError;
        vFFVoltage = kV * targetVelocity;
        vFBVoltage = powerTuning.kVP * velocityError;
        double kA = dir == Math.signum(targetVelocity) ? powerTuning.accelIncreaseKA : powerTuning.accelDecreaseKA;
        kA = Math.abs(targetAccel) > powerTuning.ignoreTargetAccelThreshold ? kA : 0;
        aVoltage = kA * targetAccel;
        return pVoltage + kF + vFFVoltage + vFBVoltage + aVoltage;
    }
    private double getLogisticKP(double errorMag) {
        return powerTuning.A / (1 + Math.exp(-powerTuning.k * (errorMag - powerTuning.x0)) ) + powerTuning.B;
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    private void updateTarget() {
        // updating position variables
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

        targetEncoder = targetRelAngleRad * turretParams.ticksPerRad;
        perpVelVec = new Vector2d(-robot.shootingSystem.futureExitPosRelativeToGoal.y, robot.shootingSystem.futureExitPosRelativeToGoal.x *1);
        perpVelVec = perpVelVec.div(robot.shootingSystem.futureExitPosGoalDistIn);
        dot = robot.shootingSystem.robotVelAtExitPosIps.dot(perpVelVec);
        targetAngularVelocity = dot / robot.shootingSystem.futureExitPosGoalDistIn - robot.shootingSystem.odoVel.headingRad;
        if(Math.abs(targetAngularVelocity) < powerTuning.ignoreAngularVelocityNoiseThreshold && Math.abs(positionError) < powerTuning.ignoreAngularVelocityPositionThreshold)
            targetAngularVelocity = 0;
        prevTargetVelocity = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        targetAccel = (targetVelocity - prevTargetVelocity) / robot.shootingSystem.dt;
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = turretParams.TICKS_PER_REV / 360.;
        int turretEncoder = robot.shootingSystem.getTurretEncoder();

        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addLine("OSCILATION DEBUGGING-----");
        if(perpVelVec != null) {
            telemetry.addData("perp vel vec mag", Math.hypot(perpVelVec.x, perpVelVec.y));
            telemetry.addData("heading vel deg", Math.toDegrees(robot.shootingSystem.odoVel.headingRad));
            telemetry.addData("dot", dot);
        }
        telemetry.addLine("-----");
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
        telemetry.addData("p Voltage", pVoltage);
        telemetry.addData("v ff Voltage", vFFVoltage);
        telemetry.addData("v fb Voltage", vFBVoltage);
        telemetry.addData("a Voltage", aVoltage);
        telemetry.addData("kP", kP);
        telemetry.addData("kf", kF);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("target angular velocity", targetAngularVelocity);
        telemetry.addData("target accel", targetAccel);
        telemetry.addLine("-----");
        telemetry.addData("current encoder", turretEncoder);
        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("turret current relative angle deg", Math.toDegrees(currentRelativeAngleRad));
        telemetry.addData("turret target relative angle deg", Math.toDegrees(targetRelAngleRad));
        telemetry.addData("dir", dir);
        telemetry.addLine("-----");
        telemetry.addData("angle degree error", positionError / turretTicksPerDegree);
        telemetry.addData("encoder error", positionError);
        telemetry.addData("velocity error", velocityError);
        telemetry.addLine("-----");
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