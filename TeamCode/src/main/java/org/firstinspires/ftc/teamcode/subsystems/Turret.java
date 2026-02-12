package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean actuallyPowerTurret = true;
    }
    public static class Params {
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public int minBound = -300;
        public int maxBound = 300;
    }
    public static class PowerTuning {
        public double ignoreAngularVelocityThreshold = Math.toRadians(5);
        public double noPowerK = 6, noPowerX0 = 1.75;
        public double staticU = .14, staticB = .06, staticK = .01, staticX0 = 100;
        public double kP = 0.0008, kV = 0.0003, kVP = 0.001;
        public double decelTime = .2;
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
    public double targetEncoder, targetVelocity, targetAngularVelocity, dot;
    private double prevTargetVelocity, firstTimeWhereTargetVelIsZero;

    private Vector2d perpVelVec;
    public double currentEncoder, currentVelocity;
    private double positionError, velocityError;
    private double kF, powerScaler, dir;

    public double targetRelAngleRad;

    public double currentAbsoluteAngleRad;
    public double currentRelativeAngleRad;
    public boolean inRange;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);
        turretState = TurretState.CENTER;
    }

    @Override
    public void update() {
        currentEncoder = robot.shootingSystem.getTurretEncoder();
        currentVelocity = robot.shootingSystem.getTurretVelTps();
        switch (turretState) {
            case TRACKING:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretPower(0);
                    break;
                }
                updateTarget();
                targetEncoder = (int) (targetRelAngleRad * turretParams.ticksPerRad);
                targetEncoder += robot.shootingSystem.distState != ShootingSystem.Dist.FAR ? nearEncoderAdjustment : farEncoderAdjustment;
                targetEncoder = Range.clip(targetEncoder, turretParams.minBound, turretParams.maxBound);

                if(testingParams.actuallyPowerTurret)
                    robot.shootingSystem.setTurretPower(calculateTurretPowerNew());
                else
                    robot.shootingSystem.setTurretPower(0);
                break;

            case CENTER:
                inRange = true;
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretPower(0);
                    break;
                }
                targetEncoder = 0;
                robot.shootingSystem.setTurretPower(calculateTurretPowerNew());
                break;
        }

        currentRelativeAngleRad = currentEncoder / turretParams.ticksPerRad;
        currentAbsoluteAngleRad = currentRelativeAngleRad + robot.drive.localizer.getPose().heading.toDouble();
    }

    public double calculateTurretPowerNew() {
        positionError = targetEncoder - currentEncoder;
        double timeSinceTargetVelFirstZero = (System.currentTimeMillis() - firstTimeWhereTargetVelIsZero) / 1000;
        velocityError = targetVelocity == 0 && timeSinceTargetVelFirstZero > powerTuning.decelTime ? 0 : targetVelocity - currentVelocity;
        dir = Math.signum(positionError);
        kF = getLogisticKf(currentEncoder, dir);
        powerScaler = getLogisticPowerScaler(Math.abs(positionError));
        double power = powerTuning.kP * positionError + kF + powerTuning.kV * targetVelocity + powerTuning.kVP * velocityError;
        return power * powerScaler;
    }
    private double getLogisticKf(double encoder, double direction) {
        if(direction == -1)
            encoder *= -1;
        double logisticPower = (powerTuning.staticU - powerTuning.staticB) / (1 + Math.exp(-powerTuning.staticK * (encoder-powerTuning.staticX0))) + powerTuning.staticB;
        return logisticPower * direction;
    }
    private double getLogisticPowerScaler(double errorMag) {
        return 1 / (1 + Math.exp(-powerTuning.noPowerK * (errorMag - powerTuning.noPowerX0)));
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
        if(Math.abs(targetAngularVelocity) < powerTuning.ignoreAngularVelocityThreshold)
            targetAngularVelocity = 0;
        prevTargetVelocity = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        if(targetVelocity == 0 && prevTargetVelocity != 0)
            firstTimeWhereTargetVelIsZero = System.currentTimeMillis();

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
        telemetry.addData("kf", kF);
        telemetry.addData("power scaler", powerScaler);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("target angular velocity", targetAngularVelocity);
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
