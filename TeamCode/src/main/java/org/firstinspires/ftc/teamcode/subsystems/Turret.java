package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean actuallyPowerTurret = true;
        public boolean testNewControl = true;
    }
    public static class Params {
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
        public double mathRes = 0.001;
    }
    public static class PowerTuning {
        public double rightKp = 0.003, rightKi = 0, rightKd = 0.000; // old kD: 0.0005
        public double leftKp = 0.003, leftKi = 0, leftKd = 0.000; // old kD: 0.001
        public double noPowerThreshold = 3;
        public double linearDistToResetKiThreshold = 1;
        public double headingDegToResetKiThreshold = 1;

        public int kDSignMult = -1;

        public double moveRightKf = -0.08;
        public double moveLeftKf = 0.08;
        public double inThresholdKfPower = 0.01;

        public double kV = 0.04, kA = 0.003;
        public double kAJoystick = 0.00;
        public double tauVel = 0.1, tauAccel = 0.1;
        public double transitionTime = 0.2;
    }
    public static TestingParams testingParams = new TestingParams();
//    public static GoalParams goalParams = new GoalParams();
    public static Params turretParams = new Params();
    public static PowerTuning powerTuning = new PowerTuning();
    public enum TurretState {
        TRACKING, CENTER
    }
    private final PIDController pidController;
    public TurretState turretState;
    private int nearEncoderAdjustment, farEncoderAdjustment;
    public int targetEncoder;
    private double motorError;
    private double kF;
    private final ElapsedTime lerpTimer;

    public double targetRelAngleRad, prevTargetRelAngleRad;
    protected double targetRelAngleRadVel, prevTargetRelAngleRadVel;
    protected double targetRelAngleRadAccel;

    public double currentAbsoluteAngleRad;
    public double currentRelativeAngleRad, prevRelativeAngleRad;
    public double currentRelativeAngleRadVel, prevRelativeAngleRadVel;
    public double currentRelativeAngleRadAccel;
    public boolean inRange;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        pidController = new PIDController(powerTuning.rightKp, powerTuning.rightKi, powerTuning.rightKd);
        turretState = TurretState.CENTER;
        lerpTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        int turretEncoder = robot.shootingSystem.getTurretEncoder();
        switch (turretState) {
            case TRACKING:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretPower(0);
                    break;
                }
                updatePosVelAccel();
                int targetTurretPosition = (int) (targetRelAngleRad * turretParams.ticksPerRad);
                targetTurretPosition += robot.shootingSystem.distState != ShootingSystem.Dist.FAR ? nearEncoderAdjustment : farEncoderAdjustment;

                if(!inRange)
                    lerpTimer.reset();

                if(testingParams.actuallyPowerTurret) {
                    if (testingParams.testNewControl )
                        robot.shootingSystem.setTurretPower(calculateTurretPowerNew(targetTurretPosition, turretEncoder));
                    else
                        robot.shootingSystem.setTurretPower(calculateTurretPower(targetTurretPosition, turretEncoder));
                }
                else
                    robot.shootingSystem.setTurretPower(0);
                break;

            case CENTER:
                lerpTimer.reset();
                inRange = true;
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    robot.shootingSystem.setTurretPower(0);
                    break;
                }

                robot.shootingSystem.setTurretPower(calculateTurretPower(0, turretEncoder));
                robot.shootingSystem.actualTurretTargetAngleRad = robot.drive.localizer.getPose().heading.toDouble();
                break;
        }

        // updating CURRENT relative angle values
        prevRelativeAngleRad = currentRelativeAngleRad;
        currentRelativeAngleRad = turretEncoder / turretParams.ticksPerRad;

        prevRelativeAngleRadVel = currentRelativeAngleRadVel;
        currentRelativeAngleRadVel = (currentRelativeAngleRad - prevRelativeAngleRad) / robot.shootingSystem.dt;

        currentRelativeAngleRadAccel = (currentRelativeAngleRadVel - prevRelativeAngleRadVel) / robot.shootingSystem.dt;

        currentAbsoluteAngleRad = currentRelativeAngleRad + robot.drive.localizer.getPose().heading.toDouble();
    }

    public double calculateTurretPowerNew(int ticks, int currentEncoder) {
        double extraPower = inRange ? powerTuning.kA * targetRelAngleRadAccel + powerTuning.kV * targetRelAngleRadVel : 0;
        extraPower = MathUtils.lerp(0, extraPower, Math.min(lerpTimer.seconds() / powerTuning.transitionTime, 1));
        extraPower = Range.clip(extraPower, -0.99, 0.99);
        return extraPower + calculateTurretPower(ticks, currentEncoder);
    }
    public double calculateTurretPower(int ticks, int currentEncoder) {
        targetEncoder = Range.clip(ticks, turretParams.RIGHT_BOUND, turretParams.LEFT_BOUND);
        motorError = currentEncoder - targetEncoder;
        boolean movingRight = motorError > 0;
        // within threshold - give 0 power
        if (Math.abs(motorError) <= powerTuning.noPowerThreshold)
            return -Math.signum(motorError) * powerTuning.inThresholdKfPower;

        // reset kI whenever the robot moves significantly
        Pose2d prevPose = robot.drive.pinpoint().lastPose;
        Pose2d curPose = robot.drive.pinpoint().getPose();
        double positionChange = Math.hypot(curPose.position.x - prevPose.position.x, curPose.position.y - prevPose.position.y);
        double headingDegChange = Math.toDegrees(curPose.heading.toDouble() - prevPose.heading.toDouble());
        if (positionChange > powerTuning.linearDistToResetKiThreshold || headingDegChange >= powerTuning.headingDegToResetKiThreshold)
            pidController.reset();

        updatePIDFValues(movingRight);

        double newPower = -pidController.updateWithError(motorError);
        newPower += kF;

        if (currentEncoder < turretParams.RIGHT_BOUND)
            newPower = Math.max(newPower, 0);
        else if (currentEncoder > turretParams.LEFT_BOUND)
            newPower = Math.min(newPower, 0);

        return newPower;
    }
    private void updatePIDFValues(boolean movingRight) {
        double kP = movingRight ? powerTuning.rightKp : powerTuning.leftKp;
        double kI = movingRight ? powerTuning.rightKi : powerTuning.leftKi;
        double kD = movingRight ? powerTuning.rightKd : powerTuning.leftKd;
        if(Math.abs(targetRelAngleRadAccel) < 2000)
            kD = 0;
        pidController.setPIDValues(kP, kI, kD);
        pidController.setPermanentKdSign(movingRight ? powerTuning.kDSignMult : -powerTuning.kDSignMult);
        if (movingRight)
            kF = powerTuning.moveRightKf;
        else
            kF = powerTuning.moveLeftKf;
        if (movingRight)
            kF = Math.min(kF, 0);
        else
            kF = Math.max(kF, 0);
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    private void updatePosVelAccel() {
        // updating position variables
        prevTargetRelAngleRad = targetRelAngleRad;
        targetRelAngleRad = MathUtils.angleNormDeltaRad(robot.shootingSystem.actualTurretTargetAngleRad - robot.shootingSystem.futureRobotPose.heading.toDouble());

        // updating velocity variables
        prevTargetRelAngleRadVel = targetRelAngleRadVel;
        double rawTargetRelAngleRadVel = -robot.shootingSystem.odoVel.headingRad;
//        double rawTargetRelAngleRadVel = MathUtils.angleNormDeltaRad(targetRelAngleRad - prevTargetRelAngleRad) / robot.shootingSystem.dt;
        // low pass filter:
        // a = e^(-dt/tau)
        // x1 = ax0 + (1-a)x
        double aVel = Math.exp(-robot.shootingSystem.dt / powerTuning.tauVel);
        targetRelAngleRadVel = aVel * prevTargetRelAngleRadVel + (1 - aVel) * rawTargetRelAngleRadVel;
        if(Math.abs(targetRelAngleRadVel) < turretParams.mathRes)
            targetRelAngleRadVel = 0;

        double rawTargetRelAngleRadAccel = (targetRelAngleRadVel - prevTargetRelAngleRadVel) / robot.shootingSystem.dt;
        double aAccel = Math.exp(-robot.shootingSystem.dt / powerTuning.tauAccel);
        targetRelAngleRadAccel = aAccel * targetRelAngleRadAccel + (1 - aAccel) * rawTargetRelAngleRadAccel;
        if(Math.abs(targetRelAngleRadAccel) < turretParams.mathRes)
            targetRelAngleRadAccel = 0;

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
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = turretParams.TICKS_PER_REV / 360.;
        int turretEncoder = robot.shootingSystem.getTurretEncoder();
        double angleDegError = motorError / turretTicksPerDegree;

        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addData("turret power", robot.shootingSystem.getTurretPower());
//        telemetry.addData("turret kF", kF);
//        telemetry.addData("turret PID integral", pidController.getIntegral());
//        telemetry.addData("turret PID derivative", pidController.getDerivatve());
        telemetry.addData("current encoder", turretEncoder);
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("encoder error", motorError);
        telemetry.addData("moving right", motorError > 0);
        telemetry.addData("angle degree error", angleDegError);
        telemetry.addLine("------------------");
        telemetry.addData("turret current relative angle deg", Math.toDegrees(currentRelativeAngleRad));
        telemetry.addData("turret current relative vel deg", Math.toDegrees(currentRelativeAngleRadVel));
        telemetry.addData("turret current relative accel deg", Math.toDegrees(currentRelativeAngleRadAccel));
        telemetry.addLine("------------------");
        telemetry.addData("turret target relative angle deg", Math.toDegrees(targetRelAngleRad));
        telemetry.addData("turret target relative angle vel", Math.toDegrees(targetRelAngleRadVel));
        telemetry.addData("turret target relative angle accel", Math.toDegrees(targetRelAngleRadAccel));
        telemetry.addLine();
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
