package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean enableKF = true;
        public boolean enableFeedForwardVelocity = true;
        public boolean enableKA = false;
        public boolean enablePID = true;
        public boolean enableTorqueMitigation = true;
        public boolean enablePower = true;
    }
    public static class TurretParams {
        public double minParkRotateVoltage = 9;
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public double angleAdjustment = Math.toRadians(3);
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public double accelFilterConcavity = 2, accelFilterMax = 200;
        public double maxAngle = Math.toRadians(90);
        public double outOfRangeAngleLerpStart = Math.toRadians(135);
    }
    public static class PowerTuning {
        public double kP = .028, kI = 0, kD = 0;
        public double maxPid = 3.5;
        public double maxIntegral = 5;
        public double motionProfileAccel = .01, motionProfileMaxVel = 1, motionProfileDeadZone = 5;
        public double dampeningRadius = Math.toRadians(1), dampeningFactor = .5;
        public double maxVoltage = 7, outOfRangeMaxVoltage = 5;

        public double kT = -.015;
        public double kV = 0;
        public double kA = 0;

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
    public static TurretParams turretParams = new TurretParams();
    public static PowerTuning powerTuning = new PowerTuning();
    private double currentAngleRad, currentVelocityRad, currentAccelerationRad;
    private double currentEncoder;


    private final PIDController pidController;
    private double fVoltage;
    private double pidVoltage;
    private double vVoltage;
    private double aVoltage;
    private double tVoltage;
    private double totalVoltage;
    private double torqueAtTurretAxisOfRotation;
    private final InterpLUT kFPosLookup, kfNegLookup;

    private boolean smoothWhenOutOfRange;

    private final DcMotorEx turretMotor;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        turretMotor = hardwareMap.get(DcMotorEx.class, RobotProperties.turretName);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kFPosLookup = new InterpLUT();
        for(int i = 0; i < powerTuning.kfPosLookupData.length; i+=2)
            kFPosLookup.add(powerTuning.kfPosLookupData[i], powerTuning.kfPosLookupData[i+1]);
        kFPosLookup.createLUT();
        kfNegLookup = new InterpLUT();
        for(int i = 0; i < powerTuning.kfNegLookupData.length; i+=2)
            kfNegLookup.add(powerTuning.kfNegLookupData[i], powerTuning.kfNegLookupData[i+1]);
        kfNegLookup.createLUT();

        pidController = new PIDController(powerTuning.kP, powerTuning.kI, powerTuning.kD);
        pidController.setMaxIntegral(powerTuning.maxIntegral);

        smoothWhenOutOfRange = false;
    }

    public void setTurretPower(double power) {
        turretMotor.setPower(power);
    }
    public void setTurretVoltage(double motorVoltage, double batteryVoltage) {
        setTurretPower(motorVoltage / batteryVoltage);
    }
    private double wrapTargetAngle(double targetAngleRad, double maxAngle, boolean smoothWhenOutOfRange) {
        boolean inRange = Math.abs(targetAngleRad) <= maxAngle;
        if (!inRange) {
            double max = Range.clip(targetAngleRad, -turretParams.maxAngle, turretParams.maxAngle);
            // mirrors the angle if the turret cannot reach it (visual cue)
            if (smoothWhenOutOfRange) {
                if (Math.abs(targetAngleRad) <= turretParams.outOfRangeAngleLerpStart)
                    targetAngleRad = max;
                else {
                    double a = Math.abs(turretParams.outOfRangeAngleLerpStart);
                    double b = Math.PI;
                    double x = Math.abs(targetAngleRad);
                    double outOfBoundsT = MathUtils.inverseLerp(a, b, x);
                    targetAngleRad = MathUtils.lerp(max, 0, outOfBoundsT);
                }
            }
            // clamps turret angle otherwise
            else
                targetAngleRad = max;
        }
        return targetAngleRad;
    }
    private double calculateExternalTorque(double robotHeading, OdoInfo robotAccel) {
        // estimating net external torque exerted on turret
        Vector2d linearRobotAccel = new Vector2d(robotAccel.x, robotAccel.y);
        // a = r * alpha
        Vector2d tangentialAccel = new Vector2d(-Math.sin(robotHeading), Math.cos(robotHeading))
                .times(RobotProperties.turretToCenterOfMassDist)
                .times(robotAccel.headingRad);
        Vector2d linearAccelAtTurret = linearRobotAccel.plus(tangentialAccel);
        double accelMagAtTurret = Math.hypot(linearAccelAtTurret.x, linearAccelAtTurret.y);
        // filtering accel mag at turret using lorenz curve
        accelMagAtTurret = Math.pow(Math.min(Math.abs(accelMagAtTurret) / turretParams.accelFilterMax, 1), turretParams.accelFilterConcavity) * Math.signum(accelMagAtTurret) * turretParams.accelFilterMax;
        double angleDif = currentEncoder / turretParams.ticksPerRad - Math.atan2(robotAccel.y, robotAccel.x);
        return accelMagAtTurret * Math.sin(angleDif);
    }
    public void updateProperties(double dt) {
        // updating turret position, velocity, acceleration
        currentEncoder = turretMotor.getCurrentPosition();
        currentAngleRad = currentEncoder / turretParams.ticksPerRad;
        double prevVelocityRad = currentVelocityRad;
        currentVelocityRad = turretMotor.getVelocity() / turretParams.ticksPerRad;
        currentAccelerationRad = (currentVelocityRad - prevVelocityRad) / dt;
    }
    public void controlTurretToTarget(double targetAngle, double targetVelocity, double targetAcceleration, double minVoltageMag, double maxVoltageMag, double robotHeading, OdoInfo robotAccel, double robotBattery) {
        torqueAtTurretAxisOfRotation = calculateExternalTorque(robotHeading, robotAccel);
        double wrappedTargetAngle = wrapTargetAngle(targetAngle, turretParams.maxAngle, smoothWhenOutOfRange);
        setTurretVoltage(calculateTurretVoltage(wrappedTargetAngle, targetVelocity, targetAcceleration, minVoltageMag, maxVoltageMag, torqueAtTurretAxisOfRotation), robotBattery);
    }

    public double calculateTurretVoltage(double targetAngle, double targetVelocity, double targetAcceleration, double minVoltageMag, double maxVoltageMag, double torqueAtTurretAxisOfRotation) {
        double positionError = targetAngle - currentAngleRad;

        if(testingParams.enableKF) {
            double dir = targetVelocity == 0 ? Math.signum(positionError) : Math.signum(targetVelocity);
            double maxEncoder = turretParams.maxAngle * turretParams.ticksPerRad;
            double input = Range.clip(currentEncoder, -maxEncoder, maxEncoder);
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

        if(testingParams.enableFeedForwardVelocity) {
            vVoltage = powerTuning.kV * targetVelocity;
        }
        else
            vVoltage = 0;

        if(testingParams.enableKA) {
            aVoltage = powerTuning.kA * targetAcceleration;
        }
        else
            aVoltage = 0;

        if(testingParams.enableTorqueMitigation)
            tVoltage = -powerTuning.kT * torqueAtTurretAxisOfRotation;
        else
            tVoltage = 0;

        if(testingParams.enablePower) {
            if(Math.abs(positionError) < powerTuning.dampeningRadius)
                fVoltage *= powerTuning.dampeningFactor;

            totalVoltage = fVoltage + pidVoltage + vVoltage + aVoltage + tVoltage;
        }
        else
            totalVoltage = 0;

        totalVoltage = Range.clip(Math.abs(totalVoltage), minVoltageMag, maxVoltageMag) * Math.signum(totalVoltage);

        if(currentAngleRad > turretParams.maxAngle)
            totalVoltage = Math.min(0, totalVoltage);
        else if(currentAngleRad < -turretParams.maxAngle)
            totalVoltage = Math.max(0, totalVoltage);

        return totalVoltage;
    }

    public void resetEncoders() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getEncoder() {
        return currentEncoder;
    }

    public double calculateMotionProfile(double targetAngle) {
        // x = (v1^2 - v0^2) / (2a)
        // v0^2 = v1^2 - 2ax
        double positionError = targetAngle - currentAngleRad;
        double motionProfileVel = Math.signum(positionError) * Math.min(powerTuning.motionProfileMaxVel, Math.sqrt(2 * powerTuning.motionProfileAccel * Math.max(0, Math.abs(positionError) - powerTuning.motionProfileDeadZone)));
        return motionProfileVel;

    }
    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
        telemetry.addLine("-----");

        telemetry.addData("voltage kF", fVoltage);
        telemetry.addData("voltage pid", pidVoltage);
        telemetry.addData("voltage kV", vVoltage);
        telemetry.addData("voltage kA", aVoltage);
        telemetry.addData("voltage kT", tVoltage);
        telemetry.addData("total voltage", totalVoltage);
        telemetry.addLine("-----");
        telemetry.addData("current encoder", currentEncoder);
        telemetry.addData("current angle", currentAngleRad);
        telemetry.addData("current vel", currentVelocityRad);
        telemetry.addData("current accel", currentAccelerationRad);
        telemetry.addLine("-----");
        telemetry.addData("external torque", torqueAtTurretAxisOfRotation);
    }

    public void setSmoothWhenOutOfRange(boolean s) {
        smoothWhenOutOfRange = s;
    }
    public double getCurAngleRad() {
        return currentAngleRad;
    }
    public double getCurVelRps() {
        return currentVelocityRad;
    }
}