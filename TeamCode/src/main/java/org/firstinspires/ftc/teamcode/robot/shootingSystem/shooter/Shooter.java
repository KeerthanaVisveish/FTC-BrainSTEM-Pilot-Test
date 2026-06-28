package org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;

@Config
public abstract class Shooter extends Component {
    public static double rampUpScale = .6, rampUpSpeed = 200;
    private double targetVelTps;
    protected double curShooterVelTps, prevVelForShotTracking;
    public static boolean useBatteryVoltage = false;

    private double pidVoltage, velocityVoltage, frictionVoltage, totalVoltage;
    protected final DcMotorEx lowShooter, highShooter;


    private double velDrop;
    private int numBallsShot;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        lowShooter = hardwareMap.get(DcMotorEx.class, RobotProperties.lowShooterName);
        lowShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lowShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lowShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        highShooter = hardwareMap.get(DcMotorEx.class, RobotProperties.highShooterName);
        highShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        highShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        highShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setShooterPower(double power) {
        lowShooter.setPower(power);
        highShooter.setPower(power);
    }
    public void setShooterVoltage(double shooterVoltage, double batteryVoltage) {
        setShooterPower(shooterVoltage / batteryVoltage);
    }
    public void setShooterVelocityPID(double targetVelTps, double batteryVoltage) {
        this.targetVelTps = targetVelTps;
        double error = targetVelTps - getVelTps();
        double kP = getKP(error);
        pidVoltage = kP * error;
        double kV = getKV();
        velocityVoltage = kV * targetVelTps;
        frictionVoltage = Math.signum(targetVelTps) * getKF();
        totalVoltage = pidVoltage + velocityVoltage + frictionVoltage;
        if(Math.abs(getVelTps()) < rampUpSpeed)
            totalVoltage *= rampUpScale;

        if(useBatteryVoltage)
            setShooterVoltage(totalVoltage, batteryVoltage);
        else
            setShooterPower(totalVoltage / 13);
    }
    protected void trackBallShots() {
        velDrop = curShooterVelTps - prevVelForShotTracking;
        if (prevVelForShotTracking > getMinVelForShot() && velDrop > getShotVelDropThreshold())
            numBallsShot++;
        prevVelForShotTracking = curShooterVelTps;
    }
    public void resetNumBallsShot() {
        numBallsShot = 0;
    }
    public int getNumBallsShot() {
        return numBallsShot;
    }
    public double getVelTps() {
        return curShooterVelTps;
    }
    public abstract void updateProperties();
    public abstract double getKP(double error);
    public abstract double getKV();
    public abstract double getKF();
    public abstract double getSpeedAdjustment();
    public abstract double getMinVelForShot();
    public abstract double getShotVelDropThreshold();
    public abstract double getNormTolerance(ShootingSystem.Location location);
    public abstract double getFirstShootTolerance();
    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("SH target speed", targetVelTps);
        telemetry.addData("SH current speed", curShooterVelTps);
        telemetry.addData("SH   shooter voltage total", totalVoltage);
        telemetry.addData("SH   shooter voltage pid", pidVoltage);
        telemetry.addData("SH   shooter voltage velocity", velocityVoltage);
        telemetry.addData("SH   shooter voltage friction", frictionVoltage);
        telemetry.addData("SH   vel drop", velDrop);
        double power = highShooter.getPower();
        telemetry.addData("SH shooter power", power);
        telemetry.addData("SH shooter power * 200", power * 200);
        telemetry.addData("SH motor combined current", highShooter.getCurrent(CurrentUnit.AMPS) + lowShooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("SH num balls shot", numBallsShot);
    }
}