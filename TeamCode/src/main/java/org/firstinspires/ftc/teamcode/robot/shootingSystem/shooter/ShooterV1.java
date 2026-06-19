package org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterV1 extends Shooter {
    public static class Params {
        public double speedAdjustment = 20;
        public double A = 20, B = 6, k = -150, x0 = .06;
        public double kF = 0, kV = -0.02005;

        public double minVelForShot = 0;
        public double shotVelDropThreshold = 0;
    }
    public static Params params = new Params();
    public ShooterV1(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }
    @Override
    public void updateProperties() {
        curShooterVelTps = (lowShooter.getVelocity() + highShooter.getVelocity()) * .5;
        trackBallShots();
    }
    @Override
    public double getKP(double error) {
        return params.A / (1 + Math.exp(params.k * (Math.abs(error) - params.x0))) + params.B;
    }
    @Override
    public double getKV() {
        return params.kV;
    }
    @Override
    public double getKF() {
        return params.kF;
    }
    @Override
    public double getSpeedAdjustment() {
        return params.speedAdjustment;
    }
    @Override
    public double getMinVelForShot() {
        return params.minVelForShot;
    }
    @Override
    public double getShotVelDropThreshold() {
        return params.shotVelDropThreshold;
    }
}