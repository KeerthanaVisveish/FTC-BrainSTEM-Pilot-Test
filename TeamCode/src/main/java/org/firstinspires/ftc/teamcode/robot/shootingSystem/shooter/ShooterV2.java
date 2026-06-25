package org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem.ShootingSystem;

@Config
public class ShooterV2 extends Shooter {
    public static class Params {
        //linear: y=0.00783937x+0.468228
        // logarithmic: y=-28.38609 + 5.28369*ln(x)
        public Function<Double, Double> getMpsFunction = tps -> -28.38609 + 5.28369 * Math.log(tps);
        public Function<Double, Double> getTpsFunction = mps -> Math.exp((mps + 28.38609) / 5.28369);
        public double firstShootTolerance = 40, farShootTolerance = 50, closeShootTolerance = 90;
        public double velocitySign = -1;
        public double kP = 0.03;
        // y=0.0101436x+1.12464
        public double kV = 0.0075;
        public double kF = 1.12464;
        public double speedAdjustment = 0;
        public double minVelForShot = 0;
        public double shotVelDropThreshold = 0;
    }
    public static Params params = new Params();
    private final SRSHub srsHub;
    public ShooterV2(HardwareMap hardwareMap, Telemetry telemetry, SRSHub srsHub) {
        super(hardwareMap, telemetry);
        this.srsHub = srsHub;
    }
    @Override
    public void updateProperties() {
        curShooterVelTps = params.velocitySign * (srsHub.getShooterLowVelocity() + srsHub.getShooterHighVelocity()) * .5;
        trackBallShots();
    }
    @Override
    public double getKP(double error) {
        return params.kP;
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

    @Override
    public double getNormTolerance(ShootingSystem.Location location) {
        return location == ShootingSystem.Location.FAR ? params.farShootTolerance : params.closeShootTolerance;
    }

    @Override
    public double getFirstShootTolerance() {
        return params.firstShootTolerance;
    }
}