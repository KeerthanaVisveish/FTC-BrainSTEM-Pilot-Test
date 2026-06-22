package org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;

@Config
public class ShooterV2 extends Shooter {
    public static class Params {
        public double kP = 0;
        public double kV = 0;
        public double kF = 0;
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
        curShooterVelTps = (srsHub.getShooterLowVelocity() + srsHub.getShooterHighVelocity()) * .5;
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
}