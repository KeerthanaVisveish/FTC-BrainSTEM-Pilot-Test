package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.RobotProperties;

@Config
public class SRSHub {
    // absolute encoders
    public static int hoodAbsoluteEncoderPort = 1;
    public static int hoodServo1Port = 4;
    public static int hoodServo2Port = 5;

    // relative encoders
    public static int shooterEncoder1Port = 2;
    public static int shooterEncoder2Port = 3;

    private final SRSHubRaw srsHubRaw;

    public SRSHub(HardwareMap hardwareMap) {
        SRSHubRaw.Config config = new SRSHubRaw.Config();

        config.setEncoder(hoodAbsoluteEncoderPort, SRSHubRaw.Encoder.PWM);
        config.setEncoder(hoodServo1Port, SRSHubRaw.Encoder.PWM);
        config.setEncoder(hoodServo2Port, SRSHubRaw.Encoder.PWM);

        config.setEncoder(shooterEncoder1Port, SRSHubRaw.Encoder.QUADRATURE);
        config.setEncoder(shooterEncoder2Port, SRSHubRaw.Encoder.QUADRATURE);

        RobotLog.clearGlobalWarningMsg();

        srsHubRaw = hardwareMap.get(SRSHubRaw.class, RobotProperties.srsHubName);
        srsHubRaw.init(config);

        while(!srsHubRaw.ready());
        this.update();
    }

    public void update() {
        srsHubRaw.update();
    }
    public int getHoodAbsEncoder() {
        return srsHubRaw.readEncoder(hoodAbsoluteEncoderPort).position;
    }
    public double getHoodVelocity() {
        return srsHubRaw.readEncoder(hoodAbsoluteEncoderPort).velocity;
    }
    public double getHoodServo1Encoder() {
        return srsHubRaw.readEncoder(hoodServo1Port).position;
    }
    public double getHoodServo2Encoder() {
        return srsHubRaw.readEncoder(hoodServo2Port).position;
    }
    public double getShooter1Velocity() {
        return srsHubRaw.readEncoder(shooterEncoder1Port).velocity;
    }
    public double getShooter2Velocity() {
        return srsHubRaw.readEncoder(shooterEncoder2Port).velocity;
    }
}
