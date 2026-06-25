package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;

@Config
public class SRSHub extends Component {
    // absolute encoders
    public static int hoodAbsoluteEncoderPort = 1;
    public static int hoodServo1Port = 4;
    public static int hoodServo2Port = 5;

    // relative encoders
    public static int shooterLowEncoderPort = 2;
    public static int shooterHighEncoderPort = 3;

    public static int shooterLowEncoderSign = 1, shooterHighEncoderSign = -1;

    private final SRSHubRaw srsHubRaw;

    public SRSHub(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        SRSHubRaw.Config config = new SRSHubRaw.Config();

        config.setEncoder(hoodAbsoluteEncoderPort, SRSHubRaw.Encoder.PWM);
        config.setEncoder(hoodServo1Port, SRSHubRaw.Encoder.PWM);
        config.setEncoder(hoodServo2Port, SRSHubRaw.Encoder.PWM);

        config.setEncoder(shooterLowEncoderPort, SRSHubRaw.Encoder.QUADRATURE);
        config.setEncoder(shooterHighEncoderPort, SRSHubRaw.Encoder.QUADRATURE);

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
    public double getHoodServo1Encoder() {
        return srsHubRaw.readEncoder(hoodServo1Port).position;
    }
    public double getHoodServo2Encoder() {
        return srsHubRaw.readEncoder(hoodServo2Port).position;
    }
    public double getShooterLowVelocity() {
        return shooterLowEncoderSign * srsHubRaw.readEncoder(shooterLowEncoderPort).velocity;
    }
    public double getShooterHighVelocity() {
        return shooterHighEncoderSign * srsHubRaw.readEncoder(shooterHighEncoderPort).velocity;
    }
    public double getShooterHighEncoder() {
        return srsHubRaw.readEncoder(shooterHighEncoderPort).position;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("SRSHUB----------");
        telemetry.addData("SRS hood encoder", getHoodAbsEncoder());
        telemetry.addData("SRS shooter low vel", getShooterLowVelocity());
        telemetry.addData("SRS shooter high vel", getShooterHighVelocity());
    }
}
