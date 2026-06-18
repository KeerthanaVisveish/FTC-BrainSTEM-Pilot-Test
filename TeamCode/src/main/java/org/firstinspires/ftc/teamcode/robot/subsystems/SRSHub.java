package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.RobotProperties;

@Config
public class SRSHub {
    public static int hoodEncoderPort = 1;

    private final SRSHubRaw srsHubRaw;

    public SRSHub(HardwareMap hardwareMap) {
        SRSHubRaw.Config config = new SRSHubRaw.Config();
        config.setEncoder(hoodEncoderPort, SRSHubRaw.Encoder.PWM);

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
        return srsHubRaw.readEncoder(hoodEncoderPort).position;
    }
    public double getHoodVelocity() {
        return srsHubRaw.readEncoder(hoodEncoderPort).velocity;
    }
}
