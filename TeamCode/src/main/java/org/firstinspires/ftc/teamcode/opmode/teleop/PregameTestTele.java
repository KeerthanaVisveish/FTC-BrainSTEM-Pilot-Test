package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.LED;

@TeleOp(name="A Pre game test", group="Competition")
public class PregameTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        ServoImplEx led = hardwareMap.get(ServoImplEx.class, "rightLED");
        waitForStart();
        while(opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if(result != null) {
                boolean valid = result.isValid();
                telemetry.addData("is result valid", valid);

                Pose3D pose = result.getBotpose();
                telemetry.addData("botpose", pose);
                if(pose.getPosition().x != 0 && pose.getPosition().y != 0 && valid)
                    led.setPosition(LED.green);
                else
                    led.setPosition(LED.red);
            }
            else {
                telemetry.addLine("botpose is null");
                led.setPosition(LED.red);
            }

            telemetry.update();
        }
    }
}
