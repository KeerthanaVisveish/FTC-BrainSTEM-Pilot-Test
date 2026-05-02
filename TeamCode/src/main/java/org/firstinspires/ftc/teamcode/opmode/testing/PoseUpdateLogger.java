package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.misc.PoseUpdatePacket;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

@Config
@TeleOp(name="Pose Update Logger", group="Testing")
public class PoseUpdateLogger extends LinearOpMode {
    public static int logLength = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        int startI = 0;
        while(opModeIsActive()) {
            int listLength = PoseUpdatePacket.poseUpdatePackets.size();
            for(int i = startI; i < Math.min(listLength, startI + logLength); i++) {
                PoseUpdatePacket packet = PoseUpdatePacket.poseUpdatePackets.get(i);
                telemetry.addData("i: " + startI, "updateType: " + packet.updateType + "pose: " + MathUtils.formatPose3(packet.pose));
            }
            telemetry.update();
            if(gamepad1.dpadDownWasPressed())
                startI = Math.min(startI + 1, listLength - logLength);
            else if(gamepad1.dpadUpWasPressed())
                startI = Math.max(startI - 1, 0);
        }
    }
}
