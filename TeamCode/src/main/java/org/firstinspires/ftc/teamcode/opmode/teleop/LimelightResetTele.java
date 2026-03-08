package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;

@TeleOp(name="A Limelight Reset Tele", group="Competition")
public class LimelightResetTele extends LinearOpMode {
    public static boolean cameraIsReset = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(Limelight.APRIL_TAG_PIPELINE);
        limelight3A.start();
        telemetry.addLine("Robot is Ready");
        telemetry.update();

        waitForStart();
        FtcDashboard.getInstance().startCameraStream(limelight3A, 5);

        while (opModeIsActive()) {
            LLResult result = limelight3A.getLatestResult();
            telemetry.addData("current pipeline index", result.getPipelineIndex());
            telemetry.addData("current pipeline type", result.getPipelineType());
            telemetry.addData("num april tags", result.getFiducialResults().size());
            telemetry.update();
        }
        cameraIsReset = true;
        limelight3A.stop();
    }
}
