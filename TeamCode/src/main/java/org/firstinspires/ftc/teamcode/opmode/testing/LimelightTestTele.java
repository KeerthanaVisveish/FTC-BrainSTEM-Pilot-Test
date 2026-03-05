package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name="LimelightTestTele", group="Limelight")
@Config
public class LimelightTestTele extends LinearOpMode {
    public static int pipeline = 0;
    public static double[] pythonInputs = { -1, 24 };
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(pipeline);
        limelight3A.start();

//        ElapsedTime initTimer = new ElapsedTime();
//        boolean canStart = false;
//        while (opModeInInit()) {
//            if (initTimer.seconds() > 1) {
//                limelight3A.stop();
//                limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
//                limelight3A.pipelineSwitch(pipeline);
//                limelight3A.start();
//                canStart = true;
//            }
//            telemetry.addData("CAN START", canStart);
//            telemetry.addData("init timer", initTimer.seconds());
//            telemetry.update();
//        }

        waitForStart();
        FtcDashboard.getInstance().startCameraStream(limelight3A, 5);

        while (opModeIsActive()) {
            if (gamepad1.a)
                limelight3A.pipelineSwitch(0);
            if (gamepad1.b)
                limelight3A.pipelineSwitch(pipeline);

            limelight3A.updatePythonInputs(pythonInputs);
            LLResult result = limelight3A.getLatestResult();
            double[] pythonOutput = result.getPythonOutput();
            telemetry.addData("current pipeline index", result.getPipelineIndex());
            telemetry.addData("current pipeline type", result.getPipelineType());
            telemetry.addData("num april tags", result.getFiducialResults().size());
            telemetry.addData("python outputs", Arrays.toString(pythonOutput));
            telemetry.update();
        }
        limelight3A.stop();
    }
}
