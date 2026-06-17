package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

@TeleOp(name="A Pre-Game Setup Tele", group="Competition")
public class PreGameSetupTele extends LinearOpMode {
    public static boolean cameraIsReset = false;
    public static double startX = 0, startY = 0, startA = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(Limelight.APRIL_TAG_PIPELINE);
        limelight3A.start();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startA));

        ServoImplEx led = hardwareMap.get(ServoImplEx.class, "rightLED");

        telemetry.addLine("Limelight is Ready");
        telemetry.update();

        waitForStart();
        FtcDashboard.getInstance().startCameraStream(limelight3A, 5);

        while (opModeIsActive()) {
            LLResult result = limelight3A.getLatestResult();

            drive.updatePoseEstimate();
            telemetry.addLine("PINPOINT---------------");
            telemetry.addData("pose", MathUtils.formatPose2(drive.pinpoint().getPose()));
            telemetry.addData("velocity", drive.pinpoint().getVelocity().toString(2));
            telemetry.addLine();
            telemetry.addLine("LIMELIGHT----------------");
            telemetry.addData("current pipeline index", result.getPipelineIndex());
            telemetry.addData("current pipeline type", result.getPipelineType());
            telemetry.addData("num april tags", result.getFiducialResults().size());

            if(!result.getFiducialResults().isEmpty())
                led.setPosition(LED.green);
            else
                led.setPosition(LED.red);
            telemetry.update();
        }
        cameraIsReset = true;
        limelight3A.stop();
    }
}
