package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Turret tuner", group="Testing")
public class TurretTuner extends LinearOpMode {
    public boolean oscillateTurret = true;
    public static double targetVel = 0, targetPos = 0;
    public static double kP = 0, kS = 0, kV = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPos = Math.abs(targetPos);
        targetVel = Math.abs(targetVel);

        waitForStart();
        while(opModeIsActive()) {

            int currentPos = turret.getCurrentPosition();
            double currentVel = turret.getVelocity();

            double positionError = targetPos - currentPos;
            if(oscillateTurret && Math.signum(positionError) != Math.signum(targetPos)) {
                targetPos *= -1;
                targetVel *= -1;
                positionError = targetPos - currentPos;
            }
            double power = positionError * kP + Math.signum(positionError) * kS + targetVel * kV;
            turret.setPower(power);

            telemetry.addData("pos error", positionError);
            telemetry.addData("target pos", targetPos);
            telemetry.addData("target vel", targetVel);
            telemetry.addData("cur pos", currentPos);
            telemetry.addData("cur vel", currentVel);
            telemetry.addData("power", turret.getPower());
            telemetry.update();
        }
    }
}
