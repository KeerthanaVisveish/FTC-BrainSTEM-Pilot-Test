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
    public int oscillateEndpoint = 200;
    public static double targetVel = 0, targetPos = 0;
    public static double kP = 0, kS = 0, kV = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {

            int currentPos = turret.getCurrentPosition();
            double currentVel = turret.getVelocity();

            double positionError = targetPos - currentPos;
            double velocityError = targetVel - currentVel;
            double power = positionError * kP + Math.signum(positionError) * kS + velocityError * kV;
            turret.setPower(power);

            telemetry.addData("pos error", positionError);
            telemetry.addData("vel error", velocityError);
            telemetry.addData("power", turret.getPower());
            telemetry.update();
        }
    }
}
