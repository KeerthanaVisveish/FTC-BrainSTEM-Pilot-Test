package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@Config
//@TeleOp(name="Turret tuner", group="Testing")
public class TurretTuner extends LinearOpMode {
    public static double noPowerK = 6, noPowerX0 = 1.75;
    public static double u = .14, b = .06, k = .01, x0 = 100; // parameters for logistic kF power
    public static boolean oscillateTurret = false;
    public static boolean joystickControlledVelocity = true;
    public static boolean alwaysUseFeedbackVelocityControl = false;
    public static double maxVel = 150, targetPos = 250;
    public static double kP = 0.001, kV = 0.0003, kVP = 0.001;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPos = Math.abs(targetPos);
        maxVel = Math.abs(maxVel);
        waitForStart();
        while(opModeIsActive()) {

            int currentPos = turret.getCurrentPosition();
            double currentVel = turret.getVelocity();

            double positionError = targetPos - currentPos;

            if(oscillateTurret && Math.signum(positionError) != Math.signum(targetPos)) {
                targetPos *= -1;
                positionError = targetPos - currentPos;
            }
            double dir = Math.signum(positionError);
            double targetVel;
            if(joystickControlledVelocity)
                targetVel = Math.abs(gamepad1.right_stick_x) * maxVel * dir;
            else
                targetVel = maxVel * dir;
            double kF = getLogisticKf(currentPos, (int) Math.signum(positionError));
            double velocityError = targetVel - currentVel;
            if(!alwaysUseFeedbackVelocityControl && targetVel == 0)
                velocityError = 0;
            double power = positionError * kP + kF + velocityError * kVP + targetVel * kV;
            double powerScaler = getLogisticPowerScaler(Math.abs(positionError));
            double finalPower = power * powerScaler;
            turret.setPower(finalPower);

            telemetry.addData("pos error", positionError);
            telemetry.addData("target pos", targetPos);
            telemetry.addData("target vel", targetVel);
            telemetry.addData("cur pos", currentPos);
            telemetry.addData("cur vel", currentVel);
            telemetry.addData("power", turret.getPower());
            telemetry.addData("kf", kF);
            telemetry.addData("power scaler", powerScaler);
            telemetry.update();
        }
    }
    private double getLogisticKf(double encoder, int direction) {
        if(direction == -1)
            encoder *= -1;
        double logisticPower = (u - b) / (1 + Math.exp(-k * (encoder-x0))) + b;
        return logisticPower * direction;
    }
    private double getLogisticPowerScaler(double errorMag) {
        return 1 / (1 + Math.exp(-noPowerK * (errorMag - noPowerX0)));
    }
}
