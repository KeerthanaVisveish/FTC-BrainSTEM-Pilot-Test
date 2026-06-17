package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.shootingSystem.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.Turret;

@TeleOp(name="A Reset Turret Encoder", group="Competition")
public class ResetTurretEncoderTele extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap, telemetry);
        turret.resetEncoders();
        turret.updateProperties(.01);
        telemetry.addData("turret encoder", turret.getEncoder());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {}

    }
}
