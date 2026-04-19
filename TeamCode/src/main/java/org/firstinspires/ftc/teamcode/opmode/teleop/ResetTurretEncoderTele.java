package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

@TeleOp(name="A Reset Turret Encoder", group="Competition")
public class ResetTurretEncoderTele extends LinearOpMode {
    @Override
    public void runOpMode() {
        ShootingSystem shootingSystem = new ShootingSystem(hardwareMap, telemetry, null);
        shootingSystem.resetTurretEncoder();
        telemetry.addData("turret encoder", shootingSystem.getTurretEncoderRaw());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {}

    }
}
