package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="A Toggle in Competition Tele", group="Competition")
public class ToggleInCompetitionTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.2)
                BrainSTEMTeleOp.inCompetition = true;
            else if (gamepad1.left_trigger > 0.2)
                BrainSTEMTeleOp.inCompetition = false;
            if(gamepad1.a)
                BrainSTEMTeleOp.allowD1Shoot = true;
            else if(gamepad1.b)
                BrainSTEMTeleOp.allowD1Shoot = false;

            telemetry.addData("IN COMPETITION", BrainSTEMTeleOp.inCompetition);
            telemetry.addLine();
            telemetry.addLine("PRESS RIGHT TRIGGER to turn inCompetition ON");
            telemetry.addLine("PRESS LEFT TRIGGER to turn inCompetition OFF");
            telemetry.addData("ALLOW D1 TO SHOOT", BrainSTEMTeleOp.allowD1Shoot);
            telemetry.addLine("press a to allow d1 to shoot");
            telemetry.addLine("press b to not allow d1 to shoot");
            telemetry.update();
        }
    }
}
