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

            telemetry.addData("IN COMPETITION", BrainSTEMTeleOp.inCompetition);
            telemetry.addLine();
            telemetry.addLine("PRESS RIGHT TRIGGER to turn inCompetition ON");
            telemetry.addLine("PRESS LEFT TRIGGER to turn inCompetition OFF");
            telemetry.update();
        }
    }
}
