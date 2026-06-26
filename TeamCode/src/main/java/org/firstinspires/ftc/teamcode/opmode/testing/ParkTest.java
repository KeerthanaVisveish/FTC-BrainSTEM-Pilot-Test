package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Parking;

@Config
@TeleOp(name="Park Test")
public class ParkTest extends LinearOpMode {
    public static double parkPosition = 0.2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Parking parking = new Parking(hardwareMap, telemetry);


        waitForStart();
        while(opModeIsActive()) {
            parking.setParkServoPosition(parkPosition);
            parking.printInfo();
            telemetry.update();
        }
    }
}
