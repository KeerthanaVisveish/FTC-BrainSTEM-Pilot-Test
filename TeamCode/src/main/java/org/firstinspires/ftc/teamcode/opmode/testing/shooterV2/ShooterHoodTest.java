package org.firstinspires.ftc.teamcode.opmode.testing.shooterV2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.hood.HoodV2;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter.ShooterV2;
import org.firstinspires.ftc.teamcode.utils.misc.BatteryVoltageFilter;

@Config
@TeleOp(name="Shooter-Hood Test", group="shooterV2")
public class ShooterHoodTest extends LinearOpMode {
    public static double targetExitAngle = Math.toRadians(45);
    public static boolean activateHood = false;

    public static double targetShooterSpeed = 0;
    public static double shooterPower = 0;
    public static boolean setRawShooterPower = true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("srshub not initialized");
        telemetry.update();

        BatteryVoltageFilter batteryVoltageFilter = BatteryVoltageFilter.getInstance(hardwareMap);
        SRSHub srsHub = new SRSHub(hardwareMap);
        ShooterV2 shooterV2 = new ShooterV2(hardwareMap, telemetry, srsHub);
        HoodV2 hoodV2 = new HoodV2(hardwareMap, telemetry, srsHub);


        telemetry.addLine("srshub initialized");
        telemetry.update();

        waitForStart();


        // TODO: in HoodV2class, configure servo directions so positive power equates to positive absolute encoder change
        // TODO: in ShooterV2class, configure shooter directions so positive power equates to positive encoder change

        
        while(opModeIsActive()) {
            batteryVoltageFilter.update();
            srsHub.update();
            shooterV2.updateProperties();

            if(activateHood)
                hoodV2.setTargetExitAngle(targetExitAngle);

            if(setRawShooterPower)
                shooterV2.setShooterPower(shooterPower);
            else
                shooterV2.setShooterVelocityPID(targetShooterSpeed, batteryVoltageFilter.getVoltage());

            hoodV2.update();

            telemetry.addLine("SRSHUB-----");
            telemetry.addData("hood abs encoder", srsHub.getHoodAbsEncoder());
            telemetry.addData("hood 1 encoder", srsHub.getHoodServo1Encoder());
            telemetry.addData("hood 2 encoder", srsHub.getHoodServo2Encoder());

            shooterV2.printInfo();
            hoodV2.printInfo();

            telemetry.update();
        }
    }
}
