package org.firstinspires.ftc.teamcode.opmode.testing.shooterV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.hood.HoodV2;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter.ShooterV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.misc.BatteryVoltageFilter;

@Config
@TeleOp(name="Shooter-Hood Test", group="shooterV2")
public class ShooterHoodTest extends LinearOpMode {
    public static boolean controlHoodWithPower = true;
    public static double hoodPower = 0;
    public static double targetExitAngleDeg = 45;


    public static double targetShooterSpeed = 0;
    public static double shooterPower = 0;
    public static boolean setRawShooterPower = true;
    public static boolean engageClutch = false;
    public static boolean runIntake = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(30);
        telemetry.addLine("srshub not initialized");
        telemetry.update();

        BatteryVoltageFilter batteryVoltageFilter = BatteryVoltageFilter.getInstance(hardwareMap);
        SRSHub srsHub = new SRSHub(hardwareMap);
        ShooterV2 shooterV2 = new ShooterV2(hardwareMap, telemetry, srsHub);
        HoodV2 hoodV2 = new HoodV2(hardwareMap, telemetry, srsHub);
        Collector collector = new Collector(hardwareMap, telemetry);


        telemetry.addLine("srshub initialized");
        telemetry.update();

        waitForStart();


        // TODO: in HoodV2class, configure servo directions so positive power equates to positive absolute encoder change
        // TODO: in ShooterV2class, configure shooter directions so positive power equates to positive encoder change

        
        while(opModeIsActive()) {
            batteryVoltageFilter.update();
            srsHub.update();
            shooterV2.updateProperties();

            if(controlHoodWithPower)
                hoodV2.setHoodPower(hoodPower);
            else
                hoodV2.setTargetExitAngle(Math.toRadians(targetExitAngleDeg));

            if(setRawShooterPower)
                shooterV2.setShooterPower(shooterPower);
            else
                shooterV2.setShooterVelocityPID(targetShooterSpeed, batteryVoltageFilter.getVoltage());

            if(engageClutch && collector.getClutchState() != Collector.ClutchState.ENGAGED)
                collector.setClutchState(Collector.ClutchState.ENGAGED);
            else if(!engageClutch && collector.getClutchState() != Collector.ClutchState.DISENGAGED)
                collector.setClutchState(Collector.ClutchState.DISENGAGED);

            if(runIntake && collector.getIntakeState() != Collector.IntakeState.INTAKE)
                collector.setIntakeState(Collector.IntakeState.INTAKE);
            else if(!runIntake && collector.getIntakeState() != Collector.IntakeState.OFF)
                collector.setIntakeState(Collector.IntakeState.OFF);

            hoodV2.update();
            collector.updateState(true);

            telemetry.addLine("SRSHUB-----");
            telemetry.addData("hood abs encoder", srsHub.getHoodAbsEncoder());
            telemetry.addData("hood 1 encoder", srsHub.getHoodServo1Encoder());
            telemetry.addData("hood 2 encoder", srsHub.getHoodServo2Encoder());
            telemetry.addData("shooter high encoder", srsHub.getShooterHighEncoder());

            collector.printInfo();
            shooterV2.printInfo();
            hoodV2.printInfo();

            telemetry.update();
        }
    }
}
