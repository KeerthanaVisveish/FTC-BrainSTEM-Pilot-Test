package org.firstinspires.ftc.teamcode.opmode.testing.shooterV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.shootingSystem.Turret;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.misc.BatteryVoltageFilter;

@Config
@TeleOp(name="Turret Tuning", group="shooterV2")
public class TurretTuning extends LinearOpMode {
    public static boolean controlTurretWithVoltage = true;
    public static double turretVoltage = 0;
    public static double targetTurretAngle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(30);
        Turret turret = new Turret(hardwareMap, telemetry);
        BatteryVoltageFilter batteryVoltageFilter = BatteryVoltageFilter.getInstance(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            batteryVoltageFilter.update();
            turret.updateProperties(.05);
            if(controlTurretWithVoltage) {
                if(Math.abs(turret.getCurAngleRad()) > Math.PI * .5)
                    turret.setTurretVoltage(0, 0);
                else
                    turret.setTurretVoltage(turretVoltage, batteryVoltageFilter.getVoltage());
            }
            else {
                double[] motionProfileData =  turret.calculateMotionProfile(targetTurretAngle);
                turret.controlTurretToTarget(targetTurretAngle, motionProfileData[0], motionProfileData[1], 0, Turret.powerTuning.maxVoltage, 0, new OdoInfo(0, 0, 0), batteryVoltageFilter.getVoltage());
            }
            turret.printInfo();
            telemetry.addData("target turret angle deg", Math.toDegrees(targetTurretAngle));
            telemetry.addData("target turret velocity", turret.calculateMotionProfile(targetTurretAngle)[0]);
            telemetry.update();
        }
    }
}
