package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@TeleOp(name="DrivePathTest")
@Config
public class DrivePathTest extends LinearOpMode {
    public static double[] p0 = new double[] { 0, 0, 0};
    public static double[] p1 = new double[] { 48, 0, 0};
    public static double[] p2 = new double[] { 0, 0, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, createPose(p0));

        telemetry.addLine("robot ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {

                Waypoint w1 = new Waypoint(createPose(p1));
                Waypoint w2 = new Waypoint(createPose(p2));
                DrivePath path = new DrivePath(drive, w1, w2);
                Actions.runBlocking(path);
            }
        }
    }
}
