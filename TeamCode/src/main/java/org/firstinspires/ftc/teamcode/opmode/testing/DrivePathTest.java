package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPid;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

@TeleOp(name="DrivePathTest", group="Testing")
@Config
public class DrivePathTest extends LinearOpMode {
    public static double[] p0 = new double[] { 0, 0, 0};
    public static double[] p1 = new double[] { 24, 24, 90 };
    public static double[] p2 = new double[] { 0, 0, 0};
    public static double distTol = 1, headingTol = Math.toRadians(3);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, createPose(AutoPid.misc.startNearRed));
        p1 = AutoPid.shoot.nearPreloadMotif;

        telemetry.addLine("robot ready");
        telemetry.update();
        DrivePath.showRobotPose = true;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                Waypoint w1 = new Waypoint(createPose(p1), new BoxTolerance(distTol, headingTol));
                Waypoint w2 = new Waypoint(createPose(p2), new BoxTolerance(distTol, headingTol));
                DrivePath path = new DrivePath(drive, w1, w2);
                Actions.runBlocking(
                        new ParallelAction(
                                path,
                                telemetryPacket -> {
                                    DrivePath.drawCurrentPath(telemetryPacket.fieldOverlay());
                                    drive.updatePoseEstimate();
                                    return true;
                                }
                        )
                );
            }
        }
    }
}
