package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

@TeleOp(name="Classifier Auto Test", group="Limelight")
public class ClassifierAutoTest extends LinearOpMode {
    public static Alliance alliance = Alliance.RED;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(20);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Limelight.startingPipeline = Limelight.CLASSIFIER_PIPELINE;
        double sign = alliance == Alliance.RED ? 1 : -1;
        double angle = Math.toRadians(90) * sign;
        BrainSTEMRobot robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, new Pose2d(-24, 24 * sign, angle));

        Waypoint w1 = new Waypoint(new Pose2d(-24, 48 * sign, angle));
        DrivePath drivePath = new DrivePath(robot.drive, w1);
        waitForStart();
        robot.startOpmode();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.5),
                                robot.lookAtClassifier(Turret.TurretState.TRACKING),
                                robot.waitXSecondsIf2BallsInClassifier(2),
                                drivePath
                        ),
                        packet -> {
                            robot.updateInfo();
                            robot.update();
                            robot.limelight.printInfo();
                            telemetry.update();
                            return true;
                        }
                )
        );
    }
}
