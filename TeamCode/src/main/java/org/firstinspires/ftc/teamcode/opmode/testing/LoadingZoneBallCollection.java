package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.math.Vec;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@TeleOp(name="Loading Zone Ball Collection", group="TestingParams")
@Config
public class LoadingZoneBallCollection extends OpMode {
    public static int streamFPS = 5;
    public static double[] startPose = { 48 + 6.5, 8, 90 };
    public static double tolerance = 3;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;
    private boolean runningAction = false;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(startPose));
        autoCommands = new AutoCommands(robot, telemetry);
        FtcDashboard.getInstance().startCameraStream(robot.limelight.limelight, streamFPS);
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.2)
            robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
        else
            robot.collection.setCollectionState(Collection.CollectionState.OFF);

        if (gamepad1.a) {
            runningAction = true;
            robot.drive.stop();

            Actions.runBlocking(
                    new CustomEndAction(
                            new ParallelAction(
                                    new SequentialAction(
                                            autoCommands.runIntake(),
                                            driveToBiggest2Blobs(),
                                            autoCommands.stopIntake(),
                                            telemetryPacket -> {runningAction = false; return false;}
                                    ),
                                    telemetryPacket -> {
                                        autoCommands.updateRobot.run(new TelemetryPacket());
                                        telemetry.update();
                                        return runningAction;
                                    }
                            ), () -> Math.abs(gamepad1.left_stick_x) > 0.1 ||
                            Math.abs(gamepad1.left_stick_y) > 0.1 ||
                            Math.abs(gamepad1.right_stick_x) > 0.1
                    )
            );
        }

        robot.update(false);

        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        telemetry.addData("time running", getRuntime());
        robot.limelight.printInfo();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robot.addRobotInfo(fieldOverlay);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public Action driveToBiggest2Blobs() {
        return new Action() {
            DrivePath drivePath = null;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drivePath == null) {
                    Pose2d currentPose = robot.drive.localizer.getPose();
                    Vector2d p1 = robot.limelight.ballDetection.getBlobPosition(0);
                    if (p1 == null)
                        return false;

                    double a1 = Math.atan2(p1.y - currentPose.position.y, p1.x - currentPose.position.x);
                    p1 = getCollectPosition(p1, a1);
                    Waypoint w1 = new Waypoint(new Pose2d(p1.x, p1.y, a1), new Tolerance(tolerance))
                            .setMinLinearPower(0.5)
                            .setMaxTime(3);

                    Vector2d p2 = robot.limelight.ballDetection.getBlobPosition(1);
                    if (p2 != null) {
                        double a2 = Math.atan2(p2.y - p1.y, p2.x - p1.x);
                        p2 = getCollectPosition(p2, a2);
                        Waypoint w2 = new Waypoint(new Pose2d(p2.x, p2.y, a2), new Tolerance(tolerance))
                                .setMaxTime(2)
                                .prioritizeHeadingInBeginning();
                        drivePath = new DrivePath(robot.drive, w1, w2);
                    }
                    else
                        drivePath = new DrivePath(robot.drive, w1);
                }
                return drivePath.run(telemetryPacket);
            }
        };
    }
    private Vector2d getCollectPosition(Vector2d ballPosition, double angle) {
        double offsetAmount = 8;
        double dx = Math.cos(angle) * offsetAmount;
        double dy = Math.sin(angle) * offsetAmount;
        return new Vector2d(ballPosition.x - dx, ballPosition.y - dy);
    }
}
