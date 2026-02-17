package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.Blob;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.PathGeneration;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;

import java.util.Arrays;

@TeleOp(name="Loading Zone Ball Collection", group="TestingParams")
@Config
public class LoadingZoneBallCollection extends OpMode {
    public static int streamFPS = 5;
    public static double[] startPose = { 48 + 6.5, 8, 90 };
    public static double tolerance = 3;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;

    private Action autoCollectAction = null;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(startPose));
        autoCommands = new AutoCommands(robot, telemetry);
        FtcDashboard.getInstance().startCameraStream(Limelight.limelight, streamFPS);
    }

    @Override
    public void init_loop() {
        if (gamepad1.start && gamepad1.backWasPressed())
            robot.shootingSystem.resetTurretEncoder();
        robot.shootingSystem.updateInfo(false);
        telemetry.addData("turret encoder", robot.shootingSystem.getTurretEncoder());
        telemetry.update();
    }
    @Override
    public void stop() {
        Limelight.limelight.stop();
    }

    @Override
    public void loop() {

        Vector2d robotPosition = robot.drive.localizer.getPose().position;

        Blob[] blobs = robot.limelight.ballDetection.getBlobs();
        Vector2d[] nodes = new Vector2d[blobs.length];
        for (int i=0; i<blobs.length; i++)
            nodes[i] = new Vector2d(blobs[i].x, blobs[i].y);

        Vector2d[] path = PathGeneration.getShortestPath(robotPosition, nodes, 3);
        boolean successful = path != null && path.length > 0;
        DrivePath autoCollectPath = successful ? PathGeneration.generateComplexDrivePath(robotPosition, robot.drive, path, false) : null;

        if (autoCollectAction == null) {
            if (gamepad1.left_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
            else
                robot.collection.setCollectionState(Collection.CollectionState.OFF);

            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }

        if (gamepad1.a && autoCollectAction == null) {
            robot.drive.stop();
            autoCollectAction = new SequentialAction(
                    autoCommands.runIntake(),
                    new CustomEndAction(
                            autoCollectPath,
                            () -> robot.collection.intakeHas3Balls()
                    ),
                    autoCommands.stopIntake()
            );
        }
        if (autoCollectAction != null) {
            if (Math.abs(gamepad1.left_stick_x) > 0.1 ||
                    Math.abs(gamepad1.left_stick_y) > 0.1 ||
                    Math.abs(gamepad1.right_stick_x) > 0.1) {
                autoCollectAction = null;
                return;
            }

            TelemetryPacket packet = new TelemetryPacket();
            boolean keepGoing = autoCollectAction.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            if (!keepGoing)
                autoCollectAction = null;
        }

        robot.update(false);

        telemetry.addData("time running", getRuntime());
        telemetry.addData("path", Arrays.toString(path));
        telemetry.addData("drive path", autoCollectPath);
        robot.limelight.printInfo();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robot.addRobotInfo(fieldOverlay);
        if (autoCollectAction == null) {
            if (path != null) {
                for (Vector2d pos : path) {
                    fieldOverlay.setStroke("purple");
                    fieldOverlay.strokeCircle(pos.x, pos.y, 2.5);
                }
                for (int i = 0; i < autoCollectPath.getWaypoints().size() - 1; i++) {
                    Vector2d start = autoCollectPath.getWaypoint(i).pose.position;
                    Vector2d end = autoCollectPath.getWaypoint(i + 1).pose.position;
                    fieldOverlay.setStroke("black");
                    fieldOverlay.strokeLine(start.x, start.y, end.x, end.y);
                    fieldOverlay.setStroke(i == 0 ? "black" : "gray");
                    fieldOverlay.strokeCircle(start.x, start.y, 3);
                }
                fieldOverlay.setStroke("gray");
                Vector2d last = autoCollectPath.getWaypoint(autoCollectPath.getWaypoints().size() - 1).pose.position;
                fieldOverlay.strokeCircle(last.x, last.y, 3);
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
