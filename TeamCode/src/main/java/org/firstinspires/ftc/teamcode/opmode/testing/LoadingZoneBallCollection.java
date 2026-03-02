package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.Blob;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathGeneration;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathInfo;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathPose;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

@TeleOp(name="Loading Zone Ball Collection", group="TestingParams")
@Config
public class LoadingZoneBallCollection extends OpMode {
    public static int streamFPS = 5;
    public static double[] startPose = { 48 + 6.5, 8, 90 };
    public static double scanAngle1 = 80, scanAngle2 = 110;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;

    private Action autoCollectAction = null;
    private ArrayList<Vector2d> mostRecentNodes;
    private PathInfo pathInfo;
    private Action scanForBallsAction = null;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Limelight.startingPipeline = Limelight.BALL_DETECTION_PIPELINE;
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(startPose));
        autoCommands = new AutoCommands(robot, telemetry);
        FtcDashboard.getInstance().startCameraStream(robot.limelight.limelight, streamFPS);
        mostRecentNodes = new ArrayList<>();
    }

    @Override
    public void init_loop() {
        if (gamepad1.start && gamepad1.backWasPressed())
            robot.shootingSystem.resetTurretEncoder();
        robot.shootingSystem.updateInfo(false);
        telemetry.addData("turret encoder", robot.shootingSystem.getTurretEncoder());

        robot.limelight.printInfo();

        telemetry.update();
    }
    @Override
    public void start() {
        robot.startOpmode();
    }
    @Override
    public void stop() {
        robot.limelight.limelight.stop();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down)
            robot.limelight.limelight.pipelineSwitch(0);
        else if (gamepad1.dpad_up)
            robot.limelight.limelight.pipelineSwitch(2);

        Pose2d robotPose = robot.drive.localizer.getPose();

        if (autoCollectAction == null && scanForBallsAction == null) {
            ArrayList<Blob> blobs = robot.limelight.ballDetection.getCurrentBlobs();
            mostRecentNodes = new ArrayList<>();
            for (int i = 0; i < blobs.size(); i++)
                mostRecentNodes.add(blobs.get(i).pos());

            pathInfo = PathGeneration.generateSimplifiedAutoCollectPath(robotPose, mostRecentNodes);
        }
        DrivePath autoCollectDrive = null;
        if (pathInfo != null) {
            autoCollectDrive = new DrivePath(robot.drive);
            for (PathPose pathPose : pathInfo.simplifiedPathPoses)
                autoCollectDrive.addWaypoint(pathPose.waypoint);
        }
        if (autoCollectAction == null && scanForBallsAction == null) {
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

        if (gamepad1.a && autoCollectAction == null && scanForBallsAction == null && autoCollectDrive != null) {
            robot.drive.stop();
            autoCollectAction = new SequentialAction(
                    autoCommands.runIntake(),
                    new CustomEndAction(
                            autoCollectDrive,
                            () -> robot.collection.autoCollectHas3Balls()
                    ),
                    autoCommands.stopIntake()
            );
        }
        if (gamepad1.y && autoCollectAction == null && scanForBallsAction == null)
            scanForBallsAction = robot.scanForBalls(() -> Math.toRadians(scanAngle1), () -> Math.toRadians(scanAngle2));
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
        if (scanForBallsAction != null) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean keepGoing = scanForBallsAction.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            if (!keepGoing)
                scanForBallsAction = null;
        }

        robot.update(false);

        telemetry.addData("time running", getRuntime());
        if (pathInfo == null)
            telemetry.addData("drive path", "null");
        else
            telemetry.addData("drive path", pathInfo.getPoses());
        robot.limelight.printInfo();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robot.addRobotInfo(fieldOverlay);
        robot.limelight.ballDetection.drawBalls(fieldOverlay, mostRecentNodes);
        if (pathInfo != null)
            robot.limelight.ballDetection.drawPath(fieldOverlay, robotPose, pathInfo.getPoses());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
