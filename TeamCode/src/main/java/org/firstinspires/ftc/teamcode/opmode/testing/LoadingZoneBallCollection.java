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
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="Loading Zone Ball Collection", group="TestingParams")
@Config
public class LoadingZoneBallCollection extends OpMode {
    public static int streamFPS = 5;
    public static double[] start = { 48 + 6.5, 8, 90 };
    public static boolean usePoseStorageStartPose = false;
    public static double scanAngle1 = 80, scanAngle2 = 110;
    public enum ShowType {
        CURRENT,
        SCAN
    }
    public static ShowType showType = ShowType.CURRENT;
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
        Pose2d startPose = usePoseStorageStartPose ? new Pose2d(PoseStorage.autoX, PoseStorage.autoY, PoseStorage.autoHeading) : createPose(start);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, startPose);
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

//        if (autoCollectAction == null && scanForBallsAction == null) {
//            Vector2d giantClump = robot.limelight.ballDetection.getCurrentGiantClumpPosition();
//            telemetry.addData("giant clump", MathUtils.formatVec1(giantClump));
//            if (giantClump == null) {
//                mostRecentNodes = showType == ShowType.CURRENT ?
//                        robot.limelight.ballDetection.getCurrentBlobPositions() :
//                        robot.limelight.ballDetection.getCombinedBlobPositions();
//            }
//            else
//                mostRecentNodes = new ArrayList<>(Arrays.asList(giantClump));
//            telemetry.addLine("MOST RECENT NODES==============");
//            for (Vector2d ballPos : mostRecentNodes)
//                telemetry.addData("ball pos: ", MathUtils.formatVec2(ballPos));
//
//            pathInfo = PathGeneration.generateSimplifiedAutoCollectPath(robotPose, mostRecentNodes);
//        }
        if (pathInfo != null)
            for (PathPose pathPose : pathInfo.optimizedPathPoses)
                telemetry.addLine("" + pathPose.approachType);
        DrivePath autoCollectDrive = null;
        if (pathInfo != null) {
            autoCollectDrive = new DrivePath(robot.drive);
            for (PathPose pathPose : pathInfo.optimizedPathPoses) {
                autoCollectDrive.addWaypoint(pathPose.waypoint);
                telemetry.addLine("" + pathPose.approachType);
            }
        }
        if (autoCollectAction == null && scanForBallsAction == null) {
            if (gamepad1.left_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
            else
                robot.collection.setCollectionState(Collection.CollectionState.OFF);

            if (gamepad1.dpad_up)
                showType = ShowType.CURRENT;
            else if (gamepad1.dpad_down)
                showType = ShowType.SCAN;

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
            showType = ShowType.SCAN;
            TelemetryPacket packet = new TelemetryPacket();
            boolean keepGoing = scanForBallsAction.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            if (!keepGoing)
                scanForBallsAction = null;
        }

        robot.updateInfo(true);
        robot.update();

        Pose2d p = robot.drive.localizer.getPose();
        PoseStorage.autoX = p.position.x;
        PoseStorage.autoY = p.position.y;
        PoseStorage.autoHeading = p.heading.toDouble();

        telemetry.addData("time running", getRuntime());
        if (pathInfo == null)
            telemetry.addData("drive path", "null");
        else
            telemetry.addData("drive path", pathInfo.getOptimizedPoses());
        robot.limelight.printInfo();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robot.drawRobotInfo(fieldOverlay);
//        robot.limelight.ballDetection.drawBalls(fieldOverlay, mostRecentNodes);
        if (pathInfo != null)
            robot.limelight.ballDetection.drawPath(fieldOverlay, robotPose, pathInfo.getOptimizedPoses());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
