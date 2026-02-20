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
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.Blob;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.PathGeneration;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;

@TeleOp(name="Loading Zone Ball Collection", group="TestingParams")
@Config
public class LoadingZoneBallCollection extends OpMode {
    public static int streamFPS = 5;
    public static double[] startPose = { 48 + 6.5, 8, 90 };
    public static double tolerance = 3;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;

    private Action autoCollectAction = null;
    private Vector2d[] mostRecentNodes;
    private ArrayList<Pose2d> mostRecentAutoCollectPathPoses;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(startPose));
        autoCommands = new AutoCommands(robot, telemetry);
        FtcDashboard.getInstance().startCameraStream(Limelight.limelight, streamFPS);
        mostRecentNodes = new Vector2d[0];
        mostRecentAutoCollectPathPoses = new ArrayList<>();
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

        Pose2d robotPose = robot.drive.localizer.getPose();

        if (autoCollectAction == null) {
            Blob[] blobs = robot.limelight.ballDetection.getBlobs();
            Vector2d[] nodes = new Vector2d[blobs.length];
            for (int i = 0; i < blobs.length; i++)
                nodes[i] = new Vector2d(blobs[i].x, blobs[i].y);

            mostRecentAutoCollectPathPoses = PathGeneration.getAutoCollectPoses(true, robotPose, nodes);
        }
        DrivePath autoCollectDrive = null;
        if (mostRecentAutoCollectPathPoses != null) {
            autoCollectDrive = new DrivePath(robot.drive);
            for (Pose2d pose : mostRecentAutoCollectPathPoses)
                autoCollectDrive.addWaypoint(new Waypoint(pose).setMinLinearPower(0.1).setMaxTime(2));
        }
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

        if (gamepad1.a && autoCollectAction == null && autoCollectDrive != null) {
            robot.drive.stop();
            autoCollectAction = new SequentialAction(
                    autoCommands.runIntake(),
                    new CustomEndAction(
                            autoCollectDrive,
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
        telemetry.addData("drive path", mostRecentAutoCollectPathPoses);
        robot.limelight.printInfo();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robot.addRobotInfo(fieldOverlay);
        if (mostRecentAutoCollectPathPoses != null)
                robot.limelight.ballDetection.drawPath(fieldOverlay, robotPose, mostRecentAutoCollectPathPoses);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
