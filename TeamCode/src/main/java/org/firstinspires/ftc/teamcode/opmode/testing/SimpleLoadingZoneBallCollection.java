package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createVec;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
//import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPid;
import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPidNew;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

//@TeleOp(name="Loading Zone Ball Collection", group="TestingParams")
@Config
public class SimpleLoadingZoneBallCollection extends LinearOpMode {
    public static int streamFPS = 5;
    public static double[] start = { 48 + 6.5, 8, 90 };
    public static boolean usePoseStorageStartPose = false;
    private BrainSTEMRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Limelight.startingPipeline = Limelight.BALL_DETECTION_PIPELINE;
        Pose2d startPose = usePoseStorageStartPose ? new Pose2d(PoseStorage.autoX, PoseStorage.autoY, PoseStorage.autoHeading) : createPose(start);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, startPose);
        FtcDashboard.getInstance().startCameraStream(robot.limelight.limelight, streamFPS);
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        Action autoAction = robot.getLimelightCollectSequence(createVec(AutoPidNew.collect.limelightScanPos1), createVec(AutoPidNew.collect.limelightScanPos2), 2);
        Action fullAutoAction = new ParallelAction(
                autoCommands.updateRobotInfo(),
                new TimedAction(autoAction, 100).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot(),
                autoCommands.savePoseContinuously(),
                packet -> {
                    robot.drawRobotInfo(packet.fieldOverlay());
                    robot.limelight.printInfo();
                    telemetry.update();
                    return true;
                }
        );

        waitForStart();
        robot.startOpmode();

        Actions.runBlocking(fullAutoAction);
    }
}
