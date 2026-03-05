package org.firstinspires.ftc.teamcode.opmode.autosBase;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPid;
import org.firstinspires.ftc.teamcode.opmode.testing.LoadingZoneBallCollection;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;

@Config
public class LimelightTestingAuto extends AutoPid {

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket field = new TelemetryPacket();
        BrainSTEMRobot robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(LoadingZoneBallCollection.start));
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        Action autoAction = getLimelightLoadingZoneCollectAndShoot(new Pose2d(40, 16, 1.57));
        Action fullAutoAction = new ParallelAction(
                packet -> {
                    field.fieldOverlay().clear();
                    return true;
                },
                autoCommands.updateRobotInfo(),
                new TimedAction(autoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot(),
                autoCommands.savePoseContinuously(),
                packet -> {
                    DrivePath.drawCurrentPath();
                    robot.drawRobotInfo(field.fieldOverlay());
                    FtcDashboard.getInstance().sendTelemetryPacket(field);
                    robot.limelight.printInfo();
                    telemetry.update();
                    return true;
                }
        );
        waitForStart();

        Actions.runBlocking(fullAutoAction);
    }

}