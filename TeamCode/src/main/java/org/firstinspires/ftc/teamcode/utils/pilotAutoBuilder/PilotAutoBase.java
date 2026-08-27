package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.TelemetryLog;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.autoReader.BrainstemPilot;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.buildingBlocks.BezierParams;
import org.firstinspires.ftc.teamcode.utils.bezierCurveDrive.tolerance.CircleTolerance;

@Config
public abstract class PilotAutoBase extends LinearOpMode {
    public static Alliance defaultAlliance = Alliance.BLUE;
    private final String autoId;
    private Alliance alliance;
    private BezierParams defaultParams;
    private Action pilotAuto;
    private Pose2d startPose;

    protected BrainSTEMRobot robot;
    public static double speedkP = 0.15, speedkF = 0.01;
    public static double headingkP = 0.15, headingkF = 0.01;
    /** Project default max velocity (in/s). Used where a record carries no constraints of its
     *  own — point connectors, and any path whose constraints are empty. Mirrors the Brainstem
     *  Pilot project default for FTC. */
    public static double maxLinearSpeed = 60;

    protected PilotAutoBase(String autoId) {
        this.autoId = autoId;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        TelemetryLog.set(telemetry);

        defaultParams = createDefaultBezierParams();
        BrainstemPilot.prepareAssets(hardwareMap.appContext, defaultParams);
        alliance = defaultAlliance;
        applyAllianceConfiguration();

        while (!isStarted() && !isStopRequested()) {
            Alliance previousAlliance = alliance;

            if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
            if (gamepad1.bWasPressed()) alliance = Alliance.RED;
            if (alliance != previousAlliance) applyAllianceConfiguration();

            telemetry.addData("Auto", autoId);
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Start pose", startPose);
            telemetry.addLine("X = Blue | B = Red");
            telemetry.addLine("Ready — waiting for START");
            telemetry.update();
        }

        waitForStart();

        robot.startOpmode();
        Actions.runBlocking(new ParallelAction(pilotAuto, this::runRobotUpdateLoop));
    }

    private void applyAllianceConfiguration() {
        startPose = BrainstemPilot.getStartingPose(autoId, alliance)
                .orElse(new Pose2d(0, 0, 0));
        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose);
        PilotCommandRegistry.registerAll(robot);
        BrainstemPilot.initialize(hardwareMap.appContext, robot.drive, alliance, defaultParams);
        robot.drive.pinpoint().setPose(startPose);
        pilotAuto = BrainstemPilot.buildAuto(autoId).build();
    }

    private BezierParams createDefaultBezierParams() {
        return new BezierParams()
            .setSpeedKp(speedkP)
            .setSpeedKf(speedkF)
            .setHeadingKp(headingkP)
            .setHeadingKf(headingkF)
            .setMaxLinearSpeed(maxLinearSpeed)
            .setTolerance(new CircleTolerance(2, 5));
    }

    private boolean runRobotUpdateLoop(TelemetryPacket packet) {
        robot.update();
        robot.drive.updatePoseEstimate();
        BrainstemPilot.draw(packet.fieldOverlay(), autoId);
        robot.drawRobotInfo(packet.fieldOverlay());
        return true;
    }
}