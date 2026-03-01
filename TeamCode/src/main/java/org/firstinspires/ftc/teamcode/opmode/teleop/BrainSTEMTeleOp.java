package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

@Config
public class BrainSTEMTeleOp extends LinearOpMode {
    public static boolean printCollector = false,
            printShooter = false, printTurret = true, printShootingSystem = false,
            printLimelight = false;
    public static boolean streamCameraToFTCDashboard = false;
    public static boolean inCompetition = false;
    public static double[] blueCornerResetPose = { 62.0618, 63.1, -90 };
    public static double[] redCornerResetPose = { 62.0618, -63.1, 90 };
    public static double noMoveJoystickThreshold = 0.1;

    BrainSTEMRobot robot;

    GamepadTracker gp1;
    GamepadTracker gp2;
    private final Alliance alliance;
    private boolean currentlyMoving;

    public BrainSTEMTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Pose2d startPose = new Pose2d(PoseStorage.autoX, PoseStorage.autoY, PoseStorage.autoHeading);
        currentlyMoving = false;
        CommandScheduler.getInstance().reset();

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        robot.setG1(gp1);
        telemetry.addData("starting pose", MathUtils.formatPose3(startPose));
        if (!robot.limelight.limelight.isConnected())
            telemetry.addLine("WARNING - LIMELIGHT IS NOT CONNECTED");
        if (!robot.limelight.limelight.isRunning())
            telemetry.addLine("WARNING - LIMELIGHT IS NOT RUNNING");
        telemetry.update();

        if (streamCameraToFTCDashboard)
            FtcDashboard.getInstance().startCameraStream(robot.limelight.limelight, 10);

        while (opModeInInit()) {
            if (!inCompetition) {
                if (gamepad1.start && gamepad1.backWasPressed())
                    robot.shootingSystem.resetTurretEncoder();
                robot.shootingSystem.updateInfo(false);
                telemetry.addData("reset turret encoder", "hold START + BACK");
                telemetry.addData("turret encoder", robot.shootingSystem.getTurretEncoder());
            }
            else
                telemetry.addLine("turret encoder reset disabled");
            telemetry.update();
        }

        waitForStart();
        robot.startOpmode();
        robot.turret.update();
        if (inCompetition) {
            robot.turret.turretState = Turret.TurretState.TRACKING;
            robot.shooter.setShooterState(Shooter.ShooterState.UPDATE);
        }
        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            //telemetry.addData("TRACKING SHOOTER DATA", robot.shooter.isTrackingData());
            //telemetry.addLine();
            //telemetry.addData("SHOOTER ADJUSTMENT", robot.shooter.adjustment);
            //telemetry.addData("TURRET ADJUSTMENT", robot.turret.adjustment);
            //telemetry.addLine();

            updateDrive();
            updateDriver2();
            updateDriver1();
            CommandScheduler.getInstance().run();

            robot.update(currentlyMoving);

            telemetry.addData("Alliance", BrainSTEMRobot.alliance);

            if (printCollector)
                robot.collection.printInfo();
            if (printLimelight)
                robot.limelight.printInfo();
            if (printTurret)
                robot.turret.printInfo();
            if (printShooter)
                robot.shooter.printInfo();
            if(printShootingSystem)
                robot.shootingSystem.printInfo(telemetry);

            telemetry.addLine();
            telemetry.addData("ljx", gamepad1.left_stick_x * 100);
            telemetry.addData("ljy", gamepad1.left_stick_y * 100);
            telemetry.addData("rjx", gamepad1.right_stick_x * 100);
            telemetry.addLine();

            telemetry.addData("dt", robot.shootingSystem.dt);
            updateDashboardField();

//            telemetry.addData("FPS", MathUtils.format2(framesRunning / timeRunning));
            telemetry.update();

            Pose2d p = robot.drive.localizer.getPose();
            PoseStorage.autoX = p.position.x;
            PoseStorage.autoY = p.position.y;
            PoseStorage.autoHeading = p.heading.toDouble();
        }
        robot.limelight.limelight.stop();
        robot.drive.stop();
    }

    private void updateDrive() {
        if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE && robot.limelight.localization.manualPoseUpdate) {
            robot.drive.stop();
            return;
        }
        currentlyMoving = Math.abs(gamepad1.left_stick_x) > noMoveJoystickThreshold || Math.abs(gamepad1.left_stick_y) > noMoveJoystickThreshold || Math.abs(gamepad1.right_stick_x) > noMoveJoystickThreshold;
        double amp = robot.shootingSystem.currentlyShootingWhileMoving ? ShootingSystem.generalParams.maxShootWhileMovingSpeed : 1;
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).times(amp),
                -gamepad1.right_stick_x * amp
        ));
    }

    private void updateDriver1() {
        robot.turret.addOscillationData = gamepad1.dpad_up;
        if(robot.collection.getClutchState() == Collection.ClutchState.UNENGAGED) {
            if (gp1.gamepad.right_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
            else if (gp1.gamepad.left_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
            else
                robot.collection.setCollectionState(Collection.CollectionState.OFF);
        }

        if(!inCompetition) {
            if (gp1.isFirstRightBumper())
                if (robot.shooter.getShooterState() == Shooter.ShooterState.UPDATE)
                    robot.shooter.setShooterState(Shooter.ShooterState.OFF);
                else
                    robot.shooter.setShooterState(Shooter.ShooterState.UPDATE);

            if (gp1.isFirstLeftBumper()) {
                if (robot.turret.turretState == Turret.TurretState.CENTER)
                    robot.turret.turretState = Turret.TurretState.TRACKING;
                else
                    robot.turret.turretState = Turret.TurretState.CENTER;
            }

            if (gp1.isFirstBack()) {
                robot.limelight.localization.maxTranslationalVariance = 0;
                robot.limelight.localization.maxHeadingVarianceDeg = 0;
                robot.limelight.localization.maxTranslationalError = 0;
                robot.limelight.localization.maxHeadingErrorDeg = 0;
            }
        }
    }

    private void updateDriver2() {
        if(robot.collection.getClutchState() == Collection.ClutchState.ENGAGED) {
            if (gp2.isFirstA())
                if (robot.collection.getCollectionState() != Collection.CollectionState.INTAKE)
                    robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
                else
                    robot.collection.setCollectionState(Collection.CollectionState.OFF);
        }
        if (gp2.isFirstB())
            if (robot.collection.getClutchState() == Collection.ClutchState.ENGAGED)
                robot.collection.setClutchState(Collection.ClutchState.UNENGAGED);
            else
                robot.collection.setClutchState(Collection.ClutchState.ENGAGED);

        if (gp2.isFirstLeftBumper())
            robot.collection.setFlickerState(Collection.FlickerState.HALF_UP_DOWN);
        if(gp2.isFirstLeftTrigger())
            robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);

        if (gp2.isFirstDpadLeft())
            robot.turret.changeEncoderAdjustment(Turret.turretParams.fineAdjust);
        if (gp2.isFirstDpadRight())
            robot.turret.changeEncoderAdjustment(-Turret.turretParams.fineAdjust);

        if (gp2.isFirstDpadUp())
            robot.shooter.changeVelocityAdjustment(10);
        if (gp2.isFirstDpadDown())
            robot.shooter.changeVelocityAdjustment(-10);

        if(gp2.isFirstRightBumper()) {
            robot.limelight.localization.manualPoseUpdate = true;
            robot.limelight.localization.setState(LimelightLocalization.LocalizationState.UPDATING_POSE);
        }
        if (gp2.isFirstRightTrigger()) {
            Pose2d resetPose = createPose(alliance == Alliance.RED ? redCornerResetPose : blueCornerResetPose);
            robot.drive.pinpoint().setPose(resetPose);
            robot.led.lastPinpointResetTimeMs = System.currentTimeMillis();
        }
    }
    private void updateDashboardField() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        robot.addRobotInfo(fieldOverlay);

        // draw goal
        fieldOverlay.setStroke("yellow");
        Vector2d defaultGoalPos = new Vector2d(robot.shootingSystem.goalPosIn.x, robot.shootingSystem.goalPosIn.z);
        fieldOverlay.strokeCircle(defaultGoalPos.x, defaultGoalPos.y, 3);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}