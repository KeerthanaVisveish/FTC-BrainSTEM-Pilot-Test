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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

@Config
public class BrainSTEMTeleOp extends LinearOpMode {
    public static boolean printCollector = false,
            printShooter = false, printTurret = false, printShootingSystem = false,
            printLimelight = false, printPark = false, printDrivetrain;
    public static boolean streamCameraToFTCDashboard = true;
    public static boolean inCompetition = true, allowD1Shoot = false;

    public static LimelightLocalization.LocalizationType localizationType = LimelightLocalization.LocalizationType.CONTINUOUS;
    // TODO: check these during driver practice
    public static double[] redCornerResetPose = { 64, -61.1, 90 };
    public static double[] blueCornerResetPose = { 64, 61.1, -90 }; // old: { 63.5, 61.9, -90 }
//    public static double[] redGateResetPose = { 10.5, 62.4, 180 }; // i didn't trust the old one
//    public static double[] blueGateResetPose = { 10.5, -62.4, 180 };
    public static boolean shouldScore = true;
    public static boolean testingPark = false;
    // RED:
    // (62.706, -62.288, 90.294)
    // (62.038, -62.863, 89.575)
    // (62.251, -62.892, 89.948)

    public static double parkDriveAxialAmp = .3, parkDriveLateralAmp = .5, slowTurnAmp = .4;
    public static double currentDriveAxialAmp = 1, currentDriveLateralAmp = 1, currentTurnAmp = 1;

    BrainSTEMRobot robot;

    GamepadTracker gp1;
    GamepadTracker gp2;
    private final Alliance alliance;

    public BrainSTEMTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        Pose2d startPose = new Pose2d(PoseStorage.autoX, PoseStorage.autoY, PoseStorage.autoHeading);

        Limelight.startingPipeline = Limelight.APRIL_TAG_PIPELINE;
        LimelightLocalization.localizationType = localizationType;
        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        robot.setG1(gp1);
        telemetry.addData("Alliance", BrainSTEMRobot.alliance);
        telemetry.addData("starting pose", MathUtils.formatPose3(startPose));
        if (!robot.limelight.limelight.isConnected())
            telemetry.addLine("WARNING - LIMELIGHT IS NOT CONNECTED");
        if (!robot.limelight.limelight.isRunning())
            telemetry.addLine("WARNING - LIMELIGHT IS NOT RUNNING");
        telemetry.update();

        if (!inCompetition && streamCameraToFTCDashboard)
            FtcDashboard.getInstance().startCameraStream(robot.limelight.limelight, 10);

        while (opModeInInit()) {
            if (!inCompetition) {
                if (gamepad1.start && gamepad1.backWasPressed())
                    robot.shootingSystem.resetTurretEncoder();
                robot.shootingSystem.updateProperties();
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
            ShootingSystem.testingParams.powerTurret = true;
            ShootingSystem.testingParams.powerHighShooter = true;
            ShootingSystem.testingParams.powerLowShooter = true;
        }
        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            updateDrive();
            updateDriver2();
            updateDriver1();
            if(!inCompetition) {
                robot.shootingSystem.setShouldScore(shouldScore);
                if(testingPark)
                    robot.parking.setParkState(Parking.ParkState.TESTING);
                Parking.PARK_PARAMS.testingPos += gamepad1.left_stick_y * Parking.PARK_PARAMS.testingInc;
            }
            robot.updateInfo();
            robot.update();


            robot.drive.pinpoint().printInfo(telemetry);

            if(!inCompetition) {
                if (printCollector)
                    robot.collector.printInfo();
                if (printLimelight)
                    robot.limelight.printInfo();
                if (printTurret)
                    robot.turret.printInfo();
                if (printShooter)
                    robot.shooter.printInfo();
                if (printShootingSystem)
                    robot.shootingSystem.printInfo(telemetry);
                if (printPark)
                    robot.parking.printInfo();
                if(printDrivetrain)
                    robot.drive.printInfo(telemetry);
            }
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.clear();
            robot.drawRobotInfo(fieldOverlay);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("dt", robot.shootingSystem.dt);

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
        if(gp1.isFirstA())
            if(currentDriveAxialAmp == 1) {
                currentDriveAxialAmp = parkDriveAxialAmp;
                currentDriveLateralAmp = parkDriveLateralAmp;
                currentTurnAmp = slowTurnAmp;
            }
            else {
                currentDriveAxialAmp = 1;
                currentDriveLateralAmp = 1;
                currentTurnAmp = 1;
            }

        double shootWhileMovingAmp = robot.shootingSystem.currentlyShootingWhileMoving ? ShootingSystem.generalParams.maxShootWhileMovingSpeed : 1;

        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * currentDriveAxialAmp,
                        -gamepad1.left_stick_x * currentDriveLateralAmp
                ).times(shootWhileMovingAmp),
                -gamepad1.right_stick_x * shootWhileMovingAmp * currentTurnAmp
        ));
    }

    private void updateDriver1() {
        if(robot.collector.getClutchState() == Collector.ClutchState.UNENGAGED) {
            if (gp1.gamepad.right_trigger > 0.2)
                robot.collector.setCollectionState(Collector.CollectionState.INTAKE);
            else if (gp1.gamepad.left_trigger > 0.2)
                robot.collector.setCollectionState(Collector.CollectionState.OUTTAKE);
            else
                robot.collector.setCollectionState(Collector.CollectionState.OFF);
        }

        if (gp1.isFirstLeftBumper()) {
            if (robot.turret.turretState == Turret.TurretState.CENTER)
                robot.turret.turretState = Turret.TurretState.TRACKING;
            else
                robot.turret.turretState = Turret.TurretState.CENTER;
        }

        if(gp1.isFirstB()) {
            if(robot.shooter.getShooterState() == Shooter.ShooterState.UPDATE) {
                robot.shooter.setShooterState(Shooter.ShooterState.OFF);
                robot.turret.setTurretState(Turret.TurretState.CENTER);
            }
            else {
                robot.shooter.setShooterState(Shooter.ShooterState.UPDATE);
                robot.turret.setTurretState(Turret.TurretState.TRACKING);
            }
        }
        if(!inCompetition) {
            if (gp1.isFirstRightBumper())
                if (robot.shooter.getShooterState() == Shooter.ShooterState.UPDATE)
                    robot.shooter.setShooterState(Shooter.ShooterState.OFF);
                else
                    robot.shooter.setShooterState(Shooter.ShooterState.UPDATE);
        }
        if(!inCompetition || allowD1Shoot) {
            if(gp1.isFirstY()) {
                if(robot.collector.getClutchState() == Collector.ClutchState.UNENGAGED) {
                    robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
                    robot.collector.setCollectionState(Collector.CollectionState.INTAKE);
                }
                else {
                    robot.collector.setClutchState(Collector.ClutchState.UNENGAGED);
                    robot.collector.setCollectionState(Collector.CollectionState.OFF);
                }
            }
        }
    }

    private void updateDriver2() {
        if (gp2.isFirstB()) {
            if (robot.collector.getClutchState() == Collector.ClutchState.ENGAGED)
                robot.collector.setClutchState(Collector.ClutchState.UNENGAGED);
            else {
                robot.collector.setCollectionState(Collector.CollectionState.OFF);
                robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            }
        }
        if(robot.collector.getClutchState() == Collector.ClutchState.ENGAGED) {
            if (gp2.isFirstA())
                if (robot.collector.getCollectionState() != Collector.CollectionState.INTAKE)
                    robot.collector.setCollectionState(Collector.CollectionState.INTAKE);
                else
                    robot.collector.setCollectionState(Collector.CollectionState.OFF);
            else if(gp2.isFirstX())
                if(robot.collector.getCollectionState() != Collector.CollectionState.INTAKE_SLOW)
                    robot.collector.setCollectionState(Collector.CollectionState.INTAKE_SLOW);
                else
                    robot.collector.setCollectionState(Collector.CollectionState.OFF);
        }
        if (gp2.isFirstLeftBumper())
            robot.collector.setFlickerState(Collector.FlickerState.HALF_UP_DOWN);
        if(gp2.isFirstLeftTrigger())
            robot.collector.setFlickerState(Collector.FlickerState.FULL_UP_DOWN);

        if (gp2.isFirstDpadLeft())
            robot.turret.changeEncoderAdjustment(Turret.turretParams.fineAdjust);
        else if (gp2.isFirstDpadRight())
            robot.turret.changeEncoderAdjustment(-Turret.turretParams.fineAdjust);
        if(gp2.isFirstLeftStickButton())
            robot.shooter.changeVelocityAdjustment(-Shooter.shooterParams.fineAdjust);
        else if(gp2.isFirstRightStickButton())
            robot.shooter.changeVelocityAdjustment(Shooter.shooterParams.fineAdjust);

        if (gamepad2.right_trigger > .5) {
            double curHeading = robot.drive.pinpoint().getPose().heading.toDouble();
            Pose2d cornerResetPose = createPose(alliance == Alliance.RED ? redCornerResetPose : blueCornerResetPose);
//            Pose2d gateResetPose = createPose(alliance == Alliance.RED ? redGateResetPose : blueGateResetPose);
            double cornerHeadingError = Math.abs(MathUtils.angleNormDeltaRad(curHeading - cornerResetPose.heading.toDouble()));
//            double gateHeadingError = Math.abs(MathUtils.angleNormDeltaRad(curHeading - gateResetPose.heading.toDouble()));
//            robot.drive.pinpoint().setPose(cornerHeadingError < gateHeadingError ? cornerResetPose : gateResetPose);
            robot.drive.pinpoint().setPose(cornerResetPose);
            robot.turret.resetAllEncoderAdjustments();
            robot.led.lastManualRelocalizationTimeMs = System.currentTimeMillis();
        }

        if (gp2.isFirstY()) {
            if (robot.parking.getParkState() != Parking.ParkState.EXTENDED) {
                robot.parking.setParkState(Parking.ParkState.EXTENDED);
                robot.turret.setTurretState(Turret.TurretState.CENTER);
                robot.shooter.setShooterState(Shooter.ShooterState.OFF);
            }
            else if (robot.turret.getTurretState() != Turret.TurretState.TRACK_CUSTOM_TARGET) {
                robot.turret.rotateToRelativeCustomTarget(Turret.turretParams.maxTeleAngle);
                robot.turret.setCustomTargetMinPower(Turret.turretParams.minParkRotateVoltage);
                robot.turret.setCustomTargetPassPosition(true);
            }
            else {
                robot.parking.setParkState(Parking.ParkState.RETRACTED);
                robot.turret.setTurretState(Turret.TurretState.CENTER);
                robot.turret.setCustomTargetMinPower(0);
                robot.turret.setCustomTargetPassPosition(false);
            }
        }
        if (!inCompetition) {
            if(Math.abs(gamepad2.left_stick_y) > .3) {
                robot.parking.setParkState(Parking.ParkState.OFF);
                robot.parking.setLeftParkPosition(robot.parking.parkLeftServo.getPosition() + Parking.PARK_PARAMS.TESTING_INC * Math.signum(gamepad2.left_stick_y));
            }
            if(Math.abs(gamepad2.right_stick_y) > .3) {
                robot.parking.setParkState(Parking.ParkState.OFF);
                robot.parking.setRightParkPosition(robot.parking.parkRightServo.getPosition() + Parking.PARK_PARAMS.TESTING_INC * Math.signum(gamepad2.right_stick_y));
            }
        }
//        if (gp2.isFirstBack())
//            robot.limelight.ballDetection.takeBallScan();
    }
}