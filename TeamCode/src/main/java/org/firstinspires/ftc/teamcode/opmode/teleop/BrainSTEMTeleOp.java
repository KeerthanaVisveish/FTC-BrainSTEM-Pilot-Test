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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.robot.subsystems.Parking;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.Turret;
import org.firstinspires.ftc.teamcode.robot.limelight.Limelight;
import org.firstinspires.ftc.teamcode.robot.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.misc.PoseUpdatePacket;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

@Config
public class BrainSTEMTeleOp extends LinearOpMode {
    public static double odoPoseStoreFrequency = 2; // in Hertz
    public static boolean printCollector = false,
            printShooter = false, printTurret = false, printShootingSystem = false,
            printLimelight = false, printPark = false, printDrivetrain = false, printHood = false, printSrsHub = false;
    public static boolean streamCameraToFTCDashboard = true;
    public static boolean inCompetition = true, allowD1Shoot = false;

    public static LimelightLocalization.LocalizationType localizationType = LimelightLocalization.LocalizationType.CONTINUOUS;
    // TODO: check these during driver practice
    public static double[] redCornerResetPose = { 62.7, -61.7, 90 };
    // 62.5, -61.6, 90
    // 62.8, -61.7, 90

    public static double[] blueCornerResetPose = { 62.9, 61.6, -90 }; // old: { 63.5, 61.9, -90 }
    // 62.8, 62.5, -90
    // 63.3, 61.7, -90
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

        ElapsedTime teleopTimer = new ElapsedTime();

        telemetry.addData("Alliance", alliance);
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
                    robot.shootingSystemV1.turret.resetEncoders();
                robot.shootingSystemV1.turret.updateProperties(.01);
                telemetry.addData("reset turret encoder", "hold START + BACK");
                telemetry.addData("turret encoder", robot.shootingSystemV1.turret.getEncoder());
            }
            else
                telemetry.addLine("turret encoder reset disabled");
            telemetry.update();
        }

        waitForStart();
        teleopTimer.reset();

        PoseUpdatePacket.poseUpdatePackets.clear();
        robot.startOpmode();
        if (inCompetition) {
            robot.shootingSystemV1.setTurretToGoalTargeting();
            robot.shootingSystemV1.setShooterToGoalTargeting();
            robot.shootingSystemV1.setHoodToGoalTargeting();
        }

        double lastOdoPoseStoreTime = -10;
        while (opModeIsActive()) {
            // pose storage debugging
            if(teleopTimer.seconds() - lastOdoPoseStoreTime > 1 / odoPoseStoreFrequency) {
                lastOdoPoseStoreTime = teleopTimer.seconds();
                PoseUpdatePacket.poseUpdatePackets.add(new PoseUpdatePacket(
                        PoseUpdatePacket.UpdateType.ODOMETRY,
                        robot.drive.pinpoint().getPose(),
                        teleopTimer.seconds()
                ));
            }

            gp1.update();
            gp2.update();

            updateDrive();
            updateDriver2();
            updateDriver1();
            if(!inCompetition) {
                if(testingPark)
                    robot.parking.setParkState(Parking.ParkState.TESTING);
                Parking.PARK_PARAMS.testingPos += gamepad1.left_stick_y * Parking.PARK_PARAMS.testingInc;
            }
            robot.update();


            robot.drive.pinpoint().printInfo(telemetry);

            if(!inCompetition) {
                if (printCollector)
                    robot.collector.printInfo();
                if (printLimelight)
                    robot.limelight.printInfo(telemetry);
                if (printTurret)
                    robot.shootingSystemV1.turret.printInfo();
                if (printShooter)
                    robot.shootingSystemV1.shooter.printInfo();
                if (printShootingSystem)
                    robot.shootingSystemV1.printInfo();
                if (printPark)
                    robot.parking.printInfo();
                if(printDrivetrain)
                    robot.drive.printInfo(telemetry);
                if(printHood)
                    robot.shootingSystemV1.hood.printInfo();
                if(printSrsHub)
                    robot.shootingSystemV1.srsHub.printInfo();
            }
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.clear();
            robot.drawRobotInfo(fieldOverlay);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("dt", robot.getDt());

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

        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * currentDriveAxialAmp,
                        -gamepad1.left_stick_x * currentDriveLateralAmp
                ),
                -gamepad1.right_stick_x * currentTurnAmp
        ));
    }

    private void updateDriver1() {
        if(robot.collector.getClutchState() == Collector.ClutchState.DISENGAGED) {
            if (gp1.gamepad.right_trigger > 0.2)
                robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
            else if (gp1.gamepad.left_trigger > 0.2)
                robot.collector.setIntakeState(Collector.IntakeState.OUTTAKE);
            else
                robot.collector.setIntakeState(Collector.IntakeState.OFF);
        }

        if (gp1.isFirstB() && !inCompetition) {
            if (robot.shootingSystemV1.turretCentered()) {
                robot.shootingSystemV1.setTurretToGoalTargeting();
                robot.shootingSystemV1.setShooterToGoalTargeting();
                robot.shootingSystemV1.setHoodToGoalTargeting();
            }
            else {
                robot.shootingSystemV1.setTurretToCenter();
                robot.shootingSystemV1.setShooterOff();
                robot.shootingSystemV1.setHoodToCustomExitAngle(Math.toRadians(45));
            }
        }
        if(gp1.isFirstLeftBumper())
            robot.collector.setFlickerState(Collector.FlickerState.FULL_UP_DOWN);
        if(gp1.isFirstRightBumper()) {
            if(robot.collector.getClutchState() == Collector.ClutchState.DISENGAGED && robot.shootingSystemV1.meetsFirstSafetyInterlocks()) {
                robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
                robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
            }
            else {
                robot.collector.setClutchState(Collector.ClutchState.DISENGAGED);
                robot.collector.setIntakeState(Collector.IntakeState.OFF);
            }
        }
        if(!inCompetition || allowD1Shoot) {
            if(gp1.isFirstY()) {
                if(robot.collector.getClutchState() == Collector.ClutchState.DISENGAGED) {
                    robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
                    robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
                }
                else {
                    robot.collector.setClutchState(Collector.ClutchState.DISENGAGED);
                    robot.collector.setIntakeState(Collector.IntakeState.OFF);
                }
            }
        }
    }

    private void updateDriver2() {
        if (gp2.isFirstB()) {
            if (robot.collector.getClutchState() == Collector.ClutchState.ENGAGED)
                robot.collector.setClutchState(Collector.ClutchState.DISENGAGED);
            else {
                robot.collector.setIntakeState(Collector.IntakeState.OFF);
                robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            }
        }
        if(robot.collector.getClutchState() == Collector.ClutchState.ENGAGED) {
            if (gp2.isFirstA())
                if (robot.collector.getIntakeState() != Collector.IntakeState.INTAKE)
                    robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
                else
                    robot.collector.setIntakeState(Collector.IntakeState.OFF);
            else if(gp2.isFirstX())
                if(robot.collector.getIntakeState() != Collector.IntakeState.INTAKE_SLOW)
                    robot.collector.setIntakeState(Collector.IntakeState.INTAKE_SLOW);
                else
                    robot.collector.setIntakeState(Collector.IntakeState.OFF);
        }
        if (gp2.isFirstLeftBumper())
            robot.collector.setFlickerState(Collector.FlickerState.HALF_UP_DOWN);
        if(gp2.isFirstLeftTrigger())
            robot.collector.setFlickerState(Collector.FlickerState.FULL_UP_DOWN);

        if (gp2.isFirstDpadLeft())
            robot.shootingSystemV1.incTurretAngleAdjustment();
        else if (gp2.isFirstDpadRight())
            robot.shootingSystemV1.decTurretAngleAdjustment();
        if(gp2.isFirstLeftStickButton())
            robot.shootingSystemV1.decShooterSpeedAdjustment();
        else if(gp2.isFirstRightStickButton())
            robot.shootingSystemV1.incShooterSpeedAdjustment();

        if (gamepad2.right_trigger > .5) {
            Pose2d cornerResetPose = createPose(alliance == Alliance.RED ? redCornerResetPose : blueCornerResetPose);
            robot.drive.pinpoint().setPose(cornerResetPose);
            robot.shootingSystemV1.resetAdjustments();
        }

        if (gp2.isFirstY()) {
            if (robot.parking.getParkState() != Parking.ParkState.EXTENDED) {
                robot.parking.setParkState(Parking.ParkState.EXTENDED);
                robot.shootingSystemV1.setTurretToCenter();
                robot.shootingSystemV1.setShooterOff();
            }
            else if (robot.shootingSystemV1.turretCentered()) {
                robot.shootingSystemV1.setTurretToCustomAngle(Turret.turretParams.maxAngle, Turret.turretParams.minParkRotateVoltage, true);
            }
            else {
                robot.parking.setParkState(Parking.ParkState.RETRACTED);
                robot.shootingSystemV1.setTurretToCenter();
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
    }
}