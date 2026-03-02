package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;
import java.util.function.DoubleSupplier;


@Config
public class BrainSTEMRobot {
    public static double width = 13, length = 16; // inches
    public static double rampWidth = .9382;
    public static boolean enableSubsystems = true;
    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableLimelight = true, enablePark = true, enableLED = true;
    public Turret turret;
    public Shooter shooter;
    public ShootingSystem shootingSystem;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
    public Limelight limelight;
    public LED led;
    public static Alliance alliance;
    private final ArrayList<Component> subsystems;
    private final Telemetry telemetry;
    public GamepadTracker g1;

    public BrainSTEMRobot(Alliance allianceColor, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        BrainSTEMRobot.alliance = allianceColor;
        subsystems = new ArrayList<>();

        drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = new Limelight(hardwareMap, telemetry, this);
        shootingSystem = new ShootingSystem(hardwareMap, this);
        turret = new Turret(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, this);
        collection = new Collection(hardwareMap, telemetry, this);
        parking = new Parking(hardwareMap, telemetry, this);
        led = new LED(hardwareMap, telemetry, this);

        if (enableTurret)
            subsystems.add(turret);
        if (enableShooter)
            subsystems.add(shooter);
        if (enableCollection)
            subsystems.add(collection);
        if (enablePark)
            subsystems.add(parking);
        if (enableLimelight)
            subsystems.add(limelight);
        if (enableLED)
            subsystems.add(led);
    }
    public void startOpmode() {
        drive.resetVoltageTimer();
    }
    public void setG1(GamepadTracker g1) {
        this.g1 = g1;
    }
    public void updateInfo(boolean useTurretLookAhead) {
        drive.updateVoltageFiltering();
        drive.updatePoseEstimate();
        shootingSystem.updateInfo(useTurretLookAhead);
    }
    public void update() {
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("Robot Pose", MathUtils.format3(pose.position.x) + ", " + MathUtils.format3(pose.position.y) + " | " + MathUtils.format3(pose.heading.toDouble()));
        if(enableSubsystems)
            for (Component c : subsystems)
                c.update();

        shootingSystem.sendHardwareInfo();
    }

    public void drawRobotInfo(Canvas fieldOverlay) {
        // draw robot, turret, exit position, and limelight pose
        Pose2d robotPose = drive.pinpoint().getPose();

        fieldOverlay.setStroke("red");
        Drawing.drawRobot(fieldOverlay, robotPose);
        fieldOverlay.setStroke("green");
        Drawing.drawRobotSimple(fieldOverlay, shootingSystem.turretPose, 5);
        fieldOverlay.setStroke("purple");
//        Drawing.drawRobotSimple(fieldOverlay, new Pose2d(shootingSystem.ballExitPos, 0), 3);
//        fieldOverlay.setStroke("purple");
//        Drawing.drawRobotSimple(fieldOverlay, new Pose2d(shootingSystem.futureBallExitPos, 0), 3);

        limelight.addLimelightInfo(fieldOverlay);

        // draw where turret is pointed
        fieldOverlay.setAlpha(1);
//        double dist = Math.hypot(shootingSystem.ballExitPos.x - shootingSystem.goalPosIn.x, shootingSystem.ballExitPos.y - shootingSystem.goalPosIn.y);
        double dist = 300;

        fieldOverlay.setStroke("purple");
        fieldOverlay.strokeLine(
                shootingSystem.turretPose.position.x,
                shootingSystem.turretPose.position.y,
                shootingSystem.turretPose.position.x + dist * Math.cos(turret.currentAbsoluteAngleRad),
                shootingSystem.turretPose.position.y + dist * Math.sin(turret.currentAbsoluteAngleRad)
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                shootingSystem.turretPose.position.x,
                shootingSystem.turretPose.position.y,
                shootingSystem.turretPose.position.x + dist * Math.cos(shootingSystem.lookAheadTurretTargetAngleRad),
                shootingSystem.turretPose.position.y + dist * Math.sin(shootingSystem.lookAheadTurretTargetAngleRad)
        );
        double speedMag = shootingSystem.lookAheadTargetExitSpeedMps;
        fieldOverlay.setStroke("red");
        fieldOverlay.strokeLine(
                shootingSystem.turretPose.position.x,
                shootingSystem.turretPose.position.y,
                shootingSystem.turretPose.position.x + dist * Math.cos(shootingSystem.desiredBallDir),
                shootingSystem.turretPose.position.y + dist * Math.sin(shootingSystem.desiredBallDir)
        );

        if(turret.perpVelVec != null) {
            fieldOverlay.setStroke("blue");
            fieldOverlay.strokeLine(
                    shootingSystem.turretPose.position.x,
                    shootingSystem.turretPose.position.y,
                    shootingSystem.turretPose.position.x + shootingSystem.robotVelAtTurretIps.x,
                    shootingSystem.turretPose.position.y + shootingSystem.robotVelAtTurretIps.y
            );
//            fieldOverlay.strokeLine(
//                    shootingSystem.turretPose.position.x,
//                    shootingSystem.turretPose.position.y,
//                    shootingSystem.turretPose.position.x + turret.perpVelVec.x * 10,
//                    shootingSystem.turretPose.position.y + turret.perpVelVec.y * 10
//            );
//            fieldOverlay.strokeLine(
//                    shootingSystem.goalPosIn.x,
//                    shootingSystem.goalPosIn.z,
//                    shootingSystem.goalPosIn.x + shootingSystem.futureTurretPosRelativeToGoal.x,
//                    shootingSystem.goalPosIn.z + shootingSystem.futureTurretPosRelativeToGoal.y
//            );
        }
    }
    public double getFilteredVoltage() {
        return drive.getFilteredVoltage();
    }
    public double getRawVoltage() {
        return drive.getRawVoltage();
    }

    public Action scanForBalls(DoubleSupplier angle1, DoubleSupplier angle2) {
        return new SequentialAction(
                turret.rotateToCustomTarget(angle1),
                new SleepAction(0.03),
                limelight.ballDetection.takeBallSnapshotAction(),
                turret.rotateToCustomTarget(angle2),
                limelight.ballDetection.takeBallSnapshotAction()
        );
    }
    public Action lookAtClassifier(Turret.TurretState endingTurretState) {
        return new SequentialAction(
                turret.rotateToCustomTarget(() -> {
                    Vector2d classifierPosition = new Vector2d(-22, alliance == Alliance.RED ? 72 : -72);
                    Vector2d turretToClassifier = shootingSystem.turretPose.position.minus(classifierPosition);
                    double robotAngle = drive.localizer.getPose().heading.toDouble();
                    double absoluteAngle = MathUtils.vecAngle(turretToClassifier);
                    return MathUtils.angleNormDeltaRad(absoluteAngle - robotAngle);
                }),
                limelight.classifier.readBallsInClassifier(),
                new InstantAction(() -> turret.setTurretState(endingTurretState))
        );
    }
    public Action waitXSecondsIf2BallsInClassifier(double maxWaitTime) {
        return new Action() {
            ElapsedTime timer = null;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (timer == null) {
                    timer = new ElapsedTime();
                    timer.reset();
                }
                int numBalls = limelight.classifier.getMostCommonNumBalls();

                telemetry.addData("num balls seen", numBalls);
                // unsuccessful read or 3 or more balls
                if (numBalls == -1 || numBalls > 2)
                    return false;

                // 0, 1, or 2 balls
                return timer.seconds() <= maxWaitTime;
            }
        };
    }
}
