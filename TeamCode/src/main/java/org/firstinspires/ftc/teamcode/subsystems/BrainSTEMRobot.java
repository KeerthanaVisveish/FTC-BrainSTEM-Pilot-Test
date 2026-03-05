package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.LimelightBallDetection;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathGeneration;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathInfo;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathPose;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;
import java.util.Collections;
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
    public final Telemetry telemetry;
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

        shootingSystem.drawShootingInfo(fieldOverlay);

        limelight.addLimelightInfo(fieldOverlay);
    }
    public double getFilteredVoltage() {
        return drive.getFilteredVoltage();
    }
    public double getRawVoltage() {
        return drive.getRawVoltage();
    }

    public Action scanForBalls(DoubleSupplier angle1Sup, DoubleSupplier angle2Sup) {
        return new SequentialAction(
                // first scan
                limelight.ballDetection.clearBallSnapshots(),
                new SequentialAction(
                        turret.rotateToCustomTarget(angle1Sup),
                        new SleepAction(LimelightBallDetection.params.waitToScanAfterTurretMove),
                        limelight.ballDetection.takeBallSnapshotAction()
                ),
                // second scan
                angle2Sup == null ?
                        new InstantAction(() -> {}) :
                        new Action() {
                            Action secondScan = null;
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                if (secondScan == null) {
                                    if (limelight.ballDetection.getCurrentBlobPositions().size() >= 2)
                                        return false;
                                    secondScan = new SequentialAction(
                                            turret.rotateToCustomTarget(angle2Sup),
                                            new SleepAction(LimelightBallDetection.params.waitToScanAfterTurretMove),
                                            limelight.ballDetection.takeBallSnapshotAction()
                                    );
                                }
                                return secondScan.run(telemetryPacket);
                            }
                        },

                new InstantAction(() -> turret.setTurretState(Turret.TurretState.TRACKING))
        );
    }
    public Action lookAtClassifier(Turret.TurretState endingTurretState) {
        return new SequentialAction(
                turret.rotateToCustomTarget(() -> {
                    Vector2d classifierPosition = new Vector2d(-22, alliance == Alliance.RED ? 72 : -72);
                    Vector2d turretToClassifier = shootingSystem.turretPose.position.minus(classifierPosition);
                    return MathUtils.vecAngle(turretToClassifier);
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
    public Action getLimelightCollectSequence(Vector2d lookPosition, double maxLimelightWaitTime) {
        return new SequentialAction(
                turret.rotateToCustomTarget(() -> MathUtils.vecAngle(lookPosition.minus(drive.pinpoint().getPose().position))),
                new SleepAction(LimelightBallDetection.params.waitToScanAfterTurretMove),
                new InstantAction(() -> collection.setCollectionState(Collection.CollectionState.INTAKE)),
                new ParallelAction(
                        generateLimelightCollectDrive(maxLimelightWaitTime),
                        new SequentialAction(
                                new SleepAction(0.2),
                                new InstantAction(() -> turret.setTurretState(Turret.TurretState.TRACKING))
                        )
                )
        );
    }
    private Action generateLimelightCollectDrive(double maxLimelightWaitTime) {
        return new Action() {
            PathInfo pathInfo = null;
            DrivePath drivePath = null;
            ElapsedTime timer = null;
            Pose2d startPose = null;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (timer == null) {
                    timer = new ElapsedTime();
                    timer.reset();
                }

                if (drivePath == null) {
                    if (timer.seconds() > maxLimelightWaitTime)
                        return false;

                    startPose = drive.localizer.getPose();
                    Vector2d giantClump = limelight.ballDetection.getCurrentGiantClumpPosition();
                    ArrayList<Vector2d> ballPositions;
                    if (giantClump == null) {
                        ballPositions = limelight.ballDetection.getCurrentBlobPositions();
                        PathGeneration.pathGenParams.alwaysUseLaneCollectNumBalls = PathGeneration.pathGenParams.defaultAlwaysUseLaneCollectNumBalls;
                    }
                    else {
                        ballPositions = new ArrayList<>(Collections.singletonList(giantClump));
                        PathGeneration.pathGenParams.alwaysUseLaneCollectNumBalls = 1;
                    }
                    pathInfo = PathGeneration.generateSimplifiedAutoCollectPath(startPose, ballPositions);
                    if (pathInfo == null) {
                        telemetry.addLine("path is null");
                        return true;
                    }
                    drivePath = new DrivePath(drive);
                    for (PathPose pathPose : pathInfo.optimizedPathPoses)
                        drivePath.addWaypoint(pathPose.waypoint);
                }
                limelight.ballDetection.drawPath(telemetryPacket.fieldOverlay(), startPose, pathInfo.getOptimizedPoses());
                return drivePath.run(telemetryPacket);
            }
        };
    }
}
