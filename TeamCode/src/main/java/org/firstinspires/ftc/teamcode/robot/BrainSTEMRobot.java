package org.firstinspires.ftc.teamcode.robot;

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
import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPid;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.limelight.Limelight;
import org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.Blob;
import org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.LimelightBallDetection;
import org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration.PathGeneration;
import org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration.PathInfo;
import org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration.PathPose;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Parking;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.ShootingSystem;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;


@Config
public class BrainSTEMRobot {
    public static boolean drawRobot = true, drawRobotDerivatives = false, drawPoseClipping = true, drawShooting = true, drawLimelight = false;

    public ShootingSystem shootingSystem;
    public Collector collector;
    public Parking parking;
    public MecanumDrive drive;
    public Limelight limelight;
//    public LED led;
    private final Alliance alliance;
    public final Telemetry telemetry;
    public GamepadTracker g1;
    private final ElapsedTime dtTimer;
    private double dt;

    public BrainSTEMRobot(Alliance allianceColor, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        this.alliance = allianceColor;

        drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = new Limelight(hardwareMap, telemetry, this);
        shootingSystem = new ShootingSystem(hardwareMap, telemetry, initialPose, alliance);
        collector = new Collector(hardwareMap, telemetry);
        parking = new Parking(hardwareMap, telemetry);
//        led = new LED(hardwareMap, telemetry);
        dtTimer = new ElapsedTime();
        dtTimer.reset();
    }
    public void startOpmode() {
        drive.resetVoltageTimer();
    }
    public void setG1(GamepadTracker g1) {
        this.g1 = g1;
    }

    public void update() {
        dt = dtTimer.seconds();
        dtTimer.reset();

        drive.updateVoltageFiltering();
        drive.updatePoseEstimate();

        limelight.update();
        shootingSystem.updateSubsystems(drive.pinpoint().getPose(), drive.pinpoint().getVelocity(), drive.pinpoint().getRawAccel(), drive.getFilteredVoltage(), dt, alliance);
        collector.updateState(shootingSystem.meetsSafetyInterlocks());
//        led.update(this);
    }

    public double getDt() {
        return dt;
    }
    public void drawRobotInfo(Canvas fieldOverlay) {
        // draw robot, turret, exit position, and limelight pose
        Pose2d robotPose = drive.pinpoint().getPose();
        OdoInfo robotVelocity = drive.pinpoint().getVelocity();
        OdoInfo robotAcceleration = drive.pinpoint().getRawAccel();

        if(drawPoseClipping)
            shootingSystem.drawClippedPoses(fieldOverlay);

        if(drawRobot) {
            fieldOverlay.setStroke("red");
            Drawing.drawRobot(fieldOverlay, robotPose);
        }

        if(drawShooting)
            shootingSystem.drawShootingInfo(fieldOverlay);

        if(drawRobotDerivatives) {
            fieldOverlay.setStroke("red");
            fieldOverlay.strokeLine(0, 0, robotVelocity.x, robotVelocity.y);
            fieldOverlay.setStroke("blue");
            fieldOverlay.strokeLine(0, 0, robotAcceleration.x, robotAcceleration.y);
        }

        if(drawLimelight)
            limelight.addLimelightInfo(fieldOverlay);
    }

    public Action scanForBalls(DoubleSupplier angle1Sup, DoubleSupplier angle2Sup) {
        return new SequentialAction(
                limelight.ballDetection.clearAllScansAction(),
                // first scan
                new SequentialAction(
                        shootingSystem.rotateTurretToCustomAngle(angle1Sup),
                        new SleepAction(LimelightBallDetection.params.waitToScanAfterTurretMove),
                        limelight.ballDetection.takeBallScanAction()
                ),
                // second scan
                angle2Sup == null ?
                        new InstantAction(() -> {}) :
                        new Action() {
                            Action secondScan = null;
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                if (secondScan == null) {
                                    if (limelight.ballDetection.getCombinedBlobsFromMostRecentScan().size() >= 2)
                                        return false;
                                    secondScan = new SequentialAction(
                                            shootingSystem.rotateTurretToCustomAngle(angle2Sup),
                                            new SleepAction(LimelightBallDetection.params.waitToScanAfterTurretMove),
                                            limelight.ballDetection.takeBallScanAction()
                                    );
                                }
                                return secondScan.run(telemetryPacket);
                            }
                        },

                new InstantAction(() -> shootingSystem.setTurretToGoalTargeting())
        );
    }
    public Action lookAtClassifier() {
        return new SequentialAction(
                shootingSystem.rotateTurretToCustomAngle(() -> {
                    Vector2d classifierPosition = new Vector2d(-22, alliance == Alliance.RED ? 72 : -72);
                    Vector2d turretToClassifier = shootingSystem.getTurretPose().position.minus(classifierPosition);
                    return MathUtils.vecAngle(turretToClassifier) - drive.pinpoint().getPose().heading.toDouble();
                }),
                limelight.classifier.readBallsInClassifier(),
                new InstantAction(() -> shootingSystem.setTurretToGoalTargeting())
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
    public Action getLimelightCollectSequence(Vector2d lookPosition1, Vector2d lookPosition2, double maxLimelightWaitTime) {
        return new SequentialAction(
                scanForBalls(
                        () -> MathUtils.vecAngle(lookPosition1.minus(drive.pinpoint().getPose().position)) - drive.pinpoint().getPose().heading.toDouble(),
                        () -> MathUtils.vecAngle(lookPosition2.minus(drive.pinpoint().getPose().position)) - drive.pinpoint().getPose().heading.toDouble()
                ),
                new InstantAction(() -> collector.setIntakeState(Collector.IntakeState.INTAKE)),
                new ParallelAction(
                        new CustomEndAction(generateLimelightCollectDrive(maxLimelightWaitTime),
                                collector::has3Balls),
                        new SequentialAction(
                                new SleepAction(0.2),
                                new InstantAction(() -> shootingSystem.setTurretToCenter())
                        )
                ),
                new InstantAction(() -> shootingSystem.setTurretToGoalTargeting())
        );
    }
    private Action generateLimelightCollectDrive(double maxLimelightWaitTime) {
        return new Action() {
            PathInfo pathInfo = null;
            TimedAction timedDrivePath = null;
            ElapsedTime timer = null;
            Pose2d startPose = null;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (timer == null) {
                    timer = new ElapsedTime();
                    timer.reset();
                }

                if (timedDrivePath == null) {
                    if (timer.seconds() > maxLimelightWaitTime)
                        return false;

                    startPose = drive.localizer.getPose();

                    ArrayList<Blob> ballBlobs = limelight.ballDetection.getBlobsFromBestScan();
                    Vector2d giantClump = limelight.ballDetection.getGiantClumpPosition(ballBlobs);
                    ArrayList<Vector2d> ballPositions = limelight.ballDetection.getBlobPositions(ballBlobs);
                    if(ballPositions.isEmpty())
                       ballPositions.add(new Vector2d(LimelightBallDetection.params.defaultX, (alliance == Alliance.RED ? 1 : -1) * LimelightBallDetection.params.defaultY));

                    PathGeneration.laneCollectParams.alwaysUseLaneCollectNumBalls = PathGeneration.laneCollectParams.defaultAlwaysUseLaneCollectNumBalls;
                    pathInfo = PathGeneration.generateSimplifiedAutoCollectPath(startPose, ballPositions);
                    if (pathInfo == null) {
                        telemetry.addLine("path is null");
                        return false;
                    }

                    if (pathInfo.pathType != PathInfo.PathType.LANE && giantClump != null) {
                        ballPositions = new ArrayList<>(Collections.singletonList(giantClump));
                        PathGeneration.laneCollectParams.alwaysUseLaneCollectNumBalls = 1;
                        pathInfo = PathGeneration.generateSimplifiedAutoCollectPath(startPose, ballPositions);
                    }

                    DrivePath drivePath = new DrivePath(drive);
                    for (PathPose pathPose : pathInfo.optimizedPathPoses) {
                       pathPose.waypoint
                               .setCustomEndCondition(
                                        () -> {
                                            OdoInfo vel = drive.pinpoint().getVelocity();
                                            double maxLinearVel = PathGeneration.driveParams.isStuckMaxLinearVel;
                                            double maxHeadingVel = Math.toRadians(PathGeneration.driveParams.isStuckMaxHeadingVel);
                                            return MathUtils.vecMag(vel.pos()) < maxLinearVel && Math.abs(vel.headingRad) < maxHeadingVel;
                                        },
                                        PathGeneration.driveParams.isStuckConfirmationTime
                                )
                               .setMinTime(PathGeneration.driveParams.startCheckingIsStuckTime);

                        drivePath.addWaypoint(pathPose.waypoint);
                    }

                    if(pathInfo.pathType == PathInfo.PathType.LANE && !drivePath.getWaypoints().isEmpty()) {
                        Waypoint lastCollectWaypoint = drivePath.getWaypoints().get(drivePath.getWaypoints().size()-1);
                        Waypoint backupWaypoint = new Waypoint(new Pose2d(
                                lastCollectWaypoint.x(),
                                lastCollectWaypoint.y() - (alliance == Alliance.RED ? 1 : -1) * PathGeneration.laneCollectParams.tryAgainBackupDist,
                                lastCollectWaypoint.headingRad()))
                                .setMinLinearPower(AutoPid.collect.loadingNormDrivePower)
                                .setPassPosition(true)
                                .setMaxTime(.5);
                        Waypoint tryAgainWaypoint = new Waypoint(new Pose2d(
                                lastCollectWaypoint.x(),
                                lastCollectWaypoint.y(),
                                lastCollectWaypoint.headingRad()))
                                .setMinLinearPower(AutoPid.collect.loadingNormDrivePower)
                                .setPassPosition(true)
                                .setMaxTime(.7);
                        drivePath.addWaypoint(backupWaypoint);
                        drivePath.addWaypoint(tryAgainWaypoint);
                    }

                    double maxTime = pathInfo.pathType == PathInfo.PathType.COMPLEX ? PathGeneration.generalParams.complexCollectMaxTime : PathGeneration.laneCollectParams.laneCollectMaxTime;
                    timedDrivePath = new TimedAction(drivePath, maxTime);

                }
                limelight.ballDetection.drawPath(telemetryPacket.fieldOverlay(), startPose, pathInfo.getOptimizedPoses());
                return timedDrivePath.run(telemetryPacket);
            }
        };
    }

    public Alliance getAlliance() {
        return alliance;
    }
}
