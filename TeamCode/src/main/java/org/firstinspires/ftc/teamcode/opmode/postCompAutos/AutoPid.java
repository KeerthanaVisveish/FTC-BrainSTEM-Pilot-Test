package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createInvertedPose;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

import java.util.ArrayList;

@Config
public abstract class AutoPid extends LinearOpMode {
    public static class Customizable {
        public AutoType autoType = AutoType.CUSTOM;
        public String collectionOrder = "n2ngngn1n3n";
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = false;
        public boolean parkAbort = false;
        public int maxCornerRetries = 0;
    }
    public enum AutoType {
        CUSTOM,
        MOTIF
    }
    public enum AutoState {
        DRIVE_TO_COLLECT,
        OPEN_GATE,
        DRIVE_TO_SHOOT,
        SHOOT
    }
    public static Customizable customizable = new Customizable();
    public static AutoParamsPid.TimeConstraints timeConstraints = new AutoParamsPid.TimeConstraints();
    public static AutoParamsPid.Collect collect = new AutoParamsPid.Collect();
    public static AutoParamsPid.Shoot shoot = new AutoParamsPid.Shoot();
    public static AutoParamsPid.Misc misc = new AutoParamsPid.Misc();

    protected Alliance alliance;
    private ElapsedTime autoTimer;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;
    private Pose2d start,
            collect1NearControlPoint, collect1FarControlPoint, collect1,
            collect2NearControlPoint, collect2FarControlPoint, collect2,
            collect3NearControlPoint, collect3FarControlPoint, preCollect3Near, collect3,
            preLoading, postLoading,
            cornerCollect, cornerCollectRetry,
            gateCollectNearControlPoint, gateCollectFarControlPoint, gateCollectOpen, postGateOpenWaypoint, gateCollect,
            gate1, gate2,
            gateNearShootControlPoint,
            parkFar;
    private Pose2d shoot1Near, shoot1Far, shoot1FarControlPoint,
            shoot2Near, shoot2Far, shoot2FarControlPoint,
            shootGateNear, shootGateFar, shootGateFarControlPoint,
            shoot3Near, shoot3Far,
            shootLoadingNear, shootLoadingFar;
    private boolean isRed;
    private AutoState autoState;
    private Vector2d perpNearParkLine;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        autoTimer = new ElapsedTime();
        isRed = alliance == Alliance.RED;

        perpNearParkLine = isRed ? new Vector2d(1, 1) : new Vector2d(1, -1);
        perpNearParkLine = perpNearParkLine.div(Math.hypot(perpNearParkLine.x, perpNearParkLine.y));

        if(customizable.collectionOrder.charAt(0) == 'n')
            start = isRed ? createPose(misc.startNearRed) : createPose(misc.startNearBlue);
        else if(customizable.collectionOrder.charAt(0) == 'f')
            start = isRed ? createPose(misc.startFar) : createInvertedPose(misc.startFar);

        // DECLARE POSES=======================
        declareShootPoses();
        declareCollectPoses();
        declareMiscPoses();

        Limelight.startingPipeline = Limelight.CLASSIFIER_PIPELINE;
        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        autoCommands = new AutoCommands(robot, telemetry);

        int numPaths = (customizable.collectionOrder.length()-1) / 2; //3
        if(numPaths == 0)
            throw new IllegalArgumentException("cannot have empty collectionOrder string");
        ArrayList<Action> actionOrder = new ArrayList<>();
        customizable.collectionOrder = customizable.collectionOrder.toLowerCase();

        boolean preloadNear = customizable.collectionOrder.charAt(0) == 'n';
        double preloadX = preloadNear ? shoot.nearPreload[0] : shoot.far[0];
        double preloadY = preloadNear ? shoot.nearPreload[1] : shoot.far[1];
        double preloadA = getShootPose(preloadNear, customizable.collectionOrder.charAt(1) + "").heading.toDouble();
        Pose2d preloadShootPose = isRed ? new Pose2d(preloadX, preloadY, preloadA) : new Pose2d(preloadX, -preloadY, preloadA);

        int numGateCollects = 0;

        for(int i=0; i<numPaths; i++) {
            boolean last = i == numPaths - 1;

            String info = customizable.collectionOrder.substring(i*2, i*2+3);
            boolean fromNear = info.charAt(0) == 'n';
            String letter = info.charAt(1) + "";
            boolean toNear = info.charAt(2) == 'n';

            Pose2d shootPose;
            if(last)
                shootPose = getFinalShootPose(toNear, letter);
            else
                shootPose = getShootPose(toNear, letter);

            telemetry.addLine("Path " + (i+1) + ": letter: " + letter + " from near: " + fromNear + " to near: " + toNear);
            telemetry.addData("Path " + (i+1) + " Shoot pose", MathUtils.formatPose3(shootPose));
            switch(letter) {
                case "1" :
                    actionOrder.add(getFirstCollectAndShoot(shootPose, fromNear, toNear, last));
                    break;
                case "2" :
                    actionOrder.add(getSecondCollectAndShoot(shootPose, fromNear, toNear));
                    break;
                case "3" :
                    actionOrder.add(getThirdCollectAndShoot(shootPose, fromNear, toNear, last));
                    break;
                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose, toNear)); break;
                case "c": actionOrder.add(getRepeatedCornerCollectAndShoot(shootPose, fromNear, toNear)); break;
                case "g":
                    numGateCollects++;
                    double waitTime = numGateCollects == 2 ? timeConstraints.secondGaitWait : numGateCollects == 3 ? timeConstraints.thirdGateWait : 0;
                    actionOrder.add(getGateCollectAndShoot(shootPose, fromNear, toNear, waitTime));
                    break;
            }
        }

        Action autoAction = new SequentialAction(
                getPreloadDriveAndShoot(preloadShootPose, customizable.collectionOrder.charAt(0) == 'n'),
                actionOrder.get(0),
                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0),
                numPaths > 4 ? actionOrder.get(4) : new SleepAction(0),
                numPaths > 5 ? actionOrder.get(5) : new SleepAction(0),
                numPaths > 6 ? actionOrder.get(6) : new SleepAction(0),
                numPaths > 7 ? actionOrder.get(7) : new SleepAction(0),
                numPaths > 8 ? actionOrder.get(8) : new SleepAction(0),
                autoCommands.stopIntake(),
                autoCommands.stopShooter()
        );

        Action timedAutoAction = new SequentialAction(
                new CustomEndAction(autoAction, () -> customizable.parkAbort && autoTimer.seconds() > timeConstraints.autoEndTime),
                autoCommands.stopIntake(),
                autoCommands.stopShooter(),
                new InstantAction(() -> robot.drive.stop())
        );

        Action forcedStopAutoAction = new ParallelAction(
                packet -> { telemetry.addData("RRAutoFar STATE", autoState); return true; },
                new TimedAction(timedAutoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot(),
                autoCommands.savePoseContinuously(),
                packet -> {
                    telemetry.update();
                    return true;
                }
        );

        robot.shootingSystem.resetTurretEncoder();
        robot.turret.setSmoothWhenOutOfRange(false);
        robot.collection.setInAuto(true);

        telemetry.addData("2nd collect pose", MathUtils.formatPose3(collect2));
        telemetry.addData("3rd collect pose", MathUtils.formatPose3(collect3));
        telemetry.addData("gate open pos", MathUtils.formatPose3(gateCollectOpen));

        telemetry.addData("alliance", alliance);
        telemetry.addData("auto string", customizable.collectionOrder);
        telemetry.addLine("READY TO RUN");
        telemetry.update();

        waitForStart();
        robot.startOpmode();
        autoTimer.reset();

        Actions.runBlocking(
                forcedStopAutoAction
        );
    }
    private Pose2d getShootPose(boolean shootClose, String letter) {
        switch (letter) {
            case "1": return shootClose ? shoot1Near : shoot1Far;
            case "2": return shootClose ? shoot2Near : shoot2Far;
            case "g": return shootClose ? shootGateNear : shoot2Far;
            case "3": return shootClose ? shoot3Near : shoot3Far;
            case "l": return shootClose ? shootLoadingNear : shootLoadingFar;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
        }
    }
    private Pose2d getFinalShootPose(boolean shootClose, String letter) {
        switch (letter) {
            case "1": return shootClose ?
                    isRed ? createPose(shoot.near1Last) : createInvertedPose(shoot.near1Last) :
                    shoot1Far;
            case "2":
            case "g": return shootClose ? shoot2Near : shoot2Far;
            case "3": return shootClose ?
                    isRed ? createPose(shoot.near3Last) : createInvertedPose(shoot.near3Last) :
                    isRed ? createPose(shoot.far, shoot.farSetup3A) : createInvertedPose(shoot.far, shoot.farSetup3A);
            case "l": case "c":
                return shootClose ?
                        shootLoadingNear : shootLoadingFar;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
        }
    }

    private Action getFarParkDrive() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double dist = Math.hypot(robot.drive.localizer.getPose().position.x - 72, robot.drive.localizer.getPose().position.y);
                if (autoState == AutoState.DRIVE_TO_COLLECT || (autoState == AutoState.DRIVE_TO_SHOOT && dist > 30)) {
                    robot.drive.stop();
                    return false;
                }
                Actions.runBlocking(new DrivePath(robot.drive,
                        new Waypoint(parkFar).setPassPosition(true).setMinLinearPower(0.4).setMaxTime(0.8)
                ));
                return false;
            }
        };
    }
    private Action getPreloadDriveAndShoot(Pose2d shootPose, boolean shootingNear) {
        DrivePath preloadShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setMinLinearPower(shoot.minDrivePower1)
                .setPassPosition(true)
                .setHeadingLerp(shootingNear ? PathParams.HeadingLerpType.TANGENT : PathParams.HeadingLerpType.LINEAR));
        Action preloadShootAction = new ParallelAction(
                preloadShootDrive,
                new SequentialAction(
                        new CustomEndAction(() -> preloadShootDrive.getWaypointDistanceError() < shoot.minPower2Dist),
                        new InstantAction(() -> preloadShootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
                )
        );
        return new SequentialAction(
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                new ParallelAction(
                        autoCommands.flickerHalfUp(),
                        autoCommands.speedUpShooter(),
                        autoCommands.enableTurretTracking(),
                        preloadShootAction
                ),
                getShootAction(0)
        );
    }
    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, DrivePath shootDrive, boolean shootingNear, double postIntakeTime, boolean runIntake, boolean notLast) {
        Action shootDriveAction = new ParallelAction(
                shootDrive,
                new SequentialAction(
                        new CustomEndAction(() -> shootDrive.getWaypointDistanceError() < shoot.minPower2Dist),
                        new InstantAction(() -> shootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
                )
        );
        return new SequentialAction(
                runIntake ? autoCommands.runIntake() : new SleepAction(0),
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
                collectDrive,
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.OPEN_GATE),
                                        gateDrive
                                ),
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                                        new CustomEndAction(shootDriveAction, () -> autoState == AutoState.SHOOT)
                                                .setEndFunction(robot.drive::stop)
                                ),
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.SHOOT),
                                        new InstantAction(() -> robot.drive.stop())
                                )
                        ),
                        new SequentialAction(
                                new SleepAction(postIntakeTime),
                                autoCommands.stopIntake(),

                                shootingNear && notLast ? new SequentialAction(
                                        new CustomEndAction(telemetryPacket -> autoState != AutoState.SHOOT, () -> {
                                            double dot = robot.drive.localizer.getPose().position.dot(perpNearParkLine);
                                            return dot < shoot.earlyEngageClutchDist;
                                        }),
                                        getShootAction(0)
                                ) : new SleepAction(0)
                        )
                ),
                !shootingNear || !notLast ? getShootAction(timeConstraints.lastShootExtraTime) : new SleepAction(0)
        );
    }
    private Action getShootAction(double extraShootTime) {
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> autoState = AutoState.SHOOT),
                        autoCommands.stopIntake(),
                        autoCommands.engageClutch(),
                        autoCommands.runIntake(),
                        new CustomEndAction(new SleepAction(timeConstraints.maxShootTime), () -> robot.shooter.ballsDoneExiting())
                ),
                new SleepAction(extraShootTime),
                new ParallelAction(
                        autoCommands.disengageClutch(),
                        autoCommands.stopIntake(),
                        autoCommands.flickerUp(),
                        new SleepAction(.1)
                )
        );
    }
    private Action getFirstCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, boolean last) {
        DrivePath firstCollectDrive;
        if(fromNear) {
            firstCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect1, new BoxTolerance(0.75, 2, Math.toRadians(3)))
                    .setFixedLinearPower(collect.collectDrivePower)
                    .setMaxTime(1.4)
                    .setControlPoint(collect1NearControlPoint, collect.firstNearT1, collect.firstNearT2)
                    .setCustomEndCondition(() -> robot.collection.has3Balls())
                    .setPassPosition(true)
                    .setCorrectiveStrength(collect.firstCorrectiveStrength));
        }
        else {
            firstCollectDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(collect1)
                            .setControlPoint(collect1FarControlPoint, collect.firstFarT1, collect.firstFarT2)
                            .setPassPosition(true)
                            .setFixedLinearPower(collect.collectDrivePower)
                            .setMaxTime(2)
                            .setCustomEndCondition(() -> robot.collection.has3Balls()));
        }
        double controlY = fromNear ? collect1NearControlPoint.position.y : collect1FarControlPoint.position.y;
        Action firstCollectAction = new ParallelAction(
                firstCollectDrive,
                new SequentialAction(
                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(controlY)),
                        new InstantAction(() -> firstCollectDrive.getCurWaypoint()
                                .setFixedLinearPower(collect.firstCollectDrivePower)
                        )
                )
        );

        double sign = isRed ? -1 : 1;
        Pose2d preGate1Pose = new Pose2d(gate1.position.x, gate1.position.y + sign * misc.gateBackupDist, gate1.heading.toDouble());

        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGate1Pose).setMinLinearPower(misc.gatePrepMinPower).setPassPosition(true),
                                new Waypoint(gate1).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);
        DrivePath firstShootDrive = toNear ?
                new DrivePath(robot.drive, new Waypoint(shootPose)
                        .setMaxTime(3)
                        .setPassPosition(true)
                        .setMinLinearPower(shoot.minDrivePower1))
                :
                new DrivePath(robot.drive, telemetry,
                        new Waypoint(shoot1FarControlPoint, shoot.waypointTol)
                                .setPassPosition(true)
                                .setSlowDownPercent(shoot.waypointSlowDown)
                                .setMaxTime(10),
                        new Waypoint(shootPose)
                                .setMaxTime(10).setPassPosition(true));

        return buildCollectAndShoot(firstCollectAction, firstGateDrive, firstShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
        DrivePath secondCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect2)
                .setPassPosition(true)
                .setMinLinearPower(collect.collectDrivePower)
                .setControlPoint(fromNear ? collect2NearControlPoint : collect2FarControlPoint, collect.secondNearT1, collect.secondNearT2));
        double controlY = fromNear ? collect2NearControlPoint.position.y : collect2FarControlPoint.position.y;
        Action secondCollectAction = new ParallelAction(
                secondCollectDrive,
                new SequentialAction(
                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(controlY)),
                        new InstantAction(() -> secondCollectDrive.getCurWaypoint()
                                .setFixedLinearPower(collect.secondCollectDrivePower)
                        )
                )
        );
        double sign = isRed ? -1 : 1;
        Pose2d preGatePose = new Pose2d(gate2.position.x, gate2.position.y + sign * misc.gateBackupDist, gate2.heading.toDouble());

        Action secondGateDrive = customizable.openGateOnSecond ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGatePose).setMinLinearPower(misc.gatePrepMinPower).setPassPosition(true),
                                new Waypoint(gate2).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);

        DrivePath secondShootDrive;
        if(toNear) {
            Waypoint w = new Waypoint(shootPose)
                    .setMaxTime(3)
                    .setPassPosition(true)
                    .setMinLinearPower(shoot.minDrivePower1)
                    .setControlPoint(gateNearShootControlPoint, shoot.gateNearShootTStartError, shoot.gateNearShootTFinishError);
            secondShootDrive = new DrivePath(robot.drive, telemetry, w);
        }
        else
            secondShootDrive = new DrivePath(robot.drive,
                    new Waypoint(shoot2FarControlPoint)
                            .setMinLinearPower(shoot.minDrivePower1)
                            .setMaxTime(1.2)
                            .setSlowDownPercent(0)
                            .setPassPosition(true),
                    new Waypoint(shootPose).setMaxTime(1.5).setPassPosition(true));

        return buildCollectAndShoot(secondCollectAction, secondGateDrive, secondShootDrive, toNear, timeConstraints.postIntakeTime, true, true);
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, boolean last) {
        Waypoint w = new Waypoint(collect3)
                .setPassPosition(true)
                .setMinLinearPower(collect.collectDrivePower);
        if(fromNear)
            w.setControlPoint(collect3NearControlPoint, collect.thirdNearT1, collect.thirdNearT2);
        else
            w.setControlPoint(collect3FarControlPoint, collect.thirdFarT1, collect.thirdFarT2);
        DrivePath thirdCollectDrive = new DrivePath(robot.drive, telemetry, w);

        if (fromNear) {
            BoxTolerance preCollect3NearTol = new BoxTolerance(collect.thirdNearWaypointXTol, collect.thirdNearWaypointYTol, Math.toRadians(collect.thirdNearWaypointHeadingTol));
            Waypoint preCollectNear = new Waypoint(preCollect3Near,
                    preCollect3NearTol)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.collectDrivePower);
            thirdCollectDrive.addWaypoint(preCollectNear, 0);
        }
        double controlY = fromNear ? collect3NearControlPoint.position.y : collect3FarControlPoint.position.y;
        Action thirdCollectAction = new ParallelAction(
                thirdCollectDrive,
                new SequentialAction(
                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(controlY)),
                        new InstantAction(() -> thirdCollectDrive.getCurWaypoint()
                                .setFixedLinearPower(collect.thirdCollectDrivePower)
                                .setCloseHeadingKP(collect.thirdCloseHeadingKp)
                        )
                )
        );

        Waypoint thirdShootDest = new Waypoint(shootPose)
                .setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT)
                .setPassPosition(true)
                .setMinLinearPower(shoot.minDrivePower1);
        thirdShootDest.setMaxTime(toNear ? 3 : 2);

        DrivePath thirdShootDrive = new DrivePath(robot.drive, telemetry, thirdShootDest);

        return buildCollectAndShoot(thirdCollectAction, new SleepAction(0), thirdShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, boolean toNear) {
        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoading, new BoxTolerance(3.5, 2, Math.toRadians(4)))
                        .setMaxTime(2).setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                new Waypoint(postLoading)
                        .setMinLinearPower(collect.loadingZoneCollectDrivePower).setMaxLinearPower(collect.loadingZoneCollectDrivePower)
                        .setMaxTime(1.5)
                        .setLateralAxialWeights(2.2, 1)
                        .setCustomEndCondition(() -> robot.collection.has3Balls()));
        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true)
                .setMinLinearPower(shoot.minDrivePower1));

        return buildCollectAndShoot(loadingCollectDrive, new SleepAction(0), loadingShootDrive, toNear, timeConstraints.loadingSlowIntakeTime, true, false);
    }

    private Action getRepeatedCornerCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
        DrivePath gateShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(4));

        return buildCollectAndShoot(repeatedCornerCollect(fromNear), new SleepAction(0), gateShootDrive, toNear, timeConstraints.loadingSlowIntakeTime, true, false);
    }
    private Action repeatedCornerCollect(boolean fromNear) {
        return new Action() {
            private Action gateCollectDrive, gateResetDrive;
            private boolean first = true;
            private boolean ranCollectDrive = false;
            private int numTimesCollected = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    resetCollectDrive();
                }
                if (!ranCollectDrive) {
                    if (!gateCollectDrive.run(telemetryPacket)) {
                        ranCollectDrive = true;
                        numTimesCollected++;
                        if (robot.collection.has3Balls() || numTimesCollected > customizable.maxCornerRetries)
                            return false;
                        resetRetryDrive();
                        resetCollectDrive();
                    }
                }
                if (ranCollectDrive) {
                    ranCollectDrive = gateResetDrive.run(telemetryPacket);
                }
                return true;
            }
            private void resetCollectDrive() {
                if (numTimesCollected == 0) {
                    double sign = isRed ? 1 : -1;
                    double collectTangent = fromNear ? sign * Math.toRadians(30) : sign * Math.toRadians(80);
                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                            .setTangent(collectTangent)
                            .splineToSplineHeading(cornerCollect, collectTangent)
                            .build(),
                            () -> robot.collection.has3Balls(), timeConstraints.cornerCollectMaxTime);
                }
                else {
                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(collect.cornerCollectRetryX, cornerCollect.position.y), cornerCollect.heading.toDouble())
                            .build(),
                            () -> robot.collection.has3Balls(), timeConstraints.cornerCollectMaxTime);
                }
            }
            private void resetRetryDrive() {
                gateResetDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                        .strafeToLinearHeading(cornerCollectRetry.position, cornerCollectRetry.heading.toDouble())
                        .build(),
                        () -> robot.collection.has3Balls() || Math.abs(robot.drive.localizer.getPose().position.y) < (cornerCollectRetry.position.y) + 1);
            }
        };
    }

    private Action getGateCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double waitTime) {
        DrivePath gateOpenDrive;
        double maxTime;
        if (fromNear) {
            maxTime = 1.9;
            gateOpenDrive = new DrivePath(robot.drive, telemetry, new Waypoint(gateCollectOpen)
                    .setMaxTime(maxTime)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.gateOpenDrive1Power)
                    .setControlPoint(gateCollectNearControlPoint, collect.gateNearT1, collect.gateNearT2)
                    .setHeadingKPMult(collect.gateOpenKPMult)
            );
        }
        else {
            maxTime = 2.4;
            gateOpenDrive = new DrivePath(robot.drive,
                    new Waypoint(gateCollectOpen)
                            .setMaxTime(maxTime)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1Power)
                            .setControlPoint(gateCollectFarControlPoint, collect.gateCollectOpenFarTStartError, collect.gateCollectOpenFarTFinishError)
            );
        }

        CustomEndAction gateOpenCustomEndAction = new CustomEndAction(gateOpenDrive,
                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold && Math.abs(robot.drive.localizer.getPose().position.y) > 45);
        Action gateOpenAction = new ParallelAction(
                gateOpenCustomEndAction,
                new SequentialAction(
                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > 45),
                        new InstantAction(() -> gateOpenDrive.getCurWaypoint().setMinLinearPower(collect.gateOpenDrive2Power))
                )
        );

        DrivePath postGateOpenDrive = new DrivePath(robot.drive,
                new Waypoint(gateCollect)
                        .setMinLinearPower(misc.gateMinPower)
                        .setPassPosition(true)
                        .setMaxTime(1));
        CustomEndAction postGateOpenAction = new CustomEndAction(postGateOpenDrive,
                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.gateCollectHitWallThreshold
                        && Math.abs(robot.drive.localizer.getPose().position.y) > 60);
        postGateOpenAction.setEndFunction(robot.drive::stop);

        double yPower = misc.gateMinPower * (isRed ? -1 : 1);
        Action completeGateCollectDrive = new SequentialAction(
                autoCommands.stopIntake(),
                new SleepAction(waitTime),
                gateOpenAction,
                new SleepAction(timeConstraints.gateCollectOpenWait),
                autoCommands.runIntake(),
                new TimedAction(telemetryPacket -> {robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, yPower), 0)); return true;}, .2),
                postGateOpenAction,
                new CustomEndAction(telemetryPacket -> { robot.drive.stop(); return true; }, () -> robot.collection.has3Balls(), timeConstraints.gateCollectMaxTime)
        );

        DrivePath gateShootDrive;
        if (toNear)
            gateShootDrive = new DrivePath(robot.drive,
                new Waypoint(shootPose)
                        .setMaxTime(2)
                        .setPassPosition(true)
                        .setControlPoint(gateNearShootControlPoint, shoot.gateNearShootTStartError, shoot.gateNearShootTFinishError)
                        .setMinLinearPower(shoot.minDrivePower1));
        else
            gateShootDrive = new DrivePath(robot.drive,
                    new Waypoint(shootPose)
                            .setPassPosition(true)
                            .setMinLinearPower(shoot.minDrivePower1)
                            .setControlPoint(shootGateFarControlPoint, shoot.gateFarTStartError, shoot.gateFarTFinishError)
            );

        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, timeConstraints.postIntakeTime, false, true);

    }
    private void declareShootPoses() {
        shoot1Near = isRed ? createPose(shoot.near1) : createInvertedPose(shoot.near1);
        shoot2Near = isRed ? createPose(shoot.near2) : createInvertedPose(shoot.near2);
        shoot2Far = isRed ? createPose(shoot.far, shoot.farSetup2A) : createInvertedPose(shoot.far, shoot.farSetup2A);
        shootGateNear = shoot2Near;
        shootGateFar = shoot2Far;
        shoot3Near = isRed ? createPose(shoot.near3) : createInvertedPose(shoot.near3);
        shoot3Far = isRed ? createPose(shoot.far, shoot.farSetup3A) : createInvertedPose(shoot.far, shoot.farSetup3A);

        shootLoadingNear = shoot3Near;
        shootLoadingFar = isRed ? createPose(shoot.far, shoot.farSetupLoadingA) : createInvertedPose(shoot.far, shoot.farSetupLoadingA);

        gateNearShootControlPoint = isRed ? createPose(shoot.gateNearControlPoint) : createInvertedPose(shoot.gateNearControlPoint);
        shoot1FarControlPoint = isRed ? createPose(shoot.far1ControlPoint) : createInvertedPose(shoot.far1ControlPoint);
        shoot2FarControlPoint = isRed ? createPose(shoot.far2ControlPoint) : createInvertedPose(shoot.far2ControlPoint);
    }
    private void declareCollectPoses() {
        collect1 = isRed ?
                createPose(collect.first) :
                createInvertedPose(collect.first);
        collect1NearControlPoint = isRed ?
                createPose(collect.firstControlPointNear) :
                createInvertedPose(collect.firstControlPointNear);
        collect1FarControlPoint = isRed ?
                createPose(collect.firstControlPointFar) :
                createInvertedPose(collect.firstControlPointFar);

        double[] second = customizable.openGateOnSecond ? collect.secondIfOpenGate : collect.second;
        collect2 = isRed ?
                createPose(second) :
                createInvertedPose(second);
        collect2NearControlPoint = isRed ?
                createPose(collect.secondNearControlPointRed) :
                createPose(collect.secondNearControlPointBlue);
        collect2FarControlPoint = isRed ?
                createPose(collect.secondFarControlPoint) :
                createInvertedPose(collect.secondFarControlPoint);

        collect3 = isRed ?
                createPose(collect.third) :
                createInvertedPose(collect.third);
        preCollect3Near = isRed ?
                createPose(collect.thirdNearWaypoint) :
                createInvertedPose(collect.thirdNearWaypoint);
        collect3NearControlPoint = isRed ?
                createPose(collect.thirdNearControlPoint) :
                createInvertedPose(collect.thirdNearControlPoint);
        collect3FarControlPoint = isRed ?
                createPose(collect.thirdFarControlPoint) :
                createInvertedPose(collect.thirdFarControlPoint);

        preLoading = isRed ?
                new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) :
                new Pose2d(collect.preLoadingXBlue, collect.preLoadingYBlue, collect.preLoadingABlue);
        postLoading = isRed ?
                new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) :
                new Pose2d(collect.postLoadingXBlue, collect.postLoadingYBlue, collect.postLoadingABlue);
        cornerCollect = isRed ?
                new Pose2d(collect.cornerCollectXRed, collect.cornerCollectYRed, collect.cornerCollectARed) :
                new Pose2d(collect.cornerCollectXBlue, collect.cornerCollectYBlue, collect.cornerCollectABlue);
        cornerCollectRetry = isRed ?
                new Pose2d(collect.cornerCollectXRed, collect.cornerCollectRetryYRed, collect.cornerCollectARed) :
                new Pose2d(collect.cornerCollectXBlue, collect.cornerCollectRetryYBlue, collect.cornerCollectABlue);

        gateCollectNearControlPoint = isRed ?
                createPose(collect.gateNearControlPoint) :
                createInvertedPose(collect.gateNearControlPoint);
        gateCollectFarControlPoint = isRed ?
                createPose(collect.gateFarControlPoint) :
                createInvertedPose(collect.gateFarControlPoint);
        gateCollectOpen = isRed ?
                createPose(collect.gateOpen) :
                createInvertedPose(collect.gateOpen);
        postGateOpenWaypoint = isRed ?
                createPose(collect.postGateOpenWaypoint) :
                createInvertedPose(collect.postGateOpenWaypoint);
        gateCollect = isRed ?
                createPose(collect.gateCollect) :
                createInvertedPose(collect.gateCollect);
    }
    private void declareMiscPoses() {
        gate1 = isRed ?
                createPose(misc.gate1) :
                createInvertedPose(misc.gate1);
        gate2 = isRed ?
                createPose(misc.gate2) :
                createInvertedPose(misc.gate2);
    }
}
