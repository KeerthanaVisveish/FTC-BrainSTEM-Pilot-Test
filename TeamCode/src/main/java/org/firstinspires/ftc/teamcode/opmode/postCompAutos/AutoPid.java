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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
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
        // if partner has 3 ball close: f2fgn1n3n
        // if partner has 6 ball close: f2fgf3flf
        public String collectionOrder = "n2ngngn1n3n";
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = false;
        public boolean abortAtEnd = false;
        public int maxCornerRetries = 0;
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
            collect1Pose, collect1ControlPointNear, collect1ControlPointFar,
            preCollect2PoseNear, preCollect2PoseFar, collect2Pose,
            preCollect3NearPose, preCollect3FarPose, collect3Pose,
            preLoadingPose, postLoadingPose,
            cornerCollectPose, cornerCollectRetryPose,
            gateCollectOpenControlPoint, gateCollectOpenNearPose, gateCollectOpenFarPose, gateCollectPose,
            gate1Pose, gate2Pose,
            gateNearShootControlPoint, gateFarWaypointPose,
            parkFarPose,
            shootFar1WaypointPose, shootFar2WaypointPose,
            shootNearPreloadPose, shootFarPreloadPose,
            shootNear1Pose, shootFar1Pose,
            shootNear2Pose, shootFar2Pose,
            shootNearGatePose,
            shootNear3Pose, shootFar3Pose,
            shootNearLoadingPose, shootFarSetupLoadingPose;
    private boolean isRed;
    private AutoState autoState;
    private boolean autoActionFinished;
    private Vector2d perpNearParkLine;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        autoActionFinished = false;

        autoTimer = new ElapsedTime();
        isRed = alliance == Alliance.RED;

        perpNearParkLine = isRed ? new Vector2d(1, 1) : new Vector2d(-1, 1);
        perpNearParkLine = perpNearParkLine.div(Math.hypot(perpNearParkLine.x, perpNearParkLine.y));

        if(customizable.collectionOrder.charAt(0) == 'n')
            start = isRed ? createPose(misc.startNear) : createInvertedPose(misc.startNear);
        else if(customizable.collectionOrder.charAt(0) == 'f')
            start = isRed ? new Pose2d(misc.startFarXRed, misc.startFarYRed, misc.startFarARed) : new Pose2d(misc.startFarXBlue, misc.startFarYBlue, misc.startFarABlue);

        // DECLARE POSES=======================
        declareShootPoses();
        declareCollectPoses();
        declareMiscPoses();

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        autoCommands = new AutoCommands(robot, telemetry);

        int numPaths = (customizable.collectionOrder.length()-1) / 2; //3
        if(numPaths == 0)
            throw new IllegalArgumentException("cannot have empty collectionOrder string");
        ArrayList<Action> actionOrder = new ArrayList<>();
        customizable.collectionOrder = customizable.collectionOrder.toLowerCase();

        Pose2d preloadShootPose = getPreloadSetupPose(customizable.collectionOrder.charAt(0));
        for(int i=0; i<numPaths; i++) {
            boolean notLast = i < numPaths - 1;
            Pose2d shootPose;
            if(notLast)
                shootPose = getShootPose(customizable.collectionOrder.substring(i*2, i*2 + 2));
            else
                shootPose = getFinalShootPose(customizable.collectionOrder.charAt(i*2+2) + "" + customizable.collectionOrder.charAt(i * 2 + 1));

            String curLetter = customizable.collectionOrder.charAt(i*2+1) + "";
            boolean fromNear = customizable.collectionOrder.charAt(i*2) == 'n';
            boolean toNear = customizable.collectionOrder.charAt(i*2+2) == 'n';
            telemetry.addLine("Path " + (i+1) + ": letter: " + curLetter + " from near: " + fromNear + " to near: " + toNear);
            telemetry.addData("Path " + (i+1) + " Shoot pose", MathUtils.formatPose3(shootPose));
            switch(curLetter) {
                case "1" :
                    actionOrder.add(getFirstCollectAndShoot(shootPose, fromNear, toNear, !notLast));
                    break;
                case "2" :
                    actionOrder.add(getSecondCollectAndShoot(shootPose, fromNear, toNear));
                    break;
                case "3" :
                    actionOrder.add(getThirdCollectAndShoot(shootPose, fromNear, toNear, !notLast));
                    break;
                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose, toNear)); break;
                case "c": actionOrder.add(getRepeatedCornerCollectAndShoot(shootPose, fromNear, toNear)); break;
                case "g": actionOrder.add(getGateCollectAndShoot(shootPose, fromNear, toNear)); break;
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
                autoCommands.stopShooter(),
                new InstantAction(() -> autoActionFinished = true)
        );

        Action timedAutoAction = new SequentialAction(
                new CustomEndAction(autoAction,
                        () -> ((customizable.abortAtEnd && autoTimer.seconds() > timeConstraints.autoEndTime) || autoActionFinished)),
                autoCommands.stopIntake(),
                autoCommands.stopShooter(),
                new InstantAction(() -> robot.drive.stop())
        );

        Action forcedStopAutoAction = new ParallelAction(
                packet -> { telemetry.addData("RRAutoFar STATE", autoState); return true; },
                new TimedAction(timedAutoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot,
                autoCommands.savePoseContinuously,
                packet -> {
//                    telemetry.addData("TIMER", autoTimer.seconds());
//                    telemetry.addData("current", robot.collection.collectorMotor.getCurrent(CurrentUnit.MILLIAMPS));
//                    telemetry.addData("balls shot", robot.shooter.getBallsShot());
//                    telemetry.addData("intake p", robot.collection.getIntakePower());
//                    telemetry.addData("shooter error", robot.shooter.shooterPID.getTarget() - robot.shooter.getAvgMotorVelocity());
//                    telemetry.addData("autoX, y, heading", PoseStorage.autoX + ", " + PoseStorage.autoY + ", " + Math.floor(PoseStorage.autoHeading * 180 / Math.PI));
                    telemetry.update();
                    return true;
                }
        );
        robot.shootingSystem.resetTurretEncoder();
        robot.turret.setSmoothWhenOutOfRange(false);
        robot.collection.setInAuto(true);
        telemetry.addData("2nd collect pose", MathUtils.formatPose3(collect2Pose));
        telemetry.addData("3rd collect pose", MathUtils.formatPose3(collect3Pose));
        telemetry.addData("gate open pos", MathUtils.formatPose3(gateCollectOpenNearPose));

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
        robot.drive.stop();
    }
    private Pose2d getPreloadSetupPose(char start) {
        boolean shootClose = start == 'n';
        if(shootClose)
            return shootNearPreloadPose;
        else
            return shootFarPreloadPose;
    }
    private Pose2d getShootPose(String info) {
        boolean shootClose = info.charAt(0) == 'n';
        String letter = info.charAt(1) + "";
        switch (letter) {
            case "1": return shootClose ? shootNear1Pose : shootFar1Pose;
            case "2": return shootClose ? shootNear2Pose : shootFar2Pose;
            case "g": return shootClose ? shootNearGatePose : shootFar2Pose;
            case "3": return shootClose ? shootNear3Pose : shootFar3Pose;
            case "l": return shootClose ? shootNearLoadingPose : shootFarSetupLoadingPose;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
        }
    }
    private Pose2d getFinalShootPose(String info) {
        boolean shootClose = info.charAt(0) == 'n';
        String letter = info.charAt(1) + "";
        switch (letter) {
            case "1": return shootClose ?
                    isRed ? createPose(shoot.firstLast) : createInvertedPose(shoot.firstLast) :
                    shootFar1Pose;
            case "2":
            case "g": return shootClose ? shootNear2Pose : shootFar2Pose;
            case "3": return shootClose ?
                    isRed ? createPose(shoot.thirdLast) : createInvertedPose(shoot.thirdLast) :
                    isRed ? new Pose2d(shoot.shootFarXRed, shoot.shootFarYRed, Math.toRadians(120)) : new Pose2d(shoot.shootFarXBlue, shoot.shootFarYBlue, Math.toRadians(-120));
            case "l": case "c": return shootClose ? shootNearLoadingPose : shootFarSetupLoadingPose;
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
                        new Waypoint(parkFarPose).setPassPosition(true).setMinLinearPower(0.4).setMaxTime(0.8)
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
                .setHeadingLerp(shoot.preloadHeadingLerp));
        Action preloadShootAction = new ParallelAction(
                preloadShootDrive,
                new SequentialAction(
                        new CustomEndAction(new SleepAction(3), () -> preloadShootDrive.getWaypointDistanceError() < shoot.minPower2Dist),
                        new InstantAction(() -> preloadShootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
                )
        );
        return new SequentialAction(
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                new ParallelAction(
                        autoCommands.flickerHalfUp(),
                        autoCommands.speedUpShooter(),
                        autoCommands.enableTurretTracking(),
                        preloadShootAction,
                        new SequentialAction(
                                autoCommands.engageClutch(),
                                autoCommands.reverseIntake(),
                                new SleepAction(Collection.shootOuttakeTimeAuto),
                                autoCommands.stopIntake(),
                                new SleepAction(Collection.postShootOuttakeWaitAuto)
                        )
                ),
                getShootAction()
        );
    }
    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, DrivePath shootDrive, boolean shootingNear, double postIntakeTime, boolean runIntake, boolean engageClutchEarly) {
        Action shootDriveAction = new ParallelAction(
                shootDrive,
                new SequentialAction(
                        new CustomEndAction(new SleepAction(100), () -> shootDrive.getWaypointDistanceError() < shoot.minPower2Dist),
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
                                ),
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.SHOOT),
                                        new InstantAction(() -> robot.drive.stop())
                                )
                        ),
                        new SequentialAction(
                                new SleepAction(postIntakeTime),
                                autoCommands.stopIntake(),

                                shootingNear && engageClutchEarly ? new SequentialAction(
                                        new CustomEndAction(telemetryPacket -> autoState != AutoState.SHOOT, () -> {
                                            double dot = robot.drive.localizer.getPose().position.dot(perpNearParkLine);
                                            return dot < shoot.earlyEngageClutchDist;
                                        }),
                                        getShootAction()
                                ) : new SleepAction(0)
                        )
                ),
                !shootingNear || !engageClutchEarly ? getShootAction() : new SleepAction(0)
        );
    }
    private Action getShootAction() {
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> autoState = AutoState.SHOOT),
                        autoCommands.reverseIntake(),
                        new SleepAction(Collection.shootOuttakeTimeAuto)
                ),
                new ParallelAction(
                        autoCommands.stopIntake(),
                        autoCommands.engageClutch(),
                        autoCommands.runIntake(),
                        new CustomEndAction(new SleepAction(timeConstraints.maxShootTime), () -> robot.shooter.ballsDoneExiting())
                ),
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
            firstCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect1Pose, new BoxTolerance(0.75, 2, Math.toRadians(3)))
                    .setMinLinearPower(collect.collectDrivePower)
                    .setMaxLinearPower(collect.collectDrivePower)
                    .setMaxTime(1.4)
                    .setControlPoint(collect1ControlPointNear, collect.firstCollectTStartError, collect.firstCollectTFinishError)
                    .setCustomEndCondition(() -> robot.collection.has3Balls())
                    .setPassPosition(true));
        }
        else {
            firstCollectDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(collect1ControlPointFar, collect.waypointTol)
                            .setPassPosition(true)
                            .setSlowDownPercent(collect.waypointSlowDown)
                            .setMaxTime(2)
                            .setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                    new Waypoint(collect1Pose)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.collectDrivePower)
                            .setMaxLinearPower(collect.collectDrivePower)
                            .setMaxTime(2)
                            .setCustomEndCondition(() -> robot.collection.has3Balls()));
        }

        double sign = isRed ? -1 : 1;
        Pose2d preGate1Pose = new Pose2d(gate1Pose.position.x, gate1Pose.position.y + sign * misc.gateBackupDist, gate1Pose.heading.toDouble());

        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGate1Pose).setMinLinearPower(misc.gatePrepMinPower).setPassPosition(true),
                                new Waypoint(gate1Pose).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
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
                        new Waypoint(shootFar1WaypointPose, shoot.waypointTol)
                                .setPassPosition(true)
                                .setSlowDownPercent(shoot.waypointSlowDown)
                                .setMaxTime(10),
                        new Waypoint(shootPose)
                                .setMaxTime(10).setPassPosition(true));

        return buildCollectAndShoot(firstCollectDrive, firstGateDrive, firstShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
        Action secondCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect2Pose)
                .setPassPosition(true)
                .setMinLinearPower(collect.collectDrivePower)
                .setControlPoint(fromNear ? preCollect2PoseNear : preCollect2PoseFar, collect.secondTValueStartError, collect.secondTValueFinishError));

        double sign = isRed ? -1 : 1;
        Pose2d preGatePose = new Pose2d(gate2Pose.position.x, gate2Pose.position.y + sign * misc.gateBackupDist, gate2Pose.heading.toDouble());

        Action secondGateDrive = customizable.openGateOnSecond ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGatePose).setMinLinearPower(misc.gatePrepMinPower).setPassPosition(true),
                                new Waypoint(gate2Pose).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);

        DrivePath secondShootDrive;
        if(toNear) {
            Waypoint w = new Waypoint(shootPose)
                    .setMaxTime(3)
                    .setPassPosition(true)
                    .setMinLinearPower(shoot.minDrivePower1);
            if(customizable.openGateOnSecond)
                w.setControlPoint(gateNearShootControlPoint, shoot.gateShootTStartError, shoot.gateShootTFinishError);
            secondShootDrive = new DrivePath(robot.drive, telemetry, w);
        }
        else
            secondShootDrive = new DrivePath(robot.drive,
                    new Waypoint(shootFar2WaypointPose)
                            .setMinLinearPower(shoot.minDrivePower1)
                            .setMaxTime(1.2)
                            .setSlowDownPercent(0)
                            .setPassPosition(true),
                    new Waypoint(shootPose).setMaxTime(1.5).setPassPosition(true));

        return buildCollectAndShoot(secondCollectDrive, secondGateDrive, secondShootDrive, toNear, timeConstraints.postIntakeTime, true, true);
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, boolean last) {
        Action thirdCollectDrive = new DrivePath(robot.drive, telemetry,
                new Waypoint(collect3Pose)
                        .setPassPosition(true)
                        .setMinLinearPower(collect.collectDrivePower)
                        .setControlPoint(fromNear ? preCollect3NearPose : preCollect3FarPose, collect.thirdTValueStartError, collect.thirdTValueFinishError));

        Waypoint thirdShootDest = toNear ?
                new Waypoint(shootPose).setMaxTime(3).setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT) :
                new Waypoint(shootPose).setMaxTime(2).setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT);
        DrivePath thirdShootDrive = new DrivePath(robot.drive, telemetry, thirdShootDest
                .setPassPosition(true)
                .setMinLinearPower(shoot.minDrivePower1));

        return buildCollectAndShoot(thirdCollectDrive, new SleepAction(0), thirdShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, boolean toNear) {
        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoadingPose, new BoxTolerance(3.5, 2, Math.toRadians(4)))
                        .setMaxTime(2).setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                new Waypoint(postLoadingPose)
                        .setMinLinearPower(collect.loadingZoneCollectDrivePower).setMaxLinearPower(collect.loadingZoneCollectDrivePower)
                        .setMaxTime(1.5)
                        .setLateralAxialWeights(2.2, 1)
                        .setCustomEndCondition(() -> robot.collection.has3Balls()));
        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3).setPassPosition(true));

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
                            .splineToSplineHeading(cornerCollectPose, collectTangent)
                            .build(),
                            () -> robot.collection.has3Balls(), timeConstraints.cornerCollectMaxTime);
                }
                else {
                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(collect.cornerCollectRetryX, cornerCollectPose.position.y), cornerCollectPose.heading.toDouble())
                            .build(),
                            () -> robot.collection.has3Balls(), timeConstraints.cornerCollectMaxTime);
                }
            }
            private void resetRetryDrive() {
                gateResetDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                        .strafeToLinearHeading(cornerCollectRetryPose.position, cornerCollectRetryPose.heading.toDouble())
                        .build(),
                        () -> robot.collection.has3Balls() || Math.abs(robot.drive.localizer.getPose().position.y) < (cornerCollectRetryPose.position.y) + 1);
            }
        };
    }

    private Action getGateCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
        DrivePath gateOpenDrive;
        Action gateOpenAction;
        double maxTime;
        if (fromNear) {
            maxTime = 1.9;
            gateOpenDrive = new DrivePath(robot.drive, telemetry, new Waypoint(gateCollectOpenNearPose, collect.gateCollectOpenTol)
                    .setMaxTime(maxTime)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.gateOpenDrivePower)
                    .setControlPoint(gateCollectOpenControlPoint, collect.gateCollectOpenTStartError, collect.gateCollectOpenTFinishError)
            );
        }
        else {
            maxTime = 2.4;
            gateOpenDrive = new DrivePath(robot.drive,
                    new Waypoint(gateFarWaypointPose, collect.waypointTol)
                            .setMaxTime(1.5)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateCollectMinPower)
                            .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                            .setSlowDownPercent(0),
                    new Waypoint(gateCollectOpenFarPose, collect.gateCollectOpenTol)
                            .setMaxTime(0.9)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateCollectMinPower)
            );
        }
        gateOpenAction = new ParallelAction(
                gateOpenDrive,
                new SequentialAction(
                        new CustomEndAction(new SleepAction(maxTime),
                                () -> gateOpenDrive.getWaypointDistanceError() < collect.gateHitActivateDist),
                        new InstantAction(() -> gateOpenDrive.getCurWaypoint().setMinLinearPower(collect.gateHitDrivePower))
                )
        );
        gateOpenAction = new CustomEndAction(gateOpenAction,
                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold && Math.abs(robot.drive.localizer.getPose().position.y) > 45);

        Action completeGateCollectDrive = new SequentialAction(
                autoCommands.stopIntake(),
                gateOpenAction,
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.gateCollectOpenWait),
                new DrivePath(robot.drive,
                        new Waypoint(gateCollectPose).setMaxTime(0.5).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
                new CustomEndAction(new SleepAction(timeConstraints.gateCollectMaxTime), () -> robot.collection.has3Balls())
        );

        DrivePath gateShootDrive;
        if (toNear)
            gateShootDrive = new DrivePath(robot.drive,
                new Waypoint(shootPose)
                        .setMaxTime(2)
                        .setPassPosition(true)
                        .setControlPoint(gateNearShootControlPoint, shoot.gateShootTStartError, shoot.gateShootTFinishError)
                        .setMinLinearPower(shoot.minDrivePower1));
        else
            gateShootDrive = new DrivePath(robot.drive,
                    new Waypoint(gateFarWaypointPose, collect.waypointTol)
                            .setMaxTime(1.2).setPassPosition(true)
                            .setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT)
                            .setSlowDownPercent(0)
                            .setMinLinearPower(shoot.minDrivePower1),
                    new Waypoint(shootPose).setMaxTime(2)
                            .setPassPosition(true)
                            .setMinLinearPower(shoot.minDrivePower1)
            );

        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, timeConstraints.postIntakeTime, false, true);

    }
    private void declareShootPoses() {
        shootNearPreloadPose = isRed ? createPose(shoot.preload) : createInvertedPose(shoot.preload);
        shootNear1Pose = isRed ? createPose(shoot.near1) : createInvertedPose(shoot.near1);
//        shootFarSetup1Pose = isRed ? shootFarRed(shoot.shootFarSetup1ARed) : shootFarBlue(shoot.shootFarSetup1ABlue);
        shootNear2Pose = isRed ? createPose(shoot.near2) : createInvertedPose(shoot.near2);
//        shootFarSetup2Pose = isRed ? shootFarRed(shoot.shootFarSetup2ARed) : shootFarBlue (shoot.shootFarSetup2ABlue);
        shootNearGatePose = shootNear2Pose;
        shootNear3Pose = isRed ? createPose(shoot.near3) : createInvertedPose(shoot.near3);
//        shootFarSetup3Pose = isRed ? shootFarRed(shoot.shootFarSetup3ARed) : shootFarBlue(shoot.shootFarSetup3ABlue);
//        shootFarSetupLoadingPose = isRed ? shootFarRed(shoot.shootFarSetupLoadingARed) : shootFarBlue(shoot.shootFarSetupLoadingABlue);

        gateNearShootControlPoint = isRed ? createPose(shoot.gateShootControlPoint) : createInvertedPose(shoot.gateShootControlPoint);
        shootFar1WaypointPose = isRed ? new Pose2d(shoot.shootFar1WaypointXRed, shoot.shootFar1WaypointYRed, shoot.shootFar1WaypointARed) : new Pose2d(shoot.shootFar1WaypointXBlue, shoot.shootFar1WaypointYBlue, shoot.shootFar1WaypointABlue);
        shootFar2WaypointPose = isRed ? new Pose2d(shoot.shootFar2WaypointXRed, shoot.shootFar2WaypointYRed, shoot.shootFar2WaypointARed) : new Pose2d(shoot.shootFar2WaypointXBlue, shoot.shootFar2WaypointYBlue, shoot.shootFar2WaypointABlue);
    }
    private void declareCollectPoses() {
        collect1Pose = isRed ? // only 1 collect pose for near (no pre collect pose, only post collect pose)
                new Pose2d(collect.firstX, collect.postFirstY, collect.lineARed) :
                new Pose2d(collect.firstX, -collect.postFirstY, collect.lineABlue);
        collect1ControlPointNear = isRed ?
                createPose(collect.firstCollectControlPointNear) :
                createInvertedPose(collect.firstCollectControlPointNear);

        preCollect2PoseNear = isRed ?
                new Pose2d(collect.preSecondXNear, collect.preSecondY, collect.curvedCollect2NearA) :
                new Pose2d(collect.preSecondXNear, -collect.preSecondY, -collect.curvedCollect2NearA);
        preCollect2PoseFar = isRed ?
                new Pose2d(collect.preSecondXFar, collect.preSecondY, collect.curvedCollect2FarA) :
                new Pose2d(collect.preSecondXFar, -collect.preSecondY, -collect.curvedCollect2FarA);
        double offset = customizable.openGateOnSecond ? collect.postSecondYOffsetIfOpenGate : 0;

        collect2Pose = isRed ?
                new Pose2d(collect.secondX, collect.postSecondY - offset, collect.lineARed) :
                new Pose2d(collect.secondX, -collect.postSecondY + offset, collect.lineABlue);
        preCollect3NearPose = isRed ?
                new Pose2d(collect.preThirdXNear, collect.preThirdY, collect.preCollect3NearA) :
                new Pose2d(collect.preThirdXNear, -collect.preThirdY, -collect.preCollect3NearA);
        preCollect3FarPose = isRed ?
                new Pose2d(collect.preThirdXFar, collect.preThirdY, collect.preCollect3FarA) :
                new Pose2d(collect.preThirdXFar, -collect.preThirdY, -collect.preCollect3FarA);
        collect3Pose = isRed ?
                new Pose2d(collect.thirdX, collect.postThirdY, collect.lineARed) :
                new Pose2d(collect.thirdX, -collect.postThirdY, collect.lineABlue);

        preLoadingPose = isRed ?
                new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) :
                new Pose2d(collect.preLoadingXBlue, collect.preLoadingYBlue, collect.preLoadingABlue);
        postLoadingPose = isRed ?
                new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) :
                new Pose2d(collect.postLoadingXBlue, collect.postLoadingYBlue, collect.postLoadingABlue);
        cornerCollectPose = isRed ?
                new Pose2d(collect.cornerCollectXRed, collect.cornerCollectYRed, collect.cornerCollectARed) :
                new Pose2d(collect.cornerCollectXBlue, collect.cornerCollectYBlue, collect.cornerCollectABlue);
        cornerCollectRetryPose = isRed ?
                new Pose2d(collect.cornerCollectXRed, collect.cornerCollectRetryYRed, collect.cornerCollectARed) :
                new Pose2d(collect.cornerCollectXBlue, collect.cornerCollectRetryYBlue, collect.cornerCollectABlue);

        gateCollectOpenControlPoint = isRed ?
                createPose(collect.gateCollectOpenControlPoint) :
                createInvertedPose(collect.gateCollectOpenControlPoint);
        gateCollectOpenNearPose = isRed ?
                createPose(collect.gateCollectOpenNear) :
                createInvertedPose(collect.gateCollectOpenNear);
        gateCollectOpenFarPose = isRed ?
                createPose(collect.gateCollectOpenFarRed) :
                createPose(collect.gateCollectOpenFarBlue);
        gateCollectPose = isRed ?
                createPose(collect.gateCollect) :
                createInvertedPose(collect.gateCollect);

    }
    private void declareMiscPoses() {
        gate1Pose = isRed ?
                new Pose2d(misc.gate1X, misc.gateY, misc.gateA) :
                new Pose2d(misc.gate1X, -misc.gateY, -misc.gateA);
        gate2Pose = isRed ?
                new Pose2d(misc.gate2X, misc.gateY, misc.gateA) :
                new Pose2d(misc.gate2X, -misc.gateY, -misc.gateA);
        gateFarWaypointPose = createPose(isRed ? misc.gateFarWaypointRed : misc.gateFarWaypointBlue);

        parkFarPose = isRed ?
                new Pose2d(misc.parkFarXRed, misc.parkFarYRed, misc.parkFarARed) :
                new Pose2d(misc.parkFarXBlue, misc.parkFarYBlue, misc.parkFarABlue);
    }

    public double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public double launchLineY(double x) {
        return isRed ? -x : x;
    }
}
