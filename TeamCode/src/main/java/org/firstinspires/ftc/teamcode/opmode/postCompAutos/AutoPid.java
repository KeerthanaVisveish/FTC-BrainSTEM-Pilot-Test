package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;

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
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;

@Config
public abstract class AutoPid extends LinearOpMode {
    public static class Customizable {
        // if partner has 3 ball close: f2fgn1n3n
        // if partner has 6 ball close: f2fgf3flf
        public String collectionOrder = "n3n";
        public String minTimes = "-1,-1,-1,-1,-1,-1";
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
            collect1Pose, preCollect1Pose,
            preCollect2Pose, collect2Pose,
            preCollect3Pose, collect3Pose,
            preLoadingPose, postLoadingPose,
            cornerCollectPose, cornerCollectRetryPose,
            gateCollectOpenControlPoint, gateCollectOpenNearPose, gateCollectOpenFarPose, gateCollectPose,
            gate1Pose, gate2Pose,
            gateFarWaypointPose,
            parkFarPose,
            shootFar1WaypointPose, shootFar2WaypointPose,
            shootNearSetup1Pose, shootFarSetup1Pose,
            shootNearSetup2Pose, shootFarSetup2Pose,
            shootNearSetupGatePose,
            shootNearSetup3Pose, shootFarSetup3Pose,
            shootNearSetupLoadingPose, shootFarSetupLoadingPose;
    private boolean isRed;
    private AutoState autoState;
    private boolean autoActionFinished;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        autoActionFinished = false;

        autoTimer = new ElapsedTime();
        isRed = alliance == Alliance.RED;

        if(customizable.collectionOrder.charAt(0) == 'n')
            start = isRed ? new Pose2d(misc.startNearXRed, misc.startNearYRed, misc.startNearARed) : new Pose2d(misc.startNearXBlue, misc.startNearYBlue, misc.startNearABlue);
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

        String[] minTimeStr = customizable.minTimes.split(",");
        double[] minTimes = new double[minTimeStr.length];
        if(minTimeStr.length != numPaths + 1)
            telemetry.addLine("min time string currently has " + minTimeStr.length + " nums. It must have " + (numPaths+1) + " nums. (1 more than collectionOrder)");
        for (int i=0; i<minTimeStr.length; i++) {
            try {
                minTimes[i] = Double.parseDouble(minTimeStr[i]);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("min time string can only contain numbers and commas. " + customizable.minTimes + " is not valid");
            }
        }

        Pose2d shootPose = new Pose2d(0, 0, 0);
        Pose2d prevShootPose;
        Pose2d preloadShootPose = getSetupPose(customizable.collectionOrder.substring(0, 2));
        for(int i=0; i<numPaths; i++) {
            prevShootPose = new Pose2d(shootPose.position.x, shootPose.position.y, shootPose.heading.toDouble());
            if(i < numPaths - 1)
                shootPose = getSetupPose(customizable.collectionOrder.substring(i*2 + 2, i*2 + 4));
            else
                  shootPose = getFinalShootPose(customizable.collectionOrder.charAt(i*2+2) + "" + customizable.collectionOrder.charAt(i * 2 + 1));

            String curLetter = customizable.collectionOrder.charAt(i*2+1) + "";
            boolean fromNear = customizable.collectionOrder.charAt(i*2) == 'n';
            boolean toNear = customizable.collectionOrder.charAt(i*2+2) == 'n';
            double minTime = minTimes[i+1];
            telemetry.addLine("Path " + (i+1) + ": letter: " + curLetter + " from near: " + fromNear + " to near: " + toNear + " min time: " + minTime);
            telemetry.addData("Path " + (i+1) + " Shoot pose", MathUtils.formatPose3(shootPose));
            switch(curLetter) {
                case "1" :
                    actionOrder.add(getFirstCollectAndShoot(shootPose, fromNear, toNear, minTime));
                    break;
                case "2" :
                    actionOrder.add(getSecondCollectAndShoot(shootPose, fromNear, toNear, minTime));
                    break;
                case "3" :
                    actionOrder.add(getThirdCollectAndShoot(shootPose, fromNear, toNear, minTime));
                    break;
                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose, toNear, minTime)); break;
                case "c": actionOrder.add(getRepeatedCornerCollectAndShoot(shootPose, fromNear, toNear, minTime)); break;
                case "g": actionOrder.add(getGateCollectAndShoot(shootPose, fromNear, toNear, minTime)); break;
            }
        }

        Action autoAction = new SequentialAction(
                getPreloadDriveAndShoot(preloadShootPose, customizable.collectionOrder.charAt(0) == 'n', minTimes[0]),
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

        telemetry.addData("alliance", alliance);
        telemetry.addData("auto string", customizable.collectionOrder);
        telemetry.addLine("READY TO RUN");
        telemetry.update();
        waitForStart();
        autoTimer.reset();

        robot.shooter.setBallsShot(0); // always start with 3 preloads

        Actions.runBlocking(
                forcedStopAutoAction

        );
        robot.drive.stop();
    }
    private Pose2d getSetupPose(String info) {
        boolean shootClose = info.charAt(0) == 'n';
        String letter = info.charAt(1) + "";
        switch (letter) {
            case "1": return shootClose ? shootNearSetup1Pose : shootFarSetup1Pose;
            case "2": return shootClose ? shootNearSetup2Pose : shootFarSetup2Pose;
            case "g": return shootClose ? shootNearSetupGatePose : shootFarSetup2Pose;
            case "3": return shootClose ? shootNearSetup3Pose : shootFarSetup3Pose;
            case "l": return shootClose ? shootNearSetupLoadingPose : shootFarSetupLoadingPose;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
        }
    }
    private Pose2d getFinalShootPose(String info) {
        boolean shootClose = info.charAt(0) == 'n';
        String letter = info.charAt(1) + "";
        switch (letter) {
            case "1": return shootClose ?
                    isRed ? shootNearLastRed(shoot.shootNearLastSetup1ARed) : shootNearLastBlue(shoot.shootNearLastSetup1ABlue) :
                    shootFarSetup1Pose;
            case "2":
            case "g": return shootClose ? shootNearSetup2Pose : shootFarSetup2Pose;
            case "3": return shootClose ?
                    isRed ? shootMidLastRed(shoot.shootNearSetup3ARed) : shootMidLastBlue(shoot.shootNearSetup3ABlue) :
                    isRed ? new Pose2d(shoot.shootFarXRed, shoot.shootFarYRed, Math.toRadians(120)) : new Pose2d(shoot.shootFarXBlue, shoot.shootFarYBlue, Math.toRadians(-120));
            case "l": case "c": return shootClose ? shootNearSetupLoadingPose : shootFarSetupLoadingPose;
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
    private Action getPreloadDriveAndShoot(Pose2d shootPose, boolean shootingNear, double minTime) {
        DrivePath preloadShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose).setMaxTime(3).setPassPosition(true).setHeadingLerp(shoot.preloadHeadingLerp));
        return new SequentialAction(
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                new ParallelAction(
                        autoCommands.flickerHalfUp(),
                        autoCommands.speedUpShooter(),
                        autoCommands.enableTurretTracking(),
                        preloadShootDrive,
                        new SequentialAction(
                                autoCommands.engageClutch(),
                                autoCommands.reverseIntake(),
                                new SleepAction(Collection.shootOuttakeTimeAuto),
                                autoCommands.stopIntake(),
                                new SleepAction(Collection.postShootOuttakeWaitAuto)
                        )
                ),
                new InstantAction(() -> autoState = AutoState.SHOOT),
                waitUntilMinTime(minTime),
                autoCommands.runIntake(),
                autoCommands.waitTillDoneShooting(Collection.params.maxTimeBetweenShots, shootingNear ? timeConstraints.shootNearMinTime : timeConstraints.shootFarMinTime),
                decideFlicker(),
                autoCommands.disengageClutch()
        );
    }
    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, Action shootDrive, boolean shootingNear, double minTime, double postIntakeTime, boolean runIntake) {
        return new SequentialAction(
                runIntake ? autoCommands.runIntake() : new SleepAction(0),
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
                collectDrive,
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(() -> autoState = AutoState.OPEN_GATE),
                                gateDrive,
                                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                                shootDrive,
                                packet -> {robot.drive.stop(); return false; }
                        ),
                        new SequentialAction(
                                new SleepAction(postIntakeTime),
                                autoCommands.stopIntake()
                        )
                ),
                new InstantAction(() -> autoState = AutoState.SHOOT),
//                autoCommands.speedUpShooter(),
                new InstantAction(() -> robot.shooter.setBallsShot(0)),
                autoCommands.flickerHalfUp(),
                autoCommands.reverseIntake(),
                new SleepAction(Collection.shootOuttakeTimeAuto),
                autoCommands.stopIntake(),
                autoCommands.engageClutch(),
                new SleepAction(Collection.postShootOuttakeWaitAuto),
                autoCommands.runIntake(),
                autoCommands.waitTillDoneShooting(Collection.params.maxTimeBetweenShots, shootingNear ? timeConstraints.shootNearMinTime : timeConstraints.shootFarMinTime),
                decideFlicker(),
                autoCommands.disengageClutch()
        );
    }
    private Action getFirstCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        DrivePath firstCollectDrive;
        if(fromNear) {
            firstCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect1Pose, new Tolerance(0.75, 2, 3))
                    .setMinLinearPower(collect.firstDrivePower)
                    .setMaxLinearPower(collect.firstDrivePower)
                    .setMaxTime(1.4)
                    .setCustomEndCondition(() -> robot.collection.intakeHas3Balls())
                    .setPassPosition(true));
        }
        else {
            Pose2d preCollectPose = new Pose2d(preCollect1Pose.position.x, preCollect1Pose.position.y, preCollect1Pose.heading.toDouble());
            firstCollectDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(preCollectPose, collect.waypointTol)
                            .setPassPosition(true)
                            .setSlowDownPercent(collect.waypointSlowDown)
                            .setMaxTime(2)
                            .setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                    new Waypoint(collect1Pose)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.collectDrivePower)
                            .setMaxLinearPower(collect.collectDrivePower)
                            .setMaxTime(2)
                            .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));
        }

        double sign = isRed ? -1 : 1;
        Pose2d preGate1Pose = new Pose2d(gate1Pose.position.x + misc.preGateXOffset, gate1Pose.position.y + sign * misc.preGateClearance, gate1Pose.heading.toDouble());

        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGate1Pose, misc.preGateTol).setMinLinearPower(misc.gateMinPower).setPassPosition(true),
                                new Waypoint(gate1Pose).setMinLinearPower(misc.gateMinPower)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);
        Action firstShootDrive = toNear ?
                new DrivePath(robot.drive, new Waypoint(shootPose)
                        .setMaxTime(3).setPassPosition(true))
                :
                new DrivePath(robot.drive, telemetry,
                        new Waypoint(shootFar1WaypointPose, shoot.waypointTol)
                                .setPassPosition(true)
                                .setSlowDownPercent(shoot.waypointSlowDown)
                                .setMaxTime(10),
                        new Waypoint(shootPose)
                                .setMaxTime(10).setPassPosition(true));

        return buildCollectAndShoot(firstCollectDrive, firstGateDrive, firstShootDrive, toNear, minTime, timeConstraints.postIntakeTime, true);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        double xOffset = fromNear ?
                (isRed ? -collect.preCollect2NearRedXOffset : -collect.preCollect2NearBlueXOffset) :
                collect.preCollect2FarOffset;
        Pose2d preCollectPose = new Pose2d(preCollect2Pose.position.x + xOffset, preCollect2Pose.position.y, preCollect2Pose.heading.toDouble());

//        Waypoint w1 = fromNear ?
//                new Waypoint(preCollectPose, collect.waypointTol).setPassPosition(true).setSlowDownPercent(collect.farWaypointSlowDown).setMaxTime(2) :
//                new Waypoint(preCollectPose, collect.waypointTol).setPassPosition(true).setHeadingLerp(PathParams.HeadingLerpType.TANGENT).setSlowDownPercent(collect.waypointSlowDown).setMaxTime(2).setHeadingTangentDeactivateThreshold(28);
//        Action secondCollectDrive = new DrivePath(robot.drive, telemetry,
//                w1,
//                new Waypoint(collect2Pose)
//                        .setPassPosition(true)
//                        .setMinLinearPower(collect.collectDrivePower)
//                        .setMaxLinearPower(collect.collectDrivePower)
//                        .setLateralAxialWeights(1, 1)
//                        .setMaxTime(2)
//                        .setCustomEndCondition(() -> robot.collection.intakeHas3Balls())
//                );
        Action newSecondCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect2Pose)
                .setPassPosition(true)
                .setMinLinearPower(collect.newCollectDrivePower)
                .setControlPoint(new Pose2d(preCollectPose.position, collect.curvedCollect2ARad), collect.secondTValueStartTime, collect.secondTValueMaxOutTime));

        double sign = isRed ? -1 : 1;
        Pose2d preGatePose = new Pose2d(gate2Pose.position.x - misc.preGateXOffset, gate2Pose.position.y + sign * misc.preGateClearance, gate2Pose.heading.toDouble());

        Action secondGateDrive = customizable.openGateOnSecond ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGatePose, misc.preGateTol).setMinLinearPower(misc.gateCollectWaypointMinPower).setPassPosition(true).setSlowDownPercent(0).setMaxTime(0.5),
                                new Waypoint(gate2Pose).setMinLinearPower(misc.gateMinPower)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);

        DrivePath secondShootDrive;
        if(toNear)
            secondShootDrive = new DrivePath(robot.drive, telemetry, new Waypoint(shootPose).setMaxTime(3).setPassPosition(true));
        else
            secondShootDrive = new DrivePath(robot.drive,
                    new Waypoint(shootFar2WaypointPose)
                            .setMaxTime(1.2)
                            .setSlowDownPercent(0)
                            .setPassPosition(true)
                            .setLateralAxialWeights(2.4, 1),
                    new Waypoint(shootPose).setMaxTime(1.5).setPassPosition(true));

        return buildCollectAndShoot(newSecondCollectDrive, secondGateDrive, secondShootDrive, toNear, minTime, timeConstraints.postIntakeTime, true);
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        double xOffset = fromNear ? -collect.preCollect3NearXOffset : collect.preCollect2NearBlueXOffset;
        Pose2d preCollectPose = new Pose2d(preCollect3Pose.position.x + xOffset, preCollect3Pose.position.y, preCollect3Pose.heading.toDouble());

//        DrivePath thirdCollectDrive = new DrivePath(robot.drive, telemetry,
//                new Waypoint(preCollectPose, collect.waypointTol)
//                        .setPassPosition(true)
//                        .setMaxTime(3)
//                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
//                        .setSlowDownPercent(fromNear ? collect.waypointSlowDown : collect.farWaypointSlowDown),
//                new Waypoint(collect3Pose)
//                        .setLateralAxialWeights(1.2, 1)
//                        .setMaxTime(2).setPassPosition(true)
//                        .setMinLinearPower(collect.collectDrivePower).setMaxLinearPower(collect.collectDrivePower)
//                        .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));
        Action thirdCollectDrive = new DrivePath(robot.drive, telemetry,
                new Waypoint(collect3Pose)
                        .setPassPosition(true)
                        .setMinLinearPower(collect.newCollectDrivePower)
                        .setControlPoint(new Pose2d(preCollectPose.position, collect.curvedCollect3ARad), collect.thirdTValueStartTime, collect.thirdTValueMaxOutTime));

        Waypoint thirdShootDest = toNear ?
                new Waypoint(shootPose).setMaxTime(3).setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT) :
                new Waypoint(shootPose).setMaxTime(2).setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT);
        Action thirdShootDrive = new DrivePath(robot.drive, telemetry, thirdShootDest.setPassPosition(true));

        return buildCollectAndShoot(thirdCollectDrive, new SleepAction(0), thirdShootDrive, toNear, minTime, timeConstraints.postIntakeTime, true);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, boolean toNear, double minTime) {
        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoadingPose, new Tolerance(3.5, 2, 4))
                        .setMaxTime(2).setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                new Waypoint(postLoadingPose)
                        .setMinLinearPower(collect.loadingZoneCollectDrivePower).setMaxLinearPower(collect.loadingZoneCollectDrivePower)
                        .setMaxTime(1.5)
                        .setLateralAxialWeights(2.2, 1)
                        .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));
        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3).setPassPosition(true));

        return buildCollectAndShoot(loadingCollectDrive, new SleepAction(0), loadingShootDrive, toNear, minTime, timeConstraints.loadingSlowIntakeTime, true);
    }

    private Action getRepeatedCornerCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        DrivePath gateShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(4));

        return buildCollectAndShoot(repeatedCornerCollect(fromNear), new SleepAction(0), gateShootDrive, toNear, minTime, timeConstraints.loadingSlowIntakeTime, true);
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
                        if (robot.collection.intakeHas3Balls() || numTimesCollected > customizable.maxCornerRetries)
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
                            () -> robot.collection.intakeHas3Balls(), timeConstraints.cornerCollectMaxTime);
                }
                else {
                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(collect.cornerCollectRetryX, cornerCollectPose.position.y), cornerCollectPose.heading.toDouble())
                            .build(),
                            () -> robot.collection.intakeHas3Balls(), timeConstraints.cornerCollectMaxTime);
                }
            }
            private void resetRetryDrive() {
                gateResetDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                        .strafeToLinearHeading(cornerCollectRetryPose.position, cornerCollectRetryPose.heading.toDouble())
                        .build(),
                        () -> robot.collection.intakeHas3Balls() || Math.abs(robot.drive.localizer.getPose().position.y) < (cornerCollectRetryPose.position.y) + 1);
            }
        };
    }

    private Action getGateCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        Action gateOpenDrive;
        if (fromNear) {
            gateOpenDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(gateCollectOpenNearPose, collect.gateCollectTol).setMaxTime(1.9).setPassPosition(true).setLateralAxialWeights(collect.gateCollectLateralWeight, 1).setMinLinearPower(collect.newCollectDrivePower)
                            .setControlPoint(gateCollectOpenControlPoint, 0, collect.gateCollectOpenTMaxTime)
            );
        }
        else
            gateOpenDrive = new DrivePath(robot.drive,
                    new Waypoint(gateFarWaypointPose, collect.waypointTol).setMaxTime(1.5).setPassPosition(true).setMinLinearPower(collect.gateCollectMinPower).setHeadingLerp(PathParams.HeadingLerpType.TANGENT).setSlowDownPercent(0),
                    new Waypoint(gateCollectOpenFarPose, collect.gateCollectTol).setMaxTime(0.9).setPassPosition(true).setMinLinearPower(collect.gateCollectMinPower)
            );

        Pose2d newGateCollectPose = fromNear ?
                gateCollectPose :
                new Pose2d(gateCollectPose.position.x + 3, gateCollectPose.position.y, gateCollectPose.heading.toDouble());

        Action completeGateCollectDrive = new SequentialAction(
                autoCommands.stopIntake(),
                gateOpenDrive,
                autoCommands.runIntake(),
                new DrivePath(robot.drive,
                        new Waypoint(newGateCollectPose).setMaxTime(0.5).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
                new CustomEndAction(new SleepAction(timeConstraints.gateCollectMaxTime), () -> robot.collection.intakeHas3Balls())
        );

        Action gateShootDrive;
        if (toNear)
            gateShootDrive = new DrivePath(robot.drive,
                new Waypoint(shootPose).setMaxTime(2).setPassPosition(true).setLateralAxialWeights(1, collect.gateShootAxialWeight));
        else
            gateShootDrive = new DrivePath(robot.drive,
                    new Waypoint(gateFarWaypointPose, collect.waypointTol).setMaxTime(1.2).setPassPosition(true).setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT).setSlowDownPercent(0),
                    new Waypoint(shootPose).setMaxTime(2).setPassPosition(true)
            );

        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, minTime, timeConstraints.postIntakeTime, false);

    }

    private Action waitUntilMinTime(double minTime) {
        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
    }
    private Action decideFlicker() {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean first = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (robot.shooter.getBallsShot() == 3)
                    return false;

                if (first) {
                    first = false;
                    timer.reset();
                }

                robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);
                robot.collection.setCollectionState(Collection.CollectionState.OFF);

                return timer.seconds() < 0.4;
            }
        };
    }
    private void declareShootPoses() {
        shootNearSetup1Pose = isRed ? shootNearRed(shoot.shootNearSetup1ARed) : shootNearBlue(shoot.shootNearSetup1ABlue);
        shootFarSetup1Pose = isRed ? shootFarRed(shoot.shootFarSetup1ARed) : shootFarBlue(shoot.shootFarSetup1ABlue);
        shootNearSetup2Pose = isRed ? shootMidRed(shoot.shootNearSetup2ARed) : shootMidBlue(shoot.shootNearSetup2ABlue);
        shootFarSetup2Pose = isRed ? shootFarRed(shoot.shootFarSetup2ARed) : shootFarBlue (shoot.shootFarSetup2ABlue);
        shootNearSetupGatePose = isRed ? shootMidRed(shoot.shootNearSetupGateARed) : shootMidBlue(shoot.shootNearSetupGateABlue);
        shootNearSetup3Pose = isRed ? shootMidRed(shoot.shootNearSetup3ARed) : shootMidBlue(shoot.shootNearSetup3ABlue);
        shootFarSetup3Pose = isRed ? shootFarRed(shoot.shootFarSetup3ARed) : shootFarBlue(shoot.shootFarSetup3ABlue);
        shootNearSetupLoadingPose = isRed ? shootNearRed(shoot.shootNearSetupLoadingARed) : shootNearBlue(shoot.shootNearSetupLoadingABlue);
        shootFarSetupLoadingPose = isRed ? shootFarRed(shoot.shootFarSetupLoadingARed) : shootFarBlue(shoot.shootFarSetupLoadingABlue);

        shootFar1WaypointPose = isRed ? new Pose2d(shoot.shootFar1WaypointXRed, shoot.shootFar1WaypointYRed, shoot.shootFar1WaypointARed) : new Pose2d(shoot.shootFar1WaypointXBlue, shoot.shootFar1WaypointYBlue, shoot.shootFar1WaypointABlue);
        shootFar2WaypointPose = isRed ? new Pose2d(shoot.shootFar2WaypointXRed, shoot.shootFar2WaypointYRed, shoot.shootFar2WaypointARed) : new Pose2d(shoot.shootFar2WaypointXBlue, shoot.shootFar2WaypointYBlue, shoot.shootFar2WaypointABlue);
    }
    private void declareCollectPoses() {
        collect1Pose = isRed ? // only 1 collect pose for near (no pre collect pose, only post collect pose)
                new Pose2d(collect.firstXRed, collect.postFirstYRed, collect.lineARed) :
                new Pose2d(collect.firstXBlue, collect.postFirstYBlue, collect.lineABlue);
        preCollect1Pose = isRed ?
                new Pose2d(collect.firstXRed, collect.preFirstYRed, collect.lineARed) :
                new Pose2d(collect.firstXBlue, collect.preFirstYBlue, collect.lineABlue);

        preCollect2Pose = isRed ?
                new Pose2d(collect.secondXRed, collect.preSecondYRed, collect.lineARed) :
                new Pose2d(collect.secondXBlue, collect.preSecondYBlue, collect.lineABlue);
        collect2Pose = isRed ?
                new Pose2d(collect.secondXRed, collect.postSecondYRed, collect.lineARed) :
                new Pose2d(collect.secondXBlue, collect.postSecondYBlue, collect.lineABlue);

        preCollect3Pose = isRed ?
                new Pose2d(collect.thirdXRed, collect.preThirdYRed, collect.preCollect3NearARed) :
                new Pose2d(collect.thirdXBlue, collect.preThirdYBlue, collect.preCollect3NearABlue);
        collect3Pose = isRed ?
                new Pose2d(collect.thirdXRed, collect.postThirdYRed, collect.lineARed) :
                new Pose2d(collect.thirdXBlue, collect.postThirdYBlue, collect.lineABlue);

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
                createPose(collect.gateCollectOpenControlPointRed) :
                createPose(collect.gateCollectOpenControlPointBlue);
        gateCollectOpenNearPose = isRed ?
                createPose(collect.gateCollectOpenNearRed) : createPose(collect.gateCollectOpenNearBlue);
        gateCollectOpenFarPose = isRed ?
                createPose(collect.gateCollectOpenFarRed) :
                createPose(collect.gateCollectOpenFarBlue);
        gateCollectPose = isRed ?
                createPose(collect.gateCollectRed) :
                createPose(collect.gateCollectBlue);

    }
    private void declareMiscPoses() {
        gate1Pose = isRed ?
                new Pose2d(misc.gate1XRed, misc.gateYRed,misc.gateARed) :
                new Pose2d(misc.gate1XBlue, misc.gateYBlue,misc.gateABlue);
        gate2Pose = isRed ?
                new Pose2d(misc.gate2XRed, misc.gateYRed, misc.gateARed) :
                new Pose2d(misc.gate2XBlue, misc.gateYBlue, misc.gateABlue);
        gateFarWaypointPose = createPose(isRed ? misc.gateFarWaypointRed : misc.gateFarWaypointBlue);

        parkFarPose = isRed ?
                new Pose2d(misc.parkFarXRed, misc.parkFarYRed, misc.parkFarARed) :
                new Pose2d(misc.parkFarXBlue, misc.parkFarYBlue, misc.parkFarABlue);
    }
    public Pose2d shootNearRed(double angleRad) { return new Pose2d(shoot.shootNearXRed, shoot.shootNearYRed, angleRad); }
    public Pose2d shootNearLastRed(double angleRad) { return new Pose2d(shoot.shootNearLastXRed, shoot.shootNearLastYRed, angleRad); }
    public Pose2d shootMidLastRed(double angleRad) { return new Pose2d(shoot.shootMidLastXRed, shoot.shootMidLastYRed, angleRad); }
    public Pose2d shootMidRed(double angleRad) { return new Pose2d(shoot.shootMidXRed, shoot.shootMidYRed, angleRad); }
    public Pose2d shootFarRed(double angleRad) { return new Pose2d(shoot.shootFarXRed, shoot.shootFarYRed, angleRad); }
    public Pose2d shootNearBlue(double angleRad) { return new Pose2d(shoot.shootNearXBlue, shoot.shootNearYBlue, angleRad); }
    public Pose2d shootNearLastBlue(double angleRad) { return new Pose2d(shoot.shootNearLastXBlue, shoot.shootNearLastYBlue, angleRad); }
    public Pose2d shootMidLastBlue(double angleRad) { return new Pose2d(shoot.shootMidLastXBlue, shoot.shootMidLastYBlue, angleRad); }
    public Pose2d shootMidBlue(double angleRad) { return new Pose2d(shoot.shootMidXBlue, shoot.shootMidYBlue, angleRad); };
    public Pose2d shootFarBlue(double angleRad) { return new Pose2d(shoot.shootFarXBlue, shoot.shootFarYBlue, angleRad); }

    public double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public double launchLineY(double x) {
        return isRed ? -x : x;
    }
}
