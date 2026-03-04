package org.firstinspires.ftc.teamcode.opmode.autosBase;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createInvertedPose;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
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
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathGeneration;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathInfo;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.pathGeneration.PathPose;
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
        public String validCollectLetters = "123gla";
        public String nearSolo = "n2ngngn1n3n", nearPartner = "n2ngngngn1n";
        public String farLoadingFirst = "flf3flf", farThirdFirst = "f3flflf";
        public String stringBuilder = "n2ngngngn1n";
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = false;
        public boolean parkAbort = false;
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
            collect3NearControlPoint, collect3FarControlPoint, preCollect3Near, preCollect3Far, collect3,
            preLoading, loadingWaypoint, postLoading,
            gateCollectNearWaypoint, gateCollectFarControlPoint, gateCollectOpen, gateCollect,
            gate1, gate2,
            shootGateNearControlPoint,
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
        TelemetryPacket fieldPacket = new TelemetryPacket();
        Canvas fieldOverlay = fieldPacket.fieldOverlay();
        DrivePath.enableFieldDrawing(fieldOverlay);

        autoTimer = new ElapsedTime();
        isRed = alliance == Alliance.RED;

        perpNearParkLine = isRed ? new Vector2d(1, 1) : new Vector2d(1, -1);
        perpNearParkLine = perpNearParkLine.div(Math.hypot(perpNearParkLine.x, perpNearParkLine.y));

        if(customizable.stringBuilder.charAt(0) == 'n')
            start = isRed ? createPose(misc.startNearRed) : createPose(misc.startNearBlue);
        else if(customizable.stringBuilder.charAt(0) == 'f')
            start = isRed ? createPose(misc.startFarRed) : createPose(misc.startFarBlue);

        // DECLARE POSES=======================
        declareShootPoses();
        declareCollectPoses();
        declareMiscPoses();

        boolean preloadNear = customizable.stringBuilder.charAt(0) == 'n';
        if (preloadNear)
            Limelight.startingPipeline = Limelight.CLASSIFIER_PIPELINE;
        else
            Limelight.startingPipeline = Limelight.BALL_DETECTION_PIPELINE;

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        autoCommands = new AutoCommands(robot, telemetry);

        int numPaths = (customizable.stringBuilder.length()-1) / 2; //3
        if(numPaths == 0)
            throw new IllegalArgumentException("cannot have empty collectionOrder string");
        ArrayList<Action> actionOrder = new ArrayList<>();
        customizable.stringBuilder = customizable.stringBuilder.toLowerCase();

//        double preloadX = preloadNear ? shoot.nearPreload[0] : shoot.farSpike[0];
//        double preloadY = preloadNear ? shoot.nearPreload[1] : shoot.farSpike[1];
//        double preloadA = getShootPose(preloadNear, customizable.stringBuilder.charAt(1) + "").heading.toDouble();
//        Pose2d preloadShootPose = isRed ? new Pose2d(preloadX, preloadY, preloadA) : new Pose2d(preloadX, -preloadY, preloadA);
        Pose2d preloadShootPose;
        if(preloadNear)
            preloadShootPose = isRed ? createPose(shoot.nearPreload) : createInvertedPose(shoot.nearPreload);
        else {
            String nextLetter = customizable.stringBuilder.charAt(1) + "";
            if(nextLetter.equals("l"))
                preloadShootPose = isRed ? createPose(shoot.farPreloadLoading) : createInvertedPose(shoot.farPreloadLoading);
            else
                preloadShootPose = getShootPose(false, nextLetter);
        }
        int numGateCollects = 0;
        boolean toNear = true;
        for(int i=0; i<numPaths; i++) {
            boolean last = i == numPaths - 1;

            String info = customizable.stringBuilder.substring(i*2, Math.min(i*2+4, customizable.stringBuilder.length()));
            boolean fromNear = info.charAt(0) == 'n';
            String letter = info.charAt(1) + "";
            toNear = info.charAt(2) == 'n';
            String nextLetter = info.length() == 3 ? letter : info.charAt(3) + "";
            if(fromNear && letter.equals("a"))
                throw new IllegalArgumentException("cannot call limelight auto collect from near at path index " + i);

            if(!customizable.validCollectLetters.contains(letter))
                throw new IllegalArgumentException("invalid collectionOrder of " + customizable.stringBuilder + "; can only contain 1, 2, 3, G/g, L/l, or A/a");

            Pose2d shootPose;
            if(last)
                shootPose = getFinalShootPose(toNear, letter);
            else {
                shootPose = getShootPose(toNear, toNear ? letter : nextLetter);
            }

            telemetry.addData("STRING BUILDER", customizable.stringBuilder);
            telemetry.addLine("Path " + (i+1) + ": letter: " + letter + " from near: " + fromNear + " to near: " + toNear + ", last: " + last);
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
                case "g":
                    numGateCollects++;
                    double waitTime = numGateCollects == 2 ? timeConstraints.secondGaitWait : numGateCollects == 3 ? timeConstraints.thirdGateWait : 0;
                    actionOrder.add(getGateCollectAndShoot(shootPose, fromNear, toNear, waitTime));
                    break;
                case "a":
                    actionOrder.add(getLimelightLoadingZoneCollectAndShoot(shootPose));
                    break;
            }
        }

        Action autoAction = new SequentialAction(
                getPreloadDriveAndShoot(preloadShootPose, customizable.stringBuilder.charAt(0) == 'n'),
                actionOrder.get(0),
                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0),
                numPaths > 4 ? actionOrder.get(4) : new SleepAction(0),
                numPaths > 5 ? actionOrder.get(5) : new SleepAction(0),
                numPaths > 6 ? actionOrder.get(6) : new SleepAction(0),
                numPaths > 7 ? actionOrder.get(7) : new SleepAction(0),
                numPaths > 8 ? actionOrder.get(8) : new SleepAction(0),
                new ParallelAction(
                        !toNear ? getFarParkDrive() : new SleepAction(0),
                        autoCommands.stopIntake(),
                        autoCommands.stopShooter()
                )
        );

        Action timedAutoAction = new SequentialAction(
                new CustomEndAction(autoAction, () -> customizable.parkAbort && autoTimer.seconds() > timeConstraints.autoEndTime),
                autoCommands.stopIntake(),
                autoCommands.stopShooter(),
                new InstantAction(() -> robot.drive.stop())
        );

        Action fullAutoAction = new ParallelAction(
                autoCommands.updateRobotInfo(),
                new TimedAction(timedAutoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot(),
                autoCommands.savePoseContinuously(),
                packet -> {
                    fieldOverlay.clear();
                    DrivePath.drawCurrentPath();
                    robot.drawRobotInfo(fieldOverlay);
                    FtcDashboard.getInstance().sendTelemetryPacket(fieldPacket);
                    telemetry.addData("auto state", autoState);
                    robot.limelight.printInfo();
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
        telemetry.addData("auto string", customizable.stringBuilder);
        telemetry.addLine("READY TO RUN");
        telemetry.update();

        waitForStart();
        robot.startOpmode();
        autoTimer.reset();

        Actions.runBlocking(fullAutoAction);
    }
    private Pose2d getShootPose(boolean shootClose, String letter) {
        switch (letter) {
            case "1": return shootClose ? shoot1Near : shoot1Far;
            case "2": return shootClose ? shoot2Near : shoot2Far;
            case "g": return shootClose ? shootGateNear : shootGateFar;
            case "3": return shootClose ? shoot3Near : shoot3Far;
            case "l": return shootClose ? shootLoadingNear : shootLoadingFar;
            case "a": return shootLoadingFar;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.stringBuilder + "; can only contain 1, 2, 3, L/l, or G/g");
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
                    isRed ? createPose(shoot.farSpike) : createInvertedPose(shoot.farSpike);
            case "l": case "a":
                return shootClose ?
                        shootLoadingNear : shootLoadingFar;
        }
        return null;
    }

    private Action getFarParkDrive() {
        return new DrivePath(robot.drive,
                new Waypoint(parkFar).setPassPosition(true).setMinLinearPower(misc.minParkFarPower).setMaxTime(0.8)
        );
    }
    private Action getPreloadDriveAndShoot(Pose2d shootPose, boolean shootingNear) {
        Waypoint preloadShootWaypoint = new Waypoint(shootPose)
                .setPassPosition(true);
        if(shootingNear)
                preloadShootWaypoint
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                        .setMaxTime(3)
                        .setMinLinearPower(shoot.nearMinDrivePower1);
        else
            preloadShootWaypoint.setMaxTime(.9);

        DrivePath preloadShootDrive = new DrivePath(robot.drive, preloadShootWaypoint);
        Action preloadShootAction;
        if(shootingNear)
            preloadShootAction = new ParallelAction(
                    preloadShootDrive,
                    new SequentialAction(
                            new CustomEndAction(() -> preloadShootDrive.getWaypointDistanceError() < shoot.nearMinDrivePower2Dist),
                            new InstantAction(() -> preloadShootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
                    )
            );
        else
            preloadShootAction = new SequentialAction(
                    new SleepAction(timeConstraints.farPreloadDriveDelay),
                    preloadShootDrive
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
        Action shootDriveAction;
        if(shootingNear) {
            shootDrive.getCurWaypoint().setMinLinearPower(shoot.nearMinDrivePower1);
            shootDriveAction = new ParallelAction(
                    shootDrive,
                    new SequentialAction(
                            new CustomEndAction(() -> shootDrive.getWaypointDistanceError() < shoot.nearMinDrivePower2Dist),
                            new InstantAction(() -> shootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
                    )
            );
        }
        else {
            shootDrive.getCurWaypoint().setMinLinearPower(shoot.farMinDrivePower);
            shootDriveAction = shootDrive;
        }
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
                !shootingNear ? getShootAction(0) :
                !notLast ? getShootAction(timeConstraints.lastShootExtraTime) : new SleepAction(0)
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
                        .setPassPosition(true))
                :
                new DrivePath(robot.drive, telemetry, new Waypoint(shootPose)
                        .setMaxTime(10)
                        .setPassPosition(true)
                        .setControlPoint(shoot1FarControlPoint, shoot.firstShootFarT1, shoot.firstShootFarT2));

        return buildCollectAndShoot(firstCollectAction, firstGateDrive, firstShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
        Waypoint collect2Dest = new Waypoint(collect2)
                .setPassPosition(true)
                .setMinLinearPower(collect.collectDrivePower);
        if(fromNear)
            collect2Dest.setControlPoint(collect2NearControlPoint, collect.secondNearT1, collect.secondNearT2);
        else
            collect2Dest.setControlPoint(collect2FarControlPoint, collect.secondFarT1, collect.secondFarT2);

        DrivePath secondCollectDrive = new DrivePath(robot.drive, telemetry, collect2Dest);

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
            secondShootDrive = new DrivePath(robot.drive, telemetry, new Waypoint(shootPose)
                    .setMaxTime(3)
                    .setPassPosition(true)
                    .setControlPoint(shootGateNearControlPoint, shoot.gateNearShootTStartError, shoot.gateNearShootTFinishError));
        }
        else
            secondShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                    .setMaxTime(1.5)
                    .setPassPosition(true)
                    .setControlPoint(shoot2FarControlPoint, shoot.secondShootFarT1, shoot.secondShootFarT2));

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
            BoxTolerance preCollect3NearTol = new BoxTolerance(collect.thirdNearWaypointTol[0], collect.thirdNearWaypointTol[1], Math.toRadians(collect.thirdNearWaypointTol[2]));
            Waypoint preCollectNear = new Waypoint(preCollect3Near, preCollect3NearTol)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.collectDrivePower);
            thirdCollectDrive.addWaypoint(preCollectNear, 0);
        }
        else {
            BoxTolerance preCollect3FarTol = new BoxTolerance(collect.thirdFarWaypointTol[0], collect.thirdFarWaypointTol[1], Math.toRadians(collect.thirdFarWaypointTol[2]));
            Waypoint preCollectFar = new Waypoint(preCollect3Far, preCollect3FarTol)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.collectDrivePower);
            thirdCollectDrive.addWaypoint(preCollectFar, 0);
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
                .setPassPosition(true)
                .setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT);
        if(toNear)
            thirdShootDest.setMaxTime(3);
        else
            thirdShootDest.setMaxTime(2);
        DrivePath thirdShootDrive = new DrivePath(robot.drive, telemetry, thirdShootDest);

        return buildCollectAndShoot(thirdCollectAction, new SleepAction(0), thirdShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, boolean toNear) {
        BoxTolerance preLoadingTol = new BoxTolerance(collect.preLoadingTol[0], collect.preLoadingTol[1], Math.toRadians(collect.preLoadingTol[2]));
        BoxTolerance postLoadingTol = new BoxTolerance(collect.postLoadingTol[0], collect.postLoadingTol[1], Math.toRadians(collect.postLoadingTol[2]));

        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoading, preLoadingTol)
                        .setMaxTime(2)
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                new Waypoint(loadingWaypoint)
                        .setFixedLinearPower(collect.loadingDrivePower)
                        .setMaxHeadingPower(collect.loadingHeadingPower)
                        .setMaxTime(.9),
                new Waypoint(postLoading, postLoadingTol)
                        .setFixedLinearPower(collect.loadingDrivePower)
                        .setMaxHeadingPower(collect.loadingHeadingPower)
                        .setMaxTime(.7)
                        .setCustomEndCondition(() -> robot.collection.has3Balls()));
        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true));

        return buildCollectAndShoot(loadingCollectDrive, new SleepAction(0), loadingShootDrive, toNear, timeConstraints.loadingSlowIntakeTime, true, false);
    }

    private Action getLimelightLoadingZoneCollectAndShoot(Pose2d shootPose) {
        Action limelightCollectAction = new SequentialAction(
                robot.scanForBalls(() -> Math.toRadians(alliance == Alliance.RED ? 90 : -90), null),
                new CustomEndAction(getLimelightCollectDrive(), robot.collection::autoCollectHas3Balls)
        );
        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true));

        return buildCollectAndShoot(limelightCollectAction, new SleepAction(0), loadingShootDrive, false, timeConstraints.loadingSlowIntakeTime, true, false);
    }
    private Action getLimelightCollectDrive() {
        return new SequentialAction(
                robot.limelight.ballDetection.takeBallSnapshotAction(),
                new Action() {
                    DrivePath path = null;
                    ElapsedTime timer = null;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (timer == null) {
                            timer = new ElapsedTime();
                            timer.reset();
                        }
                        if (timer.seconds() > timeConstraints.maxLimelightWaitTime)
                            return false;

                        if (path == null) {
                            Pose2d robotPose = robot.drive.localizer.getPose();
                            ArrayList<Vector2d> ballPositions = robot.limelight.ballDetection.getCurrentBlobPositions();
                            PathInfo pathInfo = PathGeneration.generateSimplifiedAutoCollectPath(robotPose, ballPositions);
                            if (pathInfo == null) {
                                telemetry.addLine("path is null");
                                return true;
                            }
                            path = new DrivePath(robot.drive);
                            for (PathPose pathPose : pathInfo.optimizedPathPoses)
                                path.addWaypoint(pathPose.waypoint);
                        }
                        return path.run(telemetryPacket);
                    }
                }
        );
    }
//    private Action getRepeatedCornerCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
//        DrivePath gateShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
//                .setMaxTime(4));
//
//        return buildCollectAndShoot(repeatedCornerCollect(fromNear), new SleepAction(0), gateShootDrive, toNear, timeConstraints.loadingSlowIntakeTime, true, false);
//    }
//    private Action repeatedCornerCollect(boolean fromNear) {
//        return new Action() {
//            private Action gateCollectDrive, gateResetDrive;
//            private boolean first = true;
//            private boolean ranCollectDrive = false;
//            private int numTimesCollected = 0;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (first) {
//                    first = false;
//                    resetCollectDrive();
//                }
//                if (!ranCollectDrive) {
//                    if (!gateCollectDrive.run(telemetryPacket)) {
//                        ranCollectDrive = true;
//                        numTimesCollected++;
//                        if (robot.collection.has3Balls() || numTimesCollected > customizable.maxCornerRetries)
//                            return false;
//                        resetRetryDrive();
//                        resetCollectDrive();
//                    }
//                }
//                if (ranCollectDrive) {
//                    ranCollectDrive = gateResetDrive.run(telemetryPacket);
//                }
//                return true;
//            }
//            private void resetCollectDrive() {
//                if (numTimesCollected == 0) {
//                    double sign = isRed ? 1 : -1;
//                    double collectTangent = fromNear ? sign * Math.toRadians(30) : sign * Math.toRadians(80);
//                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                            .setTangent(collectTangent)
//                            .splineToSplineHeading(cornerCollect, collectTangent)
//                            .build(),
//                            () -> robot.collection.has3Balls(), timeConstraints.cornerCollectMaxTime);
//                }
//                else {
//                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                            .strafeToLinearHeading(new Vector2d(collect.cornerCollectRetryX, cornerCollect.position.y), cornerCollect.heading.toDouble())
//                            .build(),
//                            () -> robot.collection.has3Balls(), timeConstraints.cornerCollectMaxTime);
//                }
//            }
//            private void resetRetryDrive() {
//                gateResetDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                        .strafeToLinearHeading(cornerCollectRetry.position, cornerCollectRetry.heading.toDouble())
//                        .build(),
//                        () -> robot.collection.has3Balls() || Math.abs(robot.drive.localizer.getPose().position.y) < (cornerCollectRetry.position.y) + 1);
//            }
//        };
//    }

    private Action getGateCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double waitTime) {
        DrivePath gateOpenDrive;
        if (fromNear) {
            BoxTolerance tol = new BoxTolerance(collect.gateNearWaypointTol[0], collect.gateNearWaypointTol[1], Math.toRadians(collect.gateNearWaypointTol[2]));
            gateOpenDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(gateCollectNearWaypoint, tol)
                            .setMaxTime(1.3)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1Power),
                    new Waypoint(gateCollectOpen)
                            .setMaxTime(1.5)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1Power)
            );
        }
        else {
            gateOpenDrive = new DrivePath(robot.drive,
                    new Waypoint(gateCollectOpen)
                            .setMaxTime(2.4)
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
                        .setControlPoint(shootGateNearControlPoint, shoot.gateNearShootTStartError, shoot.gateNearShootTFinishError));
        else
            gateShootDrive = new DrivePath(robot.drive,
                    new Waypoint(shootPose)
                            .setPassPosition(true)
                            .setControlPoint(shootGateFarControlPoint, shoot.gateFarTStartError, shoot.gateFarTFinishError)
            );

        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, timeConstraints.postIntakeTime, false, true);

    }
    private void declareShootPoses() {
        shoot1Near = isRed ? createPose(shoot.near1) : createInvertedPose(shoot.near1);
        shoot1Far = isRed ? createPose(shoot.farSpike) : createInvertedPose(shoot.farSpike);
        shoot2Near = isRed ? createPose(shoot.near2) : createInvertedPose(shoot.near2);
        shoot2Far = shoot1Far;
        shootGateNear = shoot2Near;
        shootGateNearControlPoint = isRed ? createPose(shoot.gateNearControlPoint) : createInvertedPose(shoot.gateNearControlPoint);
        shootGateFar = shoot2Far;
        shootGateFarControlPoint = isRed ? createPose(shoot.gateFarControlPoint) : createInvertedPose(shoot.gateFarControlPoint);
        shoot3Near = isRed ? createPose(shoot.near3) : createInvertedPose(shoot.near3);
        shoot3Far = shoot1Far;

        shootLoadingNear = shoot3Near;
        shootLoadingFar = isRed ? createPose(shoot.farLoading) : createInvertedPose(shoot.farLoading);

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
        preCollect3Far = isRed ?
                createPose(collect.thirdFarWaypoint) :
                createInvertedPose(collect.thirdFarWaypoint);
        collect3NearControlPoint = isRed ?
                createPose(collect.thirdNearControlPoint) :
                createInvertedPose(collect.thirdNearControlPoint);
        collect3FarControlPoint = isRed ?
                createPose(collect.thirdFarControlPoint) :
                createInvertedPose(collect.thirdFarControlPoint);

        preLoading = isRed ?
                createPose(collect.preLoading) :
                createInvertedPose(collect.preLoading);
        loadingWaypoint = isRed ?
                createPose(collect.loadingWaypoint) :
                createInvertedPose(collect.loadingWaypoint);
        postLoading = isRed ?
                createPose(collect.postLoading) :
                createInvertedPose(collect.postLoading);

        gateCollectNearWaypoint = isRed ?
                createPose(collect.gateNearWaypoint) :
                createInvertedPose(collect.gateNearWaypoint);
        gateCollectFarControlPoint = isRed ?
                createPose(collect.gateFarControlPoint) :
                createInvertedPose(collect.gateFarControlPoint);
        gateCollectOpen = isRed ?
                createPose(collect.gateOpen) :
                createInvertedPose(collect.gateOpen);
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
        parkFar = isRed ?
                createPose(misc.parkFar) :
                createInvertedPose(misc.parkFar);
    }
}
