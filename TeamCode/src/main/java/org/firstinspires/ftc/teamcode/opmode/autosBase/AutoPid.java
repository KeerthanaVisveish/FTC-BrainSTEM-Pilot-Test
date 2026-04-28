package org.firstinspires.ftc.teamcode.opmode.autosBase;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createInvertedPose;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createInvertedVec;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createVec;

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
import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;
import org.firstinspires.ftc.teamcode.opmode.teleop.PreGameSetupTele;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.CircleTolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

@Config
public abstract class AutoPid extends LinearOpMode {
    public static class Customizable {
        public String shotTimes = "0, 0, 0, 0, 0, 0";
        public String nearSolo = "n 2n gn g.5n 1n 3n", nearPartner = "n 2n gtn gn gn 1n";
        public String nearPattern = "n";
        public String farLoadingFirst = "f lf 3f af af", farThirdFirst = "f 3f lf af af";
        public String farNoLimelight = "f lf 3f lcf lcf lcf", far18 = "f lf 3f lcf lcf af";
        public Alliance alliance = Alliance.RED; // this is just the default it's re assigned later
        public String stringBuilder = "";
        public boolean smartPark = true;
        public boolean shouldColorSort = false;
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
    protected ElapsedTime autoTimer;
    protected BrainSTEMRobot robot;
    protected AutoCommands autoCommands;
    protected Pose2d start,
            collect1NearControlPoint, collect1FarControlPoint, collect1,
            collect2NearWaypoint, collect2NearControlPoint, collect2, collect2IfOpenGate,
            preCollect3Near, collect3NearControlPoint, collect3NearCleanup, preCollect3Far, collect3FarControlPoint, collect3,
            preLoadingWaypoint, preLoading, loadingWaypoint, postLoading, postLoading2,
            loadingCornerControlPoint, loadingCorner,
            loadingGateWaitWaypoint, loadingGateWait,
            gateCollectNearWaypoint, gateCollectNearWaypoint2, gateCollectFarControlPoint, gateCollectOpen, gateCollectOpenHold, gateCollect, gateTapBackup, gateTap,
            gate1, gate2,
            shootGateNearControlPoint,
            parkFar, hpParkFar, gatePark;
    protected Pose2d preloadShootPose, shoot1Near, shoot1Far, shoot1FarControlPoint,
            shoot2Near, shoot2Far, shoot2FarControlPoint,
            shootGateNear, shootGateFar, shootGateFarControlPoint,
            shoot3Near, shoot3Far,
            shootLoadingNear, shootLoadingFar, shootLoadingOptimizedFar, shootLoadingFarControlPoint;
    protected boolean isRed;
    protected AutoState autoState;
    protected Vector2d perpNearParkLine, perpFarParkLine, farParkLineOrigin;
    private int targetGreenPos = -1;
    protected boolean[] shouldStop = { false };
    protected boolean[] shouldPark = { false };
    protected boolean[] autoActionDone = { false };
    public AutoPid() {
        customizable.shouldColorSort = false;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        if(!BrainSTEMTeleOp.inCompetition)
            throw new IllegalStateException("In Competition is not true - run ToggleInCompetitionTele and press A");

        autoTimer = new ElapsedTime();
        alliance = customizable.alliance;
        isRed = alliance == Alliance.RED;

        perpNearParkLine = isRed ? new Vector2d(1, 1) : new Vector2d(1, -1);
        perpNearParkLine = perpNearParkLine.div(Math.hypot(perpNearParkLine.x, perpNearParkLine.y));
        perpFarParkLine = isRed ? new Vector2d(-1, 1) : new Vector2d(-1, -1);
        farParkLineOrigin = new Vector2d(48, 0);

        String stringBuilder = customizable.stringBuilder.toLowerCase().replaceAll(" ", "");
        String shotTimes = customizable.shotTimes.replaceAll(" ", "");

        if(stringBuilder.charAt(0) == 'n')
            start = isRed ? createPose(misc.startNearRed) : createPose(misc.startNearBlue);
        else if(stringBuilder.charAt(0) == 'f')
            start = isRed ? createPose(misc.startFarRed) : createPose(misc.startFarBlue);

        // DECLARE POSES=======================
        declareShootPoses();
        declareCollectPoses();
        declareMiscPoses();

        boolean preloadNear = stringBuilder.charAt(0) == 'n';
        if(!PreGameSetupTele.cameraIsReset)
            throw new RuntimeException("Run PreGameSetupTele");
        if (preloadNear)
            Limelight.startingPipeline = Limelight.APRIL_TAG_PIPELINE;
        else
            Limelight.startingPipeline = Limelight.BALL_DETECTION_PIPELINE;
        LimelightLocalization.localizationType = LimelightLocalization.LocalizationType.ON_COMMAND;

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        PreGameSetupTele.cameraIsReset = false; // set to false for next run (tele does not care about this)
        autoCommands = new AutoCommands(robot, telemetry);

        ArrayList<Action> actionOrder = new ArrayList<>();

        if(customizable.shouldColorSort)
            robot.limelight.localization.setState(LimelightLocalization.LocalizationState.SCANNING_MOTIF);

        if(preloadNear)
            preloadShootPose = customizable.shouldColorSort ?
                      (isRed ? createPose(shoot.nearPreloadMotif) : createInvertedPose(shoot.nearPreloadMotif))
                    : (isRed ? createPose(shoot.nearPreload) : createInvertedPose(shoot.nearPreload));
        else {
            String nextLetter = stringBuilder.charAt(1) + "";
            if(nextLetter.equals("l"))
                preloadShootPose = isRed ? createPose(shoot.farPreloadLoading) : createInvertedPose(shoot.farPreloadLoading);
            else
                preloadShootPose = getShootPose(false, nextLetter);
        }
        boolean toNear;
        int numPaths = 0;
        int shotTimeStartI = 0, shotTimeEndI;

        telemetry.addData("ALLIANCE", alliance);
        telemetry.addData("STRING BUILDER", stringBuilder);
        Action bigBoyAutoAction;
        if(!customizable.shouldColorSort) {
            int i = 0;
            String prevCollectLetter = "";
            while (true) {
                numPaths++;
                int shootI;
                for (shootI = i + 1; shootI < stringBuilder.length(); shootI++)
                    if (stringBuilder.charAt(shootI) == 'n' || stringBuilder.charAt(shootI) == 'f')
                        break;
                boolean last = shootI == stringBuilder.length() - 1;
                int nextShootI = -1;
                if (!last)
                    for (nextShootI = shootI + 1; nextShootI < stringBuilder.length(); nextShootI++)
                        if (stringBuilder.charAt(shootI) == 'n' || stringBuilder.charAt(shootI) == 'f')
                            break;

                boolean fromNear = stringBuilder.charAt(i) == 'n';
                toNear = stringBuilder.charAt(shootI) == 'n';

                String collectionData = stringBuilder.substring(i + 1, shootI);
                String collectionLetter = collectionData.substring(0, 1);
                boolean openGate = collectionData.length() > 1 && collectionData.charAt(1) == 'o';

                String nextCollectionData = last ? collectionData : stringBuilder.substring(shootI + 1, nextShootI + 1);
                String nextCollectionLetter = last ? collectionLetter : nextCollectionData.substring(0, 1);

                Pose2d shootPose;
                if (last)
                    shootPose = getFinalShootPose(toNear, collectionLetter);
                else if(toNear)
                    shootPose = getShootPose(true, collectionLetter);
                else {
                    shootPose = getShootPose(collectionLetter, nextCollectionLetter);
                }
                // 0,0,0
                double shotTime = -1;
                for (shotTimeEndI = shotTimeStartI + 1; shotTimeEndI <= shotTimes.length(); shotTimeEndI++) {
                    try {
                        shotTime = Double.parseDouble(shotTimes.substring(shotTimeStartI, shotTimeEndI));
                    } catch (NumberFormatException e) {
                        break;
                    }
                }
                if (shotTime == -1)
                    throw new RuntimeException("shot time is negative 1; startI " + shotTimeStartI + ", endI: " + shotTimeEndI);
                shotTimeStartI = shotTimeEndI;

                boolean initiallyExtake = prevCollectLetter.equals("g") || prevCollectLetter.equals("3") || prevCollectLetter.equals("l") || prevCollectLetter.equals("a");

                telemetry.addLine("Path " + numPaths + ": collect: " + collectionData + " from near: " + fromNear + " to near: " + toNear + ", last: " + last + ", shot time: " + shotTime);
                switch (collectionLetter) {
                    case "1":
                        actionOrder.add(getFirstCollectAndShoot(shootPose, shotTime, fromNear, toNear, openGate, last, initiallyExtake, false));
                        telemetry.addData("   open gate", openGate);
                        break;
                    case "2":
                        actionOrder.add(getSecondCollectAndShoot(shootPose, shotTime, fromNear, toNear, openGate, initiallyExtake, false));
                        telemetry.addData("   open gate", openGate);
                        break;
                    case "3":
                        actionOrder.add(getThirdCollectAndShoot(shootPose, shotTime, fromNear, toNear, last, initiallyExtake, false));
                        break;
                    case "g":
                        boolean shouldGateTap = collectionData.length() > 1 && collectionData.charAt(1) == 't';
                        double waitTime = parseWaitTime(collectionData);
                        actionOrder.add(getGateCollectAndShootNew(shootPose, shotTime, fromNear, toNear, waitTime, shouldGateTap, initiallyExtake));
                        telemetry.addData("   gate tap", shouldGateTap);
                        telemetry.addData("   wait time", waitTime);
                        break;
                    case "l":
                        Action loadingAction;
                        String type = "normal";
                        if (collectionData.length() > 1)
                            switch (collectionData.substring(1, 2)) {
                                case "g":
                                    double gateWaitTime = parseWaitTime(collectionData);
                                    loadingAction = getLoadingGateWaitCollectAndShoot(shootPose, shotTime, gateWaitTime, initiallyExtake);
                                    type = "gate wait";
                                    telemetry.addData("  gate wait time", gateWaitTime);
                                    break;
                                case "c":
                                default:
                                    type = "corner";
                                    loadingAction = getLoadingCornerCollectAndShoot(shootPose, shotTime, initiallyExtake);
                                    break;
                            }
                        else
                            loadingAction = getLoadingCollectAndShoot(shootPose, shotTime, initiallyExtake);
                        telemetry.addData("  loading collect type", type);
                        actionOrder.add(loadingAction);
                        break;
                    case "a":
                        waitTime = parseWaitTime(collectionData);
                        actionOrder.add(getLimelightLoadingZoneCollectAndShoot(shootPose, shotTime, initiallyExtake, waitTime));
                        telemetry.addData("  wait time", waitTime);
                        break;
                }
                if (last)
                    break;
                i = shootI;
                prevCollectLetter = collectionLetter;
            }
            final boolean finalToNear = toNear;
            Action noColorSortAutoAction = new SequentialAction(
                    getPreloadDriveAndShoot(preloadShootPose, stringBuilder.charAt(0) == 'n', false),
                    actionOrder.get(0),
                    numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                    numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                    numPaths > 3 ? actionOrder.get(3) : new SleepAction(0),
                    numPaths > 4 ? actionOrder.get(4) : new SleepAction(0),
                    numPaths > 5 ? actionOrder.get(5) : new SleepAction(0),
                    numPaths > 6 ? actionOrder.get(6) : new SleepAction(0),
                    numPaths > 7 ? actionOrder.get(7) : new SleepAction(0),
                    numPaths > 8 ? actionOrder.get(8) : new SleepAction(0)
            );

            BooleanSupplier nearShouldStop = () -> (autoTimer.seconds() > timeConstraints.nearParkStopTime
                    && (robot.drive.localizer.getPose().position.dot(perpNearParkLine) > misc.smartParkNearDist
                    || autoState == AutoState.DRIVE_TO_COLLECT));
            BooleanSupplier farShouldPark = () -> autoTimer.seconds() > timeConstraints.farParkTime && (robot.drive.localizer.getPose().position.minus(farParkLineOrigin)).dot(perpFarParkLine) < misc.smartParkFarDist;
            BooleanSupplier farShouldStop = () -> (autoState == AutoState.DRIVE_TO_COLLECT || autoState == AutoState.DRIVE_TO_SHOOT)
                    && autoTimer.seconds() > timeConstraints.farParkTime
                    && (robot.drive.localizer.getPose().position.minus(farParkLineOrigin)).dot(perpFarParkLine) > misc.smartParkFarDist;

            Action smartParkAutoAction = new SequentialAction(
                    new ParallelAction(
                            new CustomEndAction(noColorSortAutoAction, () -> shouldStop[0] || shouldPark[0])
                                    .setEndFunction(() -> autoActionDone[0] = true),
                            new CustomEndAction(
                                    new ParallelAction(
                                            new CustomEndAction(() -> (nearShouldStop.getAsBoolean() && finalToNear) || (farShouldStop.getAsBoolean() && !finalToNear))
                                                    .setEndFunction(() -> shouldStop[0] = true),
                                            new CustomEndAction(() -> !finalToNear && farShouldPark.getAsBoolean())
                                                    .setEndFunction(() -> shouldPark[0] = true)
                                    ),
                                    () -> !customizable.smartPark || autoActionDone[0] || shouldStop[0] || shouldPark[0]
                            )
                    ),
                    new ParallelAction(
                            new CustomEndAction(() -> shouldStop[0] || shouldPark[0] || autoActionDone[0]),
                            new InstantAction(robot.drive::stop)
                    ),
                    new Action() {
                        boolean first = true;
                        Action action;

                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            if (first) {
                                if (shouldPark[0] || (!shouldStop[0] && !finalToNear))
                                    action = getFarParkDrive();
                                else
                                    action = telemetryPacket1 -> {
                                        robot.drive.stop();
                                        return true;
                                    };
                                first = false;
                            }
                            robot.led.setAutoDone();
                            return action.run(telemetryPacket);
                        }
                    }
            );
            Action fullAutoAction = new ParallelAction(
                    autoCommands.updateRobotInfo(),
                    smartParkAutoAction,
                    autoCommands.updateRobot(),
                    autoCommands.savePoseContinuously(),
                    packet -> {
                        DrivePath.drawCurrentPath(packet.fieldOverlay());
                        robot.drawRobotInfo(packet.fieldOverlay());
                        telemetry.addData("auto state", autoState);
                        telemetry.addData("auto timer", autoTimer.seconds());
                        if(BrainSTEMTeleOp.printShooter)
                            robot.shooter.printInfo();
                        if(BrainSTEMTeleOp.printShootingSystem)
                            robot.shootingSystem.printInfo(telemetry);
                        if(BrainSTEMTeleOp.printTurret)
                            robot.turret.printInfo();
                        if(BrainSTEMTeleOp.printCollector)
                            robot.collector.printInfo();
//                    telemetry.addData("SHOULD PARK", shouldPark[0]);
//                    telemetry.addData("SHOULD STOP", shouldStop[0]);
//                    telemetry.addData("AUTO ACTION DONE", autoActionDone[0]);
//                    robot.limelight.printInfo();
//                    robot.turret.printInfo();
                        telemetry.update();
                        return true;
                    }
            );
            bigBoyAutoAction = new CustomEndAction(fullAutoAction, () -> autoTimer.seconds() > timeConstraints.stopAllTime);

        }
        else
            bigBoyAutoAction = new Action() {
            private boolean isFirst = true;
            private Action preloadAction = null;
            private boolean preloadRunning;
            private Action[] dynamicActions = null;
            private int dynamicActionI = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(isFirst) {
                    preloadAction = getPreloadDriveAndShoot(preloadShootPose, true, true);
                    isFirst = false;
                    preloadRunning = true;
                }
                if(preloadRunning) {
                    robot.updateInfo();
                    preloadRunning = preloadAction.run(telemetryPacket);
                    updateEverythingElse();
                }
                else {
                    if(dynamicActions == null) {
                        dynamicActions = new Action[4];
                        switch(targetGreenPos) {
                            case -1:
                            case 0:
                                dynamicActions[0] = getThirdCollectAndShoot(getShootPose(true, "3"), -1, true, true, false, false, false);
                                dynamicActions[1] = getSecondCollectAndShoot(getShootPose(true, "2"), -1, true, true, false, false, true);
                                dynamicActions[2] = getFirstCollectAndShoot(getShootPose(true, "2"), -1, true, true, false, false, false, true);
                                break;
                            case 1:
                                dynamicActions[0] = getSecondCollectAndShoot(getShootPose(true, "2"), -1, true, true, false, false, false);
                                dynamicActions[1] = getThirdCollectAndShoot(getShootPose(true, "3"), -1, true, true, false, false, true);
                                dynamicActions[2] = getFirstCollectAndShoot(getShootPose(true, "2"), -1, true, true, false, false, false, true);
                                break;
                            case 2:
                                dynamicActions[0] = getFirstCollectAndShoot(getShootPose(true, "2"), -1, true, true, false, false, false, false);
                                dynamicActions[1] = getSecondCollectAndShoot(getShootPose(true, "3"), -1, true, true, false, false, true);
                                dynamicActions[2] = getThirdCollectAndShoot(getShootPose(true, "3"), -1, true, true, false, false, true);
                                break;
                        }
                        dynamicActions[3] = new ParallelAction(
                                getGateParkDrive(),
                                new InstantAction(() -> robot.led.setAutoDone())
                        );
                    }
                    robot.updateInfo();
                    boolean actionRunning = dynamicActions[dynamicActionI].run(telemetryPacket);
                    if(!actionRunning) {
                        dynamicActionI++;
                        if(dynamicActionI >= dynamicActions.length)
                            return false;
                    }
                    updateEverythingElse();
                }
                return true;
            }
            private void updateEverythingElse() {
                robot.update();
                PoseStorage.autoX = robot.drive.pinpoint().getPose().position.x;
                PoseStorage.autoY = robot.drive.pinpoint().getPose().position.y;
                PoseStorage.autoHeading = robot.drive.pinpoint().getPose().heading.toDouble();
                TelemetryPacket packet = new TelemetryPacket();
                DrivePath.drawCurrentPath(packet.fieldOverlay());
                robot.drawRobotInfo(packet.fieldOverlay());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                robot.limelight.printInfo();
                telemetry.addData("auto state", autoState);
                telemetry.addData("auto timer", autoTimer.seconds());
                telemetry.addData("TARGET GREEN POS", targetGreenPos);
                telemetry.update();
            }
        };

        robot.shootingSystem.resetTurretEncoder();
        robot.turret.setSmoothWhenOutOfRange(false);
        robot.collector.setInAuto(true);
        telemetry.addLine("READY TO RUN");
        telemetry.update();

        waitForStart();
        robot.startOpmode();
        autoTimer.reset();

        Actions.runBlocking(bigBoyAutoAction);
    }
    private Pose2d getShootPose(boolean shootClose, String letter) {
        switch (letter) {
            case "1": return shootClose ? shoot1Near : shoot1Far;
            case "2": return shootClose ? shoot2Near : shoot2Far;
            case "t":
            case "g": return shootClose ? shootGateNear : shootGateFar;
            case "3": return shootClose ? shoot3Near : shoot3Far;
            case "l": return shootClose ? shootLoadingNear : shootLoadingFar;
            case "a": return shootLoadingFar;
            default: throw new IllegalArgumentException("invalid shooter letter of " + letter + " in " + customizable.stringBuilder);
        }
    }
    private Pose2d getShootPose(String letter, String nextCollectionLetter) {
        switch (nextCollectionLetter) {
            case "3": return shoot3Far;
            case "l":
            case "a":
                return letter.equals("l") || letter.equals("a") ? shootLoadingOptimizedFar : shootLoadingFar;
            default: throw new IllegalArgumentException("invalid shooter letter of " + letter + " in " + customizable.stringBuilder);
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
        boolean shortPark = customizable.stringBuilder.equals(customizable.far18);
        Pose2d parkPose = shortPark ? parkFar : hpParkFar;
        double maxTime = shortPark ? .8 : 2; // TODO: find this max time
        return new DrivePath(robot.drive,
                new Waypoint(parkPose).setPassPosition(true).setMinLinearPower(misc.minParkFarPower).setMaxTime(maxTime)
        );
    }
    private Action getGateParkDrive() {
        return new DrivePath(robot.drive,
                new Waypoint(gatePark));
    }
    private Action getPreloadDriveAndShoot(Pose2d shootPose, boolean shootingNear, boolean colorSorting) {
        Waypoint preloadShootWaypoint = new Waypoint(shootPose)
                .setPassPosition(true);
        if(shootingNear)
            preloadShootWaypoint
                    .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                    .setMaxTime(colorSorting ? 3 : 2.5)
                    .setMinLinearPower(shoot.nearMinDrivePower1);
        else
            preloadShootWaypoint.setMaxTime(.9);
        DrivePath preloadShootDrive = new DrivePath(robot.drive, telemetry, preloadShootWaypoint);

        Action preloadDriveAction;
        if(shootingNear)
            preloadDriveAction = new ParallelAction(
                    preloadShootDrive,
                    new SequentialAction(
                            new CustomEndAction(() -> preloadShootDrive.getWaypointDistanceError() < shoot.nearMinDrivePower2Dist),
                            new InstantAction(() -> preloadShootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
                    )
            );
        else
            preloadDriveAction = new SequentialAction(
                    new SleepAction(timeConstraints.farPreloadDriveDelay),
                    preloadShootDrive
            );

        boolean shootingEarly = shootingNear && !colorSorting;
        return new SequentialAction(
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                new ParallelAction(
                        autoCommands.flickerHalfUp(),
                        autoCommands.speedUpShooter(),
                        (colorSorting ?
                                new SequentialAction(
                                        autoCommands.enableCustomTurretTracking((isRed ? 1 : -1) * misc.motifScanTurretRelAngle),
                                        new TimedAction(telemetryPacket -> {
                                            if(robot.limelight.localization.foundMotif())
                                                targetGreenPos = robot.limelight.localization.getTargetGreenPos();
                                            return targetGreenPos == -1;
                                        }, timeConstraints.maxMotifScanTime),
                                        autoCommands.enableTurretTracking()
                                )
                                 : autoCommands.enableTurretTracking()
                        ),
                        preloadDriveAction,
                        shootingEarly ? getShootAction(3, timeConstraints.nearPostFlickerShootTime) : new SleepAction(0)
                ),
                !shootingEarly ? getShootAction(colorSorting ? timeConstraints.colorSortShooterInterlockMaxWait : 0, timeConstraints.farPostFlickerShootTime)
                        : new SleepAction(0)
        );
    }
    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, DrivePath shootDrive, boolean shootingNear, double postIntakeTime, boolean runIntake, boolean notLast, double shotTime, boolean initiallyExtake, boolean shootingPurple, int spikeMark) {
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
        boolean shootEarly = (shootingNear && notLast && shotTime == 0) || shootingPurple;
        return new SequentialAction(
                autoCommands.setMaxVoltage(shootingNear ? shoot.maxNearShooterVoltage : Shooter.shooterParams.maxVoltage),
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
                new ParallelAction(
                        new CustomEndAction(collectDrive, robot.collector::has3Balls),
                        new SequentialAction(
                                initiallyExtake ? new TimedAction(autoCommands.reverseIntake(), .15) : new SleepAction(0),
                                autoCommands.disengageClutch(),
                                runIntake ? new SequentialAction(new SleepAction(initiallyExtake ? .15 : .3), autoCommands.runIntake()) : new SleepAction(0)
                        )
                ),
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.OPEN_GATE),
                                        gateDrive
                                ),
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                                        new CustomEndAction(shootDriveAction, () -> autoState == AutoState.SHOOT)
                                                .setEndFunction(robot.drive::stop),
                                        autoCommands.setMaxVoltage(Shooter.shooterParams.maxVoltage)
                                ),
                                new ParallelAction(
                                        new InstantAction(() -> autoState = AutoState.SHOOT),
                                        new InstantAction(() -> robot.drive.stop())
                                )
                        ),
                        new SequentialAction(
                                new SleepAction(postIntakeTime),
                                autoCommands.stopIntake(),

                                shootEarly ? new SequentialAction(
                                        new CustomEndAction(telemetryPacket -> autoState != AutoState.SHOOT, () -> {
                                            double dot = robot.drive.localizer.getPose().position.dot(perpNearParkLine);
                                            return dot < shoot.earlyEngageClutchDist;
                                        }),
                                        shootingPurple ? getPurpleOnlyShootAction(timeConstraints.shooterInterlockMaxWait, spikeMark) : getShootAction(timeConstraints.shooterInterlockMaxWait, timeConstraints.nearPostFlickerShootTime)
                                ) : new SleepAction(0)
                        )
                ),
                shootEarly ? new SleepAction(0) : getShootAction(
                        timeConstraints.shooterInterlockMaxWait,
                        shootingNear ?
                                (notLast ? timeConstraints.nearPostFlickerShootTime : timeConstraints.nearLastShootExtraTime)
                                : timeConstraints.farPostFlickerShootTime)
        );
    }
    private Action getShootAction(double shooterInterlockMaxTime, double postFlickWaitTime) {
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> autoState = AutoState.SHOOT),
                        autoCommands.stopIntake(),
                        autoCommands.engageClutch(),
                        new SequentialAction(
                                new CustomEndAction(new SleepAction(shooterInterlockMaxTime), () -> robot.shootingSystem.shooterFirstGood() && robot.turret.onTarget()),
                                new ParallelAction(
                                        autoCommands.runIntake(),
                                        new CustomEndAction(new SleepAction(timeConstraints.maxShootTime), () -> robot.shooter.ballsDoneExiting() || robot.shooter.getNumBallsShot() == 3)
                                )
                        )
                ),
                new ParallelAction(
                        autoCommands.flickerUp(),
                        new SleepAction(postFlickWaitTime)
                ),
                autoCommands.stopIntake()
        );
    }
    private Action getPurpleOnlyShootAction(double shooterInterlockMaxTime, int spikeMark) {
        int ballToMiss = 3 - spikeMark;
        return new SequentialAction(
                new ParallelAction(
                        new ParallelAction(
                                new InstantAction(() -> autoState = AutoState.SHOOT),
                                autoCommands.stopIntake(),
                                autoCommands.engageClutch(),
                                ballToMiss == 0 ? autoCommands.setShouldScore(false) : new SleepAction(0)
                        ),
                        new SequentialAction(
                                new CustomEndAction(new SleepAction(shooterInterlockMaxTime), () -> robot.shootingSystem.shooterFirstGood() && robot.turret.onTarget()),
                                new ParallelAction(
                                        autoCommands.runIntake(),
                                        new TimedAction(new CustomEndAction(() -> robot.shooter.getNumBallsShot() == ballToMiss), ballToMiss == 1 ? timeConstraints.shoot1FirstTime : timeConstraints.shoot2FirstTime)
                                ),
                                new ParallelAction(
                                        autoCommands.stopIntake(),
                                        autoCommands.setShouldScore(false),
                                        new SleepAction(ballToMiss == 0 ? 0 : timeConstraints.missBallAdjustTime)
                                ),
                                new ParallelAction(
                                        autoCommands.runIntake(),
                                        new TimedAction(new CustomEndAction(() -> robot.shooter.getNumBallsShot() == 1), timeConstraints.shoot1SecondTime)
                                ),
                                new ParallelAction(
                                        autoCommands.setShouldScore(true),
                                        autoCommands.stopIntake(),
                                        new SleepAction(ballToMiss == 2 ? 0 : timeConstraints.missBallAdjustTime)
                                ),
                                new ParallelAction(
                                        autoCommands.runIntake(),
                                        new TimedAction(new CustomEndAction(() -> robot.shooter.getNumBallsShot() == 2 - ballToMiss), timeConstraints.shootThirdTime)
                                )

                        )
                ),
                autoCommands.flickerUp(),
                new SleepAction(timeConstraints.nearPostFlickerShootTime),
                autoCommands.stopIntake()
        );
    }
    private Action getFirstCollectAndShoot(Pose2d shootPose, double shotTime, boolean fromNear, boolean toNear, boolean openGate, boolean last, boolean initiallyExtake, boolean shootingPurple) {
        DrivePath firstCollectDrive;
        if(fromNear) {
            firstCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect1, new BoxTolerance(0.75, 2, Math.toRadians(3)))
                    .setFixedLinearPower(collect.collectDrivePower)
                    .setMaxTime(1.4)
                    .setControlPoint(collect1NearControlPoint, collect.firstNearT1, collect.firstNearT2)
                    .setCustomEndCondition(() -> robot.collector.has3Balls())
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
                            .setCustomEndCondition(() -> robot.collector.has3Balls()));
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

        double sign = isRed ? 1 : -1;
        Pose2d preGate1Pose = new Pose2d(gate1.position.x, gate1.position.y - sign * misc.gate1BackupDist, gate1.heading.toDouble());

        Action firstGateDrive = openGate ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGate1Pose, new BoxTolerance(1, 1, Math.toRadians(10))).
                                        setMinLinearPower(misc.gatePrepMinPower)
                                        .setPassPosition(true),
                                new Waypoint(gate1)
                                        .setMinLinearPower(misc.gateMinPower)
                                        .setPassPosition(true)),
                        new SleepAction(timeConstraints.spikeGateOpeningWait)
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

        return buildCollectAndShoot(firstCollectAction, firstGateDrive, firstShootDrive, toNear, timeConstraints.postIntakeTime, true, !last, shotTime, initiallyExtake, shootingPurple, 1);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, double shotTime, boolean fromNear, boolean toNear, boolean openGate, boolean initiallyExtake, boolean shootingPurple) {
        DrivePath secondCollectDrive;
        if(fromNear) {
            Pose2d second = openGate ? collect2IfOpenGate : collect2;
            BoxTolerance near2WaypointTol = new BoxTolerance(collect.secondNearWaypointTol);
            secondCollectDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(collect2NearWaypoint, near2WaypointTol)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.collectDrivePower),
                    new Waypoint(second)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.secondCollectDrivePower)
                            .setControlPoint(collect2NearControlPoint, collect.secondNearT1, collect.secondNearT2)
                            .setCloseHeadingKP(collect.secondNearCloseHeadingKP));
        }
        else {
            throw new RuntimeException("second collect not tuned from far; cannot incorporate 2f into string builder");
        }
//        double sign = isRed ? -1 : 1;
//        Pose2d preGatePose = new Pose2d(gate2.position.x, gate2.position.y + sign * misc.gate2BackupDist, gate2.heading.toDouble());

        Action gateOpen = new CustomEndAction(
                new DrivePath(robot.drive, telemetry,
//                        new Waypoint(preGatePose).setMinLinearPower(misc.gatePrepMinPower).setPassPosition(true),
                        new Waypoint(gate2).setMinLinearPower(misc.gateMinPower).setPassPosition(true).setMinTime(.15)),
                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold)
                .setEndFunction(robot.drive::stop);
        Action secondGateDrive = openGate ?
                new SequentialAction(
                        gateOpen,
                        new SleepAction(timeConstraints.spikeGateOpeningWait)
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

        return buildCollectAndShoot(secondCollectDrive, secondGateDrive, secondShootDrive, toNear, timeConstraints.postIntakeTime, true, true, shotTime, initiallyExtake, shootingPurple, 2);
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose, double shotTime, boolean fromNear, boolean toNear, boolean last, boolean initiallyExtake, boolean shootingPurple) {
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
            thirdCollectDrive.addWaypoint(new Waypoint(collect3NearCleanup)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.thirdCollectDrivePower));
        }
        else {
            BoxTolerance preCollect3FarTol = new BoxTolerance(collect.thirdFarWaypointTol[0], collect.thirdFarWaypointTol[1], Math.toRadians(collect.thirdFarWaypointTol[2]));
            Waypoint preCollectFar = new Waypoint(preCollect3Far, preCollect3FarTol)
                    .setPassPosition(true)
                    .setMinLinearPower(collect.collectDrivePower);
            thirdCollectDrive.addWaypoint(preCollectFar, 0);
        }
        double controlY = fromNear ? collect3NearControlPoint.position.y : preCollect3Far.position.y;
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

        return buildCollectAndShoot(thirdCollectAction, new SleepAction(0), thirdShootDrive, toNear, timeConstraints.postIntakeTime, true, !last, shotTime, initiallyExtake, shootingPurple, 3);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, double shotTime, boolean initiallyExtake) {
        CircleTolerance preLoadingTol = new CircleTolerance(collect.preLoadingWaypointTol);
        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoadingWaypoint, preLoadingTol)
                        .setMaxTime(1.3)
                        .setPassPosition(true)
                        .setMinLinearPower(collect.loadingNormDrivePower),
                new Waypoint(preLoading, preLoadingTol)
                        .setMaxTime(1)
                        .setPassPosition(true)
                        .setLateralAxialWeights(3, 1),
                new Waypoint(loadingWaypoint)
                        .setFixedLinearPower(collect.loadingSlowDrivePower)
                        .setMaxTime(.9),
                new Waypoint(postLoading)
                        .setMinLinearPower(collect.loadingSlowDrivePower)
                        .setMinHeadingPower(collect.loadingMinHeadingPower)
                        .setMaxTime(.65),
                new Waypoint(postLoading2)
                        .setMinLinearPower(collect.loadingNormDrivePower)
                        .setMaxTime(.6));
        Waypoint w = new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true)
                .setCloseHeadingKP(shoot.loadingShootCloseHeadingKP);
        w.setControlPoint(shootLoadingFarControlPoint, shoot.farLoadingT1, shoot.farLoadingT2);
        DrivePath loadingShootDrive = new DrivePath(robot.drive, w);

        return buildCollectAndShoot(loadingCollectDrive, new SleepAction(0), loadingShootDrive, false, timeConstraints.loadingSlowIntakeTime, true, false, shotTime, initiallyExtake, false, 0);
    }
    private Action getLoadingCornerCollectAndShoot(Pose2d shootPose, double shotTime, boolean initiallyExtake) {
        double offset = isRed ? collect.loadingCornerBackup : -collect.loadingCornerBackup;
        BooleanSupplier hitBall = () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitBallVelThreshold && Math.abs(robot.drive.localizer.getPose().position.y) > 43;
        DrivePath collectDrive = new DrivePath(robot.drive, telemetry,
                new Waypoint(loadingCorner)
                        .setMaxTime(1.7)
                        .setMinLinearPower(collect.loadingNormDrivePower)
                        .setPassPosition(true)
                        .setControlPoint(loadingCornerControlPoint, collect.loadingCornerT1, collect.loadingCornerT2)
                        .setCustomEndCondition(hitBall));
        Action collectAction = new SequentialAction(
                collectDrive,
                new Action() {
                    Pose2d backup = null;
                    DrivePath retryDrive;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if(backup == null) {
                            backup = new Pose2d(loadingCorner.position.x, robot.drive.localizer.getPose().position.y - offset, loadingCorner.heading.toDouble());
                            retryDrive = new DrivePath(robot.drive,
                                    new Waypoint(backup)
                                            .setMinLinearPower(collect.loadingNormDrivePower)
                                            .setMaxTime(.8)
                                            .setPassPosition(true),
                                    new Waypoint(loadingCorner)
                                            .setMaxTime(1)
                                            .setMinLinearPower(collect.loadingNormDrivePower)
                                            .setPassPosition(true));
                        }
                        return retryDrive.run(telemetryPacket);
                    }
                }
        );

        DrivePath shootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true));
        return buildCollectAndShoot(collectAction, new SleepAction(0), shootDrive, false, timeConstraints.loadingSlowIntakeTime, true, true, shotTime, initiallyExtake, false, 0);
    }
    private Action getLoadingGateWaitCollectAndShoot(Pose2d shootPose, double shotTime, double gateWaitTime, boolean initiallyExtake) {
        BoxTolerance loadingGateWaitWaypointTol = new BoxTolerance(collect.loadingGateWaitWaypointTol);
        DrivePath loadingGateWaitCollectDrive = new DrivePath(robot.drive,
                new Waypoint(loadingGateWaitWaypoint, loadingGateWaitWaypointTol)
                        .setMinLinearPower(collect.loadingNormDrivePower)
                        .setPassPosition(true)
                        .setMaxTime(2)
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                        .setMaxHeadingPower(.8),
                new Waypoint(loadingGateWait)
                        .setMaxTime(1));
        Action loadingGateWaitCollectAction = new SequentialAction(
                loadingGateWaitCollectDrive,
                new SleepAction(gateWaitTime)
        );

        DrivePath shootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true));

        return buildCollectAndShoot(loadingGateWaitCollectAction, new SleepAction(0), shootDrive, false, timeConstraints.loadingSlowIntakeTime, true, true, shotTime, initiallyExtake, false, 0);
    }

    protected Action getLimelightLoadingZoneCollectAndShoot(Pose2d shootPose, double shotTime, boolean initiallyExtake, double waitTime) {
        Vector2d scan1 = alliance == Alliance.RED ?
                createVec(collect.limelightScanPos1) :
                createInvertedVec(collect.limelightScanPos1);
        Vector2d scan2 = alliance == Alliance.RED ?
                createVec(collect.limelightScanPos2) :
                createInvertedVec(collect.limelightScanPos2);

        Action limelightCollectAction = new SequentialAction(
                new SleepAction(waitTime),
                robot.getLimelightCollectSequence(scan1, scan2, timeConstraints.maxLimelightWait)
        );

        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3)
                .setPassPosition(true)
                .setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT)
                .setHeadingTangentDeactivateThreshold(shoot.limelightHeadingTangentDeactivateDist));

        return buildCollectAndShoot(limelightCollectAction, new SleepAction(0), loadingShootDrive, false, timeConstraints.loadingSlowIntakeTime, true, true, shotTime, initiallyExtake, false, 0);
    }

    private Action getGateCollectAndShoot(Pose2d shootPose, double shotTime, boolean fromNear, boolean toNear, double waitTime, boolean shouldGateTap, boolean initiallyExtake) {
        DrivePath gateOpenDrive;
        if (fromNear) {
            BoxTolerance waypointTol = new BoxTolerance(collect.gateNearWaypointTol[0], collect.gateNearWaypointTol[1], Math.toRadians(collect.gateNearWaypointTol[2]));
            BoxTolerance openTol = new BoxTolerance(collect.gateOpenTol);
            gateOpenDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(gateCollectNearWaypoint, waypointTol)
                            .setMaxTime(1)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1MinPower),
                    new Waypoint(gateCollectNearWaypoint2, waypointTol)
                            .setMaxTime(.9)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1MinPower),
                    new Waypoint(gateCollectOpen, openTol)
                            .setMaxTime(.8)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive2MinPower)
            );
        }
        else {
            gateOpenDrive = new DrivePath(robot.drive,
                    new Waypoint(gateCollectOpen)
                            .setMaxTime(2)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1MinPower)
                            .setControlPoint(gateCollectFarControlPoint, collect.gateCollectOpenFarTStartError, collect.gateCollectOpenFarTFinishError)
            );
        }

        CustomEndAction gateOpenCustomEndAction = new CustomEndAction(gateOpenDrive,
                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold && Math.abs(robot.drive.localizer.getPose().position.y) > 45);
        Action gateOpenAction = new ParallelAction(
                new SequentialAction(
                        new SleepAction(.3),
                        autoCommands.runIntake()
                ),
                gateOpenCustomEndAction,
                new SequentialAction(
                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > 40),
                        new InstantAction(() -> gateOpenDrive.getCurWaypoint()
                                .setMinLinearPower(collect.gateOpenDrive2MinPower)
                                .setMaxLinearPower(collect.gateOpenDrive2MaxPower))
                )
        );
        DrivePath gateOpenHold = new DrivePath(robot.drive, new Waypoint(gateCollectOpenHold, new BoxTolerance(.1, .1, Math.toRadians(.1))).setMaxLinearPower(.2));

        DrivePath postGateOpenDrive = new DrivePath(robot.drive, new Waypoint(gateCollect)
                .setPassPosition(true)
                .setMaxTime(1));

        DrivePath gateTapDrive = new DrivePath(robot.drive,
                new Waypoint(gateTapBackup)
                        .setPassPosition(true)
                        .setMinLinearPower(misc.gateMinPower)
                        .setMaxTime(.9),
                new Waypoint(gateTap)
                        .setPassPosition(true)
                        .setMinLinearPower(misc.gateMinPower)
                        .setMaxTime(.9));
        DrivePath gateTapHold = new DrivePath(robot.drive, new Waypoint(gateTap, new BoxTolerance(.1, .1, .1)).setMaxLinearPower(.2));

        Action gateTapAction = new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                new TimedAction(new CustomEndAction(robot.collector::has3Balls), .5),
                                autoCommands.stopIntake()
                        ),
                        new SequentialAction(
                                new CustomEndAction(gateTapDrive,
                                        () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold
                                                && robot.drive.localizer.getPose().position.x < gateTapBackup.position.x),
                                new TimedAction(gateTapHold, timeConstraints.gateTapWait)
                        )
                )
        );


        Action completeGateCollectDrive = new SequentialAction(
                autoCommands.stopIntake(),
                new SleepAction(waitTime),
                gateOpenAction,
                new CustomEndAction(new SequentialAction(
                        new CustomEndAction(new TimedAction(gateOpenHold, timeConstraints.gateCollectOpenWait), robot.collector::jammed),
                        postGateOpenDrive,
                        new SleepAction(timeConstraints.gateCollectMaxTime)
                ), robot.collector::has3Balls),
                shouldGateTap ? gateTapAction : new SleepAction(0)
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

        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, timeConstraints.postIntakeTime, false, true, shotTime, initiallyExtake, false, 0);

    }

    private Action getGateCollectAndShootNew(Pose2d shootPose, double shotTime, boolean fromNear, boolean toNear, double waitTime, boolean shouldGateTap, boolean initiallyExtake) {
        DrivePath gateOpenDrive;
        if (fromNear) {
            BoxTolerance waypointTol = new BoxTolerance(collect.gateNearWaypointTol[0], collect.gateNearWaypointTol[1], Math.toRadians(collect.gateNearWaypointTol[2]));
            BoxTolerance openTol = new BoxTolerance(collect.gateOpenTol);
            Pose2d waypoint1 = shouldGateTap ?
                    new Pose2d(collect.gateTapNearWaypointX, gateCollectNearWaypoint.position.y, gateCollectNearWaypoint.heading.toDouble())
                    : gateCollectNearWaypoint;
            Pose2d waypoint2 = shouldGateTap ?
                    new Pose2d(collect.gateTapNearWaypoint2X, gateCollectNearWaypoint2.position.y, gateCollectNearWaypoint2.heading.toDouble())
                    : gateCollectNearWaypoint2;
            gateOpenDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(waypoint1, waypointTol)
                            .setMaxTime(1)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1MinPower),
                    new Waypoint(waypoint2, waypointTol)
                            .setMaxTime(.9)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1MinPower),
                    new Waypoint(shouldGateTap ? gateTap : gateCollectOpen, openTol)
                            .setMaxTime(.8)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive2MinPower)
            );
        }
        else {
            gateOpenDrive = new DrivePath(robot.drive,
                    new Waypoint(gateCollectOpen)
                            .setMaxTime(2)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.gateOpenDrive1MinPower)
                            .setControlPoint(gateCollectFarControlPoint, collect.gateCollectOpenFarTStartError, collect.gateCollectOpenFarTFinishError)
            );
        }

        CustomEndAction gateOpenCustomEndAction = new CustomEndAction(gateOpenDrive,
                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold && Math.abs(robot.drive.localizer.getPose().position.y) > 45);
        Action gateOpenAction = new ParallelAction(
                new SequentialAction(
                        new SleepAction(.3),
                        autoCommands.runIntake()
                ),
                gateOpenCustomEndAction,
                new SequentialAction(
                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > 40),
                        new InstantAction(() -> gateOpenDrive.getCurWaypoint()
                                .setMinLinearPower(collect.gateOpenDrive2MinPower)
                                .setMaxLinearPower(collect.gateOpenDrive2MaxPower))
                )
        );
        DrivePath gateOpenHold = new DrivePath(robot.drive, new Waypoint(shouldGateTap ? gateTap : gateCollectOpenHold, new BoxTolerance(.1, .1, Math.toRadians(.1))).setMaxLinearPower(.2));


        Action postGateOpenAction;
        if(shouldGateTap)
            postGateOpenAction = new SequentialAction(
                    new DrivePath(robot.drive, new Waypoint(gateCollectOpen, new BoxTolerance(.1, Math.toRadians(3)))
                            .setPassPosition(true)
                            .setMaxTime(1)
                            .setMinLinearPower(collect.postGateTapSetupGateLinearPower)
                            .setMinHeadingPower(collect.postGateTapSetupGateHeadingPower)),
                    new SleepAction(timeConstraints.gateCollectOpenWait),
                    new DrivePath(robot.drive, new Waypoint(gateCollect)
                            .setPassPosition(true)
                            .setMaxTime(1)));
        else
            postGateOpenAction = new DrivePath(robot.drive, new Waypoint(gateCollect)
                    .setPassPosition(true)
                    .setMaxTime(1));

        Action completeGateCollectDrive = new SequentialAction(
                autoCommands.stopIntake(),
                new SleepAction(waitTime),
                gateOpenAction,
                new SequentialAction(
                        shouldGateTap ? new SleepAction(timeConstraints.gateTapWait)
                        : new CustomEndAction(new TimedAction(gateOpenHold, timeConstraints.gateCollectOpenWait), robot.collector::jammed),
                        postGateOpenAction,
                        new SleepAction(timeConstraints.gateCollectMaxTime)
                )
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

        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, timeConstraints.postIntakeTime, false, true, shotTime, initiallyExtake, false, 0);

    }


    private double parseWaitTime(String data) {
        double waitTime = 0;
        for(int j = 1; j < data.length(); j++) {
            try {
                waitTime = Double.parseDouble(data.substring(j));
                break;
            }
            catch(NumberFormatException ignored) {}
        }
        return waitTime;
    }
    protected void declareShootPoses() {
        shoot1Near = isRed ? createPose(shoot.near1) : createInvertedPose(shoot.near1);
        shoot1Far = isRed ? createPose(shoot.farSpike) : createInvertedPose(shoot.farSpike);
        shoot2Near = isRed ? createPose(shoot.near2) : createInvertedPose(shoot.near2);
        shoot2Far = shoot1Far;
        shootGateNear = isRed ? createPose(shoot.nearGate) : createInvertedPose(shoot.nearGate);
        shootGateNearControlPoint = isRed ? createPose(shoot.gateNearControlPoint) : createInvertedPose(shoot.gateNearControlPoint);
        shootGateFar = shoot2Far;
        shootGateFarControlPoint = isRed ? createPose(shoot.gateFarControlPoint) : createInvertedPose(shoot.gateFarControlPoint);
        shoot3Near = isRed ? createPose(shoot.near3) : createInvertedPose(shoot.near3);
        shoot3Far = shoot1Far;

        shootLoadingNear = shoot3Near;
        shootLoadingFar = isRed ? createPose(shoot.farLoading) : createInvertedPose(shoot.farLoading);
        shootLoadingOptimizedFar = isRed ? createPose(shoot.farLoadingOptimized) : createInvertedPose(shoot.farLoadingOptimized);
        shootLoadingFarControlPoint = isRed ? createPose(shoot.farLoadingControlPoint) : createInvertedPose(shoot.farLoadingControlPoint);

        shoot1FarControlPoint = isRed ? createPose(shoot.far1ControlPoint) : createInvertedPose(shoot.far1ControlPoint);
        shoot2FarControlPoint = isRed ? createPose(shoot.far2ControlPoint) : createInvertedPose(shoot.far2ControlPoint);
    }
    protected void declareCollectPoses() {
        collect1 = isRed ?
                createPose(collect.first) :
                createInvertedPose(collect.first);
        collect1NearControlPoint = isRed ?
                createPose(collect.firstControlPointNear) :
                createInvertedPose(collect.firstControlPointNear);
        collect1FarControlPoint = isRed ?
                createPose(collect.firstControlPointFar) :
                createInvertedPose(collect.firstControlPointFar);

        collect2 = isRed ?
                createPose(collect.second) :
                createInvertedPose(collect.second);
        collect2IfOpenGate = isRed ?
                createPose(collect.secondIfOpenGate) :
                createInvertedPose(collect.secondIfOpenGate);
        collect2NearWaypoint = isRed ?
                createPose(collect.secondNearWaypoint) :
                createInvertedPose(collect.secondNearWaypoint);
        collect2NearControlPoint = isRed ?
                createPose(collect.secondNearControlPoint) :
                createInvertedPose(collect.secondNearControlPoint);

        collect3 = isRed ?
                createPose(collect.third) :
                createInvertedPose(collect.third);
        preCollect3Near = isRed ?
                createPose(collect.thirdNearWaypoint) :
                createInvertedPose(collect.thirdNearWaypoint);
        preCollect3Far = isRed ?
                createPose(collect.thirdFarWaypoint) :
                createInvertedPose(collect.thirdFarWaypoint);
        collect3FarControlPoint = isRed ?
                createPose(collect.thirdFarControlPoint) :
                createInvertedPose(collect.thirdFarControlPoint);
        collect3NearControlPoint = isRed ?
                createPose(collect.thirdNearControlPoint) :
                createInvertedPose(collect.thirdNearControlPoint);
        collect3NearCleanup = isRed ?
                createPose(collect.thirdNearCleanup) :
                createInvertedPose(collect.thirdNearCleanup);

        preLoadingWaypoint = isRed ?
                createPose(collect.preLoadingWaypoint) :
                createInvertedPose(collect.preLoadingWaypoint);
        preLoading = isRed ?
                createPose(collect.preLoading) :
                createInvertedPose(collect.preLoading);
        loadingWaypoint = isRed ?
                createPose(collect.loadingWaypoint) :
                createInvertedPose(collect.loadingWaypoint);
        postLoading = isRed ?
                createPose(collect.postLoading) :
                createInvertedPose(collect.postLoading);
        postLoading2 = isRed ?
                createPose(collect.postLoading2) :
                createInvertedPose(collect.postLoading2);
        loadingCornerControlPoint = isRed ?
                createPose(collect.loadingCornerControlPoint) :
                createInvertedPose(collect.loadingCornerControlPoint);
        loadingCorner = isRed ?
                createPose(collect.loadingCorner) :
                createInvertedPose(collect.loadingCorner);
        loadingGateWaitWaypoint = isRed ?
                createPose(collect.loadingGateWaitWaypoint) :
                createInvertedPose(collect.loadingGateWaitWaypoint);
        loadingGateWait = isRed ?
                createPose(collect.loadingGateWait) :
                createInvertedPose(collect.loadingGateWait);

        gateCollectNearWaypoint = isRed ?
                createPose(collect.gateNearWaypoint) :
                createInvertedPose(collect.gateNearWaypoint);
        gateCollectNearWaypoint2 = isRed ?
                new Pose2d(collect.gateNearWaypoint2[0] + collect.gatePropertiesRedXOffset, collect.gateNearWaypoint2[1], Math.toRadians(collect.gateNearWaypoint2[2])) :
                createInvertedPose(collect.gateNearWaypoint2);
        gateCollectFarControlPoint = isRed ?
                createPose(collect.gateFarControlPoint) :
                createInvertedPose(collect.gateFarControlPoint);
        gateCollectOpen = isRed ?
                new Pose2d(collect.gateCollectOpen[0] + collect.gatePropertiesRedXOffset, collect.gateCollectOpen[1], Math.toRadians(collect.gateCollectOpen[2])) :
                createInvertedPose(collect.gateCollectOpen);
        gateCollectOpenHold = gateCollectOpen;
        gateCollect = isRed ?
                new Pose2d(collect.gateCollect[0] + collect.gatePropertiesRedXOffset, collect.gateCollect[1], Math.toRadians(collect.gateCollect[2])) :
                createInvertedPose(collect.gateCollect);
        gateTapBackup = isRed ?
                new Pose2d(collect.gateTapBackup[0] + collect.gatePropertiesRedXOffset, collect.gateTapBackup[1], Math.toRadians(collect.gateTapBackup[2])) :
                createInvertedPose(collect.gateTapBackup);
        gateTap = isRed ?
                new Pose2d(collect.gateTap[0] + collect.gatePropertiesRedXOffset, collect.gateTap[1], Math.toRadians(collect.gateTap[2])) :
                createInvertedPose(collect.gateTap);
    }
    protected void declareMiscPoses() {
        gate1 = isRed ?
                createPose(misc.gate1) :
                createInvertedPose(misc.gate1);
        gate2 = isRed ?
                createPose(misc.gate2) :
                createInvertedPose(misc.gate2);
        parkFar = isRed ?
                createPose(misc.parkFar) :
                createInvertedPose(misc.parkFar);
        hpParkFar = isRed ?
                createPose(misc.hpParkFar) :
                createInvertedPose(misc.hpParkFar);
        gatePark = isRed ?
                createPose(misc.gatePark) :
                createInvertedPose(misc.gatePark);
    }
}
