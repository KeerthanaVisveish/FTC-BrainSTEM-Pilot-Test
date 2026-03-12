//package org.firstinspires.ftc.teamcode.opmode.autosBase;
//
//import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createInvertedPose;
//import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createInvertedVec;
//import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;
//import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createVec;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.opmode.Alliance;
//import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;
//import org.firstinspires.ftc.teamcode.opmode.teleop.LimelightResetTele;
//import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
//import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
//import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
//import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.BoxTolerance;
//import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
//import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;
//
//import java.util.ArrayList;
//import java.util.function.BooleanSupplier;
//
//@Config
//public abstract class AutoPid extends LinearOpMode {
//    public static class Customizable {
//        public String validCollectLetters = "123gtla";
//        public String nearSolo = "n2ngngn1n3n", nearPartner = "n2ngngngn1n";
//        public String farLoadingFirst = "flf3fafaf", farThirdFirst = "f3flfafaf", farLoadingLimelight = "flfafafaf";
//        public String custom = "n1n2n3n";
//        public String stringBuilder = "n2ngngngn1n";
//        public boolean openGateOnFirst = false;
//        public boolean openGateOnSecond = false;
//        public boolean smartPark = true;
//    }
//    public enum AutoState {
//        DRIVE_TO_COLLECT,
//        OPEN_GATE,
//        DRIVE_TO_SHOOT,
//        SHOOT
//    }
//    public static Customizable customizable = new Customizable();
//    public static AutoParamsPid.TimeConstraints timeConstraints = new AutoParamsPid.TimeConstraints();
//    public static AutoParamsPid.Collect collect = new AutoParamsPid.Collect();
//    public static AutoParamsPid.Shoot shoot = new AutoParamsPid.Shoot();
//    public static AutoParamsPid.Misc misc = new AutoParamsPid.Misc();
//
//    protected Alliance alliance;
//    private ElapsedTime autoTimer;
//    protected BrainSTEMRobot robot;
//    protected AutoCommands autoCommands;
//    private Pose2d start,
//            collect1NearControlPoint, collect1FarControlPoint, collect1,
//            collect2NearWaypoint, collect2NearControlPoint, collect2FarControlPoint, collect2,
//            preCollect3Near, collect3NearControlPoint, preCollect3Far, collect3FarControlPoint, collect3,
//            preLoadingControlPoint, preLoading, loadingWaypoint, postLoading,
//            gateCollectNearWaypoint, gateCollectNearControlPoint, gateCollectFarControlPoint, gateCollectOpen, gateCollectOpenHold, gateCollect, gateTapBackup, gateTap,
//            gate1, gate2,
//            shootGateNearControlPoint,
//            parkFar;
//    private Pose2d shoot1Near, shoot1Far, shoot1FarControlPoint,
//            shoot2Near, shoot2Far, shoot2FarControlPoint,
//            shootGateNear, shootGateFar, shootGateFarControlPoint,
//            shoot3Near, shoot3Far,
//            shootLoadingNear, shootLoadingFar, shootLoadingFarControlPoint;
//    private boolean isRed;
//    private AutoState autoState;
//    private Vector2d perpNearParkLine, perpFarParkLine, farParkLineOrigin;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.setMsTransmissionInterval(11);
//
//        if (!LimelightResetTele.cameraIsReset)
//            throw new IllegalStateException("Limelight has not been reset - run LimelightResetTele");
//
//        if(!BrainSTEMTeleOp.inCompetition) {
//            throw new IllegalStateException("In Competition is not true - run ToggleInCompetitionTele and press A");
//        }
//        autoTimer = new ElapsedTime();
//        isRed = alliance == Alliance.RED;
//
//        perpNearParkLine = isRed ? new Vector2d(1, 1) : new Vector2d(1, -1);
//        perpNearParkLine = perpNearParkLine.div(Math.hypot(perpNearParkLine.x, perpNearParkLine.y));
//        perpFarParkLine = isRed ? new Vector2d(-1, 1) : new Vector2d(-1, -1);
//        farParkLineOrigin = new Vector2d(48, 0);
//
//        if(customizable.stringBuilder.charAt(0) == 'n')
//            start = isRed ? createPose(misc.startNearRed) : createPose(misc.startNearBlue);
//        else if(customizable.stringBuilder.charAt(0) == 'f')
//            start = isRed ? createPose(misc.startFarRed) : createPose(misc.startFarBlue);
//
//        // DECLARE POSES=======================
//        declareShootPoses();
//        declareCollectPoses();
//        declareMiscPoses();
//
//        boolean preloadNear = customizable.stringBuilder.charAt(0) == 'n';
//        if (preloadNear)
//            Limelight.startingPipeline = Limelight.CLASSIFIER_PIPELINE;
//        else
//            Limelight.startingPipeline = Limelight.BALL_DETECTION_PIPELINE;
//
//        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
//        LimelightResetTele.cameraIsReset = false; // set to false for next run (tele does not care about this)
//        autoCommands = new AutoCommands(robot, telemetry);
//
//        int numPaths = (customizable.stringBuilder.length()-1) / 2; //3
//        if(numPaths == 0)
//            throw new IllegalArgumentException("cannot have empty collectionOrder string");
//        ArrayList<Action> actionOrder = new ArrayList<>();
//        customizable.stringBuilder = customizable.stringBuilder.toLowerCase();
//
////        double preloadX = preloadNear ? shoot.nearPreload[0] : shoot.farSpike[0];
////        double preloadY = preloadNear ? shoot.nearPreload[1] : shoot.farSpike[1];
////        double preloadA = getShootPose(preloadNear, customizable.stringBuilder.charAt(1) + "").heading.toDouble();
////        Pose2d preloadShootPose = isRed ? new Pose2d(preloadX, preloadY, preloadA) : new Pose2d(preloadX, -preloadY, preloadA);
//        Pose2d preloadShootPose;
//        if(preloadNear)
//            preloadShootPose = isRed ? createPose(shoot.nearPreload) : createInvertedPose(shoot.nearPreload);
//        else {
//            String nextLetter = customizable.stringBuilder.charAt(1) + "";
//            if(nextLetter.equals("l"))
//                preloadShootPose = isRed ? createPose(shoot.farPreloadLoading) : createInvertedPose(shoot.farPreloadLoading);
//            else
//                preloadShootPose = getShootPose(false, nextLetter);
//        }
//        int numGateCollects = 0;
//        boolean toNear = true;
//        for(int i=0; i<numPaths; i++) {
//            boolean last = i == numPaths - 1;
//
//            String info = customizable.stringBuilder.substring(i*2, Math.min(i*2+4, customizable.stringBuilder.length()));
//            boolean fromNear = info.charAt(0) == 'n';
//            String letter = info.charAt(1) + "";
//            toNear = info.charAt(2) == 'n';
//            String nextLetter = info.length() == 3 ? letter : info.charAt(3) + "";
//            if(fromNear && letter.equals("a"))
//                throw new IllegalArgumentException("cannot call limelight auto collect from near at path index " + i);
//
//            if(!customizable.validCollectLetters.contains(letter))
//                throw new IllegalArgumentException("invalid collectionOrder of " + customizable.stringBuilder + "; can only contain 1, 2, 3, G/g, L/l, or A/a");
//
//            Pose2d shootPose;
//            if(last)
//                shootPose = getFinalShootPose(toNear, letter);
//            else {
//                shootPose = getShootPose(toNear, toNear ? letter : nextLetter);
//            }
//
//            telemetry.addData("STRING BUILDER", customizable.stringBuilder);
//            telemetry.addLine("Path " + (i+1) + ": letter: " + letter + " from near: " + fromNear + " to near: " + toNear + ", last: " + last);
//            telemetry.addData("Path " + (i+1) + " Shoot pose", MathUtils.formatPose3(shootPose));
//            switch(letter) {
//                case "1" :
//                    actionOrder.add(getFirstCollectAndShoot(shootPose, fromNear, toNear, last));
//                    break;
//                case "2" :
//                    actionOrder.add(getSecondCollectAndShoot(shootPose, fromNear, toNear));
//                    break;
//                case "3" :
//                    actionOrder.add(getThirdCollectAndShoot(shootPose, fromNear, toNear, last));
//                    break;
//                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose, toNear)); break;
//                case "t":
//                case "g":
//                    numGateCollects++;
//                    double waitTime = numGateCollects == 2 ? timeConstraints.secondGaitWait : numGateCollects == 3 ? timeConstraints.thirdGateWait : 0;
//                    actionOrder.add(getGateCollectAndShoot(shootPose, fromNear, toNear, waitTime, letter.equals("t")));
//                    break;
//                case "a":
//                    actionOrder.add(getLimelightLoadingZoneCollectAndShoot(shootPose));
//                    break;
//            }
//        }
//        final boolean finalToNear = toNear;
//
//        Action autoAction = new SequentialAction(
//                getPreloadDriveAndShoot(preloadShootPose, customizable.stringBuilder.charAt(0) == 'n'),
//                actionOrder.get(0),
//                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
//                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
//                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0),
//                numPaths > 4 ? actionOrder.get(4) : new SleepAction(0),
//                numPaths > 5 ? actionOrder.get(5) : new SleepAction(0),
//                numPaths > 6 ? actionOrder.get(6) : new SleepAction(0),
//                numPaths > 7 ? actionOrder.get(7) : new SleepAction(0),
//                numPaths > 8 ? actionOrder.get(8) : new SleepAction(0)
//        );
//
////        BooleanSupplier nearShouldStop = () -> (autoTimer.seconds() > timeConstraints.parkOutsideShootingTime && robot.drive.localizer.getPose().position.dot(perpNearParkLine) > misc.smartParkNearDist);
//        BooleanSupplier farShouldParkExtra = () -> autoState == AutoState.DRIVE_TO_SHOOT && autoTimer.seconds() > timeConstraints.farParkOutsideShootingTime && (robot.drive.localizer.getPose().position.minus(farParkLineOrigin)).dot(perpFarParkLine) < misc.smartParkFarDist;
//        BooleanSupplier farShouldPark = () -> farShouldParkExtra.getAsBoolean() || (autoState == AutoState.SHOOT && autoTimer.seconds() > timeConstraints.farParkAfterShootingTime);
//        BooleanSupplier farShouldStop = () -> !farShouldParkExtra.getAsBoolean() && (autoState != AutoState.SHOOT && autoTimer.seconds() > timeConstraints.farParkOutsideShootingTime);
//        BooleanSupplier nearShouldStop = () -> false;
////        BooleanSupplier farShouldPark = () -> false;
////        BooleanSupplier farShouldStop = () -> false;
//
//        boolean[] shouldStop = { false };
//        boolean[] shouldPark = { false };
//        boolean[] autoActionDone = { false };
//
//        Action smartParkAction = new SequentialAction(
//                new ParallelAction(
//                        new CustomEndAction(autoAction, () -> shouldStop[0] || shouldPark[0])
//                            .setEndFunction(() -> autoActionDone[0] = true),
//                        new CustomEndAction(
//                                new ParallelAction(
//                                        new CustomEndAction(() -> (nearShouldStop.getAsBoolean() && finalToNear) || (farShouldStop.getAsBoolean() && !finalToNear))
//                                            .setEndFunction(() -> shouldStop[0] = true),
//                                        new CustomEndAction(() -> !finalToNear && farShouldPark.getAsBoolean())
//                                            .setEndFunction(() -> shouldPark[0] = true)
//                                ),
//                                () -> !customizable.smartPark || autoActionDone[0] || shouldStop[0] || shouldPark[0]
//                        )
//                ),
//                new CustomEndAction(() -> shouldStop[0] || shouldPark[0] || autoActionDone[0]),
//                new ParallelAction(
//                        shouldPark[0] || (!shouldPark[0] && !shouldStop[0] && !finalToNear) ? getFarParkDrive() : telemetryPacket -> {robot.drive.stop(); return true; },
//                        telemetryPacket -> {robot.led.setAutoDone(); return false; }
//                )
//        );
//
//
//        Action fullAutoAction = new ParallelAction(
//                autoCommands.updateRobotInfo(),
//                smartParkAction,
//                autoCommands.updateRobot(),
//                autoCommands.savePoseContinuously(),
//                packet -> {
//                    DrivePath.drawCurrentPath(packet.fieldOverlay());
//                    robot.drawRobotInfo(packet.fieldOverlay());
//                    telemetry.addData("auto state", autoState);
//                    telemetry.addData("SHOULD PARK", shouldPark[0]);
//                    telemetry.addData("SHOULD STOP", shouldStop[0]);
//                    telemetry.addData("AUTO ACTION DONE", autoActionDone[0]);
//                    robot.limelight.printInfo();
//                    telemetry.update();
//                    return true;
//                }
//        );
//
//        robot.shootingSystem.resetTurretEncoder();
//        robot.turret.setSmoothWhenOutOfRange(false);
//        robot.collection.setInAuto(true);
//
//        telemetry.addData("2nd collect pose", MathUtils.formatPose3(collect2));
//        telemetry.addData("3rd collect pose", MathUtils.formatPose3(collect3));
//
//        telemetry.addData("alliance", alliance);
//        telemetry.addData("auto string", customizable.stringBuilder);
//        telemetry.addLine("READY TO RUN");
//        telemetry.update();
//
//        waitForStart();
//        robot.startOpmode();
//        autoTimer.reset();
//
//        Actions.runBlocking(fullAutoAction);
//    }
//    private Pose2d getShootPose(boolean shootClose, String letter) {
//        switch (letter) {
//            case "1": return shootClose ? shoot1Near : shoot1Far;
//            case "2": return shootClose ? shoot2Near : shoot2Far;
//            case "t":
//            case "g": return shootClose ? shootGateNear : shootGateFar;
//            case "3": return shootClose ? shoot3Near : shoot3Far;
//            case "l": return shootClose ? shootLoadingNear : shootLoadingFar;
//            case "a": return shootLoadingFar;
//            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.stringBuilder + "; can only contain 1, 2, 3, L/l, or G/g");
//        }
//    }
//    private Pose2d getFinalShootPose(boolean shootClose, String letter) {
//        switch (letter) {
//            case "1": return shootClose ?
//                    isRed ? createPose(shoot.near1Last) : createInvertedPose(shoot.near1Last) :
//                    shoot1Far;
//            case "2":
//            case "t":
//            case "g": return shootClose ? shoot2Near : shoot2Far;
//            case "3": return shootClose ?
//                    isRed ? createPose(shoot.near3Last) : createInvertedPose(shoot.near3Last) :
//                    isRed ? createPose(shoot.farSpike) : createInvertedPose(shoot.farSpike);
//            case "l": case "a":
//                return shootClose ?
//                        shootLoadingNear : shootLoadingFar;
//        }
//        return null;
//    }
//
//    private Action getFarParkDrive() {
//        return new DrivePath(robot.drive,
//                new Waypoint(parkFar).setPassPosition(true).setMinLinearPower(misc.minParkFarPower).setMaxTime(0.8)
//        );
//    }
//    private Action getPreloadDriveAndShoot(Pose2d shootPose, boolean shootingNear) {
//        Waypoint preloadShootWaypoint = new Waypoint(shootPose)
//                .setPassPosition(true);
//        if(shootingNear)
//                preloadShootWaypoint
//                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
//                        .setMaxTime(3)
//                        .setMinLinearPower(shoot.nearMinDrivePower1);
//        else
//            preloadShootWaypoint.setMaxTime(.9);
//
//        DrivePath preloadShootDrive = new DrivePath(robot.drive, preloadShootWaypoint);
//        Action preloadShootAction;
//        if(shootingNear)
//            preloadShootAction = new ParallelAction(
//                    preloadShootDrive,
//                    new SequentialAction(
//                            new CustomEndAction(() -> preloadShootDrive.getWaypointDistanceError() < shoot.nearMinDrivePower2Dist),
//                            new InstantAction(() -> preloadShootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
//                    )
//            );
//        else
//            preloadShootAction = new SequentialAction(
//                    new SleepAction(timeConstraints.farPreloadDriveDelay),
//                    preloadShootDrive
//            );
//
//        return new SequentialAction(
//                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
//                new ParallelAction(
//                        autoCommands.flickerHalfUp(),
//                        autoCommands.speedUpShooter(),
//                        autoCommands.enableTurretTracking(),
//                        preloadShootAction,
//                        shootingNear ? getShootAction(0, 2.4) : new SleepAction(0)
//                ),
//                !shootingNear ? getShootAction(0) : new SleepAction(0)
//        );
//    }
//    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, DrivePath shootDrive, boolean shootingNear, double postIntakeTime, boolean runIntake, boolean notLast) {
//        Action shootDriveAction;
//        if(shootingNear) {
//            shootDrive.getCurWaypoint().setMinLinearPower(shoot.nearMinDrivePower1);
//            shootDriveAction = new ParallelAction(
//                    shootDrive,
//                    new SequentialAction(
//                            new CustomEndAction(() -> shootDrive.getWaypointDistanceError() < shoot.nearMinDrivePower2Dist),
//                            new InstantAction(() -> shootDrive.getCurWaypoint().setMinLinearPower(shoot.minDrivePower2))
//                    )
//            );
//        }
//        else {
//            shootDrive.getCurWaypoint().setMinLinearPower(shoot.farMinDrivePower);
//            shootDriveAction = shootDrive;
//        }
//        return new SequentialAction(
//                telemetryPacket -> {robot.shooter.setMaxVoltage(shootingNear ? shoot.maxNearShooterVoltage : Shooter.shooterParams.maxVoltage); return false; },
//                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
//                new ParallelAction(
//                        collectDrive,
//                        runIntake ? new SequentialAction(new SleepAction(.3), autoCommands.runIntake()) : new SleepAction(0)
//                ),
//                new ParallelAction(
//                        new SequentialAction(
//                                new ParallelAction(
//                                        new InstantAction(() -> autoState = AutoState.OPEN_GATE),
//                                        gateDrive
//                                ),
//                                new ParallelAction(
//                                        new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
//                                        new CustomEndAction(shootDriveAction, () -> autoState == AutoState.SHOOT)
//                                                .setEndFunction(robot.drive::stop)
//                                ),
//                                new ParallelAction(
//                                        new InstantAction(() -> autoState = AutoState.SHOOT),
//                                        new InstantAction(() -> robot.drive.stop())
//                                )
//                        ),
//                        new SequentialAction(
//                                new SleepAction(postIntakeTime),
//                                autoCommands.stopIntake(),
//
//                                shootingNear && notLast ? new SequentialAction(
//                                        new CustomEndAction(telemetryPacket -> autoState != AutoState.SHOOT, () -> {
//                                            double dot = robot.drive.localizer.getPose().position.dot(perpNearParkLine);
//                                            return dot < shoot.earlyEngageClutchDist;
//                                        }),
//                                        getShootAction(0)
//                                ) : new SleepAction(0)
//                        )
//                ),
//                !shootingNear ? getShootAction(0) :
//                !notLast ? getShootAction(timeConstraints.lastShootExtraTime) : new SleepAction(0)
//        );
//    }
//    private Action getShootAction(double extraShootTime, double shooterInterlockMaxTime) {
//        return new SequentialAction(
//                new ParallelAction(
//                        new InstantAction(() -> autoState = AutoState.SHOOT),
//                        autoCommands.stopIntake(),
//                        autoCommands.engageClutch(),
//                        new SequentialAction(
//                                new CustomEndAction(new SleepAction(shooterInterlockMaxTime), () -> robot.shootingSystem.shooterFirstGood() && robot.turret.onTarget()),
//                                new ParallelAction(
//                                        autoCommands.runIntake(),
//                                        new CustomEndAction(new SleepAction(timeConstraints.maxShootTime), () -> robot.shooter.ballsDoneExiting())
//                                )
//                        )
//                ),
//                new SleepAction(extraShootTime),
//                new ParallelAction(
//                        autoCommands.disengageClutch(),
//                        autoCommands.stopIntake(),
//                        autoCommands.flickerUp(),
//                        new SleepAction(.1)
//                )
//        );
//    }
//    private Action getShootAction(double extraShootTime) {
//        return getShootAction(extraShootTime, 0);
//    }
//    private Action getFirstCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, boolean last) {
//        DrivePath firstCollectDrive;
//        if(fromNear) {
//            firstCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect1, new BoxTolerance(0.75, 2, Math.toRadians(3)))
//                    .setFixedLinearPower(collect.collectDrivePower)
//                    .setMaxTime(1.4)
//                    .setControlPoint(collect1NearControlPoint, collect.firstNearT1, collect.firstNearT2)
//                    .setCustomEndCondition(() -> robot.collection.has3Balls())
//                    .setPassPosition(true)
//                    .setCorrectiveStrength(collect.firstCorrectiveStrength));
//        }
//        else {
//            firstCollectDrive = new DrivePath(robot.drive, telemetry,
//                    new Waypoint(collect1)
//                            .setControlPoint(collect1FarControlPoint, collect.firstFarT1, collect.firstFarT2)
//                            .setPassPosition(true)
//                            .setFixedLinearPower(collect.collectDrivePower)
//                            .setMaxTime(2)
//                            .setCustomEndCondition(() -> robot.collection.has3Balls()));
//        }
//        double controlY = fromNear ? collect1NearControlPoint.position.y : collect1FarControlPoint.position.y;
//        Action firstCollectAction = new ParallelAction(
//                firstCollectDrive,
//                new SequentialAction(
//                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(controlY)),
//                        new InstantAction(() -> firstCollectDrive.getCurWaypoint()
//                                .setFixedLinearPower(collect.firstCollectDrivePower)
//                        )
//                )
//        );
//
//        double sign = isRed ? 1 : -1;
//        Pose2d preGate1Pose = new Pose2d(gate1.position.x, gate1.position.y - sign * misc.gate1BackupDist, gate1.heading.toDouble());
//
//        Action firstGateDrive = customizable.openGateOnFirst ?
//                new SequentialAction(
//                        new DrivePath(robot.drive, telemetry,
//                                new Waypoint(preGate1Pose, new BoxTolerance(1, 1, Math.toRadians(10))).
//                                        setMinLinearPower(misc.gatePrepMinPower)
//                                        .setPassPosition(true),
//                                new Waypoint(gate1)
//                                        .setMinLinearPower(misc.gateMinPower)
//                                        .setPassPosition(true)),
//                        new SleepAction(timeConstraints.gateOpeningWait)
//                )
//                : new SleepAction(0);
//        DrivePath firstShootDrive = toNear ?
//                new DrivePath(robot.drive, new Waypoint(shootPose)
//                        .setMaxTime(3)
//                        .setPassPosition(true))
//                :
//                new DrivePath(robot.drive, telemetry, new Waypoint(shootPose)
//                        .setMaxTime(10)
//                        .setPassPosition(true)
//                        .setControlPoint(shoot1FarControlPoint, shoot.firstShootFarT1, shoot.firstShootFarT2));
//
//        return buildCollectAndShoot(firstCollectAction, firstGateDrive, firstShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
//    }
//    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear) {
//        DrivePath secondCollectDrive;
//        if(fromNear) {
//            BoxTolerance near2WaypointTol = new BoxTolerance(collect.secondNearWaypointTol);
//            secondCollectDrive = new DrivePath(robot.drive, telemetry,
//                    new Waypoint(collect2NearWaypoint, near2WaypointTol)
//                            .setPassPosition(true)
//                            .setMinLinearPower(collect.collectDrivePower),
//                    new Waypoint(collect2)
//                            .setPassPosition(true)
//                            .setMinLinearPower(collect.secondCollectDrivePower)
//                            .setControlPoint(collect2NearControlPoint, collect.secondNearT1, collect.secondNearT2)
//                            .setCloseHeadingKP(collect.secondNearCloseHeadingKP));
//        }
//        else {
//            throw new RuntimeException("second collect not tuned from far; cannot incorporate 2f into string builder");
//        }
//        double sign = isRed ? -1 : 1;
//        Pose2d preGatePose = new Pose2d(gate2.position.x, gate2.position.y + sign * misc.gate2BackupDist, gate2.heading.toDouble());
//
//        Action gateOpen = new CustomEndAction(
//                new DrivePath(robot.drive, telemetry,
//                    new Waypoint(preGatePose).setMinLinearPower(misc.gatePrepMinPower).setPassPosition(true),
//                    new Waypoint(gate2).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
//                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold)
//                .setEndFunction(robot.drive::stop);
//        Action secondGateDrive = customizable.openGateOnSecond ?
//                new SequentialAction(
//                        gateOpen,
//                        new SleepAction(timeConstraints.gateOpeningWait)
//                )
//                : new SleepAction(0);
//
//        DrivePath secondShootDrive;
//        if(toNear) {
//            secondShootDrive = new DrivePath(robot.drive, telemetry, new Waypoint(shootPose)
//                    .setMaxTime(3)
//                    .setPassPosition(true)
//                    .setControlPoint(shootGateNearControlPoint, shoot.gateNearShootTStartError, shoot.gateNearShootTFinishError));
//        }
//        else
//            secondShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
//                    .setMaxTime(1.5)
//                    .setPassPosition(true)
//                    .setControlPoint(shoot2FarControlPoint, shoot.secondShootFarT1, shoot.secondShootFarT2));
//
//        return buildCollectAndShoot(secondCollectDrive, secondGateDrive, secondShootDrive, toNear, timeConstraints.postIntakeTime, true, true);
//    }
//    private Action getThirdCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, boolean last) {
//        Waypoint w = new Waypoint(collect3)
//                .setPassPosition(true)
//                .setMinLinearPower(collect.collectDrivePower);
//        if(fromNear)
//            w.setControlPoint(collect3NearControlPoint, collect.thirdNearT1, collect.thirdNearT2);
//        else
//            w.setControlPoint(collect3FarControlPoint, collect.thirdFarT1, collect.thirdFarT2);
//        DrivePath thirdCollectDrive = new DrivePath(robot.drive, telemetry, w);
//
//        if (fromNear) {
//            BoxTolerance preCollect3NearTol = new BoxTolerance(collect.thirdNearWaypointTol[0], collect.thirdNearWaypointTol[1], Math.toRadians(collect.thirdNearWaypointTol[2]));
//            Waypoint preCollectNear = new Waypoint(preCollect3Near, preCollect3NearTol)
//                    .setPassPosition(true)
//                    .setMinLinearPower(collect.collectDrivePower);
//            thirdCollectDrive.addWaypoint(preCollectNear, 0);
//        }
//        else {
//            BoxTolerance preCollect3FarTol = new BoxTolerance(collect.thirdFarWaypointTol[0], collect.thirdFarWaypointTol[1], Math.toRadians(collect.thirdFarWaypointTol[2]));
//            Waypoint preCollectFar = new Waypoint(preCollect3Far, preCollect3FarTol)
//                    .setPassPosition(true)
//                    .setMinLinearPower(collect.collectDrivePower);
//            thirdCollectDrive.addWaypoint(preCollectFar, 0);
//        }
//        double controlY = fromNear ? collect3NearControlPoint.position.y : preCollect3Far.position.y;
//        Action thirdCollectAction = new ParallelAction(
//                thirdCollectDrive,
//                new SequentialAction(
//                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(controlY)),
//                        new InstantAction(() -> thirdCollectDrive.getCurWaypoint()
//                                .setFixedLinearPower(collect.thirdCollectDrivePower)
//                                .setCloseHeadingKP(collect.thirdCloseHeadingKp)
//                        )
//                )
//        );
//        Waypoint thirdShootDest = new Waypoint(shootPose)
//                .setPassPosition(true)
//                .setHeadingLerp(PathParams.HeadingLerpType.REVERSE_TANGENT);
//        if(toNear)
//            thirdShootDest.setMaxTime(3);
//        else
//            thirdShootDest.setMaxTime(2);
//        DrivePath thirdShootDrive = new DrivePath(robot.drive, telemetry, thirdShootDest);
//
////        return thirdCollectAction;
//        return buildCollectAndShoot(thirdCollectAction, new SleepAction(0), thirdShootDrive, toNear, timeConstraints.postIntakeTime, true, !last);
//    }
//    private Action getLoadingCollectAndShoot(Pose2d shootPose, boolean toNear) {
//        BoxTolerance preLoadingTol = new BoxTolerance(collect.preLoadingTol[0], collect.preLoadingTol[1], Math.toRadians(collect.preLoadingTol[2]));
//        BoxTolerance postLoadingTol = new BoxTolerance(collect.postLoadingTol[0], collect.postLoadingTol[1], Math.toRadians(collect.postLoadingTol[2]));
//        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
//                new Waypoint(preLoading, preLoadingTol)
//                        .setMaxTime(1.9)
//                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
//                        .setControlPoint(preLoadingControlPoint, collect.preLoadingT1, collect.preLoadingT2)
//                        .setPassPosition(true),
//                new Waypoint(loadingWaypoint)
//                        .setFixedLinearPower(collect.loadingDrivePower)
//                        .setMaxTime(.9),
//                new Waypoint(postLoading, postLoadingTol)
//                        .setMinLinearPower(collect.loadingDrivePower)
//                        .setMaxTime(.6));
//        Action loadingCollectAction = new CustomEndAction(loadingCollectDrive, robot.collection::has3Balls);
//        Waypoint w = new Waypoint(shootPose)
//                .setMaxTime(3)
//                .setPassPosition(true);
//        if(!toNear)
//            w.setControlPoint(shootLoadingFarControlPoint, shoot.farLoadingT1, shoot.farLoadingT2);
//        DrivePath loadingShootDrive = new DrivePath(robot.drive, w);
//
//        return buildCollectAndShoot(loadingCollectAction, new SleepAction(0), loadingShootDrive, toNear, timeConstraints.loadingSlowIntakeTime, true, false);
//    }
//
//    protected Action getLimelightLoadingZoneCollectAndShoot(Pose2d shootPose) {
//        Vector2d scan1 = alliance == Alliance.RED ?
//                createVec(collect.limelightScanPos1) :
//                createInvertedVec(collect.limelightScanPos1);
//        Vector2d scan2 = alliance == Alliance.RED ?
//                createVec(collect.limelightScanPos2) :
//                createInvertedVec(collect.limelightScanPos2);
//
////        Action limelightCollectAction = new CustomEndAction(robot.getLimelightCollectDrive(createVec(collect.limelightScanPos1), timeConstraints.maxLimelightWaitTime), robot.collection::autoCollectHas3Balls);
//        Action limelightCollectAction = robot.getLimelightCollectSequence(scan1, scan2, timeConstraints.maxLimelightWaitTime);
//
//        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
//                .setMaxTime(3)
//                .setPassPosition(true));
//
//        return buildCollectAndShoot(limelightCollectAction, new SleepAction(0), loadingShootDrive, false, timeConstraints.loadingSlowIntakeTime, true, false);
//    }
//
//    private Action getGateCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double waitTime, boolean shouldGateTap) {
//        DrivePath gateOpenDrive;
//        if (fromNear) {
//            BoxTolerance waypointTol = new BoxTolerance(collect.gateNearWaypointTol[0], collect.gateNearWaypointTol[1], Math.toRadians(collect.gateNearWaypointTol[2]));
//            BoxTolerance openTol = new BoxTolerance(collect.gateOpenTol);
//            gateOpenDrive = new DrivePath(robot.drive, telemetry,
//                    new Waypoint(gateCollectNearWaypoint, waypointTol)
//                            .setMaxTime(1.3)
//                            .setPassPosition(true)
//                            .setMinLinearPower(collect.gateOpenDrive1MinPower),
//                    new Waypoint(gateCollectOpen, openTol)
//                            .setMaxTime(1.5)
//                            .setPassPosition(true)
//                            .setMinLinearPower(collect.gateOpenDrive1MinPower)
//                            .setControlPoint(gateCollectNearControlPoint, collect.gateNearT1, collect.gateNearT2)
//            );
//        }
//        else {
//            gateOpenDrive = new DrivePath(robot.drive,
//                    new Waypoint(gateCollectOpen)
//                            .setMaxTime(2)
//                            .setPassPosition(true)
//                            .setMinLinearPower(collect.gateOpenDrive1MinPower)
//                            .setControlPoint(gateCollectFarControlPoint, collect.gateCollectOpenFarTStartError, collect.gateCollectOpenFarTFinishError)
//            );
//        }
//
//        CustomEndAction gateOpenCustomEndAction = new CustomEndAction(gateOpenDrive,
//                () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold && Math.abs(robot.drive.localizer.getPose().position.y) > 45);
//        Action gateOpenAction = new ParallelAction(
//                new SequentialAction(
//                        new SleepAction(.3),
//                        autoCommands.runIntake()
//                ),
//                gateOpenCustomEndAction,
//                new SequentialAction(
//                        new CustomEndAction(() -> Math.abs(robot.drive.localizer.getPose().position.y) > 40),
//                        new InstantAction(() -> gateOpenDrive.getCurWaypoint()
//                                .setMinLinearPower(collect.gateOpenDrive2MinPower)
//                                .setMaxLinearPower(collect.gateOpenDrive2MaxPower))
//                )
//        );
//        DrivePath gateOpenHold = new DrivePath(robot.drive, new Waypoint(gateCollectOpenHold, new BoxTolerance(.1, .1, Math.toRadians(.1))).setMaxLinearPower(.2));
//
//        DrivePath postGateOpenDrive = new DrivePath(robot.drive, new Waypoint(gateCollect)
//                .setPassPosition(true)
//                .setMaxTime(1));
//
//        DrivePath gateTapDrive = new DrivePath(robot.drive,
//                new Waypoint(gateTapBackup)
//                        .setPassPosition(true)
//                        .setMinLinearPower(misc.gateMinPower)
//                        .setMaxTime(.9),
//                new Waypoint(gateTap)
//                        .setPassPosition(true)
//                        .setMinLinearPower(misc.gateMinPower)
//                        .setMaxTime(.9));
//        DrivePath gateTapHold = new DrivePath(robot.drive, new Waypoint(gateTap, new BoxTolerance(.1, .1, .1)).setMaxLinearPower(.2).setMaxTime(timeConstraints.gateTapWait));
//
//        Action gateTapAction = new SequentialAction(
//                new ParallelAction(
//                        new SequentialAction(
//                                new CustomEndAction(robot.collection::has3Balls),
//                                autoCommands.stopIntake()
//                        ),
//                        new SequentialAction(
//                                new CustomEndAction(gateTapDrive,
//                                        () -> Math.hypot(robot.shootingSystem.robotVelCm.x, robot.shootingSystem.robotVelCm.y) < collect.hitGateVelThreshold
//                                                && robot.drive.localizer.getPose().position.x < gateTapBackup.position.x),
//                                gateTapHold
//                        )
//                )
//        );
//
//
//        Action completeGateCollectDrive = new SequentialAction(
//                autoCommands.stopIntake(),
//                new SleepAction(waitTime),
//                gateOpenAction,
//                new CustomEndAction(new SequentialAction(
//                        new TimedAction(gateOpenHold, timeConstraints.gateCollectOpenWait),
//                        postGateOpenDrive,
//                        new SleepAction(timeConstraints.gateCollectMaxTime)
//                ), robot.collection::has3Balls),
//                shouldGateTap ? gateTapAction : new SleepAction(0)
//        );
//
//        DrivePath gateShootDrive;
//        if (toNear)
//            gateShootDrive = new DrivePath(robot.drive,
//                new Waypoint(shootPose)
//                        .setMaxTime(2)
//                        .setPassPosition(true)
//                        .setControlPoint(shootGateNearControlPoint, shoot.gateNearShootTStartError, shoot.gateNearShootTFinishError));
//        else
//            gateShootDrive = new DrivePath(robot.drive,
//                    new Waypoint(shootPose)
//                            .setPassPosition(true)
//                            .setControlPoint(shootGateFarControlPoint, shoot.gateFarTStartError, shoot.gateFarTFinishError)
//            );
//
//        return buildCollectAndShoot(completeGateCollectDrive, new SleepAction(0), gateShootDrive, toNear, timeConstraints.postIntakeTime, false, true);
//
//    }
//    private void declareShootPoses() {
//        shoot1Near = isRed ? createPose(shoot.near1) : createInvertedPose(shoot.near1);
//        shoot1Far = isRed ? createPose(shoot.farSpike) : createInvertedPose(shoot.farSpike);
//        shoot2Near = isRed ? createPose(shoot.near2) : createInvertedPose(shoot.near2);
//        shoot2Far = shoot1Far;
//        shootGateNear = isRed ? createPose(shoot.nearGate) : createInvertedPose(shoot.nearGate);
//        shootGateNearControlPoint = isRed ? createPose(shoot.gateNearControlPoint) : createInvertedPose(shoot.gateNearControlPoint);
//        shootGateFar = shoot2Far;
//        shootGateFarControlPoint = isRed ? createPose(shoot.gateFarControlPoint) : createInvertedPose(shoot.gateFarControlPoint);
//        shoot3Near = isRed ? createPose(shoot.near3) : createInvertedPose(shoot.near3);
//        shoot3Far = shoot1Far;
//
//        shootLoadingNear = shoot3Near;
//        shootLoadingFar = isRed ? createPose(shoot.farLoading) : createInvertedPose(shoot.farLoading);
//        shootLoadingFarControlPoint = isRed ? createPose(shoot.farLoadingControlPoint) : createInvertedPose(shoot.farLoadingControlPoint);
//
//        shoot1FarControlPoint = isRed ? createPose(shoot.far1ControlPoint) : createInvertedPose(shoot.far1ControlPoint);
//        shoot2FarControlPoint = isRed ? createPose(shoot.far2ControlPoint) : createInvertedPose(shoot.far2ControlPoint);
//    }
//    private void declareCollectPoses() {
//        collect1 = isRed ?
//                createPose(collect.first) :
//                createInvertedPose(collect.first);
//        collect1NearControlPoint = isRed ?
//                createPose(collect.firstControlPointNear) :
//                createInvertedPose(collect.firstControlPointNear);
//        collect1FarControlPoint = isRed ?
//                createPose(collect.firstControlPointFar) :
//                createInvertedPose(collect.firstControlPointFar);
//
//        double[] second = customizable.openGateOnSecond ? collect.secondIfOpenGate : collect.second;
//        collect2 = isRed ?
//                createPose(second) :
//                createInvertedPose(second);
//        collect2NearWaypoint = isRed ?
//                createPose(collect.secondNearWaypoint) :
//                createInvertedPose(collect.secondNearWaypoint);
//        collect2NearControlPoint = isRed ?
//                createPose(collect.secondNearControlPoint) :
//                createInvertedPose(collect.secondNearControlPoint);
//        collect2FarControlPoint = isRed ?
//                createPose(collect.secondFarControlPoint) :
//                createInvertedPose(collect.secondFarControlPoint);
//
//        collect3 = isRed ?
//                createPose(collect.third) :
//                createInvertedPose(collect.third);
//        preCollect3Near = isRed ?
//                createPose(collect.thirdNearWaypoint) :
//                createInvertedPose(collect.thirdNearWaypoint);
//        preCollect3Far = isRed ?
//                createPose(collect.thirdFarWaypoint) :
//                createInvertedPose(collect.thirdFarWaypoint);
//        collect3FarControlPoint = isRed ?
//                createPose(collect.thirdFarControlPoint) :
//                createInvertedPose(collect.thirdFarControlPoint);
//        collect3NearControlPoint = isRed ?
//                createPose(collect.thirdNearControlPoint) :
//                createInvertedPose(collect.thirdNearControlPoint);
////        collect3FarControlPoint = isRed ?
////                createPose(collect.thirdFarControlPoint) :
////                createInvertedPose(collect.thirdFarControlPoint);
//
//        preLoadingControlPoint = isRed ?
//                createPose(collect.preLoadingControlPoint) :
//                createInvertedPose(collect.preLoadingControlPoint);
//        preLoading = isRed ?
//                createPose(collect.preLoading) :
//                createInvertedPose(collect.preLoading);
//        loadingWaypoint = isRed ?
//                createPose(collect.loadingWaypoint) :
//                createInvertedPose(collect.loadingWaypoint);
//        postLoading = isRed ?
//                createPose(collect.postLoading) :
//                createInvertedPose(collect.postLoading);
//
//        gateCollectNearWaypoint = isRed ?
//                createPose(collect.gateNearWaypoint) :
//                createInvertedPose(collect.gateNearWaypoint);
//        gateCollectNearControlPoint = isRed ?
//                createPose(collect.gateNearControlPoint) :
//                createInvertedPose(collect.gateNearControlPoint);
//        gateCollectFarControlPoint = isRed ?
//                createPose(collect.gateFarControlPoint) :
//                createInvertedPose(collect.gateFarControlPoint);
//        gateCollectOpen = isRed ?
//                createPose(collect.gateOpen) :
//                createInvertedPose(collect.gateOpen);
//        gateCollectOpenHold = isRed ?
//                createPose(collect.gateOpenHold) :
//                createInvertedPose(collect.gateOpenHold);
//        gateCollect = isRed ?
//                createPose(collect.gateCollect) :
//                createInvertedPose(collect.gateCollect);
//        gateTapBackup = isRed ?
//                createPose(collect.gateTapBackup) :
//                createInvertedPose(collect.gateTapBackup);
//        gateTap = isRed ?
//                createPose(collect.gateTap) :
//                createInvertedPose(collect.gateTap);
//    }
//    private void declareMiscPoses() {
//        gate1 = isRed ?
//                createPose(misc.gate1) :
//                createInvertedPose(misc.gate1);
//        gate2 = isRed ?
//                createPose(misc.gate2) :
//                createInvertedPose(misc.gate2);
//        parkFar = isRed ?
//                createPose(misc.parkFar) :
//                createInvertedPose(misc.parkFar);
//    }
//}
