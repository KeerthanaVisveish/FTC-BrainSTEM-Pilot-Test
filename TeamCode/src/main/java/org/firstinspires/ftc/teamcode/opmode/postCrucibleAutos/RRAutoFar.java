//package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.opmode.Alliance;
//import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
//import org.firstinspires.ftc.teamcode.subsystems.Collection;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
//import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
//
//import java.util.ArrayList;
//
//@Config
//public abstract class RRAutoFar extends LinearOpMode {
//    // want abilityA to decide order of collection
//    // if i get 12 ball auto:
//    //    1: if partner gets 0, 3, or 6 then always collect 2nd one first then open gate
//    //    2: if partner gets 6 + gate then order doesn't matter
//    //    3: if partner gets 9 with no gate then tell then to only run 6 and do same as scenario 1
//    //    4: if partner gets 9 + gate then collect 1st one first and don't open gate
//    // if i get 15 ball auto:
//    //    1: if partner gets 6 or more than do procedure with 12 ball
//    //    2: if partner gets 0 or 3 then collect 3rd one first, then collect 2nd and open gate
//    public static class Customizable {
//        public String collectionOrder = "flf3fgfgf";
//        public String minTimes = "-1,-1,-1,-1,-1";
//        public boolean openGateOnFirst = true;
//        public boolean openGateOnSecond = false;
//        public boolean useParkAbort = true;
//        public boolean usedCurvedSecondCollect = false;
//        public int maxGateRetries = 0;
//    }
//    public enum AutoState {
//        DRIVE_TO_COLLECT,
//        OPEN_GATE,
//        DRIVE_TO_SHOOT,
//        SHOOT
//    }
//    public static Customizable customizable = new Customizable();
//    public static AutoParams.TimeConstraints timeConstraints = new AutoParams.TimeConstraints();
//    public static AutoParams.Collect collect = new AutoParams.Collect();
//    public static AutoParams.Shoot shoot = new AutoParams.Shoot();
//    public static AutoParams.Misc misc = new AutoParams.Misc();
//
//    protected Alliance alliance;
//    private ElapsedTime autoTimer;
//    private BrainSTEMRobot robot;
//    private AutoCommands autoCommands;
//    private Pose2d start,
//            collect1NearPose, preCollect1FarPose, collect1FarPose,
//            preCollect2NearPose, collect2NearPose, preCollect2FarPose, collect2FarPose,
//            preCollect3NearPose, collect3NearPose, preCollect3FarPose, collect3FarPose,
//            preLoadingPose, postLoadingPose, gateCollectPose, gateCollectRetryPose,
//            gate1NearPose, gate1FarPose, gate2NearPose, gate2FarPose,
//            parkNearPose, parkFarPose,
//            shootNearSetup1Pose, shootFarSetup1Pose, shootNearSetup2Pose, shootFarSetup2Pose, shootNearSetup3Pose, shootFarSetup3Pose, shootNearSetupLoadingPose, shootFarSetupLoadingPose;
//    private boolean isRed;
//    private AutoState autoState;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.setMsTransmissionInterval(11);
//
//        autoTimer = new ElapsedTime();
//        isRed = alliance == Alliance.RED;
//
//        if(customizable.collectionOrder.charAt(0) == 'n')
//            start = isRed ? new Pose2d(misc.startNearXRed, misc.startNearYRed, misc.startNearARed) : new Pose2d(misc.startNearXBlue, misc.startNearYBlue, misc.startNearABlue);
//        else if(customizable.collectionOrder.charAt(0) == 'f')
//            start = isRed ? new Pose2d(misc.startFarXRed, misc.startFarYRed, misc.startFarARed) : new Pose2d(misc.startFarXBlue, misc.startFarYBlue, misc.startFarABlue);
//
//        // DECLARE POSES=======================
//        declareShootSetupPoses();
//        declareCollectPoses();
//        declareMiscPoses();
//
//        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
//        autoCommands = new AutoCommands(robot, telemetry);
//
//        int numPaths = (customizable.collectionOrder.length()-1) / 2; //3
//        if(numPaths == 0)
//            throw new IllegalArgumentException("cannot have empty collectionOrder string");
//        ArrayList<Action> actionOrder = new ArrayList<>();
//        customizable.collectionOrder = customizable.collectionOrder.toLowerCase();
//
//        String[] minTimeStr = customizable.minTimes.split(",");
//        double[] minTimes = new double[minTimeStr.length];
//        if(minTimeStr.length != numPaths + 1)
//            telemetry.addLine("min time string currently has " + minTimeStr.length + " nums. It must have " + (numPaths+1) + " nums. (1 more than collectionOrder)");
//        for (int i=0; i<minTimeStr.length; i++) {
//            try {
//                minTimes[i] = Double.parseDouble(minTimeStr[i]);
//            } catch (NumberFormatException e) {
//                throw new IllegalArgumentException("min time string can only contain numbers and commas. " + customizable.minTimes + " is not valid");
//            }
//        }
//
//        Pose2d shootPose = null;
//        Pose2d preloadShootPose = getSetupPose(customizable.collectionOrder.substring(0, 2));
//        Pose2d prevShootPose = new Pose2d(preloadShootPose.position, preloadShootPose.heading);
//        for(int i=0; i<numPaths; i++) {
//            if(i < numPaths - 1)
//                shootPose = getSetupPose(customizable.collectionOrder.substring(i*2 + 2, i*2 + 4));
//            else
//                shootPose = getSetupPose(customizable.collectionOrder.charAt(i * 2 + 2) + "" + customizable.collectionOrder.charAt(i * 2 + 1));
//
//            String curLetter = customizable.collectionOrder.charAt(i*2+1) + "";
//            boolean fromNear = customizable.collectionOrder.charAt(i*2) == 'n';
//            boolean toNear = customizable.collectionOrder.charAt(i*2+2) == 'n';
//            double minTime = minTimes[i+1];
//            telemetry.addLine("Path " + (i+1) + ": letter: " + curLetter + " from near: " + fromNear + " to near: " + toNear + " min time: " + minTime);
//
//            switch(curLetter) {
//                case "1" :
//                    actionOrder.add(getFirstCollectAndShoot(prevShootPose, shootPose, fromNear, toNear, minTime));
//                    break;
//                case "2" :
//                    actionOrder.add(getSecondCollectAndShoot(prevShootPose, shootPose, fromNear, toNear, minTime));
//                    break;
//                case "3" :
//                    actionOrder.add(getThirdCollectAndShoot(prevShootPose, shootPose, fromNear, toNear, minTime));
//                    break;
//                case "l" : actionOrder.add(getLoadingCollectAndShoot(prevShootPose, shootPose, fromNear, toNear, minTime)); break;
//                case "g": actionOrder.add(getRepeatedGateCollectAndShoot(prevShootPose, shootPose, fromNear, toNear, minTime)); break;
//            }
//            prevShootPose = new Pose2d(shootPose.position, shootPose.heading);
//        }
//
//        Action autoAction = new SequentialAction(
//                getPreloadDriveAndShoot(preloadShootPose, customizable.collectionOrder.charAt(0) == 'n', minTimes[0]),
//                actionOrder.get(0),
//                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
//                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
//                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0),
//                numPaths > 4 ? actionOrder.get(4) : new SleepAction(0),
//                numPaths > 5 ? actionOrder.get(5) : new SleepAction(0),
//                numPaths > 6 ? actionOrder.get(6) : new SleepAction(0),
//                numPaths > 7 ? actionOrder.get(7) : new SleepAction(0),
//                numPaths > 8 ? actionOrder.get(8) : new SleepAction(0),
//                autoCommands.stopIntake(),
//                autoCommands.stopShooter()
//        );
//        Action timedAutoAction = new SequentialAction(
//                new CustomEndAction(autoAction,
//                        () -> autoTimer.seconds() > timeConstraints.parkStartTime && autoState != AutoState.DRIVE_TO_COLLECT && customizable.useParkAbort, 500),
//                autoCommands.stopIntake(),
//                autoCommands.stopShooter(),
//                decideParkDrive()
//        );
//
//
//        Action forcedStopAutoAction = new ParallelAction(
//                packet -> { telemetry.addData("RRAutoFar STATE", autoState); return true; },
//                new TimedAction(timedAutoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
//                autoCommands.updateRobot,
//                autoCommands.savePoseContinuously,
//                packet -> {
//                    telemetry.addData("current", robot.collection.collectorMotor.getCurrent(CurrentUnit.MILLIAMPS));
//                    telemetry.addData("intake p", robot.collection.getIntakePower());
//                    telemetry.addData("autoX, y, heading", PoseStorage.autoX + ", " + PoseStorage.autoY + ", " + Math.floor(PoseStorage.autoHeading * 180 / Math.PI));
//                    telemetry.update();
//                    return true;
//                }
//        );
//        robot.shootingSystem.resetTurretEncoder();
//
//        telemetry.addData("alliance", alliance);
//        telemetry.addData("auto string", customizable.collectionOrder);
//        telemetry.addLine("READY TO RUN");
//        telemetry.update();
//        waitForStart();
//        autoTimer.reset();
//
//        robot.shooter.setBallsShot(0); // always start with 3 preloads
//
//        Actions.runBlocking(
//                forcedStopAutoAction
//
//        );
//        robot.drive.stop();
//
//    }
//    private Pose2d getSetupPose(String info) {
//        boolean shootClose = info.charAt(0) == 'n';
//        String letter = info.charAt(1) + "";
//        switch (letter) {
//            case "1": return shootClose ? shootNearSetup1Pose : shootFarSetup1Pose;
//            case "2": return shootClose ? shootNearSetup2Pose : shootFarSetup2Pose;
//            case "3": return shootClose ? shootNearSetup3Pose : shootFarSetup3Pose;
//            case "l": case "g": return shootClose ? shootNearSetupLoadingPose : shootFarSetupLoadingPose;
//            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
//        }
//    }
//
//    private Action getPreloadDriveAndShoot(Pose2d shootPose, boolean isNear, double minTime) {
//        Action preloadShootDrive = robot.drive.actionBuilder(start)
//                .setTangent(Math.toRadians(isRed ? shoot.preloadTangentRed : -shoot.preloadTangentRed))
//                .splineToLinearHeading(shootPose, 0)
//                .build();
//        return new SequentialAction(
//                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
//                new ParallelAction(
//                        autoCommands.speedUpShooter(),
//                        autoCommands.enableTurretTracking(),
//                        preloadShootDrive
//                ),
//                new InstantAction(() -> autoState = AutoState.SHOOT),
//                waitUntilMinTime(minTime),
//                autoCommands.engageClutch(),
//                autoCommands.runIntake(),
//                autoCommands.flickerHalfUp(),
//                new SequentialAction(
//                        new SleepAction(timeConstraints.minShootTime),
//                        autoCommands.waitTillDoneShooting(Collection.params.maxTimeBetweenShots, 1)
//                ),
//                decideFlicker(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, Action shootDrive, double minTime, double slowIntakeTime) {
//        return new SequentialAction(
//                autoCommands.runIntake(),
//                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
//                collectDrive,
//                autoCommands.intakeSlow(),
//                new ParallelAction(
//                        new SequentialAction(
//                                new InstantAction(() -> autoState = AutoState.OPEN_GATE),
//                                gateDrive,
//                                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
//                                shootDrive,
//                                packet -> {robot.drive.stop(); return false; }
//                        ),
//                        new SequentialAction(
//                                new SleepAction(slowIntakeTime),
//                                autoCommands.stopIntake()
//                        )
//                ),
//                new InstantAction(() -> autoState = AutoState.SHOOT),
//                autoCommands.speedUpShooter(),
//                waitUntilMinTime(minTime),
//                telemetryPacket -> {
//                    robot.shooter.setBallsShot(0);
//                    return false;
//                },
//                autoCommands.runIntake(),
//                autoCommands.engageClutch(),
//                autoCommands.flickerHalfUp(),
//                new SleepAction(timeConstraints.minShootTime),
//                autoCommands.waitTillDoneShooting(Collection.params.maxTimeBetweenShots, 1),
//                decideFlicker(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action getFirstCollectAndShoot(Pose2d startPose, Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
//        Action firstCollectDrive;
//        Pose2d collectPose = fromNear ? collect1NearPose : collect1FarPose;
//        if(fromNear)
//            firstCollectDrive = new CustomEndAction(
//                    robot.drive.actionBuilder(startPose)
//                            .strafeToLinearHeading(collectPose.position, isRed ? collect.lineARed : collect.lineABlue, new TranslationalVelConstraint(collect.firstNearMaxVel))
//                            .build(),
//            () -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(collect1NearPose.position.y) - 1,
//                    1.4
//            );
//        else
//            firstCollectDrive = new CustomEndAction(
//                    robot.drive.actionBuilder(startPose)
//                            .setTangent(Math.toRadians(180))
//                            .splineToSplineHeading(preCollect1FarPose, isRed ? collect.lineARed : collect.lineABlue)
//                            .splineToSplineHeading(collectPose, collectPose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                            .build(),
//                    () -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(collectPose.position.y) - 1,
//                    10
//            );
//        Pose2d gatePose = fromNear ? gate1NearPose : gate1FarPose;
//        Action firstGateDrive = customizable.openGateOnFirst ?
//                new SequentialAction(
//                        robot.drive.actionBuilder(collectPose)
//                                .setTangent(Math.toRadians(isRed ? -45 : 45))
//                                .splineToLinearHeading(gatePose, Math.toRadians(isRed ? 90 : -90))
//                                .build(),
//                        new SleepAction(timeConstraints.gateWait)
//                )
//                : new SleepAction(0);
//        Action firstShootDrive = toNear ?
//                robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collectPose)
////                .afterDisp(shoot.clutchDisp1Near, decideEarlyRunIntake(minTime))
//                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                .build() :
//                robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collectPose)
//                        .setTangent(Math.toRadians(isRed ? -45 : 45))
////                        .afterDisp(shoot.clutchDisp1Far, decideEarlyRunIntake(minTime))
//                        .splineToSplineHeading(shootPose, Math.toRadians(isRed ? -20 : 20))
//                        .build();
//
//        return buildCollectAndShoot(firstCollectDrive, firstGateDrive, firstShootDrive, minTime, timeConstraints.slowIntakeTime);
//    }
//    private Action getSecondCollectAndShoot(Pose2d startPose, Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
//            Pose2d preCollectPose = fromNear ? preCollect2NearPose : preCollect2FarPose;
//            Pose2d collectPose = fromNear ? collect2NearPose : collect2FarPose;
//            Action secondCollectDrive = new CustomEndAction(robot.drive.actionBuilder(startPose)
//                        .setTangent(fromNear ? 0 : Math.toRadians(180))
//                        .splineToSplineHeading(preCollectPose, isRed ? collect.lineARed : collect.lineABlue)
//                        .splineToLinearHeading(collectPose, collectPose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                        .build(),
//                    () -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(collect2NearPose.position.y) - 0.5,
//                    10
//            );
//
//            Pose2d gatePose = fromNear ? gate2NearPose : gate2FarPose;
//            Action secondGateDrive = customizable.openGateOnSecond ?
//                    new SequentialAction(
//                            robot.drive.actionBuilder(collectPose)
//                                    .setTangent(Math.toRadians(isRed ? -135 : 135))
//                                    .splineToLinearHeading(gatePose, Math.toRadians(isRed ? 90 : -90))
//                                    .build(),
//                            new SleepAction(timeConstraints.gateWait)
//                    )
//                    : new SleepAction(0);
//
//            double tangent;
//            if (customizable.usedCurvedSecondCollect)
//                tangent = Math.toRadians(-90);
//            else
//                tangent = Math.toRadians(toNear ? 220 : -40);
//            tangent *= isRed ? 1 : -1;
//
//            Action secondShootDrive = robot.drive.actionBuilder(customizable.openGateOnSecond ? gatePose : collectPose)
//                        .setTangent(tangent)
//                        .splineToLinearHeading(shootPose, customizable.usedCurvedSecondCollect ? Math.toRadians(toNear ? 180 : 0) : tangent)
//                        .build();
//
//            return buildCollectAndShoot(secondCollectDrive, secondGateDrive, secondShootDrive, minTime, timeConstraints.slowIntakeTime);
//    }
//    private Action getThirdCollectAndShoot(Pose2d startPose, Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
//        Pose2d preCollectPose = fromNear ? preCollect3NearPose : preCollect3FarPose;
//        Pose2d collectPose = fromNear ? collect3NearPose : collect3FarPose;
//        Action thirdCollectDrive;
//        if(fromNear)
//            thirdCollectDrive = new CustomEndAction(
//                    robot.drive.actionBuilder(startPose)
//                    .setTangent(Math.toRadians(isRed ? 20 : -20))
//                    .splineToSplineHeading(preCollectPose, preCollectPose.heading.toDouble())
//                    .splineToLinearHeading(collectPose, collectPose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                    .build(),
//                    () -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(collect3NearPose.position.y) - 0.5,
//                    10
//            );
//        else
//            thirdCollectDrive = new CustomEndAction(
//                    robot.drive.actionBuilder(startPose)
//                            .setTangent(Math.toRadians(isRed ? 135 : -135))
//                            .splineToSplineHeading(preCollectPose, isRed ? collect.lineARed : collect.lineABlue)
//                            .splineToLinearHeading(collectPose, collectPose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                            .build(),
//                    () -> Math.abs(robot.drive.localizer.getPose().position.y) > Math.abs(collect3NearPose.position.y) - 0.5,
//                    10
//            );
//
//        double sign = isRed ? 1 : -1;
//        double shootTangent = sign * Math.toRadians(toNear ? -150 : -50);
//        Action thirdShootDrive = robot.drive.actionBuilder(collectPose)
//                .setTangent(shootTangent)
////                .afterDisp(toNear ? shoot.clutchDisp3Near : shoot.clutchDisp3Far, decideEarlyRunIntake(minTime))
//                .splineToSplineHeading(shootPose, shootTangent)
//                .build();
//
//        return buildCollectAndShoot(thirdCollectDrive, new SleepAction(0), thirdShootDrive, minTime, timeConstraints.slowIntakeTime);
//    }
//    private Action getLoadingCollectAndShoot(Pose2d startPose, Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
//        double sign = isRed ? 1 : -1;
//        double collectTangent1 = sign * Math.toRadians(fromNear ? 20 : 90);
//        double collectTangent2 = sign * Math.toRadians(fromNear ? 30 : 90);
//        double collectTangent3 = 0;
//        double shootTangent = sign * Math.toRadians(toNear ? -150 : -100);
//
//        Action loadingCollectDrive = new CustomEndAction(robot.drive.actionBuilder(startPose, collect.loadingBeginEndVel)
//                .setTangent(collectTangent1)
//                .splineToLinearHeading(preLoadingPose, collectTangent2)
//                .splineToLinearHeading(postLoadingPose, collectTangent3, new TranslationalVelConstraint(collect.maxVel))
//                .build(), () -> robot.collection.intakeHas3Balls());
//        Action loadingShootDrive = new SequentialAction(
//                new TimedAction(robot.drive.actionBuilder(postLoadingPose)
//                        .setTangent(shootTangent)
//                        .splineToSplineHeading(shootPose, shootTangent)
//                        .build(),
//                        2.3
//                ),
//                telemetryPacket -> {
//                    robot.drive.stop();
//                    return false;
//                }
//        );
//
//        return buildCollectAndShoot(loadingCollectDrive, new SleepAction(0), loadingShootDrive, minTime, timeConstraints.loadingSlowIntakeTime);
//    }
//    private Action getRepeatedGateCollectAndShoot(Pose2d startPose, Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
//        double sign = isRed ? 1 : -1;
//        double shootTangent = toNear ? sign * Math.toRadians(-150) : sign * Math.toRadians(-110);
//
//        Action gateShootDrive = robot.drive.actionBuilder(gateCollectPose)
//                .setTangent(shootTangent)
//                .splineToSplineHeading(shootPose, shootTangent)
//                .build();
//
//        return buildCollectAndShoot(repeatedGateCollect(fromNear), new SleepAction(0), gateShootDrive, minTime, timeConstraints.loadingSlowIntakeTime);
//    }
//    private Action repeatedGateCollect(boolean fromNear) {
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
//                        if (robot.collection.intakeHas3Balls() || numTimesCollected > customizable.maxGateRetries)
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
//                            .splineToSplineHeading(gateCollectPose, collectTangent)
//                            .build(),
//                            () -> robot.collection.intakeHas3Balls(), timeConstraints.gateCollectMaxTime);
//                }
//                else {
//                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                            .strafeToLinearHeading(new Vector2d(collect.gateCollectRetryX, gateCollectPose.position.y), gateCollectPose.heading.toDouble())
//                            .build(),
//                            () -> robot.collection.intakeHas3Balls(), timeConstraints.gateCollectMaxTime);
//                }
//            }
//            private void resetRetryDrive() {
//                gateResetDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                        .strafeToLinearHeading(gateCollectRetryPose.position, gateCollectRetryPose.heading.toDouble())
//                        .build(),
//                        () -> robot.collection.intakeHas3Balls() || Math.abs(robot.drive.localizer.getPose().position.y) < (gateCollectRetryPose.position.y) + 1);
//            }
//        };
//    }
//
//    private Action waitUntilMinTime(double minTime) {
//        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
//    }
//    private Action decideFlicker() {
//        return new Action() {
//            private final ElapsedTime timer = new ElapsedTime();
//            private boolean first = true;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (robot.shooter.getBallsShot() == 3)
//                    return false;
//
//                if (first) {
//                    first = false;
//                    timer.reset();
//                }
//
//                robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);
//                robot.collection.setCollectionState(Collection.CollectionState.OFF);
//
//                return timer.seconds() < 0.4;
//            }
//        };
//    }
//    private Action decideParkDrive() {
//        return new Action() {
//            private boolean first = true;
//            private Action parkDrive;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (autoState == AutoState.DRIVE_TO_COLLECT) {
//                    robot.drive.stop();
//                    return false;
//                }
//
//                if (first) {
//                    first = false;
//                    declareDrive();
//                }
//
//                return parkDrive.run(telemetryPacket);
//            }
//            private void declareDrive() {
//                boolean isLastShootPoseNear = customizable.collectionOrder.charAt(customizable.collectionOrder.length() - 1) == 'n';
//                Pose2d startPose = robot.drive.localizer.getPose();
//                Pose2d parkPose = isLastShootPoseNear ? parkNearPose : parkFarPose;
//                parkDrive = robot.drive.actionBuilder(startPose)
//                        .strafeToConstantHeading(parkPose.position)
//                        .build();
//            }
//        };
//    }
//    private Action decideEarlyRunIntake(double minTime) {
//        return packet -> {
//            if (minTime == -1)
//                return autoCommands.runIntake().run(packet);
//            return false;
//        };
//    }
//    private void declareShootSetupPoses() {
//        shootNearSetup1Pose = isRed ? shootNearRed(shoot.shootNearSetup1ARed) : shootNearBlue(shoot.shootNearSetup1ABlue);
//        shootFarSetup1Pose = isRed ? shootFarRed(shoot.shootFarSetup1ARed) : shootFarBlue(shoot.shootFarSetup1ABlue);
//        shootNearSetup2Pose = isRed ? shootNearRed(shoot.shootNearSetup2ARed) : shootNearBlue(shoot.shootNearSetup2ABlue);
//        shootFarSetup2Pose = isRed ? shootFarRed(shoot.shootFarSetup2ARed) : shootFarBlue (shoot.shootFarSetup2ABlue);
//        shootNearSetup3Pose = isRed ? shootNearRed(shoot.shootNearSetup3ARed) : shootNearBlue(shoot.shootNearSetup3ABlue);
//        shootFarSetup3Pose = isRed ? shootFarRed(shoot.shootFarSetup3ARed) : shootFarBlue(shoot.shootFarSetup3ABlue);
//        shootNearSetupLoadingPose = isRed ? shootNearRed(shoot.shootNearSetupLoadingARed) : shootNearBlue(shoot.shootNearSetupLoadingABlue);
//        shootFarSetupLoadingPose = isRed ? new Pose2d(shoot.shootFarSetupLoadingXRed, shoot.shootFarSetupLoadingYRed, shoot.shootFarSetupLoadingARed) : new Pose2d(shoot.shootFarSetupLoadingXBlue, shoot.shootFarSetupLoadingYBlue, shoot.shootFarSetupLoadingABlue);
//    }
//    private void declareCollectPoses() {
//        collect1NearPose = isRed ? // only 1 collect pose for near (no pre collect pose, only post collect pose)
//                new Pose2d(collect.firstNearXRed, collect.postFirstNearYRed, collect.lineARed) :
//                new Pose2d(collect.firstNearXBlue, collect.postFirstNearYBlue, collect.lineABlue);
//        preCollect1FarPose = isRed ?
//                new Pose2d(collect.firstFarXRed, collect.preFirstFarYRed, collect.preLineFarARed) :
//                new Pose2d(collect.firstFarXBlue, collect.preFirstFarYBlue, collect.preLineFarABlue);
//        collect1FarPose = isRed ?
//                new Pose2d(collect.firstFarXRed, collect.postFirstFarYRed, collect.lineARed) :
//                new Pose2d(collect.firstFarXBlue, collect.postFirstFarYBlue, collect.lineABlue);
//
//        preCollect2NearPose = isRed ?
//                new Pose2d(collect.secondNearXRed, collect.preSecondYRed, collect.preLineNearARed) :
//                new Pose2d(collect.secondNearXBlue, collect.preSecondYBlue, collect.preLineNearABlue);
//        collect2NearPose = isRed ?
//                new Pose2d(collect.secondNearXRed, collect.postSecondYRed, collect.lineARed) :
//                new Pose2d(collect.secondNearXBlue, collect.postSecondYBlue, collect.lineABlue);
//        preCollect2FarPose = isRed ?
//                new Pose2d(collect.secondFarXRed, collect.preSecondYRed, collect.preLineFarARed) :
//                new Pose2d(collect.secondFarXBlue, collect.preSecondYBlue, collect.preLineFarABlue);
//        collect2FarPose = isRed ?
//                new Pose2d(collect.secondFarXRed, collect.postSecondYRed, collect.lineARed) :
//                new Pose2d(collect.secondFarXBlue, collect.postSecondYBlue, collect.lineABlue);
//
//        preCollect3NearPose = isRed ?
//                new Pose2d(collect.thirdNearXRed, collect.preThirdYRed, collect.preLineNearARed) :
//                new Pose2d(collect.thirdNearXBlue, collect.preThirdYBlue, collect.preLineNearABlue);
//        collect3NearPose = isRed ?
//                new Pose2d(collect.thirdNearXRed, collect.postThirdYRed, collect.lineARed) :
//                new Pose2d(collect.thirdNearXBlue, collect.postThirdYBlue, collect.lineABlue);
//        preCollect3FarPose = isRed ?
//                new Pose2d(collect.thirdFarXRed, collect.preThirdYRed, collect.preLineFarARed) :
//                new Pose2d(collect.thirdFarXBlue, collect.preThirdYBlue, collect.preLineFarABlue);
//        collect3FarPose = isRed ?
//                new Pose2d(collect.thirdFarXRed, collect.postThirdYRed, collect.lineARed) :
//                new Pose2d(collect.thirdFarXBlue, collect.postThirdYBlue, collect.lineABlue);
//
//        preLoadingPose = isRed ?
//                new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) :
//                new Pose2d(collect.preLoadingXBlue, collect.preLoadingYBlue, collect.preLoadingABlue);
//        postLoadingPose = isRed ?
//                new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) :
//                new Pose2d(collect.postLoadingXBlue, collect.postLoadingYBlue, collect.postLoadingABlue);
//        gateCollectPose = isRed ?
//                new Pose2d(collect.gateCollectXRed, collect.gateCollectYRed, collect.gateCollectARed) :
//                new Pose2d(collect.gateCollectXBlue, collect.gateCollectYBlue, collect.gateCollectABlue);
//        gateCollectRetryPose = isRed ?
//                new Pose2d(collect.gateCollectXRed, collect.gateCollectRetryYRed, collect.gateCollectARed) :
//                new Pose2d(collect.gateCollectXBlue, collect.gateCollectRetryYBlue, collect.gateCollectABlue);
//    }
//    private void declareMiscPoses() {
//        gate1NearPose = isRed ?
//                new Pose2d(misc.gateNearX1Red, misc.gateNearYRed, misc.gateRedA1) :
//                new Pose2d(misc.gateNearX1Blue, misc.gateNearYBlue, misc.gateBlueA1);
//        gate1FarPose = isRed ?
//                new Pose2d(misc.gateFarX1Red, misc.gateFarYRed, misc.gateRedA1) :
//                new Pose2d(misc.gateFarX1Blue, misc.gateFarYBlue, misc.gateBlueA1);
//        gate2NearPose = isRed ?
//                new Pose2d(misc.gateNearX2Red, misc.gateFarYRed, misc.gateRedA2) :
//                new Pose2d(misc.gateNearX2Blue, misc.gateFarYBlue, misc.gateBlueA2);
//        gate2FarPose = isRed ?
//                new Pose2d(misc.gateFarX2Red, misc.gateNearYRed, misc.gateRedA2) :
//                new Pose2d(misc.gateFarX2Blue, misc.gateNearYBlue, misc.gateBlueA2);
//
//        parkNearPose = isRed ?
//                new Pose2d(misc.parkNearX, misc.parkNearYRed, misc.parkNearARed) :
//                new Pose2d(misc.parkNearX, misc.parkNearYBlue, misc.parkNearABlue);
//        parkFarPose = isRed ?
//                new Pose2d(misc.parkFarX, misc.parkFarYRed, misc.parkFarARed) :
//                new Pose2d(misc.parkFarX, misc.parkFarYBlue, misc.parkFarABlue);
//    }
//    public Pose2d shootNearRed(double angleRad) { return new Pose2d(shoot.shootNearXRed, shoot.shootNearYRed, angleRad); }
//    public Pose2d shootFarRed(double angleRad) { return new Pose2d(shoot.shootFarXRed, shoot.shootFarYRed, angleRad); }
//    public Pose2d shootNearBlue(double angleRad) { return new Pose2d(shoot.shootNearXBlue, shoot.shootNearYBlue, angleRad); }
//    public Pose2d shootFarBlue(double angleRad) { return new Pose2d(shoot.shootFarXBlue, shoot.shootFarYBlue, angleRad); }
//}
