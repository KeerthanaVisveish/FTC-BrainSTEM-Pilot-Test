package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Collection extends Component {

    public static class Params{
        public double engagedPos = 0.1;
        public double disengagedPos = 0.65;
        public double delayPeriod = 0.5, autoCollectDelayPeriod = 0.7;
        public double normIntakePow = 0.95, autoIntakePow = .99, shootIntakePow = .99, slowShootIntakePower = .7, shooterTurretOffTargetIntakePow = 0;
        public double outtakeSpeed = -0.5;
        public double laserBallThreshold = 2.5;
        public double flickerLeftMinPwm = 1643, flickerLeftMaxPwm = 1493;
        public double flickerRightMinPwm = 1491, flickerRightMaxPwm = 1641;
        public double flickerFullUpPos = 0.8;
        public double flickerHalfUpPos = 0.4;
        public double flickerDownPos = 0.05;
        public int offDistanceSensorUpdatePeriod = 3; // when collector is off, waits this number of frames before updating distances sensors
        public double shootOuttakeTimeAuto = 0.05;
        public double postShootOuttakeWaitAuto = 0.;
        public double shootOuttakeTime = 0.05;
        public double clutchEngageRunIntakeTime = 0.3;
        public boolean useShootingSafetyInterlocks = true;
    }

    public static Params params = new Params();

    public enum CollectionState {
        OFF, INTAKE, INTAKE_SLOW, OUTTAKE,// CLUTCH_ENGAGE_INTAKE
    }

    public enum ClutchState {
        ENGAGED, UNENGAGED
    }

    public enum FlickerState {
        FULL_UP, DOWN, HALF_UP_DOWN, FULL_UP_DOWN
    }
    public final DcMotorEx collectorMotor;
    public final ServoImplEx clutchLeft;
    public final ServoImplEx clutchRight;
    public ServoImplEx flickerRight;
    private final ServoImplEx flickerLeft;

    private final ElapsedTime flickerTimer = new ElapsedTime();
    private boolean flickerStarted = false;

    private final AnalogInput frontRightLaser;
    private final AnalogInput frontLeftLaser;
    private final AnalogInput backTopLaser;
    private final AnalogInput backBottomLaser;

    private CollectionState collectionState;
    private ClutchState clutchState;
    private FlickerState flickerState;
    public boolean outtakeAfterClutchEngage;
    private double timerStart = 0;
    private boolean timerRunning = false;
    private boolean has3Balls = false, prevHas3Balls, autoCollectHas3Balls = false;
    private final ElapsedTime intake3BallsTimer = new ElapsedTime();
    public final ElapsedTime clutchTimer = new ElapsedTime();
    public double backLeftLaserDist, backRightLaserDist, frontLeftLaserDist, frontRightLaserDist;
    private int framesRunning;
    private boolean inAuto;
    private boolean shooterInitiallyGood;
    public Collection(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        collectorMotor = hardwareMap.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clutchRight = hardwareMap.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));
        clutchLeft = hardwareMap.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        flickerRight = hardwareMap.get(ServoImplEx.class, "flickerRight");
        flickerRight.setPwmRange(new PwmControl.PwmRange(params.flickerRightMinPwm, params.flickerRightMaxPwm));
        flickerLeft = hardwareMap.get(ServoImplEx.class, "flickerLeft");
        flickerLeft.setPwmRange(new PwmControl.PwmRange(params.flickerLeftMinPwm, params.flickerLeftMaxPwm));

        frontRightLaser = hardwareMap.get(AnalogInput.class, "FRLaser");
        frontLeftLaser = hardwareMap.get(AnalogInput.class, "FLLaser");
        backTopLaser = hardwareMap.get(AnalogInput.class, "BRLaser");
        backBottomLaser = hardwareMap.get(AnalogInput.class, "BLLaser");

        setCollectionState(CollectionState.OFF);
        setClutchState(ClutchState.UNENGAGED);
        setFlickerState(FlickerState.DOWN);

        intake3BallsTimer.reset();
        clutchTimer.reset();
    }

    public void setInAuto(boolean inAuto) {
        this.inAuto = inAuto;
    }
    public CollectionState getCollectionState() { return collectionState; }
    public void setCollectionState(CollectionState collectionState) {

        this.collectionState = collectionState;
        switch (collectionState) {
            case OFF:
                collectorMotor.setPower(0);
                break;
//            case CLUTCH_ENGAGE_INTAKE:
//                collectorMotor.setPower(params.clutchEngagePow);
//                break;
            case OUTTAKE:
                collectorMotor.setPower(params.outtakeSpeed);
                break;
            case INTAKE_SLOW:
            case INTAKE:
                shooterInitiallyGood = getClutchState() == ClutchState.ENGAGED && !robot.shootingSystem.shooterFirstGood();
                outtakeAfterClutchEngage = false;
                break;
        }
    }
    public ClutchState getClutchState() { return clutchState; }
    public void setClutchState(ClutchState clutchState) {
        this.clutchState = clutchState;
        switch (clutchState) {
            case ENGAGED:
                clutchRight.setPosition(params.engagedPos);
                clutchLeft.setPosition(params.engagedPos);
//                robot.collection.setCollectionState(Collection.CollectionState.CLUTCH_ENGAGE_INTAKE);
                break;
            case UNENGAGED:
                clutchRight.setPosition(params.disengagedPos);
                clutchLeft.setPosition(params.disengagedPos);
                break;
        }
    }
    public FlickerState getFlickerState() { return flickerState; }
    public void setFlickerState(FlickerState flickerState) {
        this.flickerState = flickerState;
        switch (flickerState) {
            case FULL_UP:
                flickerLeft.setPosition(params.flickerFullUpPos);
                flickerRight.setPosition(params.flickerFullUpPos);
                break;
            case DOWN:
                flickerLeft.setPosition(params.flickerDownPos);
                flickerRight.setPosition(params.flickerDownPos);
                break;
            case FULL_UP_DOWN:
                setCollectionState(CollectionState.OFF);
        }
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===COLLECTION======");
        telemetry.addData("collection state", collectionState);
        telemetry.addData("power", collectorMotor.getPower());
        telemetry.addData("flicker state", getFlickerState());
        telemetry.addData("flicker left pos", flickerLeft.getPosition());
        telemetry.addData("bl dist", backLeftLaserDist);
        telemetry.addData("br dist", backRightLaserDist);
        telemetry.addData("fl dist", frontLeftLaserDist);
        telemetry.addData("fr dist", frontRightLaserDist);
    }

    @Override
    public void update() {
        if (getCollectionState() != CollectionState.OFF || framesRunning % params.offDistanceSensorUpdatePeriod == 0) {
            backLeftLaserDist = voltageToDistance(backBottomLaser.getVoltage());
            backRightLaserDist = voltageToDistance(backTopLaser.getVoltage());
            frontLeftLaserDist = voltageToDistance(frontLeftLaser.getVoltage());
            frontRightLaserDist = voltageToDistance(frontRightLaser.getVoltage());
        }
        framesRunning++;

        switch (getCollectionState()) {
            case OFF:
            case OUTTAKE:
                break;
//            case CLUTCH_ENGAGE_INTAKE:
//                if (collectionStateTimer.seconds() >= params.clutchEngageRunIntakeTime)
//                    setCollectionState(CollectionState.OFF);
//                break;
            case INTAKE_SLOW:
            case INTAKE:
                if (getClutchState() == ClutchState.ENGAGED) {
                    boolean shouldUseSafetyInterlocks = !inAuto && params.useShootingSafetyInterlocks;
                    boolean meetsSafetyInterlocks = robot.turret.inRangeForShot() && robot.turret.onTarget() && robot.shootingSystem.shooterNormGood();
                    if (!shooterInitiallyGood && robot.shootingSystem.shooterFirstGood())
                        shooterInitiallyGood = true;
                    if (!shooterInitiallyGood)
                        meetsSafetyInterlocks = false;
                    if (shouldUseSafetyInterlocks && !meetsSafetyInterlocks)
                        collectorMotor.setPower(params.shooterTurretOffTargetIntakePow);
                    else if(getCollectionState() == CollectionState.INTAKE_SLOW)
                        collectorMotor.setPower(params.slowShootIntakePower);
                    else
                        collectorMotor.setPower(params.shootIntakePow);
                }
                else if(inAuto)
                    collectorMotor.setPower(params.autoIntakePow);
                else
                    collectorMotor.setPower(params.normIntakePow);
                break;
        }

        switch (getClutchState()) {
            case ENGAGED:
                if(clutchTimer.seconds() < params.shootOuttakeTime && outtakeAfterClutchEngage)
                    setCollectionState(CollectionState.OUTTAKE);
                else if(getCollectionState() == CollectionState.OUTTAKE)
                    setCollectionState(CollectionState.OFF);
                break;

            case UNENGAGED:
                outtakeAfterClutchEngage = true;
                clutchTimer.reset();
                break;
        }

        switch (getFlickerState()) {
            case FULL_UP:
            case DOWN:
                break;
            case HALF_UP_DOWN:
                if(!flickerStarted) {
                    flickerLeft.setPosition(params.flickerHalfUpPos);
                    flickerRight.setPosition(params.flickerHalfUpPos);
                    flickerTimer.reset();
                    flickerStarted = true;
                }
                else if(flickerTimer.seconds() > 0.4) {
                    flickerLeft.setPosition(params.flickerDownPos);
                    flickerRight.setPosition(params.flickerDownPos);
                    flickerStarted = false;
                    setFlickerState(FlickerState.DOWN);
                }
                break;
            case FULL_UP_DOWN:
                if (!flickerStarted) {
                    flickerLeft.setPosition(params.flickerFullUpPos);
                    flickerRight.setPosition(params.flickerFullUpPos);
                    flickerTimer.reset();
                    flickerStarted = true;
                } else if (flickerTimer.seconds() > 0.3) {
                    flickerLeft.setPosition(params.flickerDownPos);
                    flickerRight.setPosition(params.flickerDownPos);
                    flickerStarted = false;
                    setFlickerState(FlickerState.DOWN);
                }
                break;
        }

        checkForIntakeBalls(intake3BallsTimer.seconds());
    }

    private double voltageToDistance(double voltage) {
        return (voltage * 43.92898) - 6.01454; //tune if not accurate
    }

    public boolean isBackBallDetected() {
        return backLeftLaserDist < params.laserBallThreshold || backRightLaserDist < params.laserBallThreshold;
    }

    private boolean isFrontBallDetected() {
        return frontRightLaserDist < params.laserBallThreshold ||
                frontLeftLaserDist < params.laserBallThreshold;

//        return frontRightLaserDist < params.LASER_BALL_THRESHOLD;
    }

    public boolean has3Balls() {
        return has3Balls;
    }
    public boolean prevHas3Balls() {
        return prevHas3Balls;
    }
    public boolean autoCollectHas3Balls() {
        return autoCollectHas3Balls;
    }
    public void checkForIntakeBalls(double currentTime) {
        prevHas3Balls = has3Balls;
        if (isBackBallDetected() && isFrontBallDetected()) {
            if (!timerRunning) {
                timerStart = currentTime;
                timerRunning = true;
            } else {
                double dt = currentTime - timerStart;
                if (dt > params.delayPeriod)
                    has3Balls = true;
                if (dt > params.autoCollectDelayPeriod)
                    autoCollectHas3Balls = true;
            }
        } else {
            timerRunning = false;
            timerStart = 0;
            has3Balls = false;
            autoCollectHas3Balls = false;
        }
    }
}