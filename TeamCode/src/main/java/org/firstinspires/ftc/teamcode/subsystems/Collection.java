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
    public static double shootOuttakeTimeAuto = 0;
    public static double postShootOuttakeWaitAuto = 0.;
    public static double shootOuttakeTime = 0;
    public static boolean autoEngageClutch = false, autoUnengageClutch = false;
    public static double autoEngageClutchMaxX = 0;

    public enum CollectionState {
        OFF, INTAKE_SLOW, INTAKE, OUTTAKE, TRANSFER
    }

    public enum ClutchState {
        ENGAGED, UNENGAGED, WAITING_TO_ENGAGE
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
    private boolean has3Balls = false;
    private final ElapsedTime timer = new ElapsedTime();
    public final ElapsedTime clutch_timer = new ElapsedTime();
    public double backLeftLaserDist, backRightLaserDist, frontLeftLaserDist, frontRightLaserDist;
    public static class Params{
        public double ENGAGED_POS = 0.1;
        public double DISENGAGED_POS = 0.65;
        public double DELAY_PERIOD = 0.5;
        public double slowIntakePow = 0.3, normIntakePow = 0.95, autoIntakePow = .99, shootIntakePow = .99, turretOutOfRangeIntakePow = 0, shooterNotGoodIntakePow = .7;
        public double OUTTAKE_SPEED = -0.5;
        public double LASER_BALL_THRESHOLD = 2.5;
        public double flickerLeftMinPwm = 1643, flickerLeftMaxPwm = 1493;
        public double flickerRightMinPwm = 1491, flickerRightMaxPwm = 1641;
        public double flickerFullUpPos = 0.8;
        public double flickerHalfUpPos = 0.4;
        public double flickerDownPos = 0.05;
        public double hasBallValidationTime = 1;
        public double maxTimeBetweenShots = 0.8;
        public int offDistanceSensorUpdatePeriod = 3; // when collector is off, waits this number of frames before updating distances sensors
    }

    public static Params params = new Params();
    private int framesRunning;
    private boolean inAuto;
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

        timer.reset();
        clutch_timer.reset();
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
            case INTAKE_SLOW:
                collectorMotor.setPower(params.slowIntakePow);
                break;
            case OUTTAKE:
                collectorMotor.setPower(params.OUTTAKE_SPEED);
                break;
            case TRANSFER:
                collectorMotor.setPower(0.1);
                break;
        }
    }
    public ClutchState getClutchState() { return clutchState; }
    public void setClutchState(ClutchState clutchState) {
        this.clutchState = clutchState;
        switch (clutchState) {
            case ENGAGED:
                clutchRight.setPosition(params.ENGAGED_POS);
                clutchLeft.setPosition(params.ENGAGED_POS);
                break;
            case UNENGAGED:
            case WAITING_TO_ENGAGE:
                clutchRight.setPosition(params.DISENGAGED_POS);
                clutchLeft.setPosition(params.DISENGAGED_POS);
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
        }
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===COLLECTION======");
        telemetry.addData("collection state", collectionState);
        telemetry.addData("power", collectorMotor.getPower());
        telemetry.addData("flicker state", getFlickerState());
        telemetry.addData("flicker left pos", flickerLeft.getPosition());
//        telemetry.addData("flicker right pos", flickerRight.getPosition());
        telemetry.addData("bl dist", backLeftLaserDist);
        telemetry.addData("br dist", backRightLaserDist);
        telemetry.addData("fl dist", frontLeftLaserDist);
        telemetry.addData("fr dist", frontRightLaserDist);
    }

    @Override
    public void update() {
        if(!robot.turret.inRange && autoUnengageClutch)
            setClutchState(ClutchState.UNENGAGED);
        else if (autoEngageClutch && clutchState == ClutchState.UNENGAGED &&
                robot.drive.localizer.getPose().position.x < autoEngageClutchMaxX) {
            setClutchState(ClutchState.ENGAGED);
            setCollectionState(CollectionState.INTAKE);
        }

        boolean turretAccurate = Math.abs(robot.turret.positionError) <= Turret.turretParams.maxClutchEngageError;
        if (turretAccurate) {
            if (clutchState == ClutchState.WAITING_TO_ENGAGE) {
                setClutchState(ClutchState.ENGAGED);
                setCollectionState(CollectionState.INTAKE);
            }
        }
        else if (clutchState == ClutchState.ENGAGED) {
            setClutchState(ClutchState.WAITING_TO_ENGAGE);
            setCollectionState(CollectionState.OFF);
        }

        if (getCollectionState() != CollectionState.OFF || framesRunning % params.offDistanceSensorUpdatePeriod == 0) {
            backLeftLaserDist = voltageToDistance(backBottomLaser.getVoltage());
            backRightLaserDist = voltageToDistance(backTopLaser.getVoltage());
            frontLeftLaserDist = voltageToDistance(frontLeftLaser.getVoltage());
            frontRightLaserDist = voltageToDistance(frontRightLaser.getVoltage());
        }
        framesRunning++;

        switch (getCollectionState()) {
            case OFF:
            case INTAKE_SLOW:
            case OUTTAKE:
            case TRANSFER:
                break;
            case INTAKE:
                if (getClutchState() == ClutchState.ENGAGED) {
                    if (!robot.turret.inRangeForShot())
                        collectorMotor.setPower(params.turretOutOfRangeIntakePow);
                    else if (!robot.shootingSystem.shooterGood())
                        collectorMotor.setPower(params.shooterNotGoodIntakePow);
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
                if(clutch_timer.seconds() < shootOuttakeTime && outtakeAfterClutchEngage)
                    setCollectionState(CollectionState.OUTTAKE);
                else if(getCollectionState() == CollectionState.OUTTAKE)
                    setCollectionState(CollectionState.OFF);
                break;

            case UNENGAGED:
                outtakeAfterClutchEngage = true;
                clutch_timer.reset();
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
                else if(flickerTimer.seconds() > 0.3) {
                    flickerLeft.setPosition(params.flickerDownPos);
                    flickerRight.setPosition(params.flickerDownPos);
                    flickerStarted = false;
                    setFlickerState(FlickerState.DOWN);
                }
                break;
            case FULL_UP_DOWN:
                setCollectionState(CollectionState.OFF);
                if (!flickerStarted) {
                    flickerLeft.setPosition(params.flickerFullUpPos);
                    flickerRight.setPosition(params.flickerFullUpPos);
                    flickerTimer.reset();
                    flickerStarted = true;
                } else if (flickerTimer.seconds() > 0.3) {
                    flickerLeft.setPosition(params.flickerDownPos);
                    flickerRight.setPosition(params.flickerDownPos);
                    flickerStarted = false;
                    setCollectionState(CollectionState.INTAKE);
                    setFlickerState(FlickerState.DOWN);
                }
                break;
        }

        checkForIntakeBalls(timer.seconds());
    }
    public double getIntakePower() {
        return collectorMotor.getPower();
    }

    private double voltageToDistance(double voltage) {
        return (voltage * 43.92898) - 6.01454; //tune if not accurate
    }

    public boolean isBackBallDetected() {
        return backLeftLaserDist < params.LASER_BALL_THRESHOLD || backRightLaserDist < params.LASER_BALL_THRESHOLD;
    }

    private boolean isFrontBallDetected() {
        return frontRightLaserDist < params.LASER_BALL_THRESHOLD ||
                frontLeftLaserDist < params.LASER_BALL_THRESHOLD;

//        return frontRightLaserDist < params.LASER_BALL_THRESHOLD;
    }

    public boolean intakeHas3Balls() {
        return has3Balls;
    }


    public void checkForIntakeBalls(double currentTime) {
        if (isBackBallDetected() && isFrontBallDetected()) {
            if (!timerRunning) {
                timerStart = currentTime;
                timerRunning = true;
            } else if (currentTime - timerStart > params.DELAY_PERIOD)
                has3Balls = true;
        } else {
            timerRunning = false;
            timerStart = 0;
            has3Balls = false;
        }
    }
}