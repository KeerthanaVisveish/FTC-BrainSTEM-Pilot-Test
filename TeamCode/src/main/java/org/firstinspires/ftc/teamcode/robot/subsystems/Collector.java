package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;

@Config
public class Collector extends Component {

    public static class Params {
        public double engagedPos = 0.1;
        public double disengagedPos = 0.65;
        public double has3BallsConfirmFrames = 2, has3BallsAutoConfirmFrames = 5;
        public double fullIntakePower = .99, slowIntakePower = .85;
        public double outtakeSpeed = -0.75;
        public double laserBallThreshold = 1.5;
        public double flickerLeftMinPwm = 1643, flickerLeftMaxPwm = 1493;
        public double flickerRightMinPwm = 1491, flickerRightMaxPwm = 1641;
        public double flickerFullUpPos = 0.8;
        public double flickerHalfUpPos = 0.4;
        public double flickerDownPos = 0.05;
        public int offDistanceSensorUpdatePeriod = 2; // when collector is off, waits this number of frames before updating distances sensors
        public double jammedCurrentThreshold = 8000;
        public double confirmJamTime = .2;
    }

    public static Params params = new Params();

    public enum IntakeState {
        OFF, INTAKE, INTAKE_SLOW, OUTTAKE
    }

    public enum ClutchState {
        ENGAGED, DISENGAGED
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

    private final AnalogInput frontRightLaser;
    private final AnalogInput frontLeftLaser;
    private final AnalogInput backTopLaser;
    private final AnalogInput backBottomLaser;

    private IntakeState intakeState, cachedIntakeState;
    private int framesInState;
    private final ElapsedTime jamTimer;
    private ClutchState clutchState;
    private FlickerState flickerState;
    private int framesSaw3;
    private boolean has3Balls = false, prevHas3Balls, autoCollectHas3Balls = false;
    public final ElapsedTime clutchTimer = new ElapsedTime();
    // TODO: FRONT LEFT LASER IS BROKEN; discovered at worlds
    public double backLeftLaserDist, backRightLaserDist, frontLeftLaserDist, frontRightLaserDist;
    private int framesRunning;

    public Collector(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);

        collectorMotor = hardwareMap.get(DcMotorEx.class, RobotProperties.intakeName);
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clutchRight = hardwareMap.get(ServoImplEx.class, RobotProperties.clutchRightName);
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));
        clutchLeft = hardwareMap.get(ServoImplEx.class, RobotProperties.clutchLeftName);
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        flickerRight = hardwareMap.get(ServoImplEx.class, RobotProperties.flickerRightName);
        flickerRight.setPwmRange(new PwmControl.PwmRange(params.flickerRightMinPwm, params.flickerRightMaxPwm));
        flickerLeft = hardwareMap.get(ServoImplEx.class, RobotProperties.flickerLeftName);
        flickerLeft.setPwmRange(new PwmControl.PwmRange(params.flickerLeftMinPwm, params.flickerLeftMaxPwm));

        frontRightLaser = hardwareMap.get(AnalogInput.class, RobotProperties.frLaserName);
        frontLeftLaser = hardwareMap.get(AnalogInput.class, RobotProperties.flLaserName);
        backTopLaser = hardwareMap.get(AnalogInput.class, RobotProperties.brLaserName);
        backBottomLaser = hardwareMap.get(AnalogInput.class, RobotProperties.blLaserName);

        setIntakeState(IntakeState.OFF);
        cachedIntakeState = IntakeState.INTAKE;
        setClutchState(ClutchState.DISENGAGED);
        setFlickerState(FlickerState.DOWN);

        clutchTimer.reset();
        jamTimer = new ElapsedTime();
        jamTimer.reset();
    }
    public IntakeState getIntakeState() { return intakeState; }
    public void setIntakeState(IntakeState collectorState) {
        this.intakeState = collectorState;
        framesInState = 0;
        switch (collectorState) {
            case OFF:
                collectorMotor.setPower(0);
                break;
            case OUTTAKE:
                collectorMotor.setPower(params.outtakeSpeed);
                break;
            case INTAKE_SLOW:
                collectorMotor.setPower(params.slowIntakePower);
                break;
            case INTAKE:
                collectorMotor.setPower(params.fullIntakePower);
                break;
        }
    }
    public ClutchState getClutchState() { return clutchState; }
    public void setClutchState(ClutchState clutchState) {
        this.clutchState = clutchState;
        cachedIntakeState = IntakeState.OFF;
        switch (clutchState) {
            case ENGAGED:
                clutchRight.setPosition(params.engagedPos);
                clutchLeft.setPosition(params.engagedPos);
                break;
            case DISENGAGED:
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
                setIntakeState(IntakeState.OFF);
                flickerLeft.setPosition(params.flickerFullUpPos);
                flickerRight.setPosition(params.flickerFullUpPos);
                flickerTimer.reset();
                break;
            case HALF_UP_DOWN:
                flickerLeft.setPosition(params.flickerHalfUpPos);
                flickerRight.setPosition(params.flickerHalfUpPos);
                flickerTimer.reset();
                break;
        }
    }
    public int getFramesInState() {
        return framesInState;
    }
    public void updateState(boolean shootingInterlocksMet) {
        // checking safety interlocks for shooting
        if (getClutchState() == ClutchState.ENGAGED) {
            if(!shootingInterlocksMet && (intakeState == IntakeState.INTAKE || intakeState == IntakeState.INTAKE_SLOW)) {
                cachedIntakeState = intakeState;
                setIntakeState(IntakeState.OFF);
            }
            else if(shootingInterlocksMet && intakeState == IntakeState.OFF && cachedIntakeState != null)
                setIntakeState(cachedIntakeState);
        }

        // updating sensors
        if (getIntakeState() != IntakeState.OFF || framesRunning % params.offDistanceSensorUpdatePeriod == 0) {
            backLeftLaserDist = voltageToDistance(backBottomLaser.getVoltage());
            backRightLaserDist = voltageToDistance(backTopLaser.getVoltage());
            frontLeftLaserDist = voltageToDistance(frontLeftLaser.getVoltage());
            frontRightLaserDist = voltageToDistance(frontRightLaser.getVoltage());
        }
        framesRunning++;

        if(collectorMotor.getCurrent(CurrentUnit.MILLIAMPS) < params.jammedCurrentThreshold)
            jamTimer.reset();

        switch (getFlickerState()) {
            case FULL_UP:
            case DOWN:
                break;
            case HALF_UP_DOWN:
                if(flickerTimer.seconds() > 0.15) {
                    flickerLeft.setPosition(params.flickerDownPos);
                    flickerRight.setPosition(params.flickerDownPos);
                    setFlickerState(FlickerState.DOWN);
                }
                break;
            case FULL_UP_DOWN:
                if(flickerTimer.seconds() > .35) {
                    setFlickerState(FlickerState.DOWN);
                }
                else if (flickerTimer.seconds() > .2) {
                    flickerLeft.setPosition(params.flickerDownPos);
                    flickerRight.setPosition(params.flickerDownPos);
                }
                break;
        }


        switch (intakeState) {
            case OFF:
            case OUTTAKE:
                break;
            case INTAKE_SLOW:
                collectorMotor.setPower(params.slowIntakePower);
                break;
            case INTAKE:
                collectorMotor.setPower(params.fullIntakePower);
                break;
        }

        checkForIntakeBalls();
        framesInState++;
    }

    private double voltageToDistance(double voltage) {
        return (voltage * 43.92898) - 6.01454;
    }

    public boolean isBackBallDetected() {
        return backLeftLaserDist < params.laserBallThreshold || backRightLaserDist < params.laserBallThreshold;
    }

    private boolean isFrontBallDetected() {
        return frontRightLaserDist < params.laserBallThreshold;
//                frontLeftLaserDist < params.laserBallThreshold;

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
    public void checkForIntakeBalls() {
        prevHas3Balls = has3Balls;
        if (isBackBallDetected() && isFrontBallDetected()) {
            framesSaw3++;
            if (framesSaw3 > params.has3BallsConfirmFrames)
                has3Balls = true;
            if (framesSaw3 > params.has3BallsAutoConfirmFrames)
                autoCollectHas3Balls = true;
        } else {
            framesSaw3 = 0;
            has3Balls = false;
            autoCollectHas3Balls = false;
        }
    }
    public boolean jammed() {
        return jamTimer.seconds() > params.confirmJamTime;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===COLLECTION======");
        telemetry.addData("CO collection state", intakeState);
        telemetry.addData("CO clutch engaged", getClutchState() == ClutchState.ENGAGED ? -50 : 0);
        telemetry.addData("CO flicker state", getFlickerState());
        telemetry.addLine();
        telemetry.addData("CO power", collectorMotor.getPower());
        telemetry.addData("CO flicker left pos", flickerLeft.getPosition());
        telemetry.addData("CO flicker timer", flickerTimer.seconds());
        telemetry.addData("CO bl dist", backLeftLaserDist);
        telemetry.addData("CO br dist", backRightLaserDist);
        telemetry.addData("CO fl dist", frontLeftLaserDist);
        telemetry.addData("CO fr dist", frontRightLaserDist);
        telemetry.addData("CO intake current", collectorMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("CO jammed", jammed() ? 1 : 0);
    }
}