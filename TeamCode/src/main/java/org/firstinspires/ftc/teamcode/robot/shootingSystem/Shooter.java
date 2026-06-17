package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double speedAdjustment = 20;
        public double ignoreHoodUpdateError = Math.toRadians(.5);
        public double A = 20, B = 6, k = -150, x0 = .06;
        public double kVYInt = 1.63, kVSlope = -0.02005;
        public double shotVelDropThreshold = 40, targetVelStaticShotThreshold = .1;
        public double avg3BallShootTime = .55;
        public int startingShooterSpeedAdjustment = 0;
        public double minVoltage = -2.025, maxVoltage = 15;
        public double firstCurrentLimitedVoltage = 7;
    }
    public static class TestingParams {
        public boolean testing = false;
    }

    public static ShooterParams shooterParams = new ShooterParams();

    private double curShooterVelTps;

    public static class BallShotTracker {
        private boolean ballsCurrentlyExiting, ballsPreviouslyExiting;
        private boolean shootingInterlockRecentlyActivated;
        private int numBallsShot;
        private final ElapsedTime ballsExitingTimer;
        private double lastMax, lastMin;
        private double prevTargetVelMps;
        private boolean wasPrevIncreasing;

        public BallShotTracker() {
            lastMax = 0;
            lastMin = Double.MAX_VALUE;
            ballsExitingTimer = new ElapsedTime();
        }
        public void updateBallShotTracking(double oneFrameVelDif) {
            ballsPreviouslyExiting = ballsCurrentlyExiting;
            if(ballsCurrentlyExiting && ballsExitingTimer.seconds() > shooterParams.avg3BallShootTime)
                ballsCurrentlyExiting = false;

            boolean increasing;
            if(oneFrameVelDif > 0)
                increasing = true;
            else if(oneFrameVelDif == 0)
                increasing = wasPrevIncreasing;
            else
                increasing = false;

            // TODO: try to figure out 3 ball shoot again
            numBallsShot = 0;
            wasPrevIncreasing = increasing;
        }
    }
    private double pidVoltage, velocityVoltage, totalVoltage;
    private final DcMotorEx lowShooter, highShooter;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        lowShooter = hardwareMap.get(DcMotorEx.class, RobotProperties.lowShooterName);
        lowShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lowShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lowShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        highShooter = hardwareMap.get(DcMotorEx.class, RobotProperties.highShooterName);
        highShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        highShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        highShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setShooterPower(double power) {
        lowShooter.setPower(power);
        highShooter.setPower(power);
    }
    public void setShooterVoltage(double shooterVoltage, double batteryVoltage) {
        setShooterPower(shooterVoltage / batteryVoltage);
    }
    public void setShooterVelocityPID(double targetVelMps, double curVelMps, double batteryVoltage) {
        double error = targetVelMps - curVelMps;
        double kP = shooterParams.A / (1 + Math.exp(shooterParams.k * (Math.abs(error) - shooterParams.x0))) + shooterParams.B;
        pidVoltage = kP * error;
        double kV = shooterParams.kVYInt + shooterParams.kVSlope * targetVelMps;
        velocityVoltage = kV * targetVelMps;
        totalVoltage = pidVoltage + velocityVoltage;

        setShooterVoltage(totalVoltage, batteryVoltage);
    }
    public int getNumBallsShot() {
        return 0;
    }
    public double getVelTps() {
        return curShooterVelTps;
    }
    public void updateProperties() {
        curShooterVelTps = (lowShooter.getVelocity() + highShooter.getVelocity()) * .5;
    }
    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");

        telemetry.addData("  shooter voltage total", totalVoltage);
        telemetry.addData("  shooter voltage pid", pidVoltage);
        telemetry.addData("  shooter voltage velocity", velocityVoltage);
    }
}