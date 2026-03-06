package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double fineAdjust = .05;
        public double ignoreHoodUpdateError = Math.toRadians(.5);
//        public double kP = 0.5;
//        public double kP = 6.75;
        public double A = 20, B = 6, k = -150, x0 = .06;
        public double kI = 0.0;
        public double kD = 0.0;
//        public double kV = 0.122;
        public double kVYInt = 1.63, kVSlope = -0.02005;
        public double shotVelDropThreshold = 30;
        public double avg3BallShootTime = .7;
        public int startingShooterSpeedAdjustment = 0;
//        public double minPower = -0.15, maxPower = 0.99;
        public double minVoltage = -2.025, maxVoltage = 15;
//        public double shotRecoveryPower = 0.99;
        public double shotRecoveryVoltage = 15;
        public double shotRecoveryError = 0.08; // in meters per second
    }
    public static class TestingParams {
        public boolean testing = false;
        public double testingVel = 5.15;
        public double testingExitAngleRad = 1.0472;
    }

    public static ShooterParams shooterParams = new ShooterParams();
    public static TestingParams testingParams = new TestingParams();

    public enum ShooterState {
        OFF, UPDATE
    }
    private ShooterState shooterState;
    private double nearVelocityAdjustment, farVelocityAdjustment;
    private boolean ballsCurrentlyExiting, ballsPreviouslyExiting;
    private final ElapsedTime ballsExitingTimer;
    private double lastMax, lastMin;
    private boolean wasPrevIncreasing;
    private double kP;
    private double pidVoltage, velocityVoltage, totalVoltage;
    private double targetVelMps;
    private double lastUpdatedExitAng;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterState = ShooterState.OFF;
        lastMax = 0;
        lastMin = Double.MAX_VALUE;
        nearVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        farVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        ballsExitingTimer = new ElapsedTime();
    }
    public void setShooterVelocityPID(double targetVelMps, double curVelMps) {
        double error = targetVelMps - curVelMps;
        kP =  shooterParams.A / (1 + Math.exp(shooterParams.k * (Math.abs(error) - shooterParams.x0))) + shooterParams.B;
        pidVoltage = kP * error;
        double kV = shooterParams.kVYInt + shooterParams.kVSlope * targetVelMps;
        velocityVoltage = kV * targetVelMps;
        totalVoltage = pidVoltage + velocityVoltage;

        totalVoltage = Range.clip(totalVoltage, shooterParams.minVoltage, shooterParams.maxVoltage);
        robot.shootingSystem.setShooterVoltage(totalVoltage);
    }

    @Override
    public void update(){
        switch (shooterState) {
            case OFF:
                robot.shootingSystem.setShooterVoltage(0);
                break;

            case UPDATE:
                if(testingParams.testing)
                    setShooterVelocityPID(testingParams.testingVel, robot.shootingSystem.curExitSpeedMps);
                else {
                    if (robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
                        targetVelMps = robot.shootingSystem.lookAheadTargetExitSpeedMps + nearVelocityAdjustment;
                    else
                        targetVelMps = robot.shootingSystem.lookAheadTargetExitSpeedMps + farVelocityAdjustment;
                    setShooterVelocityPID(targetVelMps, robot.shootingSystem.curExitSpeedMps);
                }
                break;
        }
        if(testingParams.testing) {
            robot.shootingSystem.setHoodPosition(ShootingMath.getHoodServoPosition(testingParams.testingExitAngleRad));
        }
        else if((robot.shootingSystem.checkShootingWhileMoving
                || robot.shootingSystem.physicsExitAngleRads[0] != -1
                || robot.shootingSystem.robotSpeedAtTurretIps > ShootingSystem.hoodParams.robotVelThresholdToSetHood)
        && Math.abs(robot.shootingSystem.hoodExitAngleRad - lastUpdatedExitAng) > shooterParams.ignoreHoodUpdateError) {
            double targetHoodPos = ShootingMath.getHoodServoPosition(robot.shootingSystem.hoodExitAngleRad);
            robot.shootingSystem.setHoodPosition(targetHoodPos);
            lastUpdatedExitAng = robot.shootingSystem.hoodExitAngleRad;
        }
        updateBallShotTracking();
    }
    public void updateBallShotTracking() {
        ballsPreviouslyExiting = ballsCurrentlyExiting;
        if(ballsCurrentlyExiting && ballsExitingTimer.seconds() > shooterParams.avg3BallShootTime)
            ballsCurrentlyExiting = false;

        double dif = robot.shootingSystem.getFilteredShooterSpeedTps() - robot.shootingSystem.getPrevFilteredShooterSpeedTps();
        boolean increasing;
        if(dif > 0)
            increasing = true;
        else if(dif == 0)
            increasing = wasPrevIncreasing;
        else
            increasing = false;

        if(increasing && !wasPrevIncreasing) {  // means relative min detected
            lastMin = robot.shootingSystem.getPrevFilteredShooterSpeedTps();
            double velDrop = lastMax - lastMin;
            if(velDrop >= shooterParams.shotVelDropThreshold) {
                if(robot.collection.getClutchState() == Collection.ClutchState.ENGAGED && robot.collection.getCollectionState() == Collection.CollectionState.INTAKE && !ballsCurrentlyExiting) {
                    ballsCurrentlyExiting = true;
                    ballsExitingTimer.reset();
                }
            }
        }
        if(wasPrevIncreasing && !increasing) { // means relative max detected
            lastMax = robot.shootingSystem.getPrevFilteredShooterSpeedTps();
        }
        wasPrevIncreasing = increasing;
    }
    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("  shooter lookahead target vel", targetVelMps);
        telemetry.addData("  shooter no lookahead target vel mps", robot.shootingSystem.currentTargetExitSpeedMps);
        telemetry.addData("  shooter voltage total", totalVoltage);
        telemetry.addData("  shooter voltage pid", pidVoltage);
        telemetry.addData("  shooter voltage velocity", velocityVoltage);
        telemetry.addData("  shooter filtered vel tps", robot.shootingSystem.getFilteredShooterSpeedTps());
        telemetry.addData("  shooter raw vel tps", robot.shootingSystem.getRawShooterSpeedTps());
        telemetry.addData("  shooter low vel", robot.shootingSystem.getShooterLowRawVelTps());
        telemetry.addData("  shooter high vel", robot.shootingSystem.getShooterHighRawVelTps());
        telemetry.addData("  shooter high power", robot.shootingSystem.getShooterHighPower());
        telemetry.addData("  shooter low power", robot.shootingSystem.getShooterLowPower());
        telemetry.addData("  shooter filtered vel mps", robot.shootingSystem.curExitSpeedMps);
        telemetry.addData("  shooterGood", robot.shootingSystem.shooterNormGood());
        telemetry.addData("  near vel adjustment", nearVelocityAdjustment);
        telemetry.addData("  far vel adjustment", farVelocityAdjustment);
        telemetry.addLine();
        telemetry.addLine("HOOD------");
        telemetry.addData("  hood pos", robot.shootingSystem.getHoodPosition());
        telemetry.addData("  testing exit angle", Math.toDegrees(testingParams.testingExitAngleRad));
    }

    public void changeVelocityAdjustment(double amount) {
        if (robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
            nearVelocityAdjustment += amount;
        else
            farVelocityAdjustment += amount;
    }
    public double getCurVelocityAdjustment() {
        return robot.shootingSystem.distState == ShootingSystem.Dist.FAR ? farVelocityAdjustment : nearVelocityAdjustment;
    }
    public ShooterState getShooterState() {
        return shooterState;
    }
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }
    public boolean ballsCurrentlyExiting() {
        return ballsCurrentlyExiting;
    }
    public boolean ballsDoneExiting() {
        return !ballsCurrentlyExiting && ballsPreviouslyExiting;
    }
}