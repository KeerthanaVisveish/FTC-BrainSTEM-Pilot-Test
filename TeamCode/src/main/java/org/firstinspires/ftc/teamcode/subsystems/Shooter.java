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
        public double testingVel = 5.15;
        public double testingExitAngleRad = 1.0472;
    }

    public static ShooterParams shooterParams = new ShooterParams();
    public static TestingParams testingParams = new TestingParams();

    public enum ShooterState {
        OFF, UPDATE
    }
    private ShooterState shooterState, prevFrameShooterState;
    private double nearVelocityAdjustment, farVelocityAdjustment;
    private double curRawShooterSpeedTps;
    private boolean ballsCurrentlyExiting, ballsPreviouslyExiting;
    private boolean a, shootingInterlockRecentlyActivated;
    private int numBallsShot;
    private final ElapsedTime ballsExitingTimer;
    private double lastMax, lastMin;
    private double prevTargetVelMps;
    private double oneFrameVelDif;
    private boolean wasPrevIncreasing;
    private double kP;
    private double pidVoltage, velocityVoltage, totalVoltage;
    private double targetVelMps;
    private double lastUpdatedExitAng;
    private double maxVoltage;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterState = ShooterState.OFF;
        lastMax = 0;
        lastMin = Double.MAX_VALUE;
        nearVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        farVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        ballsExitingTimer = new ElapsedTime();
        maxVoltage = shooterParams.maxVoltage;
        a = true;
    }
    public void setShooterVelocityPID(double targetVelMps, double curVelMps) {
        if(prevFrameShooterState == ShooterState.OFF) {
            robot.shootingSystem.setShooterVoltage(shooterParams.firstCurrentLimitedVoltage);
            return;
        }
        double error = targetVelMps - curVelMps;
        kP =  shooterParams.A / (1 + Math.exp(shooterParams.k * (Math.abs(error) - shooterParams.x0))) + shooterParams.B;
        pidVoltage = kP * error;
        double kV = shooterParams.kVYInt + shooterParams.kVSlope * targetVelMps;
        velocityVoltage = kV * targetVelMps;
        totalVoltage = pidVoltage + velocityVoltage;

        totalVoltage = Range.clip(totalVoltage, shooterParams.minVoltage, maxVoltage);
        robot.shootingSystem.setShooterVoltage(totalVoltage);
    }
    public void setMaxVoltage(double m) {
        maxVoltage = m;
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
                    if (robot.shootingSystem.locationState != ShootingSystem.Location.FAR)
                        targetVelMps = robot.shootingSystem.lookAheadTargetExitSpeedMps + nearVelocityAdjustment;
                    else
                        targetVelMps = robot.shootingSystem.lookAheadTargetExitSpeedMps + farVelocityAdjustment;
                    setShooterVelocityPID(targetVelMps, robot.shootingSystem.curExitSpeedMps);
                }
                break;
        }
        if(testingParams.testing) {
            robot.shootingSystem.setHoodPosition(ShootingMathOld.getHoodServoPosition(testingParams.testingExitAngleRad));
        }
        else if((robot.shootingSystem.checkShootingWhileMoving
                || robot.shootingSystem.physicsExitAngleRads[0] != -1
                || robot.shootingSystem.robotSpeedAtTurretIps > ShootingSystem.hoodParams.robotVelThresholdToSetHood)
        && Math.abs(robot.shootingSystem.hoodExitAngleRad - lastUpdatedExitAng) > shooterParams.ignoreHoodUpdateError) {
            double targetHoodPos = ShootingMathOld.getHoodServoPosition(robot.shootingSystem.hoodExitAngleRad);
            robot.shootingSystem.setHoodPosition(targetHoodPos);
            lastUpdatedExitAng = robot.shootingSystem.hoodExitAngleRad;
        }
        updateBallShotTracking();
        prevTargetVelMps = targetVelMps;
        prevFrameShooterState = shooterState;
    }
    public void updateBallShotTracking() {
        ballsPreviouslyExiting = ballsCurrentlyExiting;
        if(ballsCurrentlyExiting && ballsExitingTimer.seconds() > shooterParams.avg3BallShootTime)
            ballsCurrentlyExiting = false;

        double prevRawShooterSpeedTps = curRawShooterSpeedTps;
        curRawShooterSpeedTps = robot.shootingSystem.getRawShooterSpeedTps();
        oneFrameVelDif = curRawShooterSpeedTps - prevRawShooterSpeedTps;

        boolean increasing;
        if(oneFrameVelDif > 0)
            increasing = true;
        else if(oneFrameVelDif == 0)
            increasing = wasPrevIncreasing;
        else
            increasing = false;

        if(robot.collector.getClutchState() == Collector.ClutchState.ENGAGED && robot.collector.getCollectionState() == Collector.CollectionState.INTAKE) {
            if(!robot.collector.isShooting())
                shootingInterlockRecentlyActivated = true;

            if(increasing && !wasPrevIncreasing && numBallsShot < 3) {  // means relative min detected
                lastMin = robot.shootingSystem.getPrevFilteredShooterSpeedTps();
                double velDrop = lastMax - lastMin;
                if (((numBallsShot != 0 && !shootingInterlockRecentlyActivated) || velDrop >= shooterParams.shotVelDropThreshold) && Math.abs(targetVelMps - prevTargetVelMps) < shooterParams.targetVelStaticShotThreshold) {
                    if (!ballsCurrentlyExiting) {
                        ballsCurrentlyExiting = true;
                        ballsExitingTimer.reset();
                    }
                    numBallsShot++;
                    shootingInterlockRecentlyActivated = false;
                }
            }
        }
        else
            numBallsShot = 0;

        if(wasPrevIncreasing && !increasing) { // means relative max detected
            lastMax = robot.shootingSystem.getPrevFilteredShooterSpeedTps();
        }
        wasPrevIncreasing = increasing;
    }
    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("num balls shot", numBallsShot);
        telemetry.addData("balls currently exiting", ballsCurrentlyExiting ? 50 : 0);
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
        if (robot.shootingSystem.locationState != ShootingSystem.Location.FAR)
            nearVelocityAdjustment += amount;
        else
            farVelocityAdjustment += amount;
    }
    public double getCurVelocityAdjustment() {
        return robot.shootingSystem.locationState == ShootingSystem.Location.FAR ? farVelocityAdjustment : nearVelocityAdjustment;
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
    public int getNumBallsShot() {
        return numBallsShot;
    }
}