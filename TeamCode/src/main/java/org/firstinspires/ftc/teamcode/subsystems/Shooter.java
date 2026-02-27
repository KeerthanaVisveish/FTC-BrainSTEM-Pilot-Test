package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double kP = 0.5;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kV = 0.122;
        public double shotVelDropThreshold = 30;
        public double avg3BallShootTime = .7;
        public int startingShooterSpeedAdjustment = 0;
        public double minPower = -0.15, maxPower = 0.99;
        public double shotRecoveryPower = 0.99, shotRecoveryError = 0.08;
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

    private final PIDController shooterPID;
    private double nearVelocityAdjustment, farVelocityAdjustment;
    private boolean ballsCurrentlyExiting, ballsPreviouslyExiting;
    private final ElapsedTime ballsExitingTimer;
    private double lastMax, lastMin;
    private boolean wasPrevIncreasing;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterPID = new PIDController(shooterParams.kP, shooterParams.kI, shooterParams.kD);

        shooterState = ShooterState.OFF;
        lastMax = 0;
        lastMin = Double.MAX_VALUE;
        nearVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        farVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        ballsExitingTimer = new ElapsedTime();
    }
    public void setShooterVelocityPID(double targetVelocityMps, double currentShooterVelocityMps) {
        if (robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
            shooterPID.setTarget(targetVelocityMps + nearVelocityAdjustment);
        else
            shooterPID.setTarget(targetVelocityMps + farVelocityAdjustment);

        double pidOutput = -shooterPID.update(currentShooterVelocityMps);
        double feedForward = shooterParams.kV * targetVelocityMps;
        double totalPower = pidOutput + feedForward;

        totalPower = Range.clip(totalPower, shooterParams.minPower, shooterParams.maxPower);
        double error = targetVelocityMps - currentShooterVelocityMps;
        if(error > shooterParams.shotRecoveryError)
            totalPower = shooterParams.shotRecoveryPower;

        robot.shootingSystem.setShooterPower(totalPower);
    }

    @Override
    public void update(){
        switch (shooterState) {
            case OFF:
                robot.shootingSystem.setShooterPower(0);
                break;

            case UPDATE:
                if(testingParams.testing)
                    setShooterVelocityPID(testingParams.testingVel, robot.shootingSystem.curExitSpeedMps);
                else
                    setShooterVelocityPID(robot.shootingSystem.actualTargetExitSpeedMps, robot.shootingSystem.curExitSpeedMps);
                break;
        }
        if(testingParams.testing) {
            robot.shootingSystem.setHoodPosition(ShootingMath.getHoodServoPosition(testingParams.testingExitAngleRad));
        }
        else if(robot.shootingSystem.checkShootingWhileMoving
                || robot.shootingSystem.physicsExitAngleRads[0] != -1
                || robot.shootingSystem.robotSpeedAtTurretIps > ShootingSystem.hoodParams.robotVelThresholdToSetHood)
            robot.shootingSystem.setHoodPosition(ShootingMath.getHoodServoPosition(robot.shootingSystem.hoodExitAngleRad));
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
        telemetry.addData("  pid target vel", shooterPID.getTarget());
        telemetry.addData("  shooter power", robot.shootingSystem.getShooterPower());
        telemetry.addData("  shooter filtered vel tps", robot.shootingSystem.getFilteredShooterSpeedTps());
        telemetry.addData("  shooter raw vel tps", robot.shootingSystem.getRawShooterSpeedTps());
        telemetry.addData("  shooter filtered vel mps", robot.shootingSystem.curExitSpeedMps);

        telemetry.addLine();
        telemetry.addLine("HOOD------");
        telemetry.addData("  hood pos", robot.shootingSystem.getHoodPosition());
        telemetry.addData("  testing exit angle", Math.toDegrees(testingParams.testingExitAngleRad));
    }

    public void changeVelocityAdjustment(double amount) {
        if (robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
            nearVelocityAdjustment += amount;
        farVelocityAdjustment += amount;
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