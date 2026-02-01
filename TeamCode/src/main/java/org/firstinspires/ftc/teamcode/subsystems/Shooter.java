package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

import java.util.ArrayList;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double kP = 0.5;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kV = 0.122;
        public double shotVelDropThreshold = 0.05;
        public double noiseVariance = 0.03;
        public int startingShooterSpeedAdjustment = 0;
        public double minPower = -0.15, maxPower = 0.99;
        public double shotRecoveryPower = 0.99, shotRecoveryError = 0.08;
        public boolean disableHoodInterlock = true;
    }
    public static class TestingParams {
        public boolean testing = false;
        public double testingVel = 1500;
        public double testingExitAngleRad = 1.0472;
    }

    public static ShooterParams shooterParams = new ShooterParams();
    public static TestingParams testingParams = new TestingParams();

    public enum ShooterState {
        OFF, UPDATE
    }
    public ShooterState shooterState;

    public final PIDController shooterPID;
    private double nearVelocityAdjustment, farVelocityAdjustment;
    private int ballsShot;
    public double lastMax, lastMin, lastDecel, velDropTime;
    private final ArrayList<Double> allVelDrops, allPostShotVels, allLastDecels, allVelDropTimes;
    private double mSOfLastMax;
    private boolean increasing, wasPrevIncreasing;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterPID = new PIDController(shooterParams.kP, shooterParams.kI, shooterParams.kD);

        shooterState = ShooterState.OFF;
        lastMax = 0;
        lastMin = Double.MAX_VALUE;
        ballsShot = 0;
        nearVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        farVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;

        allVelDrops = new ArrayList<>();
        allPostShotVels = new ArrayList<>();
        allLastDecels = new ArrayList<>();
        allVelDropTimes = new ArrayList<>();
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
                    setShooterVelocityPID(testingParams.testingVel, robot.shootingSystem.filteredShooterSpeedTps);
                else
                    setShooterVelocityPID(robot.shootingSystem.actualTargetExitSpeedMps, robot.shootingSystem.curExitSpeedMps);
                break;
        }
        if(testingParams.testing) {
            robot.shootingSystem.setHoodPosition(ShootingMath.getHoodServoPosition(testingParams.testingExitAngleRad));
        }
        else if(robot.shootingSystem.physicsExitAngleRads[0] != -1 || shooterParams.disableHoodInterlock)
            robot.shootingSystem.setHoodPosition(ShootingMath.getHoodServoPosition(robot.shootingSystem.hoodExitAngleRad));
        updateBallShotTracking();
    }
    public void updateBallShotTracking() {
        double dif = robot.shootingSystem.filteredShooterSpeedTps - robot.shootingSystem.getPrevShooterVelTps();
        if(dif > 0)
            increasing = true;
        else if(dif == 0)
            increasing = wasPrevIncreasing;
        else
            increasing = false;

        if(increasing && !wasPrevIncreasing) {  // means relative min detected
            lastMin = robot.shootingSystem.getPrevShooterVelTps();
            double velDrop = lastMax - lastMin;
            velDropTime = (System.currentTimeMillis() - mSOfLastMax) / 1000;
            lastDecel = velDrop / velDropTime;
            if(velDrop >= shooterParams.shotVelDropThreshold
            || shooterPID.getTarget() - lastMin >= shooterParams.noiseVariance) {
                ballsShot++;
                allVelDrops.add(velDrop);
                allPostShotVels.add(lastMin);
                allLastDecels.add(lastDecel);
                allVelDropTimes.add(velDropTime);
            }
        }
        if(wasPrevIncreasing && !increasing) { // means relative max detected
            lastMax = robot.shootingSystem.getPrevShooterVelTps();
            mSOfLastMax = System.currentTimeMillis();
        }
        wasPrevIncreasing = increasing;

        if(robot.collection.getClutchState() != Collection.ClutchState.ENGAGED) {
            ballsShot = 0;
            allVelDrops.clear();
            allPostShotVels.clear();
            allLastDecels.clear();
        }
    }
    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("  pid target vel", shooterPID.getTarget());
        telemetry.addData("  shooter power", robot.shootingSystem.getShooterPower());
        telemetry.addData("  shooter filtered vel tps", robot.shootingSystem.filteredShooterSpeedTps);
        telemetry.addData("  shooter raw vel tps", robot.shootingSystem.rawShooterSpeedTps);
        telemetry.addData("  shooter filtered vel mps", robot.shootingSystem.curExitSpeedMps);

        telemetry.addLine();
        telemetry.addLine("HOOD------");
        telemetry.addData("  hood pos", robot.shootingSystem.getHoodPosition());
    }
    public void setBallsShot(int n) {
        ballsShot = n;
    }
    public int getBallsShot() {
        return ballsShot;
    }

    public void changeVelocityAdjustment(double amount) {
        if (robot.shootingSystem.distState != ShootingSystem.Dist.FAR)
            nearVelocityAdjustment += amount;
        farVelocityAdjustment += amount;
    }
}
