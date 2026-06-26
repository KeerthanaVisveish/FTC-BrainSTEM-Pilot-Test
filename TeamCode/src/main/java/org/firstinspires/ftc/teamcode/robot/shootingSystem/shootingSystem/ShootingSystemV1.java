package org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter.ShooterV2;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShooterConversion;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMathOld;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt1;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt2;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMathNew;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;


@Config
public class ShootingSystemV1 extends ShootingSystem {
    public static class Params {
        public double x1 = 110, y1 = 0;
        public double x2 = 150, y2 = 80;
        public boolean useDynamicHood = true;
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
    }
    public static Params params = new Params();
    private final ShootingMathNew shootingMathNew;
    private AnswerKeyPt1 answerKeyPt1;
    private AnswerKeyPt2 answerKeyPt2;
    public ShootingSystemV1(HardwareMap hardwareMap, Telemetry telemetry, Pose2d robotPose, Alliance alliance) {
        super(hardwareMap, telemetry, robotPose, alliance);
        this.shootingMathNew = new ShootingMathNew();
    }
    @Override
    protected LaunchData calculateLaunchTrajectory(Vector2d robotPosIn, Vector2d turretPosIn, Vector3d goalPosIn, Vector2d robotVelocityIps, double impactAngleRad, double shooterVelTps) {
            Vector3d exitPosM = new Vector3d(turretPosIn.x, turretPosIn.y, ShootingMathOld.approximateExitHeightM(false)).times(.0254);
            Vector3d robotPosM = new Vector3d(robotPosIn.x, robotPosIn.y, 0).times(.0254);
            Vector3d goalPosM = new Vector3d(goalPosIn.x, goalPosIn.y, goalPosIn.z).times(.0254);
            double goalDist = turretPosIn.minus(new Vector2d(goalPosIn.x, goalPosIn.y)).norm();
            ShooterConversion shooterConversion = (encoderSpeed, exitAngle) -> ShooterV2.params.getMpsFunction.apply(encoderSpeed - getDragCompensation(goalDist));
            Vector3d robotVelocityMps = new Vector3d(robotVelocityIps.x, robotVelocityIps.y, 0).times(.0254);

            answerKeyPt1 = shootingMathNew.godSolvePart1(exitPosM, robotPosM, robotVelocityMps, 0, goalPosM, impactAngleRad, 0);
            answerKeyPt2 = shootingMathNew.godSolvePart2(answerKeyPt1, goalPosM, impactAngleRad, shooterVelTps, shooterConversion);

            if(answerKeyPt1.solutionExists) {
                // y - y1 = m(x - x1)
                // y = mx - mx1 + y1
                double targetShooterSpeedTps = ShooterV2.params.getTpsFunction.apply(answerKeyPt1.launchVector.speed) + getDragCompensation(goalDist);

                double targetTurretFieldAngleRad, targetHoodExitAngleRad;
                boolean useDynamicHood = params.useDynamicHood;
                if(answerKeyPt2.solutionExists && useDynamicHood) {
                    targetTurretFieldAngleRad = answerKeyPt2.launchVector.turretAng;
                    targetHoodExitAngleRad = answerKeyPt2.launchVector.exitAng;
                }
                else {
                    targetTurretFieldAngleRad = answerKeyPt1.launchVector.turretAng;
                    targetHoodExitAngleRad = answerKeyPt1.launchVector.exitAng;
                }
                return new LaunchData(targetShooterSpeedTps, targetHoodExitAngleRad, targetTurretFieldAngleRad);
            }
            return null;

    }

    private double getDragCompensation(double goalDist) {
        double slope = (params.y2 - params.y1) / (params.x2 - params.x1);
        double intercept = -slope * params.x1 + params.y1;
        return Math.max(slope * goalDist + intercept, 0);
    }

    @Override
    public void printInfo() {
        super.printInfo();
        telemetry.addData("answer key pt1 exists", answerKeyPt1.solutionExists ? 1 : 0);
        telemetry.addData("answer key pt2 exists", answerKeyPt2.solutionExists ? 2 : 0);
    }

    public boolean onTarget(double distFromGoal, double launchSpeedMps, double exitAngleRad) {
        return true;
    }
}
