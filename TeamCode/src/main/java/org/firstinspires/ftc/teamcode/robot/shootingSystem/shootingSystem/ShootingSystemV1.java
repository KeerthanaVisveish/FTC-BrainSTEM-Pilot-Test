package org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShooterConversion;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMathOld;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt1;
import org.firstinspires.ftc.teamcode.utils.shootingMath.AnswerKeyPt2;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMathNew;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;


public class ShootingSystemV1 extends ShootingSystem {
    private final ShootingMathNew shootingMathNew;
    public ShootingSystemV1(HardwareMap hardwareMap, Telemetry telemetry, Pose2d robotPose, Alliance alliance) {
        super(hardwareMap, telemetry, robotPose, alliance);
        this.shootingMathNew = new ShootingMathNew();
    }
    @Override
    protected LaunchData calculateLaunchTrajectory(Vector2d robotPosIn, Vector2d turretPosIn, Vector3d goalPosIn, Vector2d robotVelocityIps, double impactAngleRad, double shooterVelTps) {
            Vector3d exitPosM = new Vector3d(turretPosIn.x, turretPosIn.y, ShootingMathOld.approximateExitHeightM(false)).times(.0254);
            Vector3d robotPosM = new Vector3d(robotPosIn.x, robotPosIn.y, 0).times(.0254);
            Vector3d goalPosM = new Vector3d(goalPosIn.x, goalPosIn.y, goalPosIn.z).times(.0254);
//            ToDoubleFunction<Double> shooterConversion = exitAngle -> {
//                double e = calcEfficiencyCoef(exitAngle);
//                return ShootingMathOld.ticksPerSecToExitSpeedMps(1, e);
//            };
            ShooterConversion shooterConversion = (encoderSpeed, exitAngle) -> ShootingSystemV2.v2Params.shooterTpsToMpsSlope * encoderSpeed + ShootingSystemV2.v2Params.shooterTpsToMpsIntercept;
            Vector3d robotVelocityMps = new Vector3d(robotVelocityIps.x, robotVelocityIps.y, 0).times(.0254);

            AnswerKeyPt1 answerKeyPt1 = shootingMathNew.godSolvePart1(exitPosM, robotPosM, robotVelocityMps, 0, goalPosM, impactAngleRad, 0);
            AnswerKeyPt2 answerKeyPt2 = shootingMathNew.godSolvePart2(answerKeyPt1, goalPosM, impactAngleRad, shooterVelTps, shooterConversion);

            if(answerKeyPt1.solutionExists) {
                double targetShooterSpeedTps = ShootingMathOld.exitMpsToMotorTicksPerSec(answerKeyPt1.launchVector.speed, calcEfficiencyCoef(answerKeyPt1.launchVector.exitAng));

                double targetTurretFieldAngleRad, targetHoodExitAngleRad;
                if(answerKeyPt2.solutionExists) {
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
}
