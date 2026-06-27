package org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
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
        public double x1 = 105, y1 = 0;
        public double x2 = 150, y2 = 110;
        public boolean useDynamicHoodFar = false, useDynamicHoodNear = false;
    }
    public static Params params = new Params();
    private final ShootingMathNew shootingMathNew;
    private AnswerKeyPt1 answerKeyPt1;
    private AnswerKeyPt2 answerKeyPt2;
    private LaunchData prevValidPt2Data;

    private InterpLUT shooterLookup, hoodLookup;

    public ShootingSystemV1(HardwareMap hardwareMap, Telemetry telemetry, Pose2d robotPose, Alliance alliance) {
        super(hardwareMap, telemetry, robotPose, alliance);
        this.shootingMathNew = new ShootingMathNew();

        shooterLookup = new InterpLUT();
        hoodLookup = new InterpLUT();

        shooterLookup.add(0, 540);
        hoodLookup.add(0, 65);

        shooterLookup.add(40, 540);
        hoodLookup.add(40, 65);

        shooterLookup.add(49, 540);
        hoodLookup.add(49, 63);

        shooterLookup.add(68, 590);
        hoodLookup.add(68, 53);

        shooterLookup.add(88, 620);
        hoodLookup.add(88, 47);

        shooterLookup.add(107, 680);
        hoodLookup.add(107, 47);

        shooterLookup.add(121, 770);
        hoodLookup.add(121, 47);

        shooterLookup.add(130, 790);
        hoodLookup.add(130, 45);

        shooterLookup.add(132, 780);
        hoodLookup.add(132, 45);

        shooterLookup.add(140, 790);
        hoodLookup.add(140, 45);

        shooterLookup.add(150, 800);
        hoodLookup.add(150, 45);

        shooterLookup.add(155, 810);
        hoodLookup.add(155, 45);
        shooterLookup.add(1000000, 810);
        hoodLookup.add(10000000, 45);

        shooterLookup.createLUT();
        hoodLookup.createLUT();

    }
    @Override
    protected LaunchData calculateLaunchTrajectory(Vector2d robotPosIn, Vector2d turretPosIn, Vector3d goalPosIn, Vector2d robotVelocityIps, double impactAngleRad, double shooterVelTps) {
        if(true) {
            Vector2d relGoal = new Vector2d(goalPosIn.x, goalPosIn.y).minus(turretPosIn);
            double e = Math.toRadians(hoodLookup.get(distFromGoal));
            return new LaunchData(shooterLookup.get(distFromGoal), e, e, Math.atan2(relGoal.y, relGoal.x));
        }
        Vector3d exitPosM = new Vector3d(turretPosIn.x * .0254, turretPosIn.y * .0254, ShootingMathOld.approximateExitHeightM(false));
            Vector3d robotPosM = new Vector3d(robotPosIn.x, robotPosIn.y, 0).times(.0254);
            Vector3d goalPosM = new Vector3d(goalPosIn.x, goalPosIn.y, goalPosIn.z).times(.0254);
            double goalDist = turretPosIn.minus(new Vector2d(goalPosIn.x, goalPosIn.y)).norm();
            double dragCompensation = getDragCompensation(goalDist);
            ShooterConversion shooterConversion = (encoderSpeed, exitAngle) -> ShooterV2.params.getMpsFunction.apply(encoderSpeed - dragCompensation);
            Vector3d robotVelocityMps = new Vector3d(robotVelocityIps.x, robotVelocityIps.y, 0).times(.0254);

            answerKeyPt1 = shootingMathNew.godSolvePart1(exitPosM, robotPosM, robotVelocityMps, 0, goalPosM, impactAngleRad, 0);
            answerKeyPt2 = shootingMathNew.godSolvePart2(answerKeyPt1, goalPosM, impactAngleRad, shooterVelTps, shooterConversion);

            if(answerKeyPt1.solutionExists) {
                // y - y1 = m(x - x1)
                // y = mx - mx1 + y1
                double targetShooterSpeedTps = ShooterV2.params.getTpsFunction.apply(answerKeyPt1.launchVector.speed) + dragCompensation;

                boolean useDynamicHood = getLocationState() == Location.FAR || getLocationState() == Location.OPPOSITE_SIDE ? params.useDynamicHoodFar : params.useDynamicHoodNear;

                if(answerKeyPt2.solutionExists && useDynamicHood) {
                    prevValidPt2Data = new LaunchData(targetShooterSpeedTps, answerKeyPt1.launchVector.exitAng, answerKeyPt2.launchVector.exitAng, answerKeyPt2.launchVector.turretAng);
                    return prevValidPt2Data;
                }
                else if(useDynamicHood && prevValidPt2Data != null)
                    return new LaunchData(targetShooterSpeedTps, answerKeyPt1.launchVector.exitAng, prevValidPt2Data.compensatedExitAngleRad(), prevValidPt2Data.targetTurretFieldAngleRad());
                else {
                    return new LaunchData(targetShooterSpeedTps, answerKeyPt1.launchVector.exitAng, answerKeyPt1.launchVector.exitAng, answerKeyPt1.launchVector.turretAng);
                }
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
        if(answerKeyPt1 != null && answerKeyPt2 != null) {
            telemetry.addData("answer key pt1 exists", answerKeyPt1.solutionExists ? 1 : 0);
            telemetry.addData("answer key pt2 exists", answerKeyPt2.solutionExists ? 2 : 0);
        }
    }
}
