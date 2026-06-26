package org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter.ShooterV2;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.TrajectoryDistanceLUT;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.TrajectoryMath;
import org.firstinspires.ftc.teamcode.utils.shootingMath.LaunchVector;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMathNew;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.util.ArrayList;

@Config
public class ShootingSystemV2 extends ShootingSystem {
    public static class V2Params {
        public int numIterations = 5;
        public boolean useVelocityCompensation = true;
        public int drawTrajNumPoints = 200;
        public int drawTrajSparsity = 1;
    }
    public static V2Params v2Params = new V2Params();
    private final TrajectoryDistanceLUT trajectoryDistanceLUT;

    private Trajectory mostRecentTrajectory;
    private LaunchData mostRecentLaunchData;
    private Vector2d mostRecentTurretPos;

    public ShootingSystemV2(HardwareMap hardwareMap, Telemetry telemetry, Pose2d robotPose, Alliance alliance) {
        super(hardwareMap, telemetry, robotPose, alliance);

        this.trajectoryDistanceLUT = TrajectoryLoader.loadFromSettingsFile("mtiTrajectories.json");
    }

    @Override
    protected LaunchData calculateLaunchTrajectory(Vector2d robotPosIn, Vector2d turretPosIn, Vector3d goalPosIn, Vector2d robotVelocityIps, double impactAngleRad, double shooterVelTps) {
        mostRecentTurretPos = turretPosIn;
        double distFromGoalM = turretPosIn.minus(new Vector2d(goalPosIn.x, goalPosIn.y)).times(0.0254).norm();

        telemetry.addLine("OFFBOARD SHOOTING INFO======================");
        telemetry.addData("distFromGoalMeters", distFromGoalM);
        boolean distanceInRange = trajectoryDistanceLUT.distanceInRange(distFromGoalM);
        telemetry.addData("distanceInRange", distanceInRange);
        if(distanceInRange) {
            double curExitSpeedMps = getMpsFromTps(shooterVelTps);
            TrajectoryMath.TargetingInfo targetingInfo = TrajectoryMath.calculateTargetingInfo(trajectoryDistanceLUT, turretPosIn, turretPosIn, new Vector2d(goalPosIn.x, goalPosIn.y), new OdoInfo(robotVelocityIps.x, robotVelocityIps.y, 0), curExitSpeedMps, v2Params.numIterations);

            if(targetingInfo == null)
                return null;

            if(targetingInfo.idealTargetTrajectory() == null)
                return null;
            double targetShooterSpeedTps = getTpsFromMps(targetingInfo.idealTargetTrajectory().exitSpeedMps);
            telemetry.addData("targetShooterSpeedTps", targetShooterSpeedTps);
            boolean useActualTraj = v2Params.useVelocityCompensation && targetingInfo.actualTargetTrajectory() != null;
            mostRecentTrajectory = useActualTraj ? targetingInfo.actualTargetTrajectory() : targetingInfo.idealTargetTrajectory();

            double targetHoodExitAngleRad = mostRecentTrajectory.exitAngleRad;
            double targetTurretFieldAngleRad = useActualTraj ? targetingInfo.actualTurretFieldAngleRad() : targetingInfo.idealTurretFieldAngleRad();

            mostRecentLaunchData = new LaunchData(targetShooterSpeedTps, targetHoodExitAngleRad, targetHoodExitAngleRad, targetTurretFieldAngleRad);
            return mostRecentLaunchData;
        }
        return null;
    }
    private double getMpsFromTps(double tps) {
        return ShooterV2.params.getMpsFunction.apply(tps);
    }
    private double getTpsFromMps(double mps) {
        return ShooterV2.params.getTpsFunction.apply(mps);
    }

    @Override
    public void drawShootingInfo(Canvas fieldOverlay) {
        super.drawShootingInfo(fieldOverlay);

        if(mostRecentTrajectory != null) {
            ArrayList<Vector3d> points = mostRecentTrajectory.simulateTrajectory(
                    v2Params.drawTrajNumPoints,
                    v2Params.drawTrajSparsity,
                    mostRecentLaunchData.targetTurretFieldAngleRad(),
                    new Vector3d(mostRecentTurretPos.x, mostRecentTurretPos.y, 0),
                    ShootingMathNew.construct3DVector(new LaunchVector(mostRecentLaunchData.targetShooterSpeedTps(), mostRecentLaunchData.idealExitAngleRad(), mostRecentLaunchData.targetTurretFieldAngleRad())));
            fieldOverlay.setStroke("blue");
            fieldOverlay.strokeLine(points.get(0).x, points.get(0).y, points.get(points.size()-1).x, points.get(points.size()-1).y);
        }
    }

    public boolean onTarget(double distFromGoal, double exitSpeedMps, double exitAngleRad) {
        Trajectory traj1 = trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, exitSpeedMps);
        double exitAngleError = Math.abs(traj1.exitAngleRad - exitAngleRad);
        if (exitAngleError < traj1.exitAngleMOE)
            return true;
        Trajectory traj2 = trajectoryDistanceLUT.getInterpolatedExitAngleTrajectory(distFromGoal, exitAngleRad);
        double exitSpeedError = Math.abs(traj2.exitSpeedMps - exitSpeedMps);
        return exitSpeedError < traj2.exitSpeedMOE;
    }
}
