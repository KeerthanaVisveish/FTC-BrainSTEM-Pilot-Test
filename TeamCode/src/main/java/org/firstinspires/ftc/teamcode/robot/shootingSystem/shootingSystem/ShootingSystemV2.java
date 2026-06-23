package org.firstinspires.ftc.teamcode.robot.shootingSystem.shootingSystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.TrajectoryDistanceLUT;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.TrajectoryMath;
import org.firstinspires.ftc.teamcode.utils.shootingMath.LaunchVector;
import org.firstinspires.ftc.teamcode.utils.shootingMath.ShootingMathNew;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.util.ArrayList;

@Config
public class ShootingSystemV2 extends ShootingSystem {
    public static class V2Params {
        //y=0.0042825x+3.38083
        public double shooterTpsToMpsSlope = 0.0042825;
        public double shooterTpsToMpsIntercept = 3.38083;
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

        this.trajectoryDistanceLUT = new TrajectoryDistanceLUT("", new ArrayList<>());
    }

    @Override
    protected LaunchData calculateLaunchTrajectory(Vector2d robotPosIn, Vector2d turretPosIn, Vector3d goalPosIn, Vector2d robotVelocityIps, double impactAngleRad, double shooterVelTps) {
        mostRecentTurretPos = turretPosIn;
        double distFromGoalM = turretPosIn.minus(new Vector2d(goalPosIn.x, goalPosIn.y)).times(0.0254).norm();

        if(trajectoryDistanceLUT.distanceInRange(distFromGoalM)) {
            double curExitSpeedMps = getMpsFromTps(shooterVelTps);
            TrajectoryMath.TargetingInfo targetingInfo = TrajectoryMath.calculateTargetingInfo(trajectoryDistanceLUT, turretPosIn, turretPosIn, new Vector2d(goalPosIn.x, goalPosIn.y), new OdoInfo(robotVelocityIps.x, robotVelocityIps.y, 0), curExitSpeedMps, impactAngleRad, v2Params.numIterations);

            if(targetingInfo == null)
                return null;

            if(targetingInfo.idealTargetTrajectory() == null)
                return null;
            double targetShooterSpeedTps = getTpsFromMps(targetingInfo.idealTargetTrajectory().exitSpeedMps);

            boolean useActualTraj = v2Params.useVelocityCompensation && targetingInfo.actualTargetTrajectory() != null;
            mostRecentTrajectory = useActualTraj ? targetingInfo.actualTargetTrajectory() : targetingInfo.idealTargetTrajectory();

            double targetHoodExitAngleRad = mostRecentTrajectory.exitAngleRad;
            double targetTurretFieldAngleRad = useActualTraj ? targetingInfo.actualTurretFieldAngleRad() : targetingInfo.idealTurretFieldAngleRad();

            mostRecentLaunchData = new LaunchData(targetShooterSpeedTps, targetHoodExitAngleRad, targetTurretFieldAngleRad);
            return mostRecentLaunchData;
        }
        return null;
    }
    private double getMpsFromTps(double tps) {
        return v2Params.shooterTpsToMpsSlope * tps + v2Params.shooterTpsToMpsIntercept;
    }
    private double getTpsFromMps(double mps) {
        return (mps - v2Params.shooterTpsToMpsIntercept) / v2Params.shooterTpsToMpsSlope;
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
                    ShootingMathNew.construct3DVector(new LaunchVector(mostRecentLaunchData.targetShooterSpeedTps(), mostRecentLaunchData.targetHoodExitAngleRad(), mostRecentLaunchData.targetTurretFieldAngleRad())));
            fieldOverlay.setStroke("blue");
            fieldOverlay.strokeLine(points.get(0).x, points.get(0).y, points.get(points.size()-1).x, points.get(points.size()-1).y);
        }
    }
}
