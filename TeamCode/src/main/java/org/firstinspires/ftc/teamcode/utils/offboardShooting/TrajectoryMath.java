package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

/**
 * Utility methods for applying robot-motion compensation to offboard-generated
 * shooting trajectories.
 *
 * <p>The trajectory database assumes a stationary robot. This class estimates
 * the apparent displacement of the goal caused by robot and turret motion
 * during the projectile's time of flight, allowing the turret to "lead" the
 * target.
 */
public class TrajectoryMath {

    public record TargetingInfo(
        Trajectory idealTargetTrajectory,
        Trajectory actualTargetTrajectory,
        Vector3d displacedGoal,
        double turretFieldAngleRad
    ) {}

    public static TargetingInfo calculateTargetingInfo(
        TrajectoryDistanceLUT trajectoryLUT,
        Pose2d robotPose,
        Pose3d turretPose,
        Vector3d goalPos,
        ChassisSpeeds chassisSpeeds,
        double currentExitSpeed,
        double targetImpactAngleRad,
        int tofEstimationIterations
    ) {
        Vector3d robotVelocity = new Vector3d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            0
        );

        Translation2d robotToTurret = turretPose.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
        Vector3d robotToTurretPerp = new Vector3d(robotToTurret.getX(), robotToTurret.getY(), 0).perpInXY();

        Vector3d turretVel = robotToTurretPerp.times(chassisSpeeds.omegaRadiansPerSecond).add(robotVelocity);

        Vector3d displacedGoal = goalPos;
        Vector3d turretPos = translation3dToVector3d(turretPose.getTranslation());
        Vector3d turretToGoal = displacedGoal.to2D().sub(turretPos.to2D());
        double distFromGoal = Math.hypot(turretToGoal.x, turretToGoal.y);

        Trajectory idealTargetTrajectory = trajectoryLUT.getInterpolatedImpactAngleTrajectory(distFromGoal, targetImpactAngleRad);
        if (idealTargetTrajectory == null)
            return null;

        for (int i = 0; i < tofEstimationIterations; i++) {
            displacedGoal = goalPos.sub(turretVel.times(idealTargetTrajectory.timeOfFlight * 0.85));

            turretToGoal = displacedGoal.to2D().sub(turretPos.to2D());
            distFromGoal = Math.hypot(turretToGoal.x, turretToGoal.y);

            idealTargetTrajectory = trajectoryLUT.getInterpolatedImpactAngleTrajectory(distFromGoal, targetImpactAngleRad);
            if (idealTargetTrajectory == null)
                return null;
        }

        double turretFieldAngleRad = Math.atan2(turretToGoal.y, turretToGoal.x);

        Trajectory actualTargetTrajectory = trajectoryLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, currentExitSpeed);

        return new TargetingInfo(
            idealTargetTrajectory,
            actualTargetTrajectory,
            displacedGoal,
            turretFieldAngleRad
        );
    }

    private static Vector3d translation3dToVector3d(Translation3d translation) {
        return new Vector3d(translation.getX(), translation.getY(), translation.getZ());
    }
}
