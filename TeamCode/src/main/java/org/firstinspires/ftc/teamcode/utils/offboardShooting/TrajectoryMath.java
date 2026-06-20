package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;

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
        Vector2d displacedGoal,
        double turretFieldAngleRad
    ) {}

    public static TargetingInfo calculateTargetingInfo(
        TrajectoryDistanceLUT trajectoryLUT,
        Vector2d centerOfRotation,
        Vector2d turretPos,
        Vector2d goalPos,
        OdoInfo robotVel,
        double currentExitSpeed,
        double targetImpactAngleRad,
        int tofEstimationIterations
    ) {
        Vector2d robotVelocity = new Vector2d(robotVel.x, robotVel.y);

        Vector2d robotToTurret = turretPos.minus(centerOfRotation);
        Vector2d robotToTurretPerp = new Vector2d(-robotToTurret.y, robotToTurret.x *1);

        Vector2d turretVel = robotToTurretPerp.times(robotVel.headingRad).plus(robotVelocity);

        Vector2d displacedGoal = goalPos;
        Vector2d turretToGoal = displacedGoal.minus(turretPos);
        double distFromGoal = Math.hypot(turretToGoal.x, turretToGoal.y);

        Trajectory idealTargetTrajectory = trajectoryLUT.getInterpolatedImpactAngleTrajectory(distFromGoal, targetImpactAngleRad);
        if (idealTargetTrajectory == null)
            return null;

        for (int i = 0; i < tofEstimationIterations; i++) {
            displacedGoal = goalPos.minus(turretVel.times(idealTargetTrajectory.timeOfFlight * 0.85));

            turretToGoal = displacedGoal.minus(turretPos);
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
}
