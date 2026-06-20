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
        double idealTurretFieldAngleRad,
        Double actualTurretFieldAngleRad
    ) {}

    /**
     * Computes the displaced goal and the last trajectory after several iterations,
     * updating based on the provided function for next trajectory lookup.
     */
    private static TrajectoryGoalInfo computeDisplacedGoal(
        TrajectoryDistanceLUT trajectoryLUT,
        Vector2d goalPos,
        Vector2d turretPos,
        Vector2d turretVel,
        double startDistFromGoal,
        double angleOrSpeed,
        int tofEstimationIterations,
        boolean useImpactAngle // true = lookup by impact angle, false = lookup by exit speed
    ) {
        Vector2d displacedGoal = goalPos;
        Vector2d turretToGoal;
        double distFromGoal = startDistFromGoal;
        Trajectory trajectory = useImpactAngle
            ? trajectoryLUT.getInterpolatedImpactAngleTrajectory(distFromGoal, angleOrSpeed)
            : trajectoryLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, angleOrSpeed);

        if (trajectory == null) return new TrajectoryGoalInfo(null, null, 0.0);

        for (int i = 0; i < tofEstimationIterations; i++) {
            displacedGoal = goalPos.minus(turretVel.times(trajectory.timeOfFlight * 0.85));
            turretToGoal = displacedGoal.minus(turretPos);
            distFromGoal = turretToGoal.norm();

            trajectory = useImpactAngle
                ? trajectoryLUT.getInterpolatedImpactAngleTrajectory(distFromGoal, angleOrSpeed)
                : trajectoryLUT.getInterpolatedExitSpeedTrajectory(distFromGoal, angleOrSpeed);

            if (trajectory == null) return new TrajectoryGoalInfo(null, null, 0.0);
        }
        return new TrajectoryGoalInfo(trajectory, displacedGoal, distFromGoal);
    }

    private static class TrajectoryGoalInfo {
        final Trajectory trajectory;
        final Vector2d displacedGoal;
        final double distFromGoal;
        TrajectoryGoalInfo(Trajectory t, Vector2d g, double d) {
            this.trajectory = t;
            this.displacedGoal = g;
            this.distFromGoal = d;
        }
    }

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
        Vector2d robotToTurretPerp = new Vector2d(-robotToTurret.y, robotToTurret.x*1);

        Vector2d turretVel = robotToTurretPerp.times(robotVel.headingRad).plus(robotVelocity);

        Vector2d turretToGoal = goalPos.minus(turretPos);
        double startDistFromGoal = Math.hypot(turretToGoal.x, turretToGoal.y);

        // Calculate ideal trajectory (by impact angle)
        TrajectoryGoalInfo idealInfo = computeDisplacedGoal(
            trajectoryLUT,
            goalPos,
            turretPos,
            turretVel,
            startDistFromGoal,
            targetImpactAngleRad,
            tofEstimationIterations,
            true);

        if (idealInfo.trajectory == null) return null;

        double idealTurretFieldAngleRad = Math.atan2(
            idealInfo.displacedGoal.minus(turretPos).y, 
            idealInfo.displacedGoal.minus(turretPos).x
        );

        // Calculate actual trajectory (by exit speed)
        TrajectoryGoalInfo actualInfo = computeDisplacedGoal(
            trajectoryLUT,
            goalPos,
            turretPos,
            turretVel,
            idealInfo.distFromGoal,
            currentExitSpeed,
            tofEstimationIterations,
            false
        );

        Double actualTurretFieldAngleRad;
        if (actualInfo.trajectory == null)
            actualTurretFieldAngleRad = null;
        else
            actualTurretFieldAngleRad = Math.atan2(
                actualInfo.displacedGoal.minus(turretPos).y,
                actualInfo.displacedGoal.minus(turretPos).x
            );

        return new TargetingInfo(
            idealInfo.trajectory,
            actualInfo.trajectory,
            actualInfo.displacedGoal, // return displacedGoal from actual/exitSpeed
            idealTurretFieldAngleRad,
            actualTurretFieldAngleRad
        );
    }
}
