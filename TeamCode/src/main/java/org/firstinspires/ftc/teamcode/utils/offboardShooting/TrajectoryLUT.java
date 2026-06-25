package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import java.util.ArrayList;
import java.util.Comparator;

public class TrajectoryLUT {
    public final double distFromGoal;
    public final double relGoalHeight;
    public final double dragCoef;
    public final double magnusCoef;

    private final ArrayList<Trajectory> impactSortedTrajectories;
    private final ArrayList<Trajectory> speedSortedTrajectories;
    private final Trajectory optimalTrajectory;

    public TrajectoryLUT(
            double distFromGoal,
            double relGoalHeight,
            double dragCoef,
            double magnusCoef,
            int optimalTrajectoryIndex,
            ArrayList<Trajectory> trajectories) {
        this.distFromGoal = distFromGoal;
        this.relGoalHeight = relGoalHeight;
        this.dragCoef = dragCoef;
        this.magnusCoef = magnusCoef;

        if (trajectories.isEmpty())
            throw new IllegalArgumentException("trajectories must not be empty");
        if (optimalTrajectoryIndex < 0 || optimalTrajectoryIndex >= trajectories.size())
            throw new IllegalArgumentException("optimalTrajectoryIndex out of range");

        this.impactSortedTrajectories = new ArrayList<>(trajectories);
        this.speedSortedTrajectories = new ArrayList<>(trajectories);

        this.impactSortedTrajectories.sort(Comparator.comparingDouble(t -> t.impactAngleRad));
        this.speedSortedTrajectories.sort(Comparator.comparingDouble(t -> t.exitSpeedMps));

        this.optimalTrajectory = trajectories.get(optimalTrajectoryIndex);
    }

    public Trajectory getOptimalTrajectory() {
        return optimalTrajectory;
    }

    public Trajectory getInterpolatedImpactAngleTrajectory(double impactAngleRad) {
        if (impactSortedTrajectories.isEmpty())
            return null;

        if (impactAngleRad <= impactSortedTrajectories.get(0).impactAngleRad)
            return impactSortedTrajectories.get(0);

        if (impactAngleRad >= impactSortedTrajectories.get(impactSortedTrajectories.size() - 1).impactAngleRad)
            return impactSortedTrajectories.get(impactSortedTrajectories.size() - 1);

        for (int i = 0; i < impactSortedTrajectories.size() - 1; i++) {
            Trajectory lo = impactSortedTrajectories.get(i);
            Trajectory hi = impactSortedTrajectories.get(i + 1);

            double loRad = lo.impactAngleRad;
            double hiRad = hi.impactAngleRad;

            if (impactAngleRad >= loRad && impactAngleRad <= hiRad) {
                double t = (impactAngleRad - loRad) / (hiRad - loRad);
                return lo.lerp(hi, t);
            }
        }
        return null;
    }

    public Trajectory getInterpolatedExitSpeedTrajectory(double exitSpeed) {
        if (speedSortedTrajectories.isEmpty())
            return null;

        if (exitSpeed <= speedSortedTrajectories.get(0).exitSpeedMps)
            return speedSortedTrajectories.get(0);
        if (exitSpeed >= speedSortedTrajectories.get(speedSortedTrajectories.size() - 1).exitSpeedMps)
            return speedSortedTrajectories.get(speedSortedTrajectories.size() - 1);

        for (int i = 0; i < speedSortedTrajectories.size() - 1; i++) {
            Trajectory lo = speedSortedTrajectories.get(i);
            Trajectory hi = speedSortedTrajectories.get(i + 1);

            if (exitSpeed >= lo.exitSpeedMps && exitSpeed <= hi.exitSpeedMps) {
                double t = (exitSpeed - lo.exitSpeedMps) / (hi.exitSpeedMps - lo.exitSpeedMps);
                return lo.lerp(hi, t);
            }
        }
        return null;
    }

    public boolean impactAngleInRange(double impactAngleRad) {
        if (impactSortedTrajectories.isEmpty())
            return false;
        return impactAngleRad >= impactSortedTrajectories.get(0).impactAngleRad
                && impactAngleRad <= impactSortedTrajectories.get(impactSortedTrajectories.size() - 1).impactAngleRad;
    }
}
