package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;

public class TrajectoryDistanceLUT {
    private final ArrayList<TrajectoryLUT> trajectoryLUTs;

    private TrajectoryDistanceLUT() {
        this.trajectoryLUTs = new ArrayList<>();
    }

    public TrajectoryDistanceLUT(String baseDirectory, ArrayList<String> filepaths) {
        this.trajectoryLUTs = new ArrayList<>();

        for (String filepath : filepaths) {
            String fullFilepath = new File(new File(baseDirectory, "trajectoryDataBase"), filepath).getAbsolutePath();
            JSONObject json = TrajectoryLoader.getJsonObject(fullFilepath);
            if (json == null)
                continue;

            TrajectoryLUT trajectoryLUT = TrajectoryLoader.loadTrajectoryLUT(json);
            if (trajectoryLUT == null)
                continue;
            trajectoryLUTs.add(trajectoryLUT);
        }

        trajectoryLUTs.sort(Comparator.comparingDouble(t -> t.distFromGoal));
    }

    public boolean distanceInRange(double distFromGoal) {
        if (trajectoryLUTs.isEmpty())
            return false;

        return distFromGoal >= trajectoryLUTs.get(0).distFromGoal
                && distFromGoal <= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal;
    }

    public boolean impactAngleInRange(double distFromGoal, double impactAngleRad) {
        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return false;
        return neighbors.loLUT.impactAngleInRange(impactAngleRad)
                && neighbors.hiLUT.impactAngleInRange(impactAngleRad);
    }

    public Trajectory getInterpolatedImpactAngleTrajectory(double distFromGoal, double impactAngleRad) {
        if (trajectoryLUTs.isEmpty())
            return null;

        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedImpactAngleTrajectory(impactAngleRad);
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedImpactAngleTrajectory(impactAngleRad);

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return null;

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedImpactAngleTrajectory(impactAngleRad);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        Trajectory loTraj = neighbors.loLUT.getInterpolatedImpactAngleTrajectory(impactAngleRad);
        Trajectory hiTraj = neighbors.hiLUT.getInterpolatedImpactAngleTrajectory(impactAngleRad);

        if (loTraj == null || hiTraj == null)
            return null;

        return loTraj.lerp(hiTraj, t);
    }

    public Trajectory getInterpolatedExitSpeedTrajectory(double distFromGoal, double exitSpeed) {
        if (trajectoryLUTs.isEmpty())
            return null;

        if (distFromGoal <= trajectoryLUTs.get(0).distFromGoal)
            return trajectoryLUTs.get(0).getInterpolatedExitSpeedTrajectory(exitSpeed);
        if (distFromGoal >= trajectoryLUTs.get(trajectoryLUTs.size() - 1).distFromGoal)
            return trajectoryLUTs.get(trajectoryLUTs.size() - 1).getInterpolatedExitSpeedTrajectory(exitSpeed);

        NeighborTrajectoryInfo neighbors = getNeighboringTrajectoryLUTs(distFromGoal);
        if (neighbors == null)
            return null;

        double distRange = neighbors.hiDist - neighbors.loDist;
        if (distRange <= 1e-9)
            return neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed);

        double t = (distFromGoal - neighbors.loDist) / distRange;

        Trajectory loTraj = neighbors.loLUT.getInterpolatedExitSpeedTrajectory(exitSpeed);
        Trajectory hiTraj = neighbors.hiLUT.getInterpolatedExitSpeedTrajectory(exitSpeed);

        if (loTraj == null || hiTraj == null)
            return null;

        return loTraj.lerp(hiTraj, t);
    }

    private NeighborTrajectoryInfo getNeighboringTrajectoryLUTs(double distFromGoal) {
        if (!distanceInRange(distFromGoal))
            return null;

        for (int i = 0; i < trajectoryLUTs.size() - 1; i++) {
            TrajectoryLUT loLUT = trajectoryLUTs.get(i);
            TrajectoryLUT hiLUT = trajectoryLUTs.get(i + 1);

            double loDist = loLUT.distFromGoal;
            double hiDist = hiLUT.distFromGoal;

            if (distFromGoal >= loDist && distFromGoal <= hiDist)
                return new NeighborTrajectoryInfo(loLUT, hiLUT, loDist, hiDist);
        }
        return null;
    }

    private record NeighborTrajectoryInfo(TrajectoryLUT loLUT, TrajectoryLUT hiLUT, double loDist, double hiDist) {}

    public static TrajectoryDistanceLUT fromTrajectoryLUTs(ArrayList<TrajectoryLUT> trajectoryLUTs) {
        TrajectoryDistanceLUT lut = new TrajectoryDistanceLUT();
        lut.trajectoryLUTs.addAll(trajectoryLUTs);
        lut.trajectoryLUTs.sort(Comparator.comparingDouble(t -> t.distFromGoal));
        return lut;
    }
}
