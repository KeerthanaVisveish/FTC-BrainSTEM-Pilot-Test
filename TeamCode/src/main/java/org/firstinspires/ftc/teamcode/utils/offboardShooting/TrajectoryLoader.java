package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;

public class TrajectoryLoader {

    private static final double DEFAULT_DY = 0.845;
    private static final double DEFAULT_DRAG_COEFF = 0.0255312;
    private static final double DEFAULT_MAGNUS_COEFF = -0.21;

    public static Trajectory loadTrajectory(JSONObject json, double dragCoeff, double magnusCoeff) {
        try {
            double exitAngleDeg = json.getDouble("exitAngle");
            double speed = json.getDouble("speed");
            double timeOfFlight = json.getDouble("timeOfFlight");
            double impactAngleDeg = optDouble(json, 0.0, "impactAngle");
            double peakHeight = optDouble(json, 0.0, "peakHeight");
            double speedMoe = optDouble(json, 0.0, "speedMOE", "speedMoe");
            double angleMoe = optDouble(json, 0.0, "angleMOE", "angleMoe");

            return new Trajectory(
                    dragCoeff,
                    magnusCoeff,
                    speed,
                    Math.toRadians(exitAngleDeg),
                    Math.toRadians(impactAngleDeg),
                    peakHeight,
                    timeOfFlight,
                    speedMoe,
                    angleMoe
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static TrajectoryLUT loadTrajectoryLUT(JSONObject groupJson) {
        try {
            if (!groupJson.has("dx"))
                return null;

            double dx = groupJson.getDouble("dx");
            double dy = optDouble(groupJson, DEFAULT_DY, "dy");
            double dragCoeff = optDouble(groupJson, DEFAULT_DRAG_COEFF, "dragCoeff");
            double magnusCoeff = optDouble(groupJson, DEFAULT_MAGNUS_COEFF, "magnusCoeff");
            JSONArray trajectoryArray = groupJson.getJSONArray("trajectories");
            ArrayList<Trajectory> trajectories = new ArrayList<>();

            for (int i = 0; i < trajectoryArray.length(); i++) {
                JSONObject trajJson = trajectoryArray.getJSONObject(i);
                Trajectory trajectory = loadTrajectory(trajJson, dragCoeff, magnusCoeff);
                if (trajectory == null)
                    return null;
                trajectories.add(trajectory);
            }

            if (trajectories.isEmpty())
                return null;

            int optimalIndex = resolveOptimalTrajectoryIndex(groupJson, trajectories);
            if (optimalIndex < 0 || optimalIndex >= trajectories.size())
                return null;

            return new TrajectoryLUT(
                    dx,
                    dy,
                    dragCoeff,
                    magnusCoeff,
                    optimalIndex,
                    trajectories
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static TrajectoryDistanceLUT loadTrajectoryDistanceLUT(JSONObject root) {
        try {
            JSONArray groups = root.getJSONArray("groups");
            ArrayList<TrajectoryLUT> trajectoryLUTs = new ArrayList<>();

            for (int i = 0; i < groups.length(); i++) {
                TrajectoryLUT trajectoryLUT = loadTrajectoryLUT(groups.getJSONObject(i));
                if (trajectoryLUT != null)
                    trajectoryLUTs.add(trajectoryLUT);
            }

            if (trajectoryLUTs.isEmpty())
                throw new RuntimeException("No trajectory groups loaded from JSON");

            return TrajectoryDistanceLUT.fromTrajectoryLUTs(trajectoryLUTs);
        } catch (JSONException e) {
            throw new RuntimeException("Failed to parse trajectory groups from JSON", e);
        }
    }

    public static TrajectoryDistanceLUT loadFromSettingsFile(String filename) {
        return loadTrajectoryDistanceLUT(getJsonObject(filename));
    }

    public static JSONObject getJsonObject(String filepath) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filepath);
            String contents = ReadWriteFile.readFile(file);
            return new JSONObject(contents);
        } catch (Exception e) {
            throw new RuntimeException("Failed to read JSON file: " + filepath, e);
        }
    }

    private static int resolveOptimalTrajectoryIndex(JSONObject groupJson, ArrayList<Trajectory> trajectories) {
        if (groupJson.has("optimalTrajectoryIndex"))
            return groupJson.optInt("optimalTrajectoryIndex");
        if (groupJson.has("biggestMOETrajectory"))
            return groupJson.optInt("biggestMOETrajectory");

        int bestIndex = 0;
        double bestMoe = -1.0;
        for (int i = 0; i < trajectories.size(); i++) {
            Trajectory trajectory = trajectories.get(i);
            double combinedMoe = trajectory.speedMoe * trajectory.angleMoe;
            if (combinedMoe > bestMoe) {
                bestMoe = combinedMoe;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    private static double optDouble(JSONObject json, double defaultValue, String... keys) throws JSONException {
        for (String key : keys) {
            if (json.has(key))
                return json.getDouble(key);
        }
        return defaultValue;
    }
}
