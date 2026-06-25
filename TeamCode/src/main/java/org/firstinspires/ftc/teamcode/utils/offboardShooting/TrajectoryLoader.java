package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;

public class TrajectoryLoader {

    public static Trajectory loadTrajectory(JSONObject json, double dragCoeff, double magnusCoeff) {
        try {
            double exitAngleDeg = json.getDouble("exitAngle");
            double impactAngleDeg = json.getDouble("impactAngle");
            double speed = json.getDouble("speed");
            double timeOfFlight = json.getDouble("timeOfFlight");
            double peakHeight = json.getDouble("peakHeight");

            return new Trajectory(
                    dragCoeff,
                    magnusCoeff,
                    speed,
                    Math.toRadians(exitAngleDeg),
                    Math.toRadians(impactAngleDeg),
                    peakHeight,
                    timeOfFlight
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static TrajectoryLUT loadTrajectoryLUT(JSONObject groupJson) {
        try {
            double dx = groupJson.getDouble("dx");
            double dy = groupJson.getDouble("dy");
            double dragCoeff = groupJson.getDouble("dragCoeff");
            double magnusCoeff = groupJson.getDouble("magnusCoeff");
            int optimalIndex = groupJson.has("optimalTrajectoryIndex")
                    ? groupJson.getInt("optimalTrajectoryIndex")
                    : groupJson.optInt("biggestMOETrajectory", 0);
            JSONArray trajectoryArray = groupJson.getJSONArray("trajectories");
            ArrayList<Trajectory> trajectories = new ArrayList<>();

            for (int i = 0; i < trajectoryArray.length(); i++) {
                JSONObject trajJson = trajectoryArray.getJSONObject(i);
                Trajectory trajectory = loadTrajectory(trajJson, dragCoeff, magnusCoeff);
                if (trajectory == null)
                    return null;
                trajectories.add(trajectory);
            }

            if (trajectories.isEmpty() || optimalIndex < 0 || optimalIndex >= trajectories.size())
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
}
