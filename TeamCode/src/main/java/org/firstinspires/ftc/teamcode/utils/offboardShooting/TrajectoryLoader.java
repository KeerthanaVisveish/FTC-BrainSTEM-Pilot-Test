package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
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

    public static TrajectoryLUT loadTrajectoryLUT(JSONObject json) {
        try {
            double dx = json.getDouble("dx");
            double dy = json.getDouble("dy");
            double dragCoeff = json.getDouble("dragCoeff");
            double magnusCoeff = json.getDouble("magnusCoeff");
            JSONArray trajectoryArray = json.getJSONArray("trajectories");
            ArrayList<Trajectory> trajectories = new ArrayList<>();

            for (int i = 0; i < trajectoryArray.length(); i++) {
                JSONObject trajJson = trajectoryArray.getJSONObject(i);
                Trajectory trajectory = loadTrajectory(trajJson, dragCoeff, magnusCoeff);
                if (trajectory == null)
                    return null;
                trajectories.add(trajectory);
            }

            return new TrajectoryLUT(
                    dx,
                    dy,
                    dragCoeff,
                    magnusCoeff,
                    trajectories
            );
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static JSONObject getJsonObject(String filepath) {
        try (FileInputStream stream = new FileInputStream(filepath)) {
            return getJsonObject(stream);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static JSONObject getJsonObject(InputStream stream) {
        try {
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            byte[] data = new byte[1024];
            int n;
            while ((n = stream.read(data)) != -1) {
                buffer.write(data, 0, n);
            }
            return new JSONObject(buffer.toString(StandardCharsets.UTF_8.name()));
        } catch (IOException | JSONException e) {
            e.printStackTrace();
            return null;
        }
    }
}
