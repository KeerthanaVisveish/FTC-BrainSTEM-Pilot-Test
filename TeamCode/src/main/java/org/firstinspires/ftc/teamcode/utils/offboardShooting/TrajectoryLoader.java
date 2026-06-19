package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import java.io.FileReader;
import java.util.ArrayList;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class TrajectoryLoader {

    public static Trajectory loadTrajectory(JSONObject json, double dragCoeff, double magnusCoeff) {
        double exitAngleDeg = ((Number) json.get("exitAngle")).doubleValue();
        double impactAngleDeg = ((Number) json.get("impactAngle")).doubleValue();
        double speed = ((Number) json.get("speed")).doubleValue();
        double timeOfFlight = ((Number) json.get("timeOfFlight")).doubleValue();
        double peakHeight = ((Number) json.get("peakHeight")).doubleValue();

        return new Trajectory(
                dragCoeff,
                magnusCoeff,
                speed,
                Math.toRadians(exitAngleDeg),
                Math.toRadians(impactAngleDeg),
                peakHeight,
                timeOfFlight
        );
    }

    public static TrajectoryLUT loadTrajectoryLUT(JSONObject json) {
        double dx = ((Number) json.get("dx")).doubleValue();
        double dy = ((Number) json.get("dy")).doubleValue();
        double dragCoeff = ((Number) json.get("dragCoeff")).doubleValue();
        double magnusCoeff = ((Number) json.get("magnusCoeff")).doubleValue();
        JSONArray trajectoryArray = (JSONArray) json.get("trajectories");
        ArrayList<Trajectory> trajectories = new ArrayList<>();

        for (Object obj : trajectoryArray) {
            JSONObject trajJson = (JSONObject) obj;
            trajectories.add(loadTrajectory(trajJson, dragCoeff, magnusCoeff));
        }

        return new TrajectoryLUT(
                dx,
                dy,
                dragCoeff,
                magnusCoeff,
                trajectories
        );
    }

    public static JSONObject getJsonObject(String filepath) {
        JSONParser parser = new JSONParser();
        try (FileReader reader = new FileReader(filepath)) {
            Object obj = parser.parse(reader);
            return (JSONObject) obj;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}