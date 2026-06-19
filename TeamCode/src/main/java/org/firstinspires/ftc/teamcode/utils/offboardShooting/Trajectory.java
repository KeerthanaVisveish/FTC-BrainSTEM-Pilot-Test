package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class Trajectory {
    public final double dragCoef;
    public final double magnusCoef;
    public final double exitSpeedMps;
    public final double exitAngleRad;
    public final double impactAngleRad;
    public final double peakHeight;
    public final double timeOfFlight;

    public Trajectory(
            double dragCoef,
            double magnusCoef,
            double launchSpeedMps,
            double exitAngleRad,
            double impactAngleRad,
            double peakHeight,
            double timeOfFlight) {
        this.dragCoef = dragCoef;
        this.magnusCoef = magnusCoef;
        this.exitSpeedMps = launchSpeedMps;
        this.exitAngleRad = exitAngleRad;
        this.impactAngleRad = impactAngleRad;
        this.peakHeight = peakHeight;
        this.timeOfFlight = timeOfFlight;
    }

    public Trajectory lerp(Trajectory other, double t) {
        double interpLaunchSpeed = lerp(exitSpeedMps, other.exitSpeedMps, t);
        double interpExitAngle = lerp(exitAngleRad, other.exitAngleRad, t);
        double interpImpactAngle = lerp(impactAngleRad, other.impactAngleRad, t);
        double interpPeakHeight = lerp(peakHeight, other.peakHeight, t);
        double interpTOF = lerp(timeOfFlight, other.timeOfFlight, t);

        return new Trajectory(
                dragCoef,
                magnusCoef,
                interpLaunchSpeed,
                interpExitAngle,
                interpImpactAngle,
                interpPeakHeight,
                interpTOF
        );
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public ArrayList<Vector3d> simulateTrajectory(
            int numPoints,
            int sparsity,
            double turretAngleRad,
            Vector3d startPosition,
            Vector3d startVelocity) {

        ArrayList<Vector3d> points = new ArrayList<>();
        if (numPoints <= 2)
            throw new IllegalArgumentException("numPoints must be at least 3");

        double dt = timeOfFlight / (numPoints - 1);

        double x = startPosition.x;
        double y = startPosition.y;
        double z = startPosition.z;

        double launchHorizontalSpeed = exitSpeedMps * Math.cos(exitAngleRad);
        double launchVerticalSpeed = exitSpeedMps * Math.sin(exitAngleRad);

        double launchVx = launchHorizontalSpeed * Math.cos(turretAngleRad);
        double launchVy = launchHorizontalSpeed * Math.sin(turretAngleRad);
        double launchVz = launchVerticalSpeed;

        double vx = launchVx + startVelocity.x;
        double vy = launchVy + startVelocity.y;
        double vz = launchVz + startVelocity.z;

        points.add(new Vector3d(x, y, z));

        for (int i = 1; i < numPoints; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);

            double azGravity = -9.81;

            double axDrag = -dragCoef * speed * vx;
            double ayDrag = -dragCoef * speed * vy;
            double azDrag = -dragCoef * speed * vz;

            double azMagnus = magnusCoef * speed;

            double ax = axDrag;
            double ay = ayDrag;
            double az = azGravity + azDrag + azMagnus;

            vx += ax * dt;
            vy += ay * dt;
            vz += az * dt;

            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            points.add(new Vector3d(x, y, z));
        }

        ArrayList<Vector3d> pointsToPublish = IntStream.range(0, points.size())
                .filter(i -> i % sparsity == 0)
                .mapToObj(points::get)
                .collect(Collectors.toCollection(ArrayList::new));
        pointsToPublish.add(points.get(points.size() - 1));

        return pointsToPublish;
    }
}