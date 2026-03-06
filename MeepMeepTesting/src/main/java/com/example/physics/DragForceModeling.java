package com.example.physics;

public class DragForceModeling {
    public static void main(String[] args) {
        double a = Math.toRadians(45);
        double v = 1;
        double vx = Math.cos(a) * v;
        double vy = Math.sin(a) * v;
        double g = 9.81;
        double k = 0.0073;
        // drag force equation: F = 0.5pCA * v^2
        //              simplified: F = k * v^2
        //                      p = air density (~1.2 kg/m^3
        //                      C = drag coefficient (needs tuning)
        //                      A = cross sectional area of ball

        double maxTime = 2;
        double timeStep = 0.001;
        int numIterations = (int)(maxTime / timeStep);

        double curX = 0, curY = 0;

        double timeBefore = System.currentTimeMillis();
        for(int i = 0; i < numIterations; i++) {
            a = Math.atan2(vy, vx);
            v = Math.hypot(vx, vy);
            double drag = k * v * v;

            vx -= drag * Math.cos(a) * timeStep;
            vy -= (g + drag * Math.sin(a)) * timeStep;

            curX += vx * timeStep;
            curY += vy * timeStep;
        }

        double timeAfter = System.currentTimeMillis();
        System.out.println();
        System.out.println("computation time: " + (timeAfter - timeBefore) / 1000);

        System.out.println("end x: " + curX);
        System.out.println("end y: " + curY);
        System.out.println("end vx: " + vx);
        System.out.println("end vy: " + vy);
    }
}
