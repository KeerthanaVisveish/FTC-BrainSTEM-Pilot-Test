package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;

@Config
public class ShooterLookup {
    public static double[] lookupDistsI =    {20,   29.6, 34,   39,   44,   49,   53,   61,   64,   67,   73.4, 80,   86,   95.69, 98,   102,  104,  109,  127, 128,  136,  138,   141,  146.5, 154,  161};
    public static double[] lookupVelsTps =  {1200, 1250, 1260, 1260, 1280, 1280, 1310, 1370, 1420, 1450, 1490, 1530, 1580, 1650,  1680, 1700, 1710, 1720, 1870, 1870, 1960, 1960, 1960, 1990, 2060,  2090};
    public static double[] lookupExitAngs = {1.25, 1.25, 1.15, 1.1,  1,    1,    0.98, 0.95, 0.94, 0.92, 0.87, 0.81, 0.75, 0.724, 0.7,  0.7,  0.7,  0.7,  0.62, 0.62, 0.62, 0.62, 0.62, 0.62, 0.62, 0.62};
    public static double[] lookupVelsMps = { 3.735, 3.891, 4.008, 4.051, 4.202, 4.202, 4.318, 4.544, 4.720, 4.839, 5.023, 5.221, 5.456, 5.727, 5.858, 5.928, 5.963, 5.998, 6.622, 6.622, 6.941, 6.941, 6.941, 7.047, 7.295, 7.402 };

    public static double[] physicsDists =   { 20,    29.6,  34,    39,    44,    50.5,   53,    55,    57,    61,    62.99, 63.01, 64,    67,   73,   77,    80,    86,    95.69, 98,    102,   104,   109,   127,   128,   136,   138,   141,   146.5, 154,   161};
    public static double[] physicsVelsMps = { 4.050, 4.120, 4.250, 4.340, 4.450, 4.600,  4.670, 4.710, 4.760, 4.860, 4.9,   4.95,  4.97,  5.05, 5.2, 5.300, 5.321, 5.456, 5.727, 5.858, 5.928, 5.963, 5.998, 6.622, 6.622, 6.941, 6.941, 6.941, 7.047, 7.295, 7.402 };


    // for velocity independent hood
    private final InterpLUT velocityTpsLookup;
    private final InterpLUT exitAngleLookup;
    private final InterpLUT velocityMpsLookup;

    // for velocity dependent hood
    private final InterpLUT physicsVelocityLookup;
    public ShooterLookup() {
        velocityTpsLookup = new InterpLUT();
        exitAngleLookup = new InterpLUT();
        velocityMpsLookup = new InterpLUT();
        physicsVelocityLookup = new InterpLUT();

        for(int i = 0; i < lookupDistsI.length; i++) {
            velocityTpsLookup.add(lookupDistsI[i], lookupVelsTps[i]);
            velocityMpsLookup.add(lookupDistsI[i], lookupVelsMps[i]);
            exitAngleLookup.add(lookupDistsI[i], lookupExitAngs[i]);
        }
        for(int j = 0; j < physicsDists.length; j++) {
            physicsVelocityLookup.add(physicsDists[j], physicsVelsMps[j]);
        }
        velocityTpsLookup.createLUT();
        exitAngleLookup.createLUT();
        velocityMpsLookup.createLUT();
        physicsVelocityLookup.createLUT();
    }

    public double lookupVelocityTicksPerSec(double dist) {
        return velocityTpsLookup.get(dist);
    }
    public double lookupExitAngleRad(double dist) {
        return exitAngleLookup.get(dist);
    }
    public double lookupVelocityMetersPerSec(double dist) {
        return velocityMpsLookup.get(dist);
    }
    public double lookupPhysicsVelocityMetersPerSec(double distInches) { return physicsVelocityLookup.get(distInches); }

}
