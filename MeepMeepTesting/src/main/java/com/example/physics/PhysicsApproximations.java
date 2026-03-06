package com.example.physics;


public class PhysicsApproximations {
    public static double g = 9.81;

    public static void main(String[] args) {
        double exitHeight = approximateExitHeightM(true);
        double goalHeightM = 39 * 0.0254;
        double relGoalHeightM = goalHeightM - exitHeight;
        double exitPosGoalDistM = 1.5;
        double impactAng = Math.toRadians(-25);
        double filteredShooterSpeedTps = 1500;

        double[] launchVector = calculateLaunchVector(exitPosGoalDistM, relGoalHeightM, impactAng);
        double efficiencyCoef = calcEfficiencyCoef(launchVector[1]);
        double ballExitAngleRad;
        double curExitSpeedMps = ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

        System.out.println("launch vector exit speed: " + launchVector[0]);
        System.out.println("launch vector angle: " + launchVector[1]);
        System.out.println("initial curExit speed mps: " + curExitSpeedMps);
        System.out.println("initial efficiency coef: " + efficiencyCoef);
        System.out.println();
        boolean usingHighArc;

        double[] physicsExitAngleRads = new double[4];

        double highArcExitAng = calculateBallExitAngleRad(true, relGoalHeightM, exitPosGoalDistM, curExitSpeedMps);
        if(highArcExitAng != -1) {
            double lowArcExitAng = calculateBallExitAngleRad(false, relGoalHeightM, exitPosGoalDistM, curExitSpeedMps);
            double highArcImpactAng = calculateImpactAngle(exitPosGoalDistM, relGoalHeightM, curExitSpeedMps, highArcExitAng);
            double lowArcImpactAng = calculateImpactAngle(exitPosGoalDistM, relGoalHeightM, curExitSpeedMps, lowArcExitAng);
            usingHighArc = Math.abs(highArcImpactAng - impactAng) < Math.abs(lowArcImpactAng - impactAng);
            System.out.println("USING HIGH ARC: " + usingHighArc);

            physicsExitAngleRads[0] = usingHighArc ? highArcExitAng : lowArcExitAng;
            ballExitAngleRad = physicsExitAngleRads[0];
            efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
            curExitSpeedMps = ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
            System.out.println("i = 0 | estimated v, theta: " + curExitSpeedMps + ", " + ballExitAngleRad);

            for(int i = 1; i < physicsExitAngleRads.length; i++) {
                physicsExitAngleRads[i] = calculateBallExitAngleRad(usingHighArc, relGoalHeightM, exitPosGoalDistM, curExitSpeedMps);
                if(physicsExitAngleRads[i] == -1)
                    break;
                ballExitAngleRad = physicsExitAngleRads[i];
                efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                curExitSpeedMps = ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
                System.out.println("i = " + i + " | estimated v, theta: " + curExitSpeedMps + ", " + ballExitAngleRad);
            }
        }
        else
            System.out.println("VELOCITY TOO LOW");

        System.out.println("d, h: " + exitPosGoalDistM + ", " + relGoalHeightM);

    }
    public static double[] calculateLaunchVector(double d, double h, double phi) {
        double theta = Math.atan(2 * h / d - Math.tan(phi)); // desired exit angle

        double num = g * d * d;
        double denom = 2 * (d * Math.tan(theta) - h) * Math.pow(Math.cos(theta), 2);
        double v = Math.sqrt(num / denom);

        return new double[] {v, theta};
    }
    public static double calculateImpactAngle(double d, double h, double v, double theta) {
        double tanPhi = Math.tan(theta) - g * d / Math.pow(v * Math.cos(theta), 2);
        return Math.atan(tanPhi);
    }
    public static double calculateBallExitAngleRad(boolean useHighArc, double y, double x, double v) {
        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double sign = useHighArc ? 1 : -1;
        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return -1;

        double tanTheta = (v*v + sign * Math.sqrt(discriminant)) / (g * x);
        return Math.atan(tanTheta);
    }

    public static double approximateExitHeightM(boolean useHighArc) {
        double hoodAngleDeg = useHighArc ? 20 : 45;
        double exitAngleRad = Math.toRadians(90 - hoodAngleDeg);
        return getExactExitHeightMeters(exitAngleRad);
    }
    public static double getExactExitHeightMeters(double exitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - exitAngleRad;
        return 0.2413 + (0.0445 + 0.064) * Math.sin(hoodAngleRad);
    }
    public static double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = -0.0766393 * ballExitAngleRad + 0.446492;
        return Math.min(Math.max(0.3327, rawE), 0.4000);
    }
    public static double ticksPerSecToExitSpeedMps(double motorTicksPerSec, double efficiencyCoefficient) {
        double motorRevPerSec = motorTicksPerSec / 28;
        double motorAngularVel = motorRevPerSec * 2 * Math.PI;
        double flywheelAngularVel = motorAngularVel * 16 / 18;
        double flywheelTangentialVel = flywheelAngularVel * 0.0445;
        return flywheelTangentialVel * efficiencyCoefficient;
    }
}
