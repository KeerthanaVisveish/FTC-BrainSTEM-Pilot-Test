package org.firstinspires.ftc.teamcode.utils.shootingMath;

import edu.wpi.first.math.util.Units;

public class ShootingTuner {
    public static void main(String[] args) {
        runCoefficientFinder();
    }

    // finding coefficient
    // input dist traveled, delta height, exit angle deg
    // outputs exit speed and conversion coefficient
    private static void runCoefficientFinder() {
        double[] distTraveledList = new double[] {
            Units.inchesToMeters(17 + 51.82),
            Units.inchesToMeters(17 + 69.8),
            Units.inchesToMeters(17 +  112.83),
        }; // meters 
        double[] deltaHeightList = new double[] {
            Units.inchesToMeters(-17.05),
            Units.inchesToMeters(-17.05),
            Units.inchesToMeters(-17.05),
        }; // meters
        double[] exitAngleDegList = new double[] {
            65,
            65,
            65,
            // 65,
        }; // degrees
        double[] actualShooterMotorSpeedList = new double[] {
            30.4,
            35.4,
            45.5,
            // 54.8,
            // 64.8
        }; // rot/s
        
        ShootingMath shootingMath = new ShootingMath();
        for (int i=0; i<distTraveledList.length; i++) {
            double distTraveled = distTraveledList[i];
            double deltaHeight = deltaHeightList[i];
            double exitAngleRad = Math.toRadians(exitAngleDegList[i]);
            double actualShooterMotorSpeed = actualShooterMotorSpeedList[i];

            System.out.println("Trial " + (i + 1) + " - " + actualShooterMotorSpeed + " RPS | " + exitAngleDegList[i] + " deg");

            double conversionCoefficient = -1;
            try {

                double exitSpeed = shootingMath.calculateExitSpeed(distTraveled, deltaHeight, exitAngleRad);
                conversionCoefficient = exitSpeed / actualShooterMotorSpeed;
            } catch (IllegalArgumentException e) {
                System.out.println("ERROR");
                e.printStackTrace();
            }
        
            System.out.println("Conversion Coefficient: " + conversionCoefficient);
        }
    }

    private static void runCoefficientTester() {
        double[] distTraveledList = new double[] {
            Units.inchesToMeters(17 + 131.4),
            Units.inchesToMeters(17 + 265.5),
            Units.inchesToMeters(17 + 104),
            Units.inchesToMeters(17 + 205.206),
        }; // meters 
        double[] deltaHeightList = new double[] {
            Units.inchesToMeters(-47.55),
            Units.inchesToMeters(-47.55),
            Units.inchesToMeters(-47.55),
            Units.inchesToMeters(-47.55),
        }; // meters
        double[] exitAngleDegList = new double[] {
            45,
            45,
            65,
            45,
        }; // degrees
        double[] conversionCoefficientList = new double[] {
            0.1497004862165461,
            0.11969333427031964,
            0.12773841977638986,
            0.14957666034256217
        };

        ShootingMath shootingMath = new ShootingMath();
        for (int i=0; i<distTraveledList.length; i++) {
            System.out.println("Trial " + (i + 1));
            double distTraveled = distTraveledList[i];
            double deltaHeight = deltaHeightList[i];
            double exitAngleRad = Math.toRadians(exitAngleDegList[i]);
            double conversionCoefficient = conversionCoefficientList[i];

            double shooterMotorSpeed = -1;
            try {
                shooterMotorSpeed = shootingMath.calculateShooterSpeed(distTraveled, deltaHeight, exitAngleRad, x -> conversionCoefficient);
            } catch (IllegalArgumentException e) {
                System.out.println("ERROR");
                e.printStackTrace();
            }
            System.out.println("Calculated Shooter Motor Speed: " + shooterMotorSpeed);
        }
    }
}
