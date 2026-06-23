package org.firstinspires.ftc.teamcode.utils.shootingMath;

@FunctionalInterface
public interface ShooterConversion {
    public double convert(double speed, double exitAngle);
}