package org.firstinspires.ftc.teamcode.utils.shootingMath;

import androidx.annotation.NonNull;

public class LaunchVector {
    public final boolean valid;
    public final double speed;
    public final double exitAng;
    public final double turretAng;

    public LaunchVector(double speed, double exitAng, double turretAng) {
        valid = true;
        this.speed = speed;
        this.exitAng = exitAng;
        this.turretAng = turretAng;
    }
    public LaunchVector(double exitAng, double turretAng) {
        this(0, exitAng, turretAng);
    }
    public LaunchVector() {
        valid = false;
        speed = -1;
        exitAng = -1;
        turretAng = -1;
    }

    @NonNull
    public String toString() {
        return "v: " + speed + ", theta: " + exitAng + ", phi: " + turretAng;
    }
}
