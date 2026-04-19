package org.firstinspires.ftc.teamcode.utils.shootingMath;

import androidx.annotation.NonNull;

public class LaunchData {
    public final boolean valid;
    public final double speed;
    public final double exitAng;
    public final double turretAng;

    public LaunchData(double speed, double exitAng, double turretAng) {
        valid = true;
        this.speed = speed;
        this.exitAng = exitAng;
        this.turretAng = turretAng;
    }
    public LaunchData(double exitAng, double turretAng) {
        this(0, exitAng, turretAng);
    }
    public LaunchData() {
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
