package org.firstinspires.ftc.teamcode.subsystems.limelight;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

public class Blob {
    public final double tx, ty, x, y, area;
    public Blob(double tx, double ty, double x, double y, double area) {
        this.tx = tx;
        this.ty = ty;
        this.x = x;
        this.y = y;
        this.area = area;
    }
    public Vector2d pos() {
        return new Vector2d(x, y);
    }
    @NonNull
    @Override
    public String toString() {
        return "(" + MathUtils.format1(x) + " " + MathUtils.format1(y) + " " + MathUtils.format1(area) + ")";
    }
}
