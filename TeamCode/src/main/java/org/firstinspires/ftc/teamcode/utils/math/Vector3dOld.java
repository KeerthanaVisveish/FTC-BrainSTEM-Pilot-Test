package org.firstinspires.ftc.teamcode.utils.math;

public class Vector3dOld {
    public static Vector3dOld j = new Vector3dOld(0, 1, 0);
    public final double x, y, z;
    public Vector3dOld(double x, double y, double z) {
        this.x = x;
        this.y = y; this.z = z;
    }
    public Vector3dOld plus(Vector3dOld v) {
        return new Vector3dOld(x + v.x, y + v.y, z + v.z);
    }
    public Vector3dOld minus(Vector3dOld v) {
        return new Vector3dOld(x - v.x, y - v.y, z - v.z);
    }
    public Vector3dOld times(double n) {
        return new Vector3dOld(x * n, y * n, z * n);
    }
}
