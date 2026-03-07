package com.example.physics;

public class Vector3d {
    public static Vector3d j = new Vector3d(0, 1, 0);
    public final double x, y, z;
    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y; this.z = z;
    }
    public Vector3d plus(Vector3d v) {
        return new Vector3d(x + v.x, y + v.y, z + v.z);
    }
    public Vector3d minus(Vector3d v) {
        return new Vector3d(x - v.x, y - v.y, z - v.z);
    }
    public Vector3d times(double n) {
        return new Vector3d(x * n, y * n, z * n);
    }
}
