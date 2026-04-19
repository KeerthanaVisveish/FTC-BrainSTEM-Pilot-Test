package org.firstinspires.ftc.teamcode.utils.shootingMath;

public class Vector3d {
  public final double x, y, z;

  public Vector3d(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public Vector3d add(Vector3d v) {
    return new Vector3d(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  public Vector3d sub(Vector3d v) {
    return new Vector3d(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  public Vector3d times(double n) {
    return new Vector3d(this.x * n, this.y * n, this.z * n);
  }

  public Vector3d div(double n) {
    return new Vector3d(this.x / n, this.y / n, this.z / n);
  }

  public Vector3d to2D() {
    return new Vector3d(x, y, 0);
  }
  public double mag() {
    return Math.sqrt(x * x + y * y + z * z);
  }
  public double magSqrd() {
    return x * x + y * y + z * z;
  }
  public Vector3d perpInXY() {
    return new Vector3d(-y, x, 0);
  }
}
