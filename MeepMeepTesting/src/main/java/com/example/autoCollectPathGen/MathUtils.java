package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.text.DecimalFormat;
import java.util.Arrays;

// helps for concise, easy printing to telemetry
public class MathUtils {
    public static double angleNormRad(double rad) {
        if(rad >= 0 && rad < Math.PI * 2)
            return rad;
        return (rad % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);
    }
    public static double angleNormDeltaRad(double rad) {
        rad = angleNormRad(rad);
        if (rad > Math.PI)
            rad -= 2 * Math.PI;
        return rad;
    }

    public static double findAngleRadDiff(double angle2, double angle1) {
        double angleDiff = angle2 - angle1;
        double absDiff = Math.abs(angleDiff);
        if (absDiff > Math.PI)
            angleDiff = Math.signum(angleDiff) * (2 * Math.PI - absDiff);
        return angleDiff;
    }
    public static String format1(Number num) {
        return format(num, 1);
    }
    public static String format2(Number num) {
        return format(num, 2);
    }
    public static String format3(Number num) { return format(num, 3); }
    public static String formatVec2(Vector2d v) {
        return "[" + format2(v.x) + ", " + format2(v.y) + "]";
    }
    public static String format(Number num, int decimalPlaces) {
        StringBuilder decimals = new StringBuilder();
        for (int i=0; i<decimalPlaces; i++)
            decimals.append("#");
        DecimalFormat customDf = new DecimalFormat("#." + decimals);
        return customDf.format(num);
    }
    public static String format2(double[] nums) {
        StringBuilder total = new StringBuilder();
        for (double num : nums)
            total.append(format2(num)).append(", ");
        return total.substring(0, total.length() - 2);
    }
    public static String format3(double[] nums) {
        StringBuilder total = new StringBuilder();
        for (double num : nums)
            total.append(format3(num)).append(", ");
        return total.substring(0, total.length() - 2);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public static double covariance(double[] a, double[] b) {
        if(a.length != b.length)
            throw new IllegalArgumentException("lists are of length " + a.length + " & " + b.length + "respectively; they must match");
        double sum = 0;
        double meanA = mean(a);
        double meanB = mean(b);
        for(int i = 0; i < a.length; i++) {
            sum += (a[i] - meanA) * (b[i] - meanB);
        }
        return sum / (a.length - 1);
    }
    public static double mean(double[] l) {
        double sum = 0;
        for (double v : l) sum += v;
        return sum / l.length;
    }
    public static double vecMag(Vector2d v) {
        return Math.hypot(v.x, v.y);
    }

    public static String formatPose(Pose2d pose) {
        if (pose == null)
            return "null";
        return "(" + format1(pose.position.x) + ", " + format1(pose.position.y) + ") | " + format1(Math.toDegrees(pose.heading.toDouble()));
    }
    public static String formatPose2(Pose2d pose) {
        if (pose == null)
            return "null";
        return format2(pose.position.x) + ", " + format2(pose.position.y) + " " + format2(Math.toDegrees(pose.heading.toDouble()));
    }
    public static String formatPose3(Pose2d pose) {
        if (pose == null)
            return "null";
        return format3(pose.position.x) + ", " + format3(pose.position.y) + " " + format3(Math.toDegrees(pose.heading.toDouble()));
    }
    public static String formatVec3(Vector2d vec) {
        if(vec == null)
            return "null";
        return format3(vec.x) + ", " + format3(vec.y);
    }
    public static Pose2d createPose(double[] pose) {
        if (pose.length != 3)
            throw new IllegalArgumentException("cannot call createPose on " + Arrays.toString(pose) + " - must contain EXACTLY 3 elements");
        return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
    }
    public static Vector2d createVec(double[] vec) {
        if(vec.length != 2)
            throw new IllegalArgumentException("cannot call createVec on " + Arrays.toString(vec) + " - must contain EXACTLY 2 elements");
        return new Vector2d(vec[0], vec[1]);
    }
}
