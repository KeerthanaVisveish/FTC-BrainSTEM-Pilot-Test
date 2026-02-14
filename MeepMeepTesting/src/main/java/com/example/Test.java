package com.example;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;

public class Test {
    public static void main(String[] args) {
        Vector2d robotVector = new Vector2d(1, 0);
        double rHeading = Math.toRadians(90);
        Vector2d fieldVector = robotVectorToFieldVector(robotVector, rHeading);
        Vector2d robotVector2 = fieldVectorToRobotVector(fieldVector, rHeading);
        System.out.println(fieldVector.x + ", " + fieldVector.y);
        System.out.println(robotVector2.x + ", " + robotVector2.y);
    }

    private static Vector2d fieldVectorToRobotVector(Vector2d fieldVector, double rHeadingRad) {
        double cos = Math.cos(-rHeadingRad);
        double sin = Math.sin(-rHeadingRad);
        // cos, -sin
        // sin,  cos
        return new Vector2d(fieldVector.x * cos - fieldVector.y * sin, fieldVector.x * sin + fieldVector.y * cos);
    }
    private static Vector2d robotVectorToFieldVector(Vector2d robotVector, double rHeadingRad) {
        double cos = Math.cos(rHeadingRad);
        double sin = Math.sin(rHeadingRad);
        // cos, -sin
        // sin,  cos
        return new Vector2d(robotVector.x * cos - robotVector.y * sin, robotVector.x * sin + robotVector.y * cos);
    }
}
