package com.example;

import com.acmerobotics.roadrunner.Vector2d;

public class Test {
    public static void main(String[] args) {
        Vector2d startPosition = new Vector2d(52, 8);
        Vector2d[] nodes = {
//                new Vector2d(70, 40),
//                new Vector2d(30, 60),
//                new Vector2d(36, 70)
        };
        double startTime = System.currentTimeMillis();
        Vector2d[] shortestPath = PathFinder.findShortestPath(startPosition, nodes, 3);
        double endTime = System.currentTimeMillis();
        System.out.println("calculation dt (ms): " + (endTime - startTime));
        System.out.println("FINAL PATH");
        if (shortestPath == null)
            System.out.println("path is null");
        else {
            for (int i = 0; i < shortestPath.length; i++)
                System.out.println(i + ": (" + shortestPath[i].x + ", " + shortestPath[i].y + ")");
        }
    }
}
