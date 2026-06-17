package org.firstinspires.ftc.teamcode.robot.limelight.ballDetection.pathGeneration;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.ArrayList;

public class Ball {

    public enum BallType {
        NORMAL,
        CORNER,
        CLASSIFIER_WALL,
        BACK_WALL
    }
    public final static Ball NULL = new Ball(new Vector2d(10000, 10000), BallType.NORMAL);
    public static ArrayList<Vector2d> toVecList(ArrayList<Ball> balls) {
        ArrayList<Vector2d> vecList = new ArrayList<>();
        for (Ball ball : balls)
            vecList.add(ball.pos);
        return vecList;
    }
    public static ArrayList<Ball> toBallList(ArrayList<Vector2d> positions) {
        ArrayList<Ball> ballList = new ArrayList<>();
        for (Vector2d pos : positions)
            ballList.add(new Ball(pos));
        return ballList;
    }
    public final Vector2d pos;
    public final BallType type;
    public int pathIndex;
    public Ball(Vector2d pos) {
        this.pos = pos;
        this.type = getBallType(pos);
        this.pathIndex = -1;
    }
    private Ball(Vector2d pos, BallType type) {
        this.pos = pos;
        this.type = type;
        this.pathIndex = -1;
    }
    @Override
    public String toString() {
        return "(" + pathIndex + ": " + type + " | " + MathUtils.formatVec0(pos) + ")";
    }
    private BallType getBallType(Vector2d ballPosition) {
        Vector2d cornerPosition = new Vector2d(ballPosition.x > 0 ? 72 : -72, ballPosition.y > 0 ? 72 : -72);
        double distFromClassifierWall = Math.abs(cornerPosition.y - ballPosition.y);
        double distFromBackWall = Math.abs(cornerPosition.x - ballPosition.x);
        double distFromCorner = Math.hypot(distFromClassifierWall, distFromBackWall);

        BallType ballType = BallType.NORMAL;
        if (distFromCorner < PathGeneration.cornerParams.cornerBallDistance)
            ballType = BallType.CORNER;
        else if (distFromClassifierWall < PathGeneration.wallStrafeParams.classifierWallDistance)
            ballType = BallType.CLASSIFIER_WALL;
        else if (distFromBackWall < PathGeneration.wallStrafeParams.backWallDistance)
            ballType = BallType.BACK_WALL;
        return ballType;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Ball)
             return ((Ball) other).pos.equals(pos);
        return false;
    }
}
