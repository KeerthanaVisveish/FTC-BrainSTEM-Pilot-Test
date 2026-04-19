package org.firstinspires.ftc.teamcode.utils.shootingMath;

public class AnswerKeyPt1 {
    public final boolean solutionExists;
    public final LaunchData launchData;
    
    public final Vector3d lookAheadExitPos;
    public final Vector3d lookAheadVelAtExitPos;
    public final Vector3d currentVelAtExitPos;

    public AnswerKeyPt1(Vector3d exitPos, Vector3d lookAheadVelAtExitPos, Vector3d currentVelAtExitPos) {
        solutionExists = false;
        launchData = null;
        this.lookAheadExitPos = exitPos;
        this.lookAheadVelAtExitPos = lookAheadVelAtExitPos;
        this.currentVelAtExitPos = currentVelAtExitPos;
    }
    public AnswerKeyPt1(LaunchData launchData, Vector3d lookAheadExitPos, Vector3d lookAheadVelAtExitPos, Vector3d currentVelAtExitPos) {
        solutionExists = true;
        this.launchData = launchData;
        this.lookAheadExitPos = lookAheadExitPos;
        this.lookAheadVelAtExitPos = lookAheadVelAtExitPos;
        this.currentVelAtExitPos = currentVelAtExitPos;
    }

    public String toString() {
        return "" + launchData;
    }
}
