package org.firstinspires.ftc.teamcode.utils.shootingMath;

public class AnswerKeyPt2 {
    public final boolean solutionExists;
    public final LaunchVector launchVector;

    public AnswerKeyPt2() {
        solutionExists = false;
        launchVector = null;
    }
    
    public AnswerKeyPt2(double curExitSpeedMps) {
        solutionExists = false;
        launchVector = new LaunchVector(curExitSpeedMps, -1, -1);
    }
    public AnswerKeyPt2(LaunchVector launchVector) {
        solutionExists = true;
        this.launchVector = launchVector;
    }

    public String toString() {
        return "solution: " + solutionExists + ", launch data: " + launchVector;
    }
}
