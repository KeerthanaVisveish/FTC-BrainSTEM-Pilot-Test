package org.firstinspires.ftc.teamcode.utils.shootingMath;

public class AnswerKeyPt2 {
    public final boolean solutionExists;
    public final LaunchData launchData;

    public AnswerKeyPt2() {
        solutionExists = false;
        launchData = null;
    }
    
    public AnswerKeyPt2(double curExitSpeedMps) {
        solutionExists = false;
        launchData = new LaunchData(curExitSpeedMps, -1, -1);
    }
    public AnswerKeyPt2(LaunchData launchData) {
        solutionExists = true;
        this.launchData = launchData;
    }

    public String toString() {
        return "solution: " + solutionExists + ", launch data: " + launchData;
    }
}
