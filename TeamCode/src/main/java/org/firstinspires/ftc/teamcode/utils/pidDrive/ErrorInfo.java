package org.firstinspires.ftc.teamcode.utils.pidDrive;

public class ErrorInfo {
    public final double xError, yError, distanceError, headingRadError, headingPowerDirFlip;
    public ErrorInfo(double xError, double yError, double distanceError, double headingRadError, double headingPowerDirFlip) {
        this.xError = xError;
        this.yError = yError;
        this.distanceError = distanceError;
        this.headingRadError = headingRadError;
        this.headingPowerDirFlip = headingPowerDirFlip;
    }
}
