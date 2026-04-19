package org.firstinspires.ftc.teamcode.utils.misc;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoImplCacher {
    private double curPos, targetPos;
    private final ServoImplEx servo;
    public ServoImplCacher(ServoImplEx servo) {
        this.servo = servo;
    }
    public void updateInfo() {
        curPos = servo.getPosition();
    }
    public double getPosition() {
        return curPos;
    }
    public double getTargetPosition() {
        return targetPos;
    }
    public void setPosition(double p) {
        targetPos = p;
    }
    public void sendInfo() {
        servo.setPosition(targetPos);
    }
}
