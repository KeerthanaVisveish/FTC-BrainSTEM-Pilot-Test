package org.firstinspires.ftc.teamcode.utils.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorCacher {
    private final DcMotorEx motor;
    private int encoder;
    private double vel, prevVel;
    private double power;
    private boolean updatedYet;
    private double current;
    public MotorCacher(DcMotorEx motor) {
        this.motor = motor;
        updateInfo();
    }
    public void updateInfo() {
        prevVel = vel;
        encoder = motor.getCurrentPosition();
        vel = motor.getVelocity();
        current = motor.getCurrent(CurrentUnit.AMPS);
        updatedYet = true;
    }
    public void sendInfo() {
        setPowerRaw(power);
        updatedYet = false;
    }
    public double getVelTps() {
        return vel;
    }
    public double getPrevVelTps() {
        return prevVel;
    }
    public int getCurrentPosition() {
        return encoder;
    }
    public int getCurrentPositionRaw() {
        return motor.getCurrentPosition();
    }
    public void setPower(double p) {
        power = p;
    }
    public void setPowerRaw(double p) {
        if(!updatedYet)
            throw new RuntimeException("cannot call set power raw twice in one frame");
        motor.setPower(p);
    }
    public double getPower() {
        return power;
    }
    public void resetEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getCurrent() {
        return current;
    }
}
