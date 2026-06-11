package org.firstinspires.ftc.teamcode.utils.math;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
public class PIDController {

    private double target;
    private double kP, kI, kD;
    private double proportional;
    private double integral;
    private double maxIntegral = Double.MAX_VALUE;
    private double derivative;
    private boolean shouldReset;

    private double previousError;
    private final ElapsedTime timer;

    private double lowerInputBound = Double.NEGATIVE_INFINITY, higherInputBound = Double.POSITIVE_INFINITY;
    private double lowerOutputBound = Double.NEGATIVE_INFINITY, higherOutputBound = Double.POSITIVE_INFINITY;
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        timer = new ElapsedTime();
        timer.reset();

        shouldReset = true;
    }
    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }
    public void setPIDValues(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void setKp(double kP) {
        this.kP = kP;
    }
    public void setKi(double kI) {
        this.kI = kI;
    }
    public void setKd(double kD) {
        this.kD = kD;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setInputBounds(double lowerInputBound, double higherInputBound) {
        this.lowerInputBound = lowerInputBound;
        this.higherInputBound = higherInputBound;
    }

    public void setOutputBounds(double lowerOutputBound, double higherOutputBound) {
        this.lowerOutputBound = lowerOutputBound;
        this.higherOutputBound = higherOutputBound;
    }

    public double getLowerInputBound(){
        return this.lowerInputBound;
    }

    public double getUpperInputBound(){
        return this.higherInputBound;
    }


    public void reset() {
        shouldReset = true;
    }

    public double update(double value) {
        value = Range.clip(value, lowerInputBound, higherInputBound);

        double error = value - target;

        return updateWithError(error);
    }

    public double updateWithError(double error) {
        if (Double.isNaN(error) || Double.isInfinite(error))
            return 0;

        proportional = kP * error;


        if (shouldReset) {
            shouldReset = false;
            integral = 0;
            derivative = 0;
        } else {

            double dT = timer.seconds();

            integral += kI * error * dT;
            integral = Range.clip(integral, -maxIntegral, maxIntegral);

            if(dT > .00001)
                derivative = kD * (error - previousError) / dT;
        }
        timer.reset();
        previousError = error;

        double correction = proportional + integral + derivative;

        return Range.clip(correction,
                lowerOutputBound, higherOutputBound);
    }
}