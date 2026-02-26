package org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.function.BooleanSupplier;

public class PathParams {
    public enum HeadingLerpType {
        LINEAR,
        TANGENT,
        REVERSE_TANGENT
    }
    public enum PathType {
        NORMAL,
        CURVED
    }
    private static final double noMaxTime = -1;

    public static class DefaultParams {
        public double bigSpeedKp = 0.02, smallSpeedKp = 0.013, bigSpeedKd = 0, smallSpeedKd = 0.0001;
        public double speedKi = 0, speedKf = 0.075;
        public double correctiveKp = 0.05;
        public double correctiveStrength = 1;
        public double applyCloseSpeedPIDError = 5;
        public double closeHeadingKp = 0.01, closeHeadingKi = 0, closeHeadingKd = 0.001, headingKf = 0.1;
        public double farHeadingKp = 0.012, farHeadingKi = 0, farHeadingKd = 0;
        public double applyCloseHeadingPIDErrorDeg = 15;
        public double lateralWeight = 1.9, axialWeight = 1; // weight the drive powers to correct for differences in driving
        public double minSpeed = 0, maxSpeed = 1;
        public double minHeadingSpeed = 0, maxHeadingSpeed = 1;
        public double maxTime = 100;
        public HeadingLerpType headingLerpType = HeadingLerpType.LINEAR;
        public PathType pathType = PathType.NORMAL;
        public double tValueMaxOutTime = 1;
        public double tangentHeadingActivateThreshold = 23;
        public boolean prioritizeHeadingInBeginning = false;
        public double prioritizeHeadingThresholdDeg = 5, maxLinearPowerWhilePrioritizingHeading = 0.1;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    public double lateralWeight, axialWeight;
    public double minLinearPower, maxLinearPower;
    public double minHeadingPower, maxHeadingPower;
    // decides how much the robot will slow down at this waypoint
    // if this equals 1, then the drivetrain will completely stop at this waypoint
    // if this equals 0, this waypoint will have no influence on slowing down the drivetrain as it approaches this point
    public double slowDownPercent;
    // if passPosition is true, the robot only needs to pass its target position for drive path to consider it "in tolerance", not fall within tolerance of it
    public boolean passPosition;
    public double maxTime;
    public BooleanSupplier customEndCondition = () -> false;

    public double bigSpeedKp, smallSpeedKp, bigSpeedKd, smallSpeedKd;
    public double speedKi, speedKf;
    public double correctiveKp;
    public double correctiveStrength = 1; // percent between [0, 1] - decides strongly the corrective vector is weighted relative to the drive vector
    public double closeHeadingKp, closeHeadingKi, closeHeadingKd, farHeadingKp, farHeadingKi, farHeadingKd, headingKf;
    public double applyCloseSpeedPIDError;
    public HeadingLerpType headingLerpType;
    public PathType pathType;
    public Pose2d controlPoint;
    public double tValueStartDist, tValueFinishDist;
    public double tangentHeadingDeactivateThreshold, applyCloseHeadingPIDErrorDeg;
    public boolean prioritizeHeadingInBeginning;
    public PathParams() {
        this(defaultParams.bigSpeedKp, defaultParams.smallSpeedKp, defaultParams.speedKi, defaultParams.bigSpeedKd, defaultParams.smallSpeedKd, defaultParams.speedKf, defaultParams.closeHeadingKp, defaultParams.closeHeadingKi, defaultParams.closeHeadingKd, defaultParams.farHeadingKp, defaultParams.farHeadingKi, defaultParams.farHeadingKd, defaultParams.headingKf);
    }
    public PathParams(double bigSpeedKp, double smallSpeedKp, double speedKi, double bigSpeedKd, double smallSpeedKd, double speedKf, double closeHeadingKp, double closeHeadingKi, double closeHeadingKd, double farHeadingKp, double farHeadingKi, double farHeadingKd, double headingKf) {
        this.bigSpeedKp = bigSpeedKp;
        this.smallSpeedKp = smallSpeedKp;
        this.speedKi = speedKi;
        this.bigSpeedKd = bigSpeedKd;
        this.smallSpeedKd = smallSpeedKd;
        this.speedKf = speedKf;
        this.closeHeadingKp = closeHeadingKp;
        this.closeHeadingKi = closeHeadingKi;
        this.closeHeadingKd = closeHeadingKd;
        this.farHeadingKp = farHeadingKp;
        this.farHeadingKi = farHeadingKi;
        this.farHeadingKd = farHeadingKd;
        this.headingKf = headingKf;

        initializeDefault();
    }
    private void initializeDefault() {
        correctiveKp = defaultParams.correctiveKp;
        correctiveStrength = defaultParams.correctiveStrength;
        maxTime = defaultParams.maxTime;
        minLinearPower = defaultParams.minSpeed;
        maxLinearPower = defaultParams.maxSpeed;
        minHeadingPower = defaultParams.minHeadingSpeed;
        maxHeadingPower = defaultParams.maxHeadingSpeed;
        lateralWeight = defaultParams.lateralWeight;
        axialWeight = defaultParams.axialWeight;
        passPosition = false;
        applyCloseSpeedPIDError = defaultParams.applyCloseSpeedPIDError;
        headingLerpType = defaultParams.headingLerpType;
        pathType = defaultParams.pathType;
        tValueFinishDist = defaultParams.tValueMaxOutTime;
        tangentHeadingDeactivateThreshold = defaultParams.tangentHeadingActivateThreshold;
        applyCloseHeadingPIDErrorDeg = defaultParams.applyCloseHeadingPIDErrorDeg;
        slowDownPercent = 1;
        prioritizeHeadingInBeginning = defaultParams.prioritizeHeadingInBeginning;
        controlPoint = new Pose2d(0, 0, 0);
    }
    public boolean hasMaxTime() {
        return maxTime != noMaxTime;
    }
}
