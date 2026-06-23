package org.firstinspires.ftc.teamcode.robot.shootingSystem.hood;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;

@Config
public class HoodV1 extends Hood {

    public static class Params {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);

        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double servoRangeMm = 30;
        public double minAngleDeg = 15, maxAngleDeg = 55;
    }
    public static Params params = new Params();


    private final ServoImplEx hoodLeft, hoodRight;
    public HoodV1(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        hoodLeft = hardwareMap.get(ServoImplEx.class, RobotProperties.hoodLeftName);
        hoodLeft.setPwmRange(new PwmControl.PwmRange(params.downPWM, params.upPWM));
        hoodRight = hardwareMap.get(ServoImplEx.class, RobotProperties.hoodRightName);
        hoodRight.setPwmRange(new PwmControl.PwmRange(params.downPWM, params.upPWM));
    }

    public void setPosition(double position) {
        hoodLeft.setPosition(position);
        hoodRight.setPosition(position);
    }
    public void setTargetExitAngle(double exitAngle) {
        setPosition(getPositionFromExitAngle(Range.clip(exitAngle, params.minExitAngRad, params.maxExitAngRad)));
    }

    @Override
    public boolean onTarget() {
        return true;
    }

    public double getPositionFromExitAngle(double ballExitAngleRadians) {
        double hoodAngleFromXAxisRadians = Math.PI * 0.5 - ballExitAngleRadians;
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), params.minAngleDeg, params.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + params.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - params.restingDistanceMm;
        return Range.clip(linearDistanceToExtendMm / params.servoRangeMm, 0, 1);
    }

    @Override
    public void printInfo() {

    }
}