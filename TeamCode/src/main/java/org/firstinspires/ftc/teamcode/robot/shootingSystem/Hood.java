package org.firstinspires.ftc.teamcode.robot.shootingSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.subsystems.Component;

@Config
public class Hood extends Component {

    public static class Params {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
    }
    public static Params params = new Params();


    private static class HoodSystemParams {
        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double servoRangeMm = 30;
        public double minAngleDeg = 15, maxAngleDeg = 55;
    }
    private static final HoodSystemParams hoodSystemParams = new HoodSystemParams();


    private final ServoImplEx hoodLeft, hoodRight;
    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
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
    public void setExitAngle(double exitAngle) {
        setPosition(getServoPosFromExitAngle(exitAngle));
    }

    public double getServoPosFromExitAngle(double ballExitAngleRadians) {
        double hoodAngleFromXAxisRadians = Math.PI * 0.5 - ballExitAngleRadians;
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), hoodSystemParams.minAngleDeg, hoodSystemParams.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + hoodSystemParams.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - hoodSystemParams.restingDistanceMm;
        return linearDistanceToExtendMm / hoodSystemParams.servoRangeMm;
    }

    @Override
    public void printInfo() {

    }
}