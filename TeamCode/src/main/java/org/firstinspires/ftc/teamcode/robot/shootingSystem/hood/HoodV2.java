package org.firstinspires.ftc.teamcode.robot.shootingSystem.hood;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

@Config
public class HoodV2 extends Hood {

    public static class Params {
        public double minExitAngle = Math.toRadians(30);
        public double maxExitAngle = Math.toRadians(65);
        public double kP = 2;
        public double kI = 0;
        public double kD = 0.1;
        public double kF = 0.08;
        public double kG = 0.01414;
        public double kGLookAheadScale = 1;
        public double maxPower = .99;
        //y=-0.37015x+261.88606
        public double encoderToExitAngleSlope = -0.37015, encoderToExitAngleIntercept = 261.88606;
        public double externalAngularOffset = 3.72; // figure out through CAD bc i don't know what part of hood corresponds to exit angle
        // assuming the conversion function: y = mx + b
        public double onTargetErrorThreshold = Math.toRadians(5);

        public double dampeningErrorThreshold = Math.toRadians(1);
        public double dampeningFactor = .3;
    }
    public static Params params = new Params();

    private final CRServo hoodLeft, hoodRight;

    private final PIDController pid;
    private double currentExitAngle, lookAheadExitAngle;
    private double pidPower, frictionPower, gravityPower, totalPower;
    private final SRSHub srsHub;

    // assuming srshub has already been initialized
    public HoodV2(HardwareMap hardwareMap, Telemetry telemetry, SRSHub srsHub) {
        super(hardwareMap, telemetry);

        this.srsHub = srsHub;

        hoodLeft = hardwareMap.get(CRServo.class, RobotProperties.hoodLeftName);
        hoodLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodRight = hardwareMap.get(CRServo.class, RobotProperties.hoodRightName);
        hoodRight.setDirection(DcMotorSimple.Direction.FORWARD);
        pid = new PIDController(params.kP, params.kI, params.kD);
    }
    @Override
    public void setTargetExitAngle(double exitAngle) {
        exitAngle = Range.clip(exitAngle, params.minExitAngle, params.maxExitAngle);
        pid.setTarget(exitAngle);
        pid.setPIDValues(params.kP, params.kI, params.kD);
        double prevExitAngle = currentExitAngle;
        currentExitAngle = getExitAngleFromPosition(srsHub.getHoodAbsEncoder());
        double error = pid.getTarget() - currentExitAngle;
        pidPower = pid.updateWithError(error);
        frictionPower = Math.signum(error) * params.kF;
        lookAheadExitAngle = currentExitAngle + (currentExitAngle - prevExitAngle) * params.kGLookAheadScale;
        gravityPower = -Math.sin(lookAheadExitAngle) * params.kG;
        totalPower = pidPower + frictionPower + gravityPower;
        totalPower = Range.clip(totalPower, -params.maxPower, params.maxPower);
        if(Math.abs(pid.getTarget() - currentExitAngle) < params.dampeningErrorThreshold)
            totalPower *= params.dampeningFactor;
        setHoodPower(totalPower);
    }
    @Override
    public boolean onTarget() {
        return Math.abs(pid.getTarget() - currentExitAngle) < params.onTargetErrorThreshold;
    }
    public void setHoodPower(double p) {
        hoodLeft.setPower(p);
        hoodRight.setPower(p);
    }
    private double getExitAngleFromPosition(double pos) {
        double deg = pos * params.encoderToExitAngleSlope + params.encoderToExitAngleIntercept + params.externalAngularOffset;
        return Math.toRadians(deg);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("HOOD------");
        telemetry.addData("HO target angle deg", Math.toDegrees(pid.getTarget()));
        telemetry.addData("HO current angle deg", Math.toDegrees(currentExitAngle));
        telemetry.addData("HO lookahead angle deg", Math.toDegrees(lookAheadExitAngle));
        telemetry.addData("HO on target", onTarget());
        telemetry.addData("HO pid power", pidPower);
        telemetry.addData("HO friction power", frictionPower);
        telemetry.addData("HO gravity power", gravityPower);
        telemetry.addData("HO total power", totalPower);
        telemetry.addData("HO raw encoder", srsHub.getHoodAbsEncoder());
    }
}
