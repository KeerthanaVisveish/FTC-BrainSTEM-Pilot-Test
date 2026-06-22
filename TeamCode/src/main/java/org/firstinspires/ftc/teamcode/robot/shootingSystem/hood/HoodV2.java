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
        public double minExitAngle = Math.toRadians(20);
        public double maxExitAngle = Math.toRadians(69);
        public double kP = 2;
        public double kI = 0;
        public double kD = 0.01;
        public double kF = 0.08;
        public double kG = 0.01414;
        public double kGLookAhead = 0; // assumes 50Hz loop times
        public double maxPower = .99;
        //y=-0.00668771x+3.47549
        public double encoderToExitAngleSlope = -0.00668771, encoderToExitAngleIntercept = 3.47549;
        public double externalAngularOffset = 0; // figure out through CAD bc i don't know what part of hood corresponds to exit angle
        // assuming the conversion function: y = mx + b
        public double onTargetErrorThreshold = Math.toRadians(.5);
        public double onTargetDampeningFactor = .3;
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

        // TODO: configure hood directions so positive power equates to positive exit angle change
        hoodLeft = hardwareMap.get(CRServo.class, RobotProperties.hoodLeftName);
        hoodLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodRight = hardwareMap.get(CRServo.class, RobotProperties.hoodRightName);
        hoodRight.setDirection(DcMotorSimple.Direction.FORWARD);
        pid = new PIDController(params.kP, params.kI, params.kD);
    }

    @Override
    public void update() {
    }
    @Override
    public void setTargetExitAngle(double exitAngle) {
        exitAngle = Range.clip(exitAngle, params.minExitAngle, params.maxExitAngle);
        pid.setTarget(exitAngle);
        pid.setPIDValues(params.kP, params.kI, params.kD);
        currentExitAngle = getExitAngleFromPosition(srsHub.getHoodAbsEncoder());
        double error = pid.getTarget() - currentExitAngle;
        pidPower = pid.updateWithError(error);
        frictionPower = Math.signum(error) * params.kF;
        lookAheadExitAngle = currentExitAngle + getAngularVelocityFromEncoder(srsHub.getHoodVelocity()) * params.kGLookAhead;
        gravityPower = -Math.sin(lookAheadExitAngle) * params.kG;
        totalPower = pidPower + frictionPower + gravityPower;
        totalPower = Range.clip(totalPower, -params.maxPower, params.maxPower);
        if(onTarget())
            totalPower *= params.onTargetDampeningFactor;
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
        return pos * params.encoderToExitAngleSlope + params.encoderToExitAngleIntercept + params.externalAngularOffset;
    }
    private double getAngularVelocityFromEncoder(double encoderVelocity) {
        return encoderVelocity * params.encoderToExitAngleSlope;
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
