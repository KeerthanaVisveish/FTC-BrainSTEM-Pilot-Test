package org.firstinspires.ftc.teamcode.robot.shootingSystem.hood;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotProperties;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

@Config
public class HoodV2 extends Hood {

    public static class Params {
        public double kP = 0;
        public double kI = 0;
        public double kD = 0;
        public double kF = 0;

        public double radiansPerEncoder = 0;

        public double errorThreshold = Math.toRadians(.5);
    }
    public static Params params = new Params();

    private final CRServo hoodLeft, hoodRight;

    private final PIDController pid;
    private double currentAngle;
    private double pidPower, totalPower;
    private final SRSHub srsHub;

    // assuming srshub has already been initialized
    public HoodV2(HardwareMap hardwareMap, Telemetry telemetry, SRSHub srsHub) {
        super(hardwareMap, telemetry);

        this.srsHub = srsHub;

        // TODO: configure hood directions so positive power equates to positive encoder change
        hoodLeft = hardwareMap.get(CRServo.class, RobotProperties.hoodLeftName);
        hoodLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        hoodRight = hardwareMap.get(CRServo.class, RobotProperties.hoodRightName);
        hoodRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PIDController(params.kP, params.kI, params.kD);
    }

    @Override
    public void update() {
        pid.setPIDValues(params.kP, params.kI, params.kD);
        currentAngle = getExitAngleFromPosition(srsHub.getHoodAbsEncoder());
        double error = pid.getTarget() - currentAngle;
        pidPower = pid.updateWithError(error);
        totalPower = pidPower + Math.signum(error) * params.kF;
        setHoodPower(totalPower);
    }
    @Override
    public void setTargetExitAngle(double exitAngle) {
        pid.setTarget(exitAngle);
    }
    @Override
    public boolean onTarget() {
        return Math.abs(pid.getTarget() - currentAngle) < params.errorThreshold;
    }
    public void setHoodPower(double p) {
        hoodLeft.setPower(p);
        hoodRight.setPower(p);
    }
    private double getExitAngleFromPosition(double pos) {
        return pos * params.radiansPerEncoder;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("HOOD------");
        telemetry.addData("HO target angle deg", Math.toDegrees(pid.getTarget()));
        telemetry.addData("HO current angle deg", Math.toDegrees(currentAngle));
        telemetry.addData("HO on target", onTarget());
        telemetry.addData("HO pid power", pidPower);
        telemetry.addData("HO total power", totalPower);
        telemetry.addData("HO raw encoder", srsHub.getHoodAbsEncoder());
    }
}
