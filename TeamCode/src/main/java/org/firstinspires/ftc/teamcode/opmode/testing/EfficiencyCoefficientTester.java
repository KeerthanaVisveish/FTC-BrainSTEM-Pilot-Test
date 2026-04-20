package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMathOld;

//@TeleOp(name="Power Efficiency Tester", group="TestingParams")
//@Config
public class EfficiencyCoefficientTester extends OpMode {
    public static class Controls {
        public double ballExitAngleRad = Math.toRadians(40);
        public boolean powerShooter = true;
        public boolean powerIntake = true;
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double distanceToShootBallInches = 120; // distance to shoot ball from exit position of robot
        public double velToShootBallTps = 1200;
        public boolean useVelOverDistance = true;
        public double heightToShootBallInches = 0; // height to shoot ball from ground
    }
    public static class Experiment {
        public double gravityAcceleration = 9.81;
    }
    public static Controls controls = new Controls();
    public static Experiment experiment = new Experiment();

    private BrainSTEMRobot robot;

    double totalDistanceTraveledMeters, changeInYMeters;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, new Pose2d(0, 0, 0));

        robot.collection.clutchRight.setPosition(Collection.params.engagedPos);
        robot.collection.clutchLeft.setPosition(Collection.params.engagedPos);
    }
    @Override
    public void loop() {
        robot.shootingSystem.updateProperties();
        robot.shootingSystem.setHoodPosition(ShootingMathOld.getHoodServoPosition(controls.ballExitAngleRad));

        if (gamepad1.aWasPressed())
            controls.powerShooter = !controls.powerShooter;
        if(gamepad1.rightBumperWasPressed())
            controls.powerIntake = !controls.powerIntake;

        double collectPower = controls.powerIntake ? Collection.params.normIntakePow : 0;
        robot.collection.collectorMotor.setPower(collectPower);

        if (!controls.powerShooter) {
//            robot.shootingSystem.setShooterPower(0);
        }
        else {
            // under the assumption that the turret is facing the robot's direction, only the x offset of the exit position matters
            Pose2d start = new Pose2d(0, 0, 0);
            Vector2d exitPosition = ShootingMathOld.getExitPositionInches(ShootingMathOld.getTurretPose(start, 0), controls.ballExitAngleRad);

            double efficiencyCoefficient = controls.efficiencyCoefB + controls.ballExitAngleRad * controls.efficiencyCoefM;
            double targetVelMetersPerSec;

            double targetExitVelTicksPerSec;

            if(!controls.useVelOverDistance) {
                totalDistanceTraveledMeters = (controls.distanceToShootBallInches - BrainSTEMRobot.length / 2 + Math.abs(exitPosition.x)) * 0.0254;

                double initialHeightMeters = ShootingMathOld.getExactExitHeightMeters(controls.ballExitAngleRad);
                double finalHeightMeters = ShootingMathOld.shooterSystemParams.ballRadiusMeters + (controls.heightToShootBallInches * 0.0254);
                changeInYMeters = finalHeightMeters - initialHeightMeters;

                targetVelMetersPerSec = calculateTargetExitVelocity(totalDistanceTraveledMeters, changeInYMeters, controls.ballExitAngleRad);
                targetExitVelTicksPerSec = ShootingMathOld.exitMpsToMotorTicksPerSec(targetVelMetersPerSec, efficiencyCoefficient);
            }
            else {
                targetVelMetersPerSec = ShootingMathOld.ticksPerSecToExitSpeedMps(controls.velToShootBallTps, efficiencyCoefficient);
                targetExitVelTicksPerSec = controls.velToShootBallTps;
            }

            robot.shooter.setShooterVelocityPID(targetExitVelTicksPerSec, robot.shootingSystem.getFilteredShooterSpeedTps());
            telemetry.addData("a| exit pos X inches", exitPosition.x);
            telemetry.addData("b| distance meters", totalDistanceTraveledMeters);
            telemetry.addData("c| change in Y from ball exit position (m)", changeInYMeters);
            telemetry.addData("d| ball exit angle degrees", Math.toDegrees(controls.ballExitAngleRad));
            telemetry.addData("e| efficiency coefficient", efficiencyCoefficient);
            telemetry.addData("f| target exit velocity mps", targetVelMetersPerSec);
            telemetry.addData("g| target exit velocity tps", targetExitVelTicksPerSec);
            telemetry.addData("i| current shooter vel tps", robot.shootingSystem.getFilteredShooterSpeedTps());
            telemetry.addData("j| shooter power", robot.shootingSystem.getShooterHighPower());
            telemetry.addData("k| shooter high vel", robot.shootingSystem.getShooterHighRawVelTps());
            telemetry.addData("l| shooter low vel", robot.shootingSystem.getShooterLowRawVelTps());
            telemetry.addData("m| dt", robot.shootingSystem.dt);
            telemetry.update();
        }
        robot.shootingSystem.sendHardwareInfo();
    }

    public static double calculateTargetExitVelocity(double distanceMeters, double changeInYMeters, double exitAngleRad) {
        double cosTheta = Math.cos(exitAngleRad);
        double tanTheta = Math.tan(exitAngleRad);
        double denominator = 2 * cosTheta * cosTheta * (distanceMeters * tanTheta - changeInYMeters);
        if (denominator <= 0)
            return -1;
        double numerator = experiment.gravityAcceleration * distanceMeters * distanceMeters;
        return Math.sqrt(numerator / denominator);
    }
}
