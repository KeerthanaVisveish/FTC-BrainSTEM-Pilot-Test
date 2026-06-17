//package org.firstinspires.ftc.teamcode.opmode.testing;
//
//import static org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils.createPose;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.opmode.Alliance;
//import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
//import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
//import org.firstinspires.ftc.teamcode.robot.shootingSystem.ShootingMathOld;
//import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
//
////@TeleOp(name="Power Efficiency Finder", group="TestingParams")
////@Config
//public class EfficiencyCoefficientFinder extends OpMode {
//    public static class Controls {
//        public double targetShooterVelocityTicksPerSec = 1500;
//        public double ballExitAngleRad = 0.6981317;
//        public boolean powerShooter = true;
//        public boolean powerIntake = true;
//    }
//    public static class Experiment {
//        public double gravityAcceleration = 9.81;
//        public double[] startPose = { 0, 0, 0 };
//
//        // represents distances the the balls landed from the center of the robot
//        public double[] distanceInches = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
//    }
//    public static Controls controls = new Controls();
//    public static Experiment experiment = new Experiment();
//
//    private BrainSTEMRobot robot;
//
//    double avgDistMeters, changeInYMeters;
//    double shooterVelTicksPerSec, shooterVelMetersPerSec;
//    double[] theoreticalDistMeters;
//    double actualExitVelocityMetersPerSecUsingRealWorldData;
//    double powerEfficiencyCoefficient;
//    double actualExitVelocityMetersPerSecUsingPowerLoss;
//    double[] expectedDistancesOfTravel;
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.setMsTransmissionInterval(20);
//        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(experiment.startPose));
//
//        robot.collector.clutchRight.setPosition(Collector.params.engagedPos);
//        robot.collector.clutchLeft.setPosition(Collector.params.engagedPos);
//    }
//    @Override
//    public void loop() {
//        if (gamepad1.aWasPressed())
//            controls.powerShooter = !controls.powerShooter;
//        if(gamepad1.rightBumperWasPressed())
//            controls.powerIntake = !controls.powerIntake;
//
//        double collectPower = controls.powerIntake ? Collector.params.fullIntakePower : 0;
//        robot.collector.collectorMotor.setPower(collectPower);
//
//        if (controls.targetShooterVelocityTicksPerSec == 0 || !controls.powerShooter) {
////            robot.shootingSystem.setShooterPower(0);
//        }
//        else {
//            shooterVelTicksPerSec = robot.shootingSystem.shooter.getVelTps();
//            robot.shootingSystem.shooter.setShooterVelocityPID(controls.targetShooterVelocityTicksPerSec, shooterVelTicksPerSec);
//            robot.shootingSystem.setHoodPosition(ShootingMathOld.getHoodServoPosition(controls.ballExitAngleRad));
//
//            Pose2d start = createPose(experiment.startPose);
//            // under the assumption that the turret is facing the robot's direction, only the x offset of the exit position matters
//            Pose2d turretPose = ShootingMathOld.calcTurretPose(start, 0);
//            Vector2d exitPosition = ShootingMathOld.getExitPositionInches(turretPose, controls.ballExitAngleRad);
//            avgDistMeters = calculateAvgDist(experiment.distanceInches, exitPosition) * 0.0254;
//            if (avgDistMeters < 0)
//                telemetry.addLine("__INPUT DISTANCES ARE NOT VALID");
//
//            // change in y = final y - initial y
//            changeInYMeters = ShootingMathOld.shooterSystemParams.ballRadiusMeters - ShootingMathOld.getExactExitHeightMeters(controls.ballExitAngleRad);
//
//            shooterVelMetersPerSec = ShootingMathOld.ticksPerSecToExitSpeedMps(shooterVelTicksPerSec, 1);
//            theoreticalDistMeters = calculateExpectedDistanceOfTravel(changeInYMeters, controls.ballExitAngleRad, shooterVelMetersPerSec);
//
//            actualExitVelocityMetersPerSecUsingRealWorldData = calculateActualExitVelocity(avgDistMeters, changeInYMeters, controls.ballExitAngleRad);
//            if (actualExitVelocityMetersPerSecUsingRealWorldData < 0) {
//                telemetry.addLine("actual exit velocity using real world data is not valid");
//                telemetry.update();
//                return;
//            }
//
//            powerEfficiencyCoefficient = actualExitVelocityMetersPerSecUsingRealWorldData / shooterVelMetersPerSec;
//            actualExitVelocityMetersPerSecUsingPowerLoss = ShootingMathOld.ticksPerSecToExitSpeedMps(shooterVelTicksPerSec, powerEfficiencyCoefficient); // this should equal actualExitVelocityMetersPerSecUsingRealWorldData
//            expectedDistancesOfTravel = calculateExpectedDistanceOfTravel(changeInYMeters, controls.ballExitAngleRad, actualExitVelocityMetersPerSecUsingPowerLoss); // this should equal avgDistMeters
//
//            robot.shootingSystem.sendHardwareInfo();
//            telemetry.addLine("a MISC=====");
//            telemetry.addData("b turret pose", MathUtils.formatPose3(turretPose));
//            telemetry.addData("c exit pos X inches", exitPosition.x);
//            telemetry.addData("d left hood pos", robot.shootingSystem.getHoodPosition());
//            telemetry.addLine();
//            telemetry.addLine("e CALCULATIONS============");
//            telemetry.addData("f change in Y from ball exit position (meters)", changeInYMeters);
//            telemetry.addData("g avg recorded landing distance from ball exit position (meters)", MathUtils.format3(avgDistMeters));
//            telemetry.addData("h avg theoretical landing distance without power loss (meters)", MathUtils.format3(theoreticalDistMeters));
//            telemetry.addLine("i");
//
//            telemetry.addData("j shooter angular velocity ticks per sec", MathUtils.format3(shooterVelTicksPerSec));
//            telemetry.addData("k tangential velocity meters per sec", MathUtils.format3(shooterVelMetersPerSec));
//            telemetry.addLine("l");
//            telemetry.addData("m actual ball exit velocity using real world data (meters per sec)", MathUtils.format3(actualExitVelocityMetersPerSecUsingRealWorldData));
//            telemetry.addData("n actual ball exit velocity using power loss (meters per sec)", MathUtils.format3(actualExitVelocityMetersPerSecUsingPowerLoss));
//            telemetry.addLine("o");
//            telemetry.addData("p expected distance of travel 1 with power loss (meters)", MathUtils.format3(expectedDistancesOfTravel[0]));
//            telemetry.addData("q expected distance of travel 2 with power loss (meters)", MathUtils.format3(expectedDistancesOfTravel[1]));
//            telemetry.addLine("r");
//            telemetry.addData("s calculated power efficiency coefficient", MathUtils.format(powerEfficiencyCoefficient, 8));
//            telemetry.addData("t reverse engineered shooter vel (t/s)", MathUtils.format3(ShootingMathOld.exitMpsToMotorTicksPerSec(actualExitVelocityMetersPerSecUsingRealWorldData, powerEfficiencyCoefficient)));
//
//            telemetry.update();
//        }
//    }
//
//    private double calculateAvgDist(double[] distances, Vector2d exitPosition) {
//        double zeroPosition = exitPosition.x;
//        double tapeMeasureSetupX = -BrainSTEMRobot.length/2;
//
//        double total = 0;
//        int numDists = 0;
//        for (double dist : distances) {
//            if (dist < 0)
//                continue;
//            numDists++;
//
//            double actualDist = (dist + tapeMeasureSetupX) - zeroPosition;
//            total += actualDist;
//        }
//        if (numDists == 0)
//            return -1;
//        return total / numDists;
//    }
//
//    private double calculateActualExitVelocity(double distanceMeters, double changeInYMeters, double exitAngleRad) {
//        double cosTheta = Math.cos(exitAngleRad);
//        double tanTheta = Math.tan(exitAngleRad);
//        double denominator = 2 * cosTheta * cosTheta * (distanceMeters * tanTheta - changeInYMeters);
//        if (denominator <= 0)
//            return -1;
//        double numerator = experiment.gravityAcceleration * distanceMeters * distanceMeters;
//        return Math.sqrt(numerator / denominator);
//    }
//    private double[] calculateExpectedDistanceOfTravel(double changeInYMeters, double exitAngleRad, double exitVelMetersPerSec) {
//        double g = experiment.gravityAcceleration;
//        double y = changeInYMeters;
//        double v = exitVelMetersPerSec;
//        double cosTheta = Math.cos(exitAngleRad);
//        double tanTheta = Math.tan(exitAngleRad);
//
//        double discriminantSquared = tanTheta * tanTheta - 2 * g * y / (v * v * cosTheta * cosTheta);
//        if (discriminantSquared < 0)
//            return new double[] { -1, -1 };
//
//        double discriminant = Math.sqrt(discriminantSquared);
//        double term1 = v * v * cosTheta * cosTheta / g;
//
//        double solution1 = term1 * (tanTheta + discriminant);
//        double solution2 = term1 * (tanTheta - discriminant);
//        return new double[] { solution1, solution2 };
//    }
//}
