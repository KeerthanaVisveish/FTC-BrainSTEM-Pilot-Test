package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.turretParams;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.math.Vector3dOld;

@Config
public class ShootingMathOld {
    public static double g = 9.81;
    // stores all parameters of the shooter/hood/turret system
    public static class ShooterSystemParams {
        public double flywheelHeightMeters = 0.2413;
        public double flywheelOffsetFromTurretInches = 2.4783465;
        public double flywheelRadiusMeters = 0.0445;
        public double ballRadiusMeters = 0.064;
        public double shooterMotorTicksPerRev = 28;
    }
    public static class HoodSystemParams {
        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double servoRangeMm = 30;
        public double minAngleDeg = 15, maxAngleDeg = 55;
        public double highArcHoodAngleDegEstimation = 20, lowArcHoodAngleDegEstimation = 45;
    }
    public static ShooterSystemParams shooterSystemParams = new ShooterSystemParams();
    public static HoodSystemParams hoodSystemParams = new HoodSystemParams();

    public static Vector2d getExitPositionInches(Pose2d turretPose, double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        double shooterCombinedRadiusInches = (shooterSystemParams.flywheelRadiusMeters + shooterSystemParams.ballRadiusMeters) / 0.0254;
        double offsetFromTurretInches = shooterSystemParams.flywheelOffsetFromTurretInches - Math.cos(hoodAngleRad) * shooterCombinedRadiusInches;

        return new Vector2d(
                turretPose.position.x + offsetFromTurretInches * Math.cos(turretPose.heading.toDouble()),
                turretPose.position.y + offsetFromTurretInches * Math.sin(turretPose.heading.toDouble())
        );
    }

    // approximates ball exit height to avoid hood jitter in tele
    public static double approximateExitHeightM(boolean useHighArc) {
        double hoodAngleDeg = useHighArc ? hoodSystemParams.highArcHoodAngleDegEstimation : hoodSystemParams.lowArcHoodAngleDegEstimation;
        double exitAngleRad = Math.toRadians(90 - hoodAngleDeg);
        return getExactExitHeightMeters(exitAngleRad);
    }
    public static double getExactExitHeightMeters(double exitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - exitAngleRad;
        return shooterSystemParams.flywheelHeightMeters + (shooterSystemParams.flywheelRadiusMeters + shooterSystemParams.ballRadiusMeters) * Math.sin(hoodAngleRad);
    }

    public static double ticksPerSecToExitSpeedMps(double motorTicksPerSec, double efficiencyCoefficient) {
        double motorRevPerSec = motorTicksPerSec / shooterSystemParams.shooterMotorTicksPerRev;
        double motorAngularVel = motorRevPerSec * 2 * Math.PI;
        double flywheelAngularVel = motorAngularVel * 16 / 18;
        double flywheelTangentialVel = flywheelAngularVel * shooterSystemParams.flywheelRadiusMeters;
        return flywheelTangentialVel * efficiencyCoefficient;
    }
    public static double exitMpsToMotorTicksPerSec(double ballExitMps, double efficiencyCoefficient) {
        double flywheelTangentialVel = ballExitMps / efficiencyCoefficient;
        double flywheelAngularVel = flywheelTangentialVel / shooterSystemParams.flywheelRadiusMeters;
        double motorAngularVel = flywheelAngularVel * 18 / 16;
        double motorRevPerSec = motorAngularVel / (2 * Math.PI);
        return motorRevPerSec * shooterSystemParams.shooterMotorTicksPerRev;
    }



    ///  THE BIG FIVEEEEE
    public static double[] calculateLaunchVectorWithImpactAngle(double d, double h, double phi) {
        double theta = Math.atan(2 * h / d - Math.tan(phi)); // desired exit angle
        double v = calculateLaunchVelocityWithExitAngle(d, h, theta);
        return new double[] {v, theta};
    }
    public static double calculateLaunchVelocityWithExitAngle(double d, double h, double theta) {
        double num = g * d * d;
        double denom = 2 * (d * Math.tan(theta) - h) * Math.pow(Math.cos(theta), 2);
        return Math.sqrt(num / denom);
    }
    public static double calculateImpactAngle(double d, double h, double v, double theta) {
        double tanPhi = Math.tan(theta) - g * d / Math.pow(v * Math.cos(theta), 2);
        return Math.atan(tanPhi);
    }
    public static Vector3dOld calculateActualTargetExitVel(double topViewBallDir, double ballExitAngleRad, double targetVelMps, Vector2d robotVelAtExitPosMps) {
        Vector3dOld ballTravelDir = new Vector3dOld(Math.cos(topViewBallDir),0, Math.sin(topViewBallDir));
        Vector2d shootingAngle = new Vector2d(Math.cos(ballExitAngleRad), Math.sin(ballExitAngleRad));
        Vector3dOld absoluteTargetVelocity = ballTravelDir.times(shootingAngle.x).plus(Vector3dOld.j.times(shootingAngle.y));
        absoluteTargetVelocity = absoluteTargetVelocity.times(targetVelMps);

        return absoluteTargetVelocity.minus(new Vector3dOld(robotVelAtExitPosMps.x, 0, robotVelAtExitPosMps.y));
    }
    public static double calculateBallExitAngleRad(boolean useHighArc, double y, double x, double v) {
        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double sign = useHighArc ? 1 : -1;
        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return -1;

        double tanTheta = (v*v + sign * Math.sqrt(discriminant)) / (g * x);
        return Math.atan(tanTheta);
    }



    public static double getHoodServoPosition(double ballExitAngleRadians) {
        double hoodAngleFromXAxisRadians = Math.PI * 0.5 - ballExitAngleRadians;
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), hoodSystemParams.minAngleDeg, hoodSystemParams.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + hoodSystemParams.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - hoodSystemParams.restingDistanceMm;
        return linearDistanceToExtendMm / hoodSystemParams.servoRangeMm;
    }

    public static Pose2d getTurretPose(Pose2d robotPose, double turretRelativeAngleRad) {
        double robotHeading = robotPose.heading.toDouble();
        double xOffset = -Math.cos(robotHeading) * turretParams.offsetFromCenter;
        double yOffset = -Math.sin(robotHeading) * turretParams.offsetFromCenter;
        return new Pose2d(robotPose.position.x + xOffset, robotPose.position.y + yOffset, robotHeading + turretRelativeAngleRad);
    }
    public static Pose2d getRobotPose(Pose2d turretPose, double turretRelativeAngleRad) {
        double robotHeading = turretPose.heading.toDouble() - turretRelativeAngleRad;
        if(robotHeading > Math.PI)
            robotHeading -= Math.PI * 2;
        Vector2d robotTurretVec = new Vector2d(
                turretParams.offsetFromCenter * Math.cos(robotHeading),
                turretParams.offsetFromCenter * Math.sin(robotHeading)
        );
        return new Pose2d(turretPose.position.x + robotTurretVec.x, turretPose.position.y + robotTurretVec.y, robotHeading);
    }
}
