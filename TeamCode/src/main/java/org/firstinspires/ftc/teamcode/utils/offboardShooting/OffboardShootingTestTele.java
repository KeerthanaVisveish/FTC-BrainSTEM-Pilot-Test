package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.hood.HoodV2;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter.ShooterV2;
import org.firstinspires.ftc.teamcode.utils.misc.BatteryVoltageFilter;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.util.ArrayList;

@TeleOp(name="OffboardShootingTestTele")
@Config
public class OffboardShootingTestTele extends OpMode {
    public static double feetFromGoal;

    public enum ControlType {
        EXIT_SPEED,
        EXIT_ANGLE,
        IMPACT_ANGLE,
        OPTIMAL
    }
    public static boolean setShooterHoodToTrajectory = false;
    public static ControlType controlType = ControlType.OPTIMAL;
    public static double exitSpeed, exitAngleDeg, impactAngleDeg;
    private TrajectoryDistanceLUT trajectoryDistanceLUT;
    private ShooterV2 shooter;
    private HoodV2 hood;
    private BatteryVoltageFilter batteryVoltageFilter;
    @Override
    public void init() {
        trajectoryDistanceLUT = TrajectoryLoader.loadFromSettingsFile("mtiTrajectories.json");
        SRSHub srsHub = new SRSHub(hardwareMap, telemetry);
        shooter = new ShooterV2(hardwareMap, telemetry, srsHub);
        hood = new HoodV2(hardwareMap, telemetry, srsHub);
        batteryVoltageFilter = BatteryVoltageFilter.getInstance(hardwareMap);

    }

    @Override
    public void loop() {
        batteryVoltageFilter.update();

        telemetry.addData("Control Type", controlType.name());
        telemetry.addLine("Aiming " + feetFromGoal + "feet away");

        Trajectory trajectory = chooseTrajectory(controlType);

        telemetry.addData("Chosen Trajectory", trajectory);

        if (trajectory != null) {
            if (setShooterHoodToTrajectory) {
                double shooterSpeedTps = ShooterV2.params.getTpsFunction.apply(trajectory.exitSpeedMps);
                shooter.setShooterVelocityPID(shooterSpeedTps, batteryVoltageFilter.getVoltage());

                hood.setTargetExitAngle(trajectory.exitAngleRad);
            }

            ArrayList<Vector3d> points = trajectory.simulateTrajectory(300, 10, 0, new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
            Vector3d startingPositionMeters = points.get(0);
            Vector3d startingPositionFeet = startingPositionMeters.times(3.281);
            Vector3d landingPositionMeters = points.get(points.size() - 1);
            Vector3d landingPositionFeet = landingPositionMeters.times(3.281);

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("green");
            packet.fieldOverlay().fillCircle(startingPositionFeet.x, startingPositionFeet.y, 3);
            packet.fieldOverlay().setStroke("red");
            packet.fieldOverlay().fillCircle(landingPositionFeet.x, landingPositionFeet.y, 3);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        telemetry.update();
    }

    private Trajectory chooseTrajectory(ControlType controlType) {
        double metersFromGoal = feetFromGoal * 3.28084;

        return switch (controlType) {
            case EXIT_SPEED ->
                    trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(metersFromGoal, exitSpeed);
            case EXIT_ANGLE ->
                    trajectoryDistanceLUT.getInterpolatedExitAngleTrajectory(metersFromGoal, exitAngleDeg);
            case IMPACT_ANGLE ->
                    trajectoryDistanceLUT.getInterpolatedImpactAngleTrajectory(metersFromGoal, impactAngleDeg);
            case OPTIMAL -> trajectoryDistanceLUT.getInterpolatedOptimalTrajectory(metersFromGoal);
        };
    }
}
