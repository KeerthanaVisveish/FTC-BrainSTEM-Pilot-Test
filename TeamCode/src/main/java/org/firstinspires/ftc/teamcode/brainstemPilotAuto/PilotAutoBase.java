package org.firstinspires.ftc.teamcode.brainstemPilotAuto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.TelemetryLog;
import org.brainstemfirst.pilot.ftc.PilotOpMode;
import org.brainstemfirst.pilot.ftc.PilotRegistry;
import org.brainstemfirst.pilot.ftc.bezier.follower.BezierFollowerConfig;
import org.brainstemfirst.pilot.ftc.model.PilotAlliance;
import org.brainstemfirst.pilot.ftc.model.PilotDrive;

/**
 * TEAM-OWNED — Brainstem Pilot UI creates this file once and will not overwrite it.
 * Wire your robot, drive, and {@code PilotRegistry.addCommand} calls here.
 * Generated OpModes in {@code opmodeAutos/} extend this class.
 *
 * Road Runner feedforward ({@code kV}/{@code kS}/{@code kA}) lives on
 * {@code MecanumDrive.PARAMS}, not here.
 */
public abstract class PilotAutoBase extends PilotOpMode {
    protected BrainSTEMRobot robot;

    protected PilotAutoBase(String autoId) {
        super(autoId);
        configureFollower();
    }

    /** Bézier follower gains. FTC Dashboard can still override these at runtime. */
    private void configureFollower() {
        BezierFollowerConfig.useVelocityProfile = true;
        BezierFollowerConfig.velKv = 0.014;
        BezierFollowerConfig.velKs = 0.03;
        BezierFollowerConfig.velKp = 0.05;
        BezierFollowerConfig.crossTrackKp = 0.05;
        BezierFollowerConfig.speedkP = 0.05;
        BezierFollowerConfig.speedkF = 0.05;
        BezierFollowerConfig.speedkD = 0.0;
        BezierFollowerConfig.correctivePower = 0.7;
        BezierFollowerConfig.headingkP = 0.05;
        BezierFollowerConfig.headingkF = 0.05;
        BezierFollowerConfig.overrideCruiseVel = false;
        BezierFollowerConfig.cruiseVel = 30;
        BezierFollowerConfig.overrideProfileDecel = false;
        BezierFollowerConfig.profileDecel = 40;
    }

    @Override
    protected void setupRobot(PilotAlliance alliance, Pose2d startPose) {
        TelemetryLog.set(telemetry);
        Alliance teamAlliance = alliance == PilotAlliance.RED ? Alliance.RED : Alliance.BLUE;
        robot = new BrainSTEMRobot(teamAlliance, telemetry, hardwareMap, startPose);
        robot.drive.pinpoint().setPose(startPose);
    }

    @Override
    protected PilotDrive getDrive() {
        return robot.drive;
    }

    @Override
    protected void registerCommands() {
        PilotRegistry.addCommand("Collector", "Intake On", () -> packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
            return false;
        });

        PilotRegistry.addCommand("Collector", "Intake Off", () -> packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.OFF);
            return false;
        });

        PilotRegistry.addCommand("Shooter", "Shooter On", () -> packet -> {
            robot.shootingSystem.setShooterToGoalTargeting();
            robot.shootingSystem.setHoodToGoalTargeting();
            return false;
        });

        PilotRegistry.addCommand("Turret", "Track Turret", () -> packet -> {
            robot.shootingSystem.setTurretToGoalTargeting();
            return false;
        });

        PilotRegistry.addCommand("Transfer", "Engage Clutch", () -> packet -> {
            robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            return false;
        });

        PilotRegistry.addCommand("Transfer", "Disengage Clutch", () -> packet -> {
            robot.collector.setClutchState(Collector.ClutchState.DISENGAGED);
            return false;
        });

        PilotRegistry.addCommand("Transfer", "Flicker", () -> packet -> {
            robot.collector.setFlickerState(Collector.FlickerState.FULL_UP_DOWN);
            return false;
        });
    }

    @Override
    protected void onOpModeStart() {
        robot.startOpmode();
    }

    @Override
    protected boolean updateRobot(TelemetryPacket packet) {
        robot.update();
        robot.drive.updatePoseEstimate();
        return true;
    }

    @Override
    protected void drawRobot(Canvas canvas) {
        robot.drawRobotInfo(canvas);
    }
}
