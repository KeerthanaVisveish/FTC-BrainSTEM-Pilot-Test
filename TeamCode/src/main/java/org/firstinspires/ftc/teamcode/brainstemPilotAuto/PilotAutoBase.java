package org.firstinspires.ftc.teamcode.brainstemPilotAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.brainstemfirst.pilot.ftc.bezier.follower.BezierDrivePath;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.TelemetryLog;
import org.brainstemfirst.pilot.ftc.PilotOpMode;
import org.brainstemfirst.pilot.ftc.PilotRegistry;
import org.brainstemfirst.pilot.ftc.model.FieldConstants;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * TEAM-OWNED — Brainstem Pilot UI creates this file once and will not overwrite it.
 * Wire your robot, drive, and {@code PilotRegistry.addCommand} calls here.
 * Generated OpModes in {@code opmodeAutos/} extend this class.
 */
public abstract class PilotAutoBase extends PilotOpMode {
    protected BrainSTEMRobot robot;

    protected PilotAutoBase(String autoId) {
        super(autoId);
        configureFollower();
    }

    /** Dashboard knobs live on {@link BezierFollower}; this copies them onto the library. */
    private void configureFollower() {
        BezierFollower.apply();
    }

    @Override
    protected void setupRobot(FieldConstants.Alliance alliance, Pose2d startPose) {
        TelemetryLog.set(telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Alliance teamAlliance = alliance == FieldConstants.Alliance.RED ? Alliance.RED : Alliance.BLUE;
        robot = new BrainSTEMRobot(teamAlliance, telemetry, hardwareMap, startPose);
        robot.drive.pinpoint().setPose(startPose);
    }

    @Override
    protected Supplier<Pose2d> pose() {
        return robot.drive::getPose;
    }

    @Override
    protected Supplier<PoseVelocity2d> lastVelRobot() {
        return robot.drive::lastVelRobot;
    }

    @Override
    protected Consumer<PoseVelocity2d> setDrivePowers() {
        return robot.drive::setDrivePowers;
    }

    @Override
    protected DoubleSupplier maxAngVel() {
        return robot.drive::maxAngVel;
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
        BezierFollower.apply();
        robot.update();
        robot.drive.updatePoseEstimate();

        BezierDrivePath.Status s = BezierDrivePath.status();
        Pose2d poseNow = robot.drive.getPose();
        telemetry.addData("target X", s.targetPoint.x);
        telemetry.addData("target Y", s.targetPoint.y);
        telemetry.addData("current X", poseNow.position.x);
        telemetry.addData("current Y", poseNow.position.y);
        telemetry.addData("isFinished", s.finished);
        telemetry.addData("heading rad", poseNow.heading.toDouble());
        telemetry.addData("x vel", robot.drive.lastVelRobot().linearVel.x);
        telemetry.addData("y vel", robot.drive.lastVelRobot().linearVel.y);
        telemetry.addData("mag vel", robot.drive.lastVelRobot().linearVel.norm());
        telemetry.update();

        return true;
    }
}
