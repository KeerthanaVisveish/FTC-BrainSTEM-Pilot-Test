package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.Turret;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public Action updateRobotInfo() {
        return packet -> {
            robot.updateInfo();
            return true;
        };
    }
    public Action updateRobot(){
        return packet -> {
            robot.update();
            return true;
        };
    }

    public Action savePoseContinuously() {
        return packet -> {
            Pose2d cur = robot.drive.localizer.getPose();
            PoseStorage.autoX = cur.position.x;
            PoseStorage.autoY = cur.position.y;
            PoseStorage.autoHeading = cur.heading.toDouble();
            return true;
        };
    }

    // TURRET
    public Action enableTurretTracking() {
        return packet -> {
            robot.shootingSystem.setTurretState(ShootingSystem.TurretState.TRACKING);
            return false;
        };
    }
    public Action enableCustomTurretTracking(double targetRelAngle) {
        return telemetryPacket -> {
            robot.shootingSystem.trackCustomTarget(targetRelAngle);
            return false;
        };
    }

    public Action turretCenter() {
        return packet -> {
            robot.shootingSystem.setTurretState(ShootingSystem.TurretState.CENTER);
            return false;
        };
    }

    // SHOOTER
    public Action speedUpShooter() {
        return packet -> {
            robot.shootingSystem.setShooterState(ShootingSystem.ShooterState.ON);
            return !robot.shootingSystem.shooterFirstGood();
        };
    }
    public Action setShouldScore(boolean shouldScore) {
        return new SleepAction(0); // TODO: actually implement this
    }

    // COLLECTIONS
    public Action engageClutch() {
        return packet -> {
            robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            robot.collector.clutchTimer.reset();
            return false;
        };
    }

    public Action disengageClutch() {
        return packet -> {
            robot.collector.setClutchState(Collector.ClutchState.UNENGAGED);
            return false;
        };
    }

    public Action flickerUp() {
        return packet -> {
            robot.collector.setFlickerState(Collector.FlickerState.FULL_UP_DOWN);
            return false;
        };
    }
    public Action flickerHalfUp() {
        return packet -> {
            robot.collector.setFlickerState(Collector.FlickerState.HALF_UP_DOWN);
            return false;
        };
    }

    public Action flickerDown() {
        return packet -> {
            robot.collector.setFlickerState(Collector.FlickerState.DOWN);
            return false;
        };
    }


    public Action runIntake() {
        return packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
            return false;
        };
    }

    public Action reverseIntake() {
        return packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.OUTTAKE);
            return false;
        };
    }

    public Action stopIntake() {
        return packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.OFF);
            return false;
        };
    }
}