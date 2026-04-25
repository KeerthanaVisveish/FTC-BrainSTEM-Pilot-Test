package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
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
            robot.turret.setTurretState(Turret.TurretState.TRACKING);
            return false;
        };
    }
    public Action enableCustomTurretTracking(double targetRelAngle) {
        return telemetryPacket -> {
            robot.turret.rotateToRelativeCustomTarget(targetRelAngle);
            robot.turret.setCustomTargetPassPosition(true);
            return false;
        };
    }

    public Action turretCenter() {
        return packet -> {
            robot.turret.setTurretState(Turret.TurretState.CENTER);
            return false;
        };
    }

    // SHOOTER
    public Action speedUpShooter() {
        return packet -> {
            robot.shooter.setShooterState(Shooter.ShooterState.UPDATE);
            return !robot.shootingSystem.shooterFirstGood();
        };
    }
    public Action setShouldScore(boolean shouldScore) {
        return new InstantAction(() -> robot.shootingSystem.setShouldScore(shouldScore));
    }
    public Action setMaxVoltage(double m) {
        return new InstantAction(() -> robot.shooter.setMaxVoltage(m));
    }

    // COLLECTIONS
    public Action engageClutch() {
        return packet -> {
            robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            robot.collector.outtakeAfterClutchEngage = false;
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
            robot.collector.setCollectionState(Collector.CollectionState.INTAKE);
            return false;
        };
    }

    public Action reverseIntake() {
        return packet -> {
            robot.collector.setCollectionState(Collector.CollectionState.OUTTAKE);
            return false;
        };
    }

    public Action stopIntake() {
        return packet -> {
            robot.collector.setCollectionState(Collector.CollectionState.OFF);
            return false;
        };
    }
}