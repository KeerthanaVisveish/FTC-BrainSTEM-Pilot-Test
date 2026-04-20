package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
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
        return telemetryPacket -> {
            robot.shootingSystem.setShouldScore(shouldScore);
            return false;
        };
    }

    public Action stopShooter() {
        return packet -> {
            robot.shooter.setShooterState(Shooter.ShooterState.OFF);
            return false;
        };
    }

    // COLLECTIONS
    public Action engageClutchAndIntake() {
        return new ParallelAction(
                engageClutch(),
                runIntake()
        );
    }
    public Action engageClutch() {
        return packet -> {
            robot.collection.setClutchState(Collection.ClutchState.ENGAGED);
            robot.collection.outtakeAfterClutchEngage = false;
            robot.collection.clutchTimer.reset();
            return false;
        };
    }

    public Action disengageClutch() {
        return packet -> {
            robot.collection.setClutchState(Collection.ClutchState.UNENGAGED);
            return false;
        };
    }

    public Action flickerUp() {
        return packet -> {
            robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);
            return false;
        };
    }
    public Action flickerHalfUp() {
        return packet -> {
            robot.collection.setFlickerState(Collection.FlickerState.HALF_UP_DOWN);
            return false;
        };
    }

    public Action flickerDown() {
        return packet -> {
            robot.collection.setFlickerState(Collection.FlickerState.DOWN);
            return false;
        };
    }


    public Action runIntake() {
        return packet -> {
            robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
            return false;
        };
    }

    public Action reverseIntake() {
        return packet -> {
            robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
            return false;
        };
    }

    public Action stopIntake() {
        return packet -> {
            robot.collection.setCollectionState(Collection.CollectionState.OFF);
            return false;
        };
    }
}