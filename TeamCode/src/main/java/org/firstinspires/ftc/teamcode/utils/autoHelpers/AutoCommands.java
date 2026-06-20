package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.autosBase.AutoPid;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
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
            robot.shootingSystemV1.setTurretToGoalTargeting();
            return false;
        };
    }
    public Action enableCustomTurretTracking(double targetRelAngle) {
        return telemetryPacket -> {
            robot.shootingSystemV1.setTurretToCustomAngle(targetRelAngle, 0, false);
            return false;
        };
    }


    // SHOOTER
    public Action speedUpShooter() {
        return packet -> {
            robot.shootingSystemV1.setShooterToGoalTargeting();
            return !robot.shootingSystemV1.shooterFirstGood();
        };
    }
    public Action setShouldScore(boolean shouldScore) {
        if(shouldScore) {
            return new InstantAction(() -> {
                robot.shootingSystemV1.setShooterToGoalTargeting();
                robot.shootingSystemV1.setHoodToGoalTargeting();
            });
        }
        else {
            return new InstantAction(() -> {
                robot.shootingSystemV1.setShooterToCustomVoltage(AutoPid.shoot.shooterMissVoltage);
                robot.shootingSystemV1.setHoodToCustomExitAngle(AutoPid.shoot.hoodMissExitAngle);
            });
        }
    }

    // COLLECTIONS
    public Action engageClutch() {
        return packet -> {
            robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            robot.collector.clutchTimer.reset();
            robot.shootingSystemV1.shooter.resetNumBallsShot();
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