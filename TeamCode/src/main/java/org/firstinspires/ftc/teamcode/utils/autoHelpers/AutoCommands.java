package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public Action waitTillDoneShooting(double maxTimeBetweenShots, double minTime) {
        return new Action() {
            private final ElapsedTime distanceSensorTimer = new ElapsedTime();
            private final ElapsedTime timeSinceLastVelDrop = new ElapsedTime();
            private final ElapsedTime totalTimer = new ElapsedTime();
            private final ElapsedTime timeSinceIntakeSwitch = new ElapsedTime();
            private int oldBallsShot;
            private boolean first = true;
            boolean alreadyOuttaked = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    distanceSensorTimer.reset();
                    timeSinceLastVelDrop.reset();
                    oldBallsShot = robot.shooter.getBallsShot();
                    totalTimer.reset();
                }

                if (oldBallsShot != robot.shooter.getBallsShot())
                    timeSinceLastVelDrop.reset();
                oldBallsShot = robot.shooter.getBallsShot();
//                telemetry.addData("time since last vel drop", timeSinceLastVelDrop.seconds());

                if(robot.collection.isBackBallDetected())
                    distanceSensorTimer.reset();

                if(robot.collection.getIntakePower() == Collection.params.impossibleShotIntakePow) {
                    distanceSensorTimer.reset();
                    timeSinceLastVelDrop.reset();
                }

                if(!alreadyOuttaked && totalTimer.seconds() > 1 && robot.shooter.getBallsShot() == 0 && robot.collection.getCollectionState() == Collection.CollectionState.INTAKE) {
                    robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
                    timeSinceIntakeSwitch.reset();
                    timeSinceLastVelDrop.reset();
                    alreadyOuttaked = true;
                }
                if(alreadyOuttaked && timeSinceIntakeSwitch.seconds() > Collection.shootOuttakeTimeAuto)
                    robot.collection.setCollectionState(Collection.CollectionState.INTAKE);

                boolean done = totalTimer.seconds() > 3.5 || ((robot.shooter.getBallsShot() == 3 || timeSinceLastVelDrop.seconds() > 1.1) && distanceSensorTimer.seconds() >= 0.15);
//                return totalTimer.seconds() < 2 && timeSinceFirstVelDrop.seconds() < 0.9;
                return !done;
//                return (timeSinceLastVelDrop.seconds() < maxTimeBetweenShots && robot.shooter.getBallsShot() < 3) || distanceSensorTimer.seconds() < 0.5;
//                return timeSinceLastVelDrop.seconds() < maxTimeBetweenShots && timer.seconds() < intakeCurrentValidationTime && robot.shooter.getBallsShot() < 3;
            }
        };
    }
    // CONSTANT UPDATES
    public Action updateRobot = packet -> {
        robot.update(true);
        return true;
    };

    public Action savePoseContinuously = packet -> {
        Pose2d cur = robot.drive.localizer.getPose();
        PoseStorage.autoX = cur.position.x;
        PoseStorage.autoY = cur.position.y;
        PoseStorage.autoHeading = cur.heading.toDouble();
        return true;
    };

    // TURRET
    public Action enableTurretTracking() {
        return packet -> {
            robot.turret.turretState = Turret.TurretState.TRACKING;
            return false;
        };
    }

    public Action turretCenter() {
        return packet -> {
            robot.turret.turretState = Turret.TurretState.CENTER;
            return false;
        };
    }

    // SHOOTER
    public Action speedUpShooter() {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.UPDATE;
            return !robot.shootingSystem.shooterGood();
        };
    }

    public Action stopShooter() {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.OFF;
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
            robot.collection.clutch_timer.reset();
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
            robot.collection.setCollectionState(Collection.CollectionState.OFF);
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
    public Action intakeSlow() {
        return telemetryPacket -> {
            robot.collection.setCollectionState(Collection.CollectionState.INTAKE_SLOW);
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