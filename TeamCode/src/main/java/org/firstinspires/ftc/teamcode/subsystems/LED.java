package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;

@Config
public class LED extends Component {
    public static double white = 0.99, green = 0.45, yellow = 0.35, orange = .3, lightBlue = .55, blue = 0.6, purple = 0.666, red = 0.279;
    public static double shooterFlashOnTime = 0.3, shooterFlashOffTime = 0.2;
    public static double turretFlashOnTime = 0.07, turretFlashOffTime = 0.07;
    public static double parkFlashTime = .4;
    public static double confirmSuccessfulPoseUpdateTime = 0.3;
    private final ServoImplEx left_led;
    private final ServoImplEx right_led;
    private final ElapsedTime shooterFlashTimer, turretFlashTimer, parkFlashTimer;
    public double lastPinpointResetTimeMs;
    private boolean autoDone;
    public LED(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        right_led = hardwareMap.get(ServoImplEx.class, "rightLED");
        left_led = hardwareMap.get(ServoImplEx.class, "leftLED");
        shooterFlashTimer = new ElapsedTime();
        shooterFlashTimer.reset();
        turretFlashTimer = new ElapsedTime();
        turretFlashTimer.reset();
        parkFlashTimer = new ElapsedTime();
        parkFlashTimer.reset();
        lastPinpointResetTimeMs = -1000000;
    }

    @Override
    public void printInfo() {}

    @Override
    public void update() {
        if(robot.parking.getParkState() == Parking.ParkState.EXTENDED) {
            if(parkFlashTimer.seconds() > parkFlashTime * 2)
                parkFlashTimer.reset();
            else if(parkFlashTimer.seconds() > parkFlashTime)
                setLed(0);
            else
                setLed(orange);
            return;
        }
        if(autoDone || BrainSTEMTeleOp.currentDriveAxialAmp != 1) {
            setLed(orange);
            return;
        }
        if (robot.turret.getTurretState() == Turret.TurretState.TRACK_CUSTOM_TARGET) {
            setLed(white);
            return;
        }
        if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
            setLed(white);
            return;
        }
        if (robot.limelight.localization.getPrevState() == LimelightLocalization.LocalizationState.UPDATING_POSE &&
                robot.limelight.localization.successfullyFoundPose &&
                robot.limelight.localization.getStateTime() < confirmSuccessfulPoseUpdateTime) {
            setLed(blue);
            return;
        }
        if (System.currentTimeMillis() - lastPinpointResetTimeMs < confirmSuccessfulPoseUpdateTime * 1000) {
            setLed(blue);
            return;
        }

        if (robot.shooter.getShooterState() == Shooter.ShooterState.UPDATE && !robot.shootingSystem.shooterNormGood()) {
            if (shooterFlashTimer.seconds() > shooterFlashOnTime + shooterFlashOffTime)
                shooterFlashTimer.reset();
            else if (shooterFlashTimer.seconds() > shooterFlashOnTime) {
                setLed(0);
                return;
            }
        }
        if((!robot.turret.inRange() || !robot.turret.onTarget()) && robot.turret.getTurretState() == Turret.TurretState.TRACKING) {
            if(turretFlashTimer.seconds() > turretFlashOnTime + turretFlashOffTime)
                turretFlashTimer.reset();
            else if(turretFlashTimer.seconds() > turretFlashOnTime) {
                setLed(0);
                return;
            }
        }
        if (robot.collection.getClutchState() == Collection.ClutchState.ENGAGED) {
            if (robot.collection.getCollectionState() == Collection.CollectionState.INTAKE || robot.collection.getCollectionState() == Collection.CollectionState.INTAKE_SLOW)
                if(robot.shooter.ballsCurrentlyExiting() && robot.collection.inAuto())
                    setLed(lightBlue);
                else
                    setLed(green);
            else
                setLed(yellow);
        }
        else if (robot.collection.has3Balls())
            setLed(purple);
        else
            setLed(red);
    }
    public void setLed(double position) {
        left_led.setPosition(position);
        right_led.setPosition(position);
    }
    public void setAutoDone() {
        autoDone = true;
    }
}