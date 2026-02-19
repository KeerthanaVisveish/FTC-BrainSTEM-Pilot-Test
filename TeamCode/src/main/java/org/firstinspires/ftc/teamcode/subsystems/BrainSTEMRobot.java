package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;


@Config
public class BrainSTEMRobot {
    public static double width = 13, length = 16; // inches
    public static boolean enablePinpoint = true, enableSubsystems = true;
    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableLimelight = true, enablePark = true, enableLED = true;
    public static double voltageAlpha = .99, voltageDataBuildupTime = 1;
    public Turret turret;
    public Shooter shooter;
    public ShootingSystem shootingSystem;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
    public Limelight limelight;
    public LED led;
    public static Alliance alliance;
    private final ArrayList<Component> subsystems;
    private final Telemetry telemetry;
    public GamepadTracker g1;

    private double rawVoltage, filteredVoltage;
    private final ElapsedTime timer;

    public BrainSTEMRobot(Alliance allianceColor, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        BrainSTEMRobot.alliance = allianceColor;
        subsystems = new ArrayList<>();

        drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = new Limelight(hardwareMap, telemetry, this);
        shootingSystem = new ShootingSystem(hardwareMap, this);
        turret = new Turret(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, this);
        collection = new Collection(hardwareMap, telemetry, this);
        parking = new Parking(hardwareMap, telemetry, this);
        led = new LED(hardwareMap, telemetry, this);

        if (enableTurret)
            subsystems.add(turret);
        if (enableShooter)
            subsystems.add(shooter);
        if (enableCollection)
            subsystems.add(collection);
        if (enablePark)
            subsystems.add(parking);
        if (enableLimelight)
            subsystems.add(limelight);
        if (enableLED)
            subsystems.add(led);

        timer = new ElapsedTime();
        timer.reset();
    }
    public void startOpmode() {
        timer.reset();
    }
    public void setG1(GamepadTracker g1) {
        this.g1 = g1;
    }
    public void update(boolean useTurretLookAhead) {
        rawVoltage = drive.voltageSensor.getVoltage();
        double a = timer.seconds() < voltageDataBuildupTime ? 0 : voltageAlpha;
        filteredVoltage = filteredVoltage * a + (1 - a) * rawVoltage;
        if(enablePinpoint)
            drive.updatePoseEstimate();
        shootingSystem.updateInfo(useTurretLookAhead);


        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("Robot Pose", MathUtils.format3(pose.position.x) + ", " + MathUtils.format3(pose.position.y) + " | " + MathUtils.format3(pose.heading.toDouble()));
        if(enableSubsystems)
            for (Component c : subsystems)
                c.update();

        shootingSystem.sendHardwareInfo();
    }

    public void addRobotInfo(Canvas fieldOverlay) {
        // draw robot, turret, exit position, and limelight pose
        Pose2d robotPose = drive.pinpoint().getPose();

        fieldOverlay.setStroke("red");
        Drawing.drawRobot(fieldOverlay, robotPose);
        fieldOverlay.setStroke("green");
        Drawing.drawRobotSimple(fieldOverlay, shootingSystem.turretPose, 5);
        fieldOverlay.setStroke("purple");
        Drawing.drawRobotSimple(fieldOverlay, new Pose2d(shootingSystem.ballExitPos, 0), 3);
//        fieldOverlay.setStroke("purple");
//        Drawing.drawRobotSimple(fieldOverlay, new Pose2d(shootingSystem.futureBallExitPos, 0), 3);

        limelight.addLimelightInfo(fieldOverlay);

        // draw where turret is pointed
        fieldOverlay.setAlpha(1);
        double dist = Math.hypot(shootingSystem.ballExitPos.x - shootingSystem.goalPosIn.x, shootingSystem.ballExitPos.y - shootingSystem.goalPosIn.y);

        fieldOverlay.setStroke("purple");
        fieldOverlay.strokeLine(
                shootingSystem.ballExitPos.x,
                shootingSystem.ballExitPos.y,
                shootingSystem.ballExitPos.x + dist * Math.cos(turret.currentAbsoluteAngleRad),
                shootingSystem.ballExitPos.y + dist * Math.sin(turret.currentAbsoluteAngleRad)
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                shootingSystem.ballExitPos.x,
                shootingSystem.ballExitPos.y,
                shootingSystem.ballExitPos.x + dist * Math.cos(shootingSystem.actualTurretTargetAngleRad),
                shootingSystem.ballExitPos.y + dist * Math.sin(shootingSystem.actualTurretTargetAngleRad)
        );
        double speedMag = shootingSystem.actualTargetExitSpeedMps;
        fieldOverlay.setStroke("red");
        fieldOverlay.strokeLine(
                shootingSystem.ballExitPos.x,
                shootingSystem.ballExitPos.y,
                shootingSystem.ballExitPos.x + 1.5 * speedMag * Math.cos(shootingSystem.actualTurretTargetAngleRad),
                shootingSystem.ballExitPos.y + 1.5 * speedMag * Math.sin(shootingSystem.actualTurretTargetAngleRad)
        );
    }
    public double getFilteredVoltage() {
        return filteredVoltage;
    }
    public double getRawVoltage() {
        return rawVoltage;
    }
}
