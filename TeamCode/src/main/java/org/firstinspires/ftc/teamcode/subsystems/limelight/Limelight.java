
package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Component;
import org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection.LimelightBallDetection;
import org.firstinspires.ftc.teamcode.subsystems.limelight.classifier.LimelightClassifier;
import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;

import java.util.ArrayList;

@Config
public class Limelight extends Component {
    public static class HardwareParams {
        public double distFromTurret = 5.9148;
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        public double verticalAngleOffset = -1.5;
        public double axialDistanceOffset = 5;
        public double hFOV = 54.5;
        public double vFOVYInt = 50, vFOVSlope = 0;
        public double cameraHeight = 11.9685; // equal to .3039999 meters (this is what is set in limelight hardware for localization)
    }
    public static class DrawingParams {
        public double FOVDist = 24;
    }
    public static HardwareParams hardwareParams = new HardwareParams();
    public static DrawingParams drawingParams = new DrawingParams();

    // i should tune the camera so that it gives me the turret center position
    public final Limelight3A limelight;
    public static int APRIL_TAG_PIPELINE = 3, CLASSIFIER_PIPELINE = 1, BALL_DETECTION_PIPELINE = 2;
    public static int startingPipeline = APRIL_TAG_PIPELINE;
    private int pipeline;
    public final LimelightLocalization localization; // april tag localization
    public final LimelightClassifier classifier; // counts # balls in classifier
    public final LimelightBallDetection ballDetection; // detects balls in loading zone

    // classifier detection data
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        localization = new LimelightLocalization(robot, limelight);
        classifier = new LimelightClassifier(robot, limelight);
        ballDetection = new LimelightBallDetection(robot, limelight);

        switchPipeline(startingPipeline);
        limelight.start();
    }
    public void takeSnapshot(String fileName) {
        limelight.captureSnapshot(fileName);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("LIMELIGHT");
        telemetry.addData("pipeline type (from java)", pipeline);
        telemetry.addData("limelight is running", limelight.isRunning());
        telemetry.addData("limelight is connected", limelight.isConnected());
        telemetry.addData("connection info", limelight.getConnectionInfo());
        telemetry.addLine();

        switch (pipeline) {
            case 3:
                localization.updateTelemetry(telemetry);
                break;
            case 1:
                classifier.updateTelemetry(telemetry);
                break;
            case 2:
                ballDetection.updateTelemetry(telemetry);
                break;
        }
    }
    @Override
    public void update() {
        switch (pipeline) {
            case 3:
                localization.update();
                break;
            case 1:
                classifier.update();
                break;
            case 2:
                ballDetection.update();
                break;
        }
    }

    public void switchPipeline(int num) {
        pipeline = num;
        limelight.pipelineSwitch(num);
    }

    public void addLimelightInfo(Canvas fieldOverlay) {
        switch (pipeline) {
            case 3:
                localization.addLocalizationInfo(fieldOverlay);
                break;
            case 1:
                classifier.addClassifierInfo(fieldOverlay);
                break;
            case 2:
                ballDetection.addBallInfo(fieldOverlay);
                break;
        }

        if (drawingParams.FOVDist > 0) {
            Pose2d cameraPose = getLimelightPose(robot.shootingSystem.turretPose);
            Vector2d cameraPos = cameraPose.position;
            double maxAngleRad = cameraPose.heading.toDouble() + Math.toRadians(hardwareParams.hFOV * 0.5);
            double minAngleRad = cameraPose.heading.toDouble() - Math.toRadians(hardwareParams.hFOV * 0.5);
            Vector2d maxPoint = cameraPose.position.plus(new Vector2d(
                    drawingParams.FOVDist * Math.cos(maxAngleRad),
                    drawingParams.FOVDist * Math.sin(maxAngleRad)
            ));
            Vector2d minPoint = cameraPose.position.plus(new Vector2d(
                    drawingParams.FOVDist * Math.cos(minAngleRad),
                    drawingParams.FOVDist * Math.sin(minAngleRad)
            ));
            fieldOverlay.setStroke("black");
            fieldOverlay.strokeLine(cameraPos.x, cameraPos.y, maxPoint.x, maxPoint.y);
            fieldOverlay.strokeLine(cameraPos.x, cameraPos.y, minPoint.x, minPoint.y);
        }
    }
    public static Pose2d getTurretPose(Pose2d cameraPose) {
        double dx = Math.cos(cameraPose.heading.toDouble()) * hardwareParams.distFromTurret;
        double dy = Math.sin(cameraPose.heading.toDouble()) * hardwareParams.distFromTurret;
        return new Pose2d(cameraPose.position.x - dx, cameraPose.position.y - dy, cameraPose.heading.toDouble());
    }
    public static Pose2d getLimelightPose(Pose2d turretPose) {
        double angle = turretPose.heading.toDouble();
        double dx = Math.cos(angle) * hardwareParams.distFromTurret;
        double dy = Math.sin(angle) * hardwareParams.distFromTurret;
        return new Pose2d(turretPose.position.x + dx, turretPose.position.y + dy, angle);
    }


    // gets the horizontal angle of a pixel in degrees
    public static double pixelXToTx(double pixelX) {
        // focal length in pixels = resolution * 0.5 / tan(horizontal fov * 0.5)
        double focalLengthXPixels = hardwareParams.resolutionWidth * 0.5 / Math.tan(Math.toRadians(hardwareParams.hFOV * 0.5));
        return Math.toDegrees(Math.atan((pixelX - hardwareParams.resolutionWidth * 0.5) / focalLengthXPixels));
    }
    // gets the vertical angle of a pixel in degrees
    public static double pixelYToTy(double pixelY) {
        double vFOV = pixelY * hardwareParams.vFOVSlope + hardwareParams.vFOVYInt;
        double focalLengthYPixels = hardwareParams.resolutionHeight * 0.5 / Math.tan(Math.toRadians(vFOV * 0.5));
        double ty = Math.toDegrees(Math.atan((hardwareParams.resolutionHeight * 0.5 - pixelY) / focalLengthYPixels));
        return ty + hardwareParams.verticalAngleOffset;
    }
    public static Vector2d calculateBallFieldPosition(Pose2d cameraPose, double tx, double ty) {
        double height = hardwareParams.cameraHeight - 2.5;

        double axialOffset = height / Math.tan(Math.toRadians(-ty));
        axialOffset += hardwareParams.axialDistanceOffset;
        double lateralOffset = Math.hypot(axialOffset, height) * Math.tan(Math.toRadians(tx));

        Vector2d relativeOffset = new Vector2d(axialOffset, -lateralOffset);
        return GeometryUtils.rotateVector(relativeOffset, cameraPose.heading.toDouble()).plus(cameraPose.position);
    }
}