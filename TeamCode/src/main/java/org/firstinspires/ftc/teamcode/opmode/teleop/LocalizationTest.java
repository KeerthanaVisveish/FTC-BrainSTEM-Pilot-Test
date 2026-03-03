package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

import java.util.ArrayList;

@Config
@TeleOp(name="localization test", group="Competition")
public class LocalizationTest extends LinearOpMode {
    public static double startX = 0, startY = 0, startA = 0;
    public static double mt2HeadingOffset = 0;
    public static int numPrevPosesToAvg = 10;
    public static boolean drawRobotPoses = true, drawTurretPoses = true, drawCameraPose = true, drawFilteredPoses = true;
    public static boolean drawRobotVelocity = true;
    public static boolean useMegaTag2 = true;
    public static int ftcDashboardFPS = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startA));
        LED led = new LED(hardwareMap, telemetry, null);

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        FtcDashboard.getInstance().startCameraStream(limelight3A, ftcDashboardFPS);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArrayList<Pose2d> prevLlCameraPoses = new ArrayList<>();

        double maxXError = 0;
        double maxYError = 0;
        double maxTranslationalError = 0;
        double maxHeadingErrorRad = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                maxXError = 0;
                maxYError = 0;
                maxTranslationalError = 0;
                maxHeadingErrorRad = 0;
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();


            Pose2d pinpointRobotPose = drive.localizer.getPose();
            double relativeTurretAngle = Turret.getTurretRelativeAngleRad(motor.getCurrentPosition());

            Pose2d pinpointTurretPose = ShootingMath.getTurretPose(pinpointRobotPose, relativeTurretAngle);
            Pose2d pinpointCameraPose = Limelight.getLimelightPose(pinpointTurretPose);

            if (useMegaTag2) {
                double heading = Math.toDegrees(pinpointTurretPose.heading.toDouble()) + mt2HeadingOffset;
                limelight3A.updateRobotOrientation(heading);
            }

            LLResult result = limelight3A.getLatestResult();
            Pose2d llCameraPose = new Pose2d(0, 0, 0);
            Pose2d llTurretPose = new Pose2d(0, 0, 0);
            Pose2d llRobotPose = new Pose2d(0, 0, 0);
            Pose2d filteredLlCameraPose = new Pose2d(0, 0, 0);
            Pose2d filteredLlTurretPose = new Pose2d(0, 0, 0);
            Pose2d filteredLlRobotPose = new Pose2d(0, 0, 0);

            if (result != null && result.isValid()) {
                Pose3D cameraPose3D = useMegaTag2 ? result.getBotpose_MT2() : result.getBotpose();
                Position cameraPos = cameraPose3D.getPosition().toUnit(DistanceUnit.INCH);
                double cameraHeading = cameraPose3D.getOrientation().getYaw(AngleUnit.RADIANS);
                if (useMegaTag2)
                    cameraHeading -= mt2HeadingOffset;

                llCameraPose = new Pose2d(cameraPos.x, cameraPos.y, cameraHeading);
                if (llCameraPose.position.x != 0 || llCameraPose.position.y != 0 || llCameraPose.heading.toDouble() != 0) {
                    llTurretPose = Limelight.getTurretPose(llCameraPose);
                    llRobotPose = ShootingMath.getRobotPose(llTurretPose, relativeTurretAngle);
                }
            }

            boolean invalidPose = llCameraPose.position.x == 0 && llCameraPose.position.y == 0 && llCameraPose.heading.toDouble() == 0;
            if (invalidPose)
                led.setLed(LED.red);
            else {
                led.setLed(LED.green);
                prevLlCameraPoses.add(llCameraPose);
                if (prevLlCameraPoses.size() > numPrevPosesToAvg)
                    prevLlCameraPoses.remove(0);
                double x = 0, y = 0, hRad = 0;
                for (Pose2d llCamera : prevLlCameraPoses) {
                    x += llCamera.position.x;
                    y += llCamera.position.y;
                    hRad += llCamera.heading.toDouble();
                }
                filteredLlCameraPose = new Pose2d(x / prevLlCameraPoses.size(), y / prevLlCameraPoses.size(), hRad / prevLlCameraPoses.size());
                filteredLlTurretPose = Limelight.getTurretPose(filteredLlCameraPose);
                filteredLlRobotPose = ShootingMath.getRobotPose(filteredLlTurretPose, relativeTurretAngle);
            }

            if (gamepad1.y)
                drive.localizer.setPose(filteredLlRobotPose);

            double xError = pinpointRobotPose.position.x - llRobotPose.position.x;
            double yError = pinpointRobotPose.position.y - llRobotPose.position.y;
            double translError = Math.hypot(xError, yError);
            double headingErrorRad = Math.abs(pinpointRobotPose.heading.toDouble() - llRobotPose.heading.toDouble());

            maxXError = Math.max(maxXError, xError);
            maxYError = Math.max(maxYError, yError);
            maxTranslationalError = Math.max(maxTranslationalError, translError);
            maxHeadingErrorRad = Math.max(maxHeadingErrorRad, headingErrorRad);

            telemetry.addData("reset errors", "gamepad1.a");
            telemetry.addData("reset odo pose to ll", "gamepad1.y");
            telemetry.addLine("Pinpoint======================");
            telemetry.addData("pinpoint robot pose", MathUtils.formatPose3(pinpointRobotPose));
            telemetry.addData("pinpoint turret pose", MathUtils.formatPose3(pinpointTurretPose));
            telemetry.addData("pinpoint camera pose", MathUtils.formatPose3(pinpointCameraPose));
            double velX = drive.pinpoint().driver.getVelX(DistanceUnit.INCH);
            double velY = drive.pinpoint().driver.getVelX(DistanceUnit.INCH);
            Vector2d rawVel = new Vector2d(velX, velY);
            OdoInfo fieldVelocity = drive.pinpoint().getVelocity();
            telemetry.addData("pinpoint raw velocity", MathUtils.formatVec3(rawVel));
            telemetry.addData("pinpoint field velocity", fieldVelocity.toString(3));
            telemetry.addLine();

            telemetry.addLine("Limelight======================");
            telemetry.addData("ll robot pose", MathUtils.formatPose3(llRobotPose));
            telemetry.addData("ll turret pose", MathUtils.formatPose3(llTurretPose));
            telemetry.addData("ll camera pose", MathUtils.formatPose3(llCameraPose));
            telemetry.addData("ll camera pose filtered", MathUtils.formatPose3(filteredLlCameraPose));
            telemetry.addLine();

            telemetry.addLine("Errors==========================");
            telemetry.addData("max x error", MathUtils.format3(maxXError));
            telemetry.addData("max y error", MathUtils.format3(maxYError));
            telemetry.addData("max translational error", MathUtils.format3(maxTranslationalError));
            telemetry.addData("max heading error deg", MathUtils.format3(Math.toDegrees(maxHeadingErrorRad)));


            telemetry.update();

            if (drawRobotPoses || drawTurretPoses) {
                TelemetryPacket packet = new TelemetryPacket();
                if (drawRobotPoses) {
                    packet.fieldOverlay().setStroke("green");
                    Drawing.drawRobot(packet.fieldOverlay(), pinpointRobotPose);
                    packet.fieldOverlay().setStroke("gray");
                    Drawing.drawRobot(packet.fieldOverlay(), llRobotPose);
                }
                if (drawTurretPoses) {
                    packet.fieldOverlay().setStroke("green");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), pinpointTurretPose, 3);
                    packet.fieldOverlay().setStroke("gray");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), llTurretPose, 3);
                }
                if (drawCameraPose) {
                    packet.fieldOverlay().setStroke("black");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), pinpointCameraPose, 2);
                    Drawing.drawRobotSimple(packet.fieldOverlay(), llCameraPose, 2);
                }
                if (drawFilteredPoses) {
                    packet.fieldOverlay().setStroke("blue");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), filteredLlCameraPose, 2);
                    Drawing.drawRobotSimple(packet.fieldOverlay(), filteredLlTurretPose, 3);
                    Drawing.drawRobot(packet.fieldOverlay(), filteredLlRobotPose);
                }
                if (drawRobotVelocity) {
                    packet.fieldOverlay().setStroke("black");
                    Vector2d pos = pinpointRobotPose.position;
                    Vector2d vel = fieldVelocity.pos();
                    packet.fieldOverlay().strokeLine(pos.x, pos.y, pos.x + vel.x, pos.y + vel.y);
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }
}
