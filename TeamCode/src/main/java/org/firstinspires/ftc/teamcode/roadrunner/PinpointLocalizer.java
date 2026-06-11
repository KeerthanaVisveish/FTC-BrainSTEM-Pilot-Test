package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static boolean usePositionDerivedVelocity = false;

    public static class Params {
        public double parYTicks = -0.946; // y position of the parallel encoder (in inches)
        public double perpXTicks = -7.328; // x position of the perpendicular encoder (in inches)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpointPose; // pose that converts txPinpointRobot into field pose
    private Pose2d txPinpointRobotPose = new Pose2d(0, 0, 0); // raw pose data from pinpoint
    private OdoInfo velocity = new OdoInfo();
    public Pose2d lastPose;
    public OdoInfo rawAccel;
    private final ElapsedTime dtTimer;
    private boolean successfulHardwareRead = false;
    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        driver.setEncoderResolution(20, DistanceUnit.MM); //1.0 / mmPerTick (FIX VALUE)
        driver.setOffsets(
                DistanceUnit.MM.fromInches(PARAMS.parYTicks),
                DistanceUnit.MM.fromInches(PARAMS.perpXTicks),
                DistanceUnit.MM
        );

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpointPose = initialPose;
        lastPose = initialPose;

        dtTimer = new ElapsedTime();
        dtTimer.reset();
    }

    @Override
    public void setPose(Pose2d pose) {
//        txPinpointRobotPose = new Pose2d(0, 0, 0);
        txWorldPinpointPose = pose.times(txPinpointRobotPose.inverse());
//        txWorldPinpointPose = pose;
        lastPose = pose;
    }

    // inches
    @Override
    public Pose2d getPose() {
        return txWorldPinpointPose.times(txPinpointRobotPose);
    }
    // inches/sec, rad/sec
    public OdoInfo getVelocity() {
        return velocity;
    }
    public OdoInfo getRawAccel() {
        return rawAccel;
    }

    @Override
    public PoseVelocity2d update() {
        double dt = dtTimer.seconds();
        dtTimer.reset();

        driver.update();

        successfulHardwareRead = Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY;
        if (successfulHardwareRead) {
            Pose2d prevPose = lastPose;
            lastPose = getPose();
            OdoInfo positionDerivedVelocity = new OdoInfo((lastPose.position.x - prevPose.position.x) / dt, (lastPose.position.y - prevPose.position.y) / dt, (lastPose.heading.toDouble() - prevPose.heading.toDouble()) / dt);

            txPinpointRobotPose = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            double velX, velY;
            if(usePositionDerivedVelocity) {
                velX = positionDerivedVelocity.x;
                velY = positionDerivedVelocity.y;
            }
            else {
                velX = driver.getVelX(DistanceUnit.INCH);
                velY = driver.getVelY(DistanceUnit.INCH);
            }
            Vector2d pinpointTranslationalVelocity = new Vector2d(velX, velY);
            Vector2d robotTranslationalVelocity = Rotation2d.fromDouble(-txPinpointRobotPose.heading.log()).times(pinpointTranslationalVelocity);
            double angularVelocity = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

            // custom velocity calculation
            Vector2d worldTranslationalVelocity = GeometryUtils.rotateVector(pinpointTranslationalVelocity, txWorldPinpointPose.heading.toDouble());

            OdoInfo prevVelocity = velocity;
            velocity = new OdoInfo(worldTranslationalVelocity.x, worldTranslationalVelocity.y, angularVelocity);

            rawAccel = new OdoInfo(velocity.x - prevVelocity.x, velocity.y - prevVelocity.y, velocity.headingRad - prevVelocity.headingRad);
            rawAccel = new OdoInfo(rawAccel.x / dt, rawAccel.y / dt, rawAccel.headingRad / dt);
            return new PoseVelocity2d(robotTranslationalVelocity, angularVelocity);
        }
        rawAccel = new OdoInfo(0, 0, 0);
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    // predicts on most recent velocity
    public Pose2d getNextPoseSimple(double time) {
        Pose2d pose = getPose();
        OdoInfo velocity = getVelocity();
        return new Pose2d(
                pose.position.x + velocity.x * time,
                pose.position.y + velocity.y * time,
                pose.heading.toDouble() + velocity.headingRad * time
        );
    }

    public void printInfo(Telemetry telemetry) {
        if (!successfulHardwareRead)
            telemetry.addLine("PINPOINT LAST READ WAS NOT SUCCESSFUL=================");
        else
            telemetry.addLine("pinpoint successful last read");
    }
}
