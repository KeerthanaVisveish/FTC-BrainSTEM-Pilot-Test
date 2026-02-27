package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.video.KalmanFilter;

import java.util.ArrayList;
import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static boolean usePositionDerivedVelocity = true;
    public static class PosePredictParams {
        public int numPrevVelocitiesToTrack = 4;
        public double maxLinearSpeedPerSecond = 10, maxHeadingDegSpeedPerSecond = 20;
        public boolean clampSpeeds = false;
        public double velocityDamping = 0.75;
        public double accelTau = 0.1;
    }
    public static class Params {
        public double parYTicks = -0.946; // y position of the parallel encoder (in inches)
        public double perpXTicks = -7.328; // x position of the perpendicular encoder (in inches)
    }

    public static Params PARAMS = new Params();
    public static PosePredictParams posePredictParams = new PosePredictParams();


    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    // previousVelocities[0] = most recent
    // previousAccelerations[0] is most recent, calculated with previousVelocities[0] and previousVelocities[1]
    public final ArrayList<OdoInfo> previousVelocities, previousAccelerations;
    private long lastUpdateTimeNano;
    public Pose2d lastPose;
    public OdoInfo filteredAccel;
    public double dt;
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

        txWorldPinpoint = initialPose;
        lastPose = initialPose;

        lastUpdateTimeNano = 0;
        previousVelocities = new ArrayList<>();
        previousAccelerations = new ArrayList<>();
        filteredAccel = new OdoInfo(0, 0, 0);
    }

    @Override
    public void setPose(Pose2d pose) {
        driver.resetPosAndIMU();
        txPinpointRobot = new Pose2d(0, 0, 0);
//        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
        txWorldPinpoint = pose;
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }


    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            Pose2d prevPose = lastPose;
            lastPose = getPose();
            OdoInfo positionDerivedVelocity = new OdoInfo((lastPose.position.x - prevPose.position.x) / dt, (lastPose.position.y - prevPose.position.y) / dt, (lastPose.heading.toDouble() - prevPose.heading.toDouble()) / dt);
            
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            double velX, velY, velHeadingRad;
            if(usePositionDerivedVelocity) {
                velX = positionDerivedVelocity.x;
                velY = positionDerivedVelocity.y;
                velHeadingRad = positionDerivedVelocity.headingRad;
            }
            else {
                velX = driver.getVelX(DistanceUnit.INCH);
                velY = driver.getVelY(DistanceUnit.INCH);
                velHeadingRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            }
            Vector2d worldVelocity = new Vector2d(velX, velY);
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            updatePreviousVelocitiesAndAccelerations(velX, velY, velHeadingRad);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    private void updatePreviousVelocitiesAndAccelerations(double vx, double vy, double vh) {
        dt = (System.nanoTime() - lastUpdateTimeNano) * 1.0 * 1e-9; // delta time is in seconds

        // remove oldest velocity
        if (previousVelocities.size() >= posePredictParams.numPrevVelocitiesToTrack)
            previousVelocities.remove(previousVelocities.size() - 1);

        previousVelocities.add(0, new OdoInfo(vx, vy, vh));

        // remove oldest acceleration
        if (previousAccelerations.size() >= posePredictParams.numPrevVelocitiesToTrack - 1)
            previousAccelerations.remove(previousAccelerations.size() - 1);

        // update acceleration
        // acceleration = current velocity - old velocity
        // not dividing by time b/c velocity is already in the desired "time" unit - change from last frame to this frame
        // so this acceleration actually represents the change in velocity from last frame to this frame
        if (previousVelocities.size() > 1) {
            previousAccelerations.add(0, new OdoInfo(
                    (previousVelocities.get(0).x - previousVelocities.get(1).x) / dt,
                    (previousVelocities.get(0).y - previousVelocities.get(1).y) / dt,
                    (previousVelocities.get(0).headingRad - previousVelocities.get(1).headingRad) / dt
            ));

            filteredAccel = getFilteredAccel(dt, previousAccelerations.get(0));
        }
        lastUpdateTimeNano = System.nanoTime();
    }
    private OdoInfo getFilteredAccel(double dt, OdoInfo raw) {
        double a = Math.exp(-dt / posePredictParams.accelTau);
        double nx = filteredAccel.x * a + raw.x * (1-a);
        double ny = filteredAccel.y * a + raw.y * (1-a);
        double nh = filteredAccel.headingRad * a + raw.headingRad * (1-a);
        return new OdoInfo(nx, ny, nh);
    }
    // predicts on most recent velocity
    public Pose2d getNextPoseSimple(double time) {
        if (previousVelocities.isEmpty())
            return getPose();
        double velocityMultiplier = time * posePredictParams.velocityDamping;
        Pose2d pose = getPose();
        return new Pose2d(
                pose.position.x + previousVelocities.get(0).x * velocityMultiplier,
                pose.position.y + previousVelocities.get(0).y * velocityMultiplier,
                pose.heading.toDouble() + previousVelocities.get(0).headingRad * velocityMultiplier
        );
    }
    public OdoInfo getMostRecentVelocity() {
        if (previousVelocities.isEmpty())
            return new OdoInfo(0, 0, 0);
        return previousVelocities.get(0).clone();
    }

}
