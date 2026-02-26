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
    public static class PosePredictParams {
        public int numPrevVelocitiesToTrack = 4;
        public double maxLinearSpeedPerSecond = 10, maxHeadingDegSpeedPerSecond = 20;
        public boolean clampSpeeds = false;
        public double velocityDamping = 0.75;
        public double accelTau = 0.1;
    }
    public static class KalmanParams {
        public double velXYProcessNoise = 0.1, velHeadingProcessNoise = 0.05;
        public double accelXYProcessNoise = 0.2, accelHeadingProcessNoise = 0.1;
        public double velXYMeasurementNoise = 0.1, velHeadingMeasurementNoise = 0.05;

        public double accelTau = .1;


    }
    public static class Params {
        public double parYTicks = -0.946; // y position of the parallel encoder (in inches)
        public double perpXTicks = -7.328; // x position of the perpendicular encoder (in inches)
    }

    public static Params PARAMS = new Params();
    public static PosePredictParams posePredictParams = new PosePredictParams();
    public static KalmanParams kalmanParams = new KalmanParams();


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
    private OdoInfo nextVelAdvanced;
    public double dt;

    public KalmanFilter kalmanFilter;
    private Mat A;
    private int stateSize, type;
    public OdoInfo kalmanVelPrediction, kalmanVelEstimation;
    public OdoInfo kalmanAccelPrediction, kalmanAccelEstimation;
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
        nextVelAdvanced = new OdoInfo(0, 0, 0);

        stateSize = 6;
        int numMeasurements = 3;
        type = CvType.CV_32F;
        kalmanFilter = new KalmanFilter(stateSize, numMeasurements, 0, type);
        Mat H = Mat.eye(numMeasurements, stateSize, type);
        Mat Q = Mat.zeros(stateSize, stateSize, type);
        Q.put(0, 0, kalmanParams.velXYProcessNoise);
        Q.put(1, 1, kalmanParams.velXYProcessNoise);
        Q.put(2, 2, kalmanParams.velHeadingProcessNoise);
        Q.put(3, 3, kalmanParams.accelXYProcessNoise);
        Q.put(4, 4, kalmanParams.accelXYProcessNoise);
        Q.put(3, 5, kalmanParams.accelHeadingProcessNoise);
        Mat R = Mat.zeros(numMeasurements, numMeasurements, type);
        R.put(0, 0, kalmanParams.velXYMeasurementNoise);
        R.put(1, 1, kalmanParams.velXYMeasurementNoise);
        R.put(2, 2, kalmanParams.velHeadingMeasurementNoise);

        kalmanFilter.set_measurementMatrix(H);
        kalmanFilter.set_processNoiseCov(Q);
        kalmanFilter.set_measurementNoiseCov(R);
    }

    @Override
    public void setPose(Pose2d pose) {
        driver.resetPosAndIMU();
        txPinpointRobot = new Pose2d(0, 0, 0);
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }


    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            lastPose = getPose();
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            double velX = driver.getVelX(DistanceUnit.INCH), velY = driver.getVelY(DistanceUnit.INCH), velHeadingRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            Vector2d worldVelocity = new Vector2d(velX, velY);
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            updatePreviousVelocitiesAndAccelerations(velX, velY, velHeadingRad);
            if(!previousVelocities.isEmpty()) {
                A = Mat.eye(stateSize, stateSize, type);
                A.put(0, 3, dt);
                A.put(1, 4, dt);
                A.put(2, 5, dt);
                kalmanFilter.set_transitionMatrix(A);
                Mat prediction = kalmanFilter.predict();
                kalmanVelPrediction = new OdoInfo(prediction.get(0, 0)[0], prediction.get(1, 0)[0], prediction.get(2, 0)[0]);
                kalmanAccelPrediction = new OdoInfo(prediction.get(3, 0)[0], prediction.get(4, 0)[0], prediction.get(5, 0)[0]);
                Mat measurement = new Mat(3, 1, CvType.CV_32F);
                measurement.put(0, 0, previousVelocities.get(0).x);
                measurement.put(1, 0, previousVelocities.get(0).y);
                measurement.put(2, 0, previousVelocities.get(0).headingRad);
                Mat estimated = kalmanFilter.correct(measurement);
                kalmanVelEstimation = new OdoInfo(estimated.get(0, 0)[0], estimated.get(1, 0)[0], estimated.get(2, 0)[0]);
                kalmanAccelEstimation = new OdoInfo(estimated.get(3, 0)[0], estimated.get(4, 0)[0], estimated.get(5, 0)[0]);
            }

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    private void updatePreviousVelocitiesAndAccelerations(double vx, double vy, double vh) {
        dt = (System.nanoTime() - lastUpdateTimeNano) * 1.0 * 1e-9; // delta time is in seconds

        // remove oldest velocity
        if (previousVelocities.size() >= posePredictParams.numPrevVelocitiesToTrack)
            previousVelocities.remove(previousVelocities.size() - 1);

        // add most recent velocity (and clamp if necessary)
        if (posePredictParams.clampSpeeds) {
            double linearSpeedPerSecond = Math.sqrt(vx * vx + vy * vy);
            if (linearSpeedPerSecond > posePredictParams.maxLinearSpeedPerSecond) {
                vx *= posePredictParams.maxLinearSpeedPerSecond / linearSpeedPerSecond;
                vy *= posePredictParams.maxLinearSpeedPerSecond / linearSpeedPerSecond;
            }
            if (Math.abs(vh) > Math.toRadians(posePredictParams.maxHeadingDegSpeedPerSecond))
                vh = Math.signum(vh) * Math.toRadians(posePredictParams.maxHeadingDegSpeedPerSecond);
        }
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
    public double[] getPreviousXAccels() {
        double[] xAccels = new double[previousAccelerations.size()];
        for(int i = 0; i < xAccels.length; i++)
            xAccels[i] = previousAccelerations.get(i).x;
        return xAccels;
    }
    public double[] getPreviousYAccels() {
        double[] yAccels = new double[previousAccelerations.size()];
        for(int i = 0; i < yAccels.length; i++)
            yAccels[i] = previousAccelerations.get(i).y;
        return yAccels;
    }
    public double[] getPreviousHeadingAccels() {
        double[] headingAccels = new double[previousAccelerations.size()];
        for(int i = 0; i < headingAccels.length; i++)
            headingAccels[i] = previousAccelerations.get(i).headingRad;
        return headingAccels;
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
    public OdoInfo getNextVelAdvanced(){
        OdoInfo vel = getMostRecentVelocity();
        return new OdoInfo(vel.x + filteredAccel.x * dt, vel.y + filteredAccel.y * dt, vel.headingRad + filteredAccel.headingRad * dt);
    }
    public OdoInfo getMostRecentVelocity() {
        if (previousVelocities.isEmpty())
            return new OdoInfo(0, 0, 0);
        return previousVelocities.get(0).clone();
    }
    public OdoInfo getMostRecentAcceleration() {
        if(previousAccelerations.isEmpty())
            return new OdoInfo(0, 0, 0);
        return previousAccelerations.get(0).clone();
    }
}
