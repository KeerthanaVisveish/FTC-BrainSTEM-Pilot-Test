package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMathOld;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightLocalization extends LLParent {
    public enum LocalizationType {
        CONTINUOUS,
        ON_COMMAND
    }
    public enum LocalizationState {
        OFF,
        SCANNING_MOTIF,
        PASSIVE_READING,
        UPDATING_POSE
    }

    public static class Params {
        public double limelightTrust = .5;
        public boolean showValidLocalizationZones = true;
        public double[] nearZoneLocalizeCircle = { -24, 0, 48 };
        public double maxUpdateTranslationalVel = 2, maxUpdateHeadingDegVel = 2; // inches and degrees
        public int maxUpdateTurretVelTicksPerSec = 1;
        public boolean allowUpdateAnywhereForFirst = false;
        public double ableToUpdateConfirmationTime = 0.2;
        public int numPrevFramesToAvg = 4;
        public double minTimeBetweenUpdates = 5;
        public boolean useMT2 = true;
        public double jankPerpOffset = 0, jankParallelOffset = 0;
        public int numPrevPosesToPrint = 0;
        public LocalizationState offLocalizationState = LocalizationState.PASSIVE_READING;
    }

    public static LocalizationType localizationType = LocalizationType.CONTINUOUS;
    public static Params params = new Params();
    public Pose2d cameraPose, robotPose, rawCameraPose, rawRobotPose;
    private LLResult aprilTagResult;
    private Pose2d lastAvgTurretPose;
    private final ArrayList<Pose3D> lastCameraPoses;
    public double maxTranslationalVariance, maxHeadingVarianceDeg; // how much the limelight jitters
    public double maxTranslationalError, maxHeadingErrorDeg; // how much the limelight differs from pinpoint
    private boolean drivetrainGoodForUpdate, turretGoodForUpdate, inLocalizationZone;
    public boolean successfullyFoundPose;

    private boolean foundMotif;
    private int targetGreenPos;

    private LocalizationState state, prevState;
    private final ElapsedTime stateTimer;
    private int numSetPoses = 0;
    private double lastUpdatePoseTimeMs = 0;
    public boolean manualPoseUpdate;
    public List<LLResultTypes.FiducialResult> visibleTagInfo;
    private final ElapsedTime ableToUpdateTimer;
    private int numFramesInState;

    public LimelightLocalization(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);

        robotPose = null;
        cameraPose = null;
        lastAvgTurretPose = new Pose2d(0, 0, 0);
        lastCameraPoses = new ArrayList<>();
        stateTimer = new ElapsedTime();
        prevState = LocalizationState.OFF;
        setState(LocalizationState.OFF);
        successfullyFoundPose = false;
        drivetrainGoodForUpdate = false;
        turretGoodForUpdate = false;
        manualPoseUpdate = false;
        maxTranslationalVariance = 0;
        maxHeadingVarianceDeg = 0;
        maxHeadingErrorDeg = 0;
        maxTranslationalError = 0;
        numSetPoses = 0;
        visibleTagInfo = new ArrayList<>();
        ableToUpdateTimer = new ElapsedTime();
    }

    public LocalizationState getState() {
        return state;
    }
    public LocalizationState getPrevState() {
        return prevState;
    }
    public double getStateTime() {
        return stateTimer.seconds();
    }
    public void setState(LocalizationState state) {
        if (state == this.state)
            return;
        stateTimer.reset();
        numFramesInState = 0;
        prevState = this.state;
        this.state = state;
        if (state == LocalizationState.UPDATING_POSE) {
            lastCameraPoses.clear();
            successfullyFoundPose = false;
        }
    }

    public void update() {
        numFramesInState++;
        drivetrainGoodForUpdate = canUpdateDrivetrainReliably();
        turretGoodForUpdate = canUpdateTurretReliably();
        inLocalizationZone = isInLocalizationZone();
        if (!inLocalizationZone && state == LocalizationState.UPDATING_POSE)
            setState(params.offLocalizationState);

        if (!drivetrainGoodForUpdate || !turretGoodForUpdate || !inLocalizationZone) {
            // want to update again immediately if current update is interrupted
            if (state == LocalizationState.UPDATING_POSE)
                lastUpdatePoseTimeMs = -1;

            setState(params.offLocalizationState);
            lastCameraPoses.clear();
            ableToUpdateTimer.reset();
        }

        // only want to update again if robot moves, not if turret moves
        if (!drivetrainGoodForUpdate)
            successfullyFoundPose = false;

        double curTimeMs = System.currentTimeMillis();
        double timeSinceUpdate = (curTimeMs - lastUpdatePoseTimeMs) * 0.001;

        boolean canUpdate = drivetrainGoodForUpdate && turretGoodForUpdate && inLocalizationZone &&
                ableToUpdateTimer.seconds() >= params.ableToUpdateConfirmationTime &&
                !successfullyFoundPose && localizationType == LocalizationType.CONTINUOUS;

        // if the code has reached this ine, everything is ready to update the pose
        if (canUpdate && timeSinceUpdate >= params.minTimeBetweenUpdates && state != LocalizationState.UPDATING_POSE) {
            manualPoseUpdate = false;
            setState(LocalizationState.UPDATING_POSE);
            lastUpdatePoseTimeMs = System.currentTimeMillis();
        }

        switch (state) {
            case OFF:
                break;
            case SCANNING_MOTIF:
                updateMotifData();
                break;
            case PASSIVE_READING:
                updatePoseFromCamera();
                if (robotPose == null || !aprilTagResult.isValid())
                    break;
                updateMaxErrors(lastAvgTurretPose, cameraPose);

                break;
            case UPDATING_POSE:
                if (numFramesInState > params.numPrevFramesToAvg * 2) {
                    setState(params.offLocalizationState);
                    break;
                }
                updatePoseFromCamera();

                successfullyFoundPose = robotPose != null;
                if (!successfullyFoundPose)
                    break;

                updateMaxErrors(lastAvgTurretPose, cameraPose);

                Pose2d pinpointPose = robot.drive.localizer.getPose();
                Vector2d avgPos = pinpointPose.position.times(1-params.limelightTrust).plus(robotPose.position.times(params.limelightTrust));
                double avgHeading = pinpointPose.heading.toDouble() * (1-params.limelightTrust) + robotPose.heading.toDouble() * params.limelightTrust;
                robot.drive.localizer.setPose(new Pose2d(avgPos, avgHeading));
                numSetPoses++;
                setState(params.offLocalizationState);
                break;
        }
    }
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("   localization state", state);
        if(aprilTagResult != null) {
            telemetry.addData("   isValid", aprilTagResult.isValid());
            telemetry.addData("   bot pose is null", aprilTagResult.getBotpose() == null);
            telemetry.addData("   camera pose", MathUtils.formatPose2(cameraPose));
            telemetry.addData("   robot pose", MathUtils.formatPose2(robotPose));
            telemetry.addLine();
            telemetry.addData("   max translational variance", MathUtils.format3(maxTranslationalVariance));
            telemetry.addData("   max heading variance", MathUtils.format3(maxHeadingVarianceDeg));
            telemetry.addData("   max translational error", MathUtils.format3(maxTranslationalError));
            telemetry.addData("   max heading error", MathUtils.format3(maxHeadingErrorDeg));
            telemetry.addLine();
            telemetry.addData("   drivetrain good for update", drivetrainGoodForUpdate);
            telemetry.addData("   turret good for update", turretGoodForUpdate);
            telemetry.addData("   in localization zone", inLocalizationZone);
            telemetry.addData("   time since last update", MathUtils.format3((System.currentTimeMillis() - lastUpdatePoseTimeMs) * 0.001));
            telemetry.addData("   successfully found pose", successfullyFoundPose);
            telemetry.addLine();
            telemetry.addData("   num visible tags", visibleTagInfo.size());
            StringBuilder tagIDs = new StringBuilder();
            for (LLResultTypes.FiducialResult result : visibleTagInfo)
                tagIDs.append(result.getFiducialId()).append(" ");
            telemetry.addData("   tag IDs", tagIDs);

            for (int i = 0; i<Math.min(params.numPrevPosesToPrint, lastCameraPoses.size()); i++)
                telemetry.addData("   last pose " + (i + 1),
                        MathUtils.format2(lastCameraPoses.get(i).getPosition().x) + " " +
                                MathUtils.format2(lastCameraPoses.get(i).getPosition().y) + " " +
                                MathUtils.format2(lastCameraPoses.get(i).getOrientation().getYaw(AngleUnit.DEGREES)));
        }
        else
            telemetry.addLine("   result is null");
    }
    private void updateMotifData() {
        aprilTagResult = limelight.getLatestResult();
        visibleTagInfo.clear();

        if(aprilTagResult == null || !aprilTagResult.isValid())
            return;

        visibleTagInfo = aprilTagResult.getFiducialResults();
        foundMotif = false;
        targetGreenPos = -1;
        for(int i = 0; i < visibleTagInfo.size(); i++) {
            int id = visibleTagInfo.get(i).getFiducialId();
            if (id >= 21 && id <= 23) {
                foundMotif = true;
                targetGreenPos = id - 21;
            }
        }
    }
    public boolean foundMotif() {
        return foundMotif;
    }
    public int getTargetGreenPos() {
        return targetGreenPos;
    }
    private void updatePoseFromCamera() {
        if (cameraPose != null)
            lastAvgTurretPose = new Pose2d(cameraPose.position, cameraPose.heading);
        else
            lastAvgTurretPose = new Pose2d(0, 0, 0);

        if (params.useMT2) {
            double turretHeadingDeg = Math.toDegrees(robot.turret.currentAbsoluteAngleRad);
            limelight.updateRobotOrientation(turretHeadingDeg);
        }

        aprilTagResult = limelight.getLatestResult();
        visibleTagInfo.clear();

        if (aprilTagResult == null || !aprilTagResult.isValid())
            return;

        visibleTagInfo = aprilTagResult.getFiducialResults();
        boolean validTags = false;
        for (LLResultTypes.FiducialResult tagResult : visibleTagInfo)
            if (tagResult.getFiducialId() == 20 || tagResult.getFiducialId() == 24) {
                validTags = true;
                break;
            }
        if(!validTags) {
            setAllPosesToNull();
            setState(params.offLocalizationState);
            lastCameraPoses.clear();
            return;
        }
        Pose3D curFrameCameraPose = params.useMT2 ? aprilTagResult.getBotpose_MT2() : aprilTagResult.getBotpose();
        if (curFrameCameraPose == null) {
            setAllPosesToNull();
            setState(params.offLocalizationState);
            lastCameraPoses.clear();
            return;
        }
        Position curFrameCameraPosition = curFrameCameraPose.getPosition().toUnit(DistanceUnit.INCH);
        if (Math.abs(curFrameCameraPosition.x) < 0.00001 &&
            Math.abs(curFrameCameraPosition.y) < 0.00001 &&
            Math.abs(curFrameCameraPose.getOrientation().getYaw(AngleUnit.DEGREES)) < 0.00001) {
            setAllPosesToNull();
            setState(params.offLocalizationState);
            lastCameraPoses.clear();
            return;
        }

        rawCameraPose = new Pose2d(curFrameCameraPosition.x, curFrameCameraPosition.y, curFrameCameraPose.getOrientation().getYaw(AngleUnit.RADIANS));
        rawRobotPose = calculateRobotPose(rawCameraPose);

        lastCameraPoses.add(new Pose3D(curFrameCameraPosition, curFrameCameraPose.getOrientation()));
        if (lastCameraPoses.size() > params.numPrevFramesToAvg)
            lastCameraPoses.remove(0);
        else {
            setAllPosesToNull();
            return;
        }

        cameraPose = getAvgCameraPose(lastCameraPoses);
        double a = cameraPose.heading.toDouble();
        Vector2d parallelBasis = new Vector2d(Math.cos(a), Math.sin(a));
        Vector2d perpBasis = new Vector2d(parallelBasis.y*1, -parallelBasis.x);
        Vector2d jankOffset = parallelBasis.times(params.jankParallelOffset).plus(perpBasis.times(params.jankPerpOffset));
        cameraPose = new Pose2d(cameraPose.position.plus(jankOffset), cameraPose.heading.toDouble());
        robotPose = calculateRobotPose(cameraPose);
    }
    private void setAllPosesToNull() {
        cameraPose = null;
        robotPose = null;
        rawCameraPose = null;
        rawRobotPose = null;
    }
    private void updateMaxErrors(Pose2d lastAvgTurretPose, Pose2d curAvgTurretPose) {
        if (lastAvgTurretPose == null || curAvgTurretPose == null)
            return;
        double translationalVariance = Math.hypot(curAvgTurretPose.position.x - lastAvgTurretPose.position.x, curAvgTurretPose.position.y - lastAvgTurretPose.position.y);
        double headingVarianceDeg = Math.abs(Math.toDegrees(curAvgTurretPose.heading.toDouble() - lastAvgTurretPose.heading.toDouble()));
        maxTranslationalVariance = Math.max(maxTranslationalVariance, translationalVariance);
        maxHeadingVarianceDeg = Math.max(maxHeadingVarianceDeg, headingVarianceDeg);

        Pose2d odoPose = robot.drive.localizer.getPose();
        double translationalError = Math.hypot(curAvgTurretPose.position.x - odoPose.position.x, curAvgTurretPose.position.y - odoPose.position.y);
        double headingErrorDeg = Math.abs(Math.toDegrees(curAvgTurretPose.heading.toDouble() - odoPose.heading.toDouble()));
        maxTranslationalError = Math.max(maxTranslationalError, translationalError);
        maxHeadingErrorDeg = Math.max(maxHeadingErrorDeg, headingErrorDeg);
    }
    private Pose2d getAvgCameraPose(ArrayList<Pose3D> lastTurretPoses) {
        double x = 0, y = 0, hRad = 0;
        for (Pose3D pose : lastTurretPoses) {
            x += pose.getPosition().x;
            y += pose.getPosition().y;
            hRad += pose.getOrientation().getYaw(AngleUnit.RADIANS);
        }
        int num = lastTurretPoses.size();
        return new Pose2d(x / num, y / num, hRad / num);
    }
    private Pose2d calculateRobotPose(Pose2d cameraPose) {
        Pose2d turretPose = Limelight.getTurretPose(cameraPose);
        return ShootingMathOld.getRobotPose(turretPose, robot.turret.curRelAngleRad);
    }
    private boolean canUpdateDrivetrainReliably() {
        OdoInfo odoVel = robot.drive.pinpoint().getVelocity();
        return Math.abs(Math.toDegrees(odoVel.headingRad)) < params.maxUpdateHeadingDegVel && Math.hypot(odoVel.x, odoVel.y) < params.maxUpdateTranslationalVel;
    }
    private boolean canUpdateTurretReliably() {
        return robot.shootingSystem.getTurretVelTps() < params.maxUpdateTurretVelTicksPerSec;
    }
    private boolean isInLocalizationZone() {
        Pose2d odoPose = robot.drive.localizer.getPose();
        if (params.allowUpdateAnywhereForFirst && numSetPoses == 0)
            return true;
        return Math.hypot(odoPose.position.x - params.nearZoneLocalizeCircle[0], odoPose.position.y -  params.nearZoneLocalizeCircle[1]) < params.nearZoneLocalizeCircle[2];
    }

    public void addLocalizationInfo(Canvas fieldOverlay) {

        if (params.showValidLocalizationZones) {
            fieldOverlay.setStroke("yellow");
            fieldOverlay.strokeCircle(params.nearZoneLocalizeCircle[0], params.nearZoneLocalizeCircle[1], params.nearZoneLocalizeCircle[2]);
        }

        fieldOverlay.setStroke("blue");
        Pose2d robotPoseToDraw = robotPose == null ? new Pose2d(0, 0, 0) : new Pose2d(robotPose.position, robotPose.heading);
        Drawing.drawRobot(fieldOverlay, robotPoseToDraw);

        Pose2d turretPose = ShootingMathOld.getTurretPose(robotPoseToDraw, robot.turret.curRelAngleRad);
        Drawing.drawRobotSimple(fieldOverlay, turretPose, 3);

        Pose2d limelightPose = Limelight.getLimelightPose(turretPose);
        Drawing.drawRobotSimple(fieldOverlay, limelightPose, 2);
    }
}
