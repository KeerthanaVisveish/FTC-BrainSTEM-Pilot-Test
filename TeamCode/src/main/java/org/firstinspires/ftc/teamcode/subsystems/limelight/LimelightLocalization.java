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
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
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
        SCANNING_MOTIF,
        PASSIVE_READING,
        UPDATING_POSE
    }

    public static class Params {
        public double limelightTrust = .5;
        public boolean showValidLocalizationZones = true;
        public double[] validLocalizeZone = { -24, 0, 96 };
        public double maxUpdateTranslationalVel = 2, maxUpdateHeadingDegVel = 2; // inches and degrees
        public int maxUpdateTurretVelTicksPerSec = 1;
        public int nearNumPrevFramesToAvg = 5, farNumPrevFramesToAvg = 10;
        public double minTimeBetweenUpdates = 5;
        public boolean useMT2 = true;
        public double jankPerpOffset = 0, jankParallelOffset = 0;
        public int numPrevPosesToPrint = 0;
    }

    public static LocalizationType localizationType = LocalizationType.CONTINUOUS;
    public static Params params = new Params();
    private Pose2d avgCameraPose, avgRobotPose;
    private LLResult aprilTagResult;
    private Pose2d prevAvgCameraPose;
    private final ArrayList<Pose3D> prevCameraPoses;
    private double maxTranslationalVariance, maxHeadingVarianceDeg; // how much the limelight jitters
    private double maxTranslationalError, maxHeadingErrorDeg; // how much the limelight differs from pinpoint
    private boolean drivetrainGoodForUpdate, turretGoodForUpdate, inLocalizationZone;
    private boolean localizeInterlocksMet;
    private final ElapsedTime timeSinceLastLocalizeTimer;
    private boolean poseUpdateSuccessful;

    private boolean foundMotif;
    private int targetGreenPos;

    private LocalizationState state, prevState;
    private final ElapsedTime stateTimer;
    private List<LLResultTypes.FiducialResult> visibleTagInfo;

    public LimelightLocalization(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);

        avgRobotPose = null;
        avgCameraPose = null;
        prevAvgCameraPose = new Pose2d(0, 0, 0);
        prevCameraPoses = new ArrayList<>();
        stateTimer = new ElapsedTime();
        prevState = LocalizationState.PASSIVE_READING;
        setState(LocalizationState.PASSIVE_READING);
        drivetrainGoodForUpdate = false;
        turretGoodForUpdate = false;
        resetMaxErrors();
        visibleTagInfo = new ArrayList<>();
        timeSinceLastLocalizeTimer = new ElapsedTime();
        timeSinceLastLocalizeTimer.reset();
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
        prevState = this.state;
        this.state = state;
        if (state == LocalizationState.UPDATING_POSE) {
            prevCameraPoses.clear();
        }
    }

    public void update() {
        switch (state) {
            case SCANNING_MOTIF:
                updateMotifData();
                break;

            case PASSIVE_READING:
            case UPDATING_POSE:
                Pose3D rawCameraPose3 = getRawCameraPoseFromLimelight();
                localizeInterlocksMet = updateInterlocks();

                if(rawCameraPose3 == null || !localizeInterlocksMet) {
                    setAllPosesToNull();
                    resetMaxErrors();
                    prevCameraPoses.clear();
                    if(state == LocalizationState.UPDATING_POSE) {
                        setState(LocalizationState.PASSIVE_READING);
                        poseUpdateSuccessful = false;
                    }
                    break;
                }
                if (state == LocalizationState.PASSIVE_READING
                        && localizationType == LocalizationType.CONTINUOUS
                        && (timeSinceLastLocalizeTimer.seconds() > params.minTimeBetweenUpdates || !poseUpdateSuccessful)) {
                    setState(LocalizationState.UPDATING_POSE);
                }

                prevCameraPoses.add(rawCameraPose3);
                double desiredNumToAvg = robot.shootingSystem.locationState == ShootingSystem.Location.FAR ? params.farNumPrevFramesToAvg : params.nearNumPrevFramesToAvg;
                if(prevCameraPoses.size() > desiredNumToAvg)
                    prevCameraPoses.remove(0);

                prevAvgCameraPose = avgCameraPose;
                avgCameraPose = getAvgPose(prevCameraPoses);
                avgRobotPose = calculateRobotPose(avgCameraPose);
                updateMaxErrors(prevAvgCameraPose, avgCameraPose);

                if(state == LocalizationState.UPDATING_POSE && prevCameraPoses.size() >= desiredNumToAvg) {
                    Pose2d pinpointPose = robot.drive.localizer.getPose();
                    Vector2d avgPos = pinpointPose.position.times(1 - params.limelightTrust).plus(avgRobotPose.position.times(params.limelightTrust));
                    double avgHeading = pinpointPose.heading.toDouble() * (1 - params.limelightTrust) + avgRobotPose.heading.toDouble() * params.limelightTrust;

                    robot.drive.localizer.setPose(new Pose2d(avgPos, avgHeading));
                    timeSinceLastLocalizeTimer.reset();
                    setState(LocalizationState.PASSIVE_READING);
                    poseUpdateSuccessful = true;
                }
                break;
        }
    }
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("   localization state", state);
        if(aprilTagResult != null) {
            telemetry.addData("   LLResult valid", aprilTagResult.isValid());
            if(state == LocalizationState.SCANNING_MOTIF) {
                telemetry.addData("found motif", foundMotif);
                telemetry.addData("target green pos", targetGreenPos);
            }
            else {
                telemetry.addData("   bot pose is null", aprilTagResult.getBotpose() == null);
                if(avgCameraPose != null) {
                    telemetry.addData("   camera pose", MathUtils.formatPose2(avgCameraPose));
                    telemetry.addData("   robot pose", MathUtils.formatPose2(avgRobotPose));
                    telemetry.addLine();
                }
                telemetry.addData("   max translational variance", MathUtils.format3(maxTranslationalVariance));
                telemetry.addData("   max heading variance", MathUtils.format3(maxHeadingVarianceDeg));
                telemetry.addData("   max translational error", MathUtils.format3(maxTranslationalError));
                telemetry.addData("   max heading error", MathUtils.format3(maxHeadingErrorDeg));
                telemetry.addLine();
                telemetry.addData("   drivetrain good for update", drivetrainGoodForUpdate);
                telemetry.addData("   turret good for update", turretGoodForUpdate);
                telemetry.addData("   in localization zone", inLocalizationZone);
                telemetry.addData("   localize interlocks met", localizeInterlocksMet);
                telemetry.addData("   time since last update", timeSinceLastLocalizeTimer.seconds());
                telemetry.addData("   pose update successful", poseUpdateSuccessful);
                telemetry.addLine();
                telemetry.addData("   num visible tags", visibleTagInfo.size());
                StringBuilder tagIDs = new StringBuilder();
                for (LLResultTypes.FiducialResult result : visibleTagInfo)
                    tagIDs.append(result.getFiducialId()).append(" ");
                telemetry.addData("   tag IDs", tagIDs);

                for (int i = 0; i < Math.min(params.numPrevPosesToPrint, prevCameraPoses.size()); i++)
                    telemetry.addData("   last pose " + (i + 1),
                            MathUtils.format2(prevCameraPoses.get(i).getPosition().x) + " " +
                                    MathUtils.format2(prevCameraPoses.get(i).getPosition().y) + " " +
                                    MathUtils.format2(prevCameraPoses.get(i).getOrientation().getYaw(AngleUnit.DEGREES)));
            }
        }
        else
            telemetry.addLine("   result is null");
    }
    private void updateMotifData() {
        aprilTagResult = limelight.getLatestResult();
        visibleTagInfo.clear();

        boolean motifDataNull = aprilTagResult == null || !aprilTagResult.isValid();
        if(motifDataNull) {
            return;
        }

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
    private Pose2d calculateRobotPose(Pose2d avgCameraPose) {
        double a = avgCameraPose.heading.toDouble();
        Vector2d parallelBasis = new Vector2d(Math.cos(a), Math.sin(a));
        Vector2d perpBasis = new Vector2d(parallelBasis.y * 1, -parallelBasis.x);
        Vector2d jankOffset = parallelBasis.times(params.jankParallelOffset).plus(perpBasis.times(params.jankPerpOffset));
        avgCameraPose = new Pose2d(avgCameraPose.position.plus(jankOffset), avgCameraPose.heading.toDouble());
        Pose2d turretPose = Limelight.getTurretPose(avgCameraPose);
        return ShootingMathOld.getRobotPose(turretPose, robot.turret.curRelAngleRad);
    }
    private boolean updateInterlocks() {
        drivetrainGoodForUpdate = isDrivetrainStable();
        turretGoodForUpdate = isTurretStable();
        inLocalizationZone = isInLocalizationZone();
        return drivetrainGoodForUpdate && turretGoodForUpdate && inLocalizationZone;
    }
    public boolean foundMotif() {
        return foundMotif;
    }
    public int getTargetGreenPos() {
        return targetGreenPos;
    }
    private Pose3D getRawCameraPoseFromLimelight() {
        if (params.useMT2) {
            double turretHeadingDeg = Math.toDegrees(robot.turret.currentAbsoluteAngleRad);
            limelight.updateRobotOrientation(turretHeadingDeg);
        }

        aprilTagResult = limelight.getLatestResult();
        visibleTagInfo.clear();

        if (aprilTagResult == null || !aprilTagResult.isValid())
            return null;

        visibleTagInfo = aprilTagResult.getFiducialResults();
        Pose3D curFrameCameraPose = params.useMT2 ? aprilTagResult.getBotpose_MT2() : aprilTagResult.getBotpose();
        if (curFrameCameraPose == null)
            return null;

        Position curFrameCameraPosition = curFrameCameraPose.getPosition().toUnit(DistanceUnit.INCH);
        if (Math.abs(curFrameCameraPosition.x) < 0.00001 &&
            Math.abs(curFrameCameraPosition.y) < 0.00001 &&
            Math.abs(curFrameCameraPose.getOrientation().getYaw(AngleUnit.DEGREES)) < 0.00001)
            return null;

        return new Pose3D(curFrameCameraPosition, curFrameCameraPose.getOrientation());
    }
    private void setAllPosesToNull() {
        avgCameraPose = null;
        avgRobotPose = null;
    }
    private void resetMaxErrors() {
        maxTranslationalVariance = 0;
        maxHeadingVarianceDeg = 0;
        maxTranslationalError = 0;
        maxHeadingErrorDeg = 0;
    }
    private void updateMaxErrors(Pose2d lastAvgPose, Pose2d curAvgPose) {
        if (lastAvgPose == null || curAvgPose == null)
            return;
        double translationalVariance = Math.hypot(curAvgPose.position.x - lastAvgPose.position.x, curAvgPose.position.y - lastAvgPose.position.y);
        double headingVarianceDeg = Math.abs(Math.toDegrees(curAvgPose.heading.toDouble() - lastAvgPose.heading.toDouble()));
        maxTranslationalVariance = Math.max(maxTranslationalVariance, translationalVariance);
        maxHeadingVarianceDeg = Math.max(maxHeadingVarianceDeg, headingVarianceDeg);

        Pose2d odoPose = robot.drive.localizer.getPose();
        double translationalError = Math.hypot(curAvgPose.position.x - odoPose.position.x, curAvgPose.position.y - odoPose.position.y);
        double headingErrorDeg = Math.abs(Math.toDegrees(curAvgPose.heading.toDouble() - odoPose.heading.toDouble()));
        maxTranslationalError = Math.max(maxTranslationalError, translationalError);
        maxHeadingErrorDeg = Math.max(maxHeadingErrorDeg, headingErrorDeg);
    }
    private Pose2d getAvgPose(ArrayList<Pose3D> poses) {
        double x = 0, y = 0, hRad = 0;
        for (Pose3D pose : poses) {
            x += pose.getPosition().x;
            y += pose.getPosition().y;
            hRad += pose.getOrientation().getYaw(AngleUnit.RADIANS);
        }
        int num = poses.size();
        return new Pose2d(x / num, y / num, hRad / num);
    }
    private boolean isDrivetrainStable() {
        OdoInfo odoVel = robot.drive.pinpoint().getVelocity();
        return Math.abs(Math.toDegrees(odoVel.headingRad)) < params.maxUpdateHeadingDegVel && Math.hypot(odoVel.x, odoVel.y) < params.maxUpdateTranslationalVel;
    }
    private boolean isTurretStable() {
        return robot.shootingSystem.getTurretVelTps() < params.maxUpdateTurretVelTicksPerSec;
    }
    private boolean isInLocalizationZone() {
        Pose2d odoPose = robot.drive.localizer.getPose();
        return Math.hypot(odoPose.position.x - params.validLocalizeZone[0], odoPose.position.y -  params.validLocalizeZone[1]) < params.validLocalizeZone[2];
    }
    public boolean poseUpdateSuccessful() {
        return poseUpdateSuccessful;
    }
    public void attemptManualPoseUpdate() {
        setState(LocalizationState.UPDATING_POSE);
    }

    public void addLocalizationInfo(Canvas fieldOverlay) {

        if (params.showValidLocalizationZones) {
            fieldOverlay.setStroke("yellow");
            fieldOverlay.strokeCircle(params.validLocalizeZone[0], params.validLocalizeZone[1], params.validLocalizeZone[2]);
        }

        fieldOverlay.setStroke("blue");
        Pose2d robotPoseToDraw = avgRobotPose == null ? new Pose2d(0, 0, 0) : new Pose2d(avgRobotPose.position, avgRobotPose.heading);
        Drawing.drawRobot(fieldOverlay, robotPoseToDraw);

        Pose2d turretPose = ShootingMathOld.getTurretPose(robotPoseToDraw, robot.turret.curRelAngleRad);
        Drawing.drawRobotSimple(fieldOverlay, turretPose, 3);

        Pose2d limelightPose = Limelight.getLimelightPose(turretPose);
        Drawing.drawRobotSimple(fieldOverlay, limelightPose, 2);
    }
}
