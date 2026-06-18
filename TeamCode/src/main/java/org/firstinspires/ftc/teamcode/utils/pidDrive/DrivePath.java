package org.firstinspires.ftc.teamcode.utils.pidDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.pathParams.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class DrivePath implements Action {
    public enum PathDrawType {
        IDEAL,
        ACTUAL
    }
    public static PathDrawType pathDrawType = PathDrawType.ACTUAL;
    public static boolean showRobotPose = false;
    private static DrivePath mostRecentPath = null;
    public static void drawCurrentPath(Canvas fieldOverlay) {
        if (fieldOverlay == null)
            throw new IllegalStateException("fieldOverlay is null - need to call DrivePath.enableFieldDrawing(fieldOverlay)");
        if (mostRecentPath != null) {
            mostRecentPath.drawPath(fieldOverlay);

            if (showRobotPose)
                mostRecentPath.drawRobotPose(mostRecentPath.odo.getPose(), fieldOverlay);
        }
    }
    public static double baseVoltage = 13.5;
    private final MecanumDrive drivetrain;
    private final PinpointLocalizer odo;
    private final Telemetry telemetry;
    private final ArrayList<Waypoint> waypoints; // list of all waypoints in drive path
    private int curWaypointIndex; // the waypoint index the drivetrain is currently trying to go to
    private PidDrivePidController totalDistancePID, waypointDistancePID, headingRadCloseErrorPID, headingRadFarErrorPID;
    private final ElapsedTime waypointTimer;
    private double angleRadToTargetWaypoint;
    private boolean first;
    private Pose2d startPose;
    private double splineT;
    private Pose2d prevWaypointPose, targetPose;
    private final ArrayList<Vector2d> prevPositions = new ArrayList<>();
    private Vector2d driveVector, correctiveVector, customForceVector, combinedDirectionVector;
    private boolean shouldUpdatePose = false;
    private double waypointDistanceError;
    private final ElapsedTime customEndConfirmationTimer = new ElapsedTime();
    private boolean prevCustomEndTriggered = false;
    public DrivePath(MecanumDrive drivetrain, Waypoint ...waypoints) {
        this(drivetrain, null, waypoints);
    }
    public DrivePath(MecanumDrive drivetrain, Telemetry telemetry, Waypoint ...waypoints) {
        this.drivetrain = drivetrain;
        this.odo = drivetrain.pinpoint();
        this.telemetry = telemetry;

        this.waypoints = new ArrayList<>();
        this.waypoints.addAll(Arrays.asList(waypoints));
        for(int i = 0; i < this.waypoints.size() - 1; i++)
            this.waypoints.get(i).setNextWaypoint(this.waypoints.get(i+1));

        curWaypointIndex = 0;

        waypointTimer = new ElapsedTime();
        first = true;
        startPose = new Pose2d(0, 0, 0);
        angleRadToTargetWaypoint = 0;
    }
    public void setShouldUpdatePose(boolean shouldUpdatePose) {
        this.shouldUpdatePose = shouldUpdatePose;
    }
    // adds another waypoint to the end of the list
    public void addWaypoint(Waypoint waypoint) {
        addWaypoint(waypoint, waypoints.size());
    }
    // inserts a waypoint at the given index and pushes everything at and after that index back
    public void addWaypoint(Waypoint waypoint, int index) {
        waypoints.add(index, waypoint);
        if (waypoints.size() == 1)
            return;
        Waypoint prevWaypoint = getWaypoint(index - 1);
        Waypoint nextWaypoint = getWaypoint(index + 1);
        if (prevWaypoint != null)
            prevWaypoint.setNextWaypoint(waypoint);
        if (nextWaypoint != null)
            waypoint.setNextWaypoint(nextWaypoint);
    }
    public Waypoint getWaypoint(int index) {
        if (index < 0 || index >= waypoints.size())
            return null;
        return waypoints.get(index);
    }
    public ArrayList<Waypoint> getWaypoints() {
        return waypoints;
    }
    public Waypoint getCurWaypoint() {
        return waypoints.get(Math.min(waypoints.size() - 1, curWaypointIndex));
    }
    public PathParams getCurParams() {
        return getCurWaypoint().params;
    }
    private double getWaypointDistanceToTarget(int waypointIndex) {
        if (waypointIndex == waypoints.size() - 1)
            return 0;
        double dist = 0;
        for (int i=waypointIndex; i<waypoints.size() - 1; i++)
            dist += waypoints.get(i).getDistToNextWaypoint();
        return dist;
    }
    // x: axial
    // y: negative lateral
    private Vector2d updateTargetDriveDir(Pose2d robotPose) {
        Vector2d robotPos = robotPose.position;
        double headingRad = robotPose.heading.toDouble();
        if(getCurParams().pathType == PathParams.PathType.CURVED) {
            double targetX = MathUtils.lerp(getCurParams().controlPoint.position.x, getCurWaypoint().x(), splineT);
            double targetY = MathUtils.lerp(getCurParams().controlPoint.position.y, getCurWaypoint().y(), splineT);
            double targetHeadingRad = MathUtils.lerp(getCurParams().controlPoint.heading.toDouble(), getCurWaypoint().headingRad(), splineT);
            targetPose = new Pose2d(targetX, targetY, targetHeadingRad);
        }
        angleRadToTargetWaypoint = MathUtils.vecAngle(targetPose.position.minus(robotPos));

        // translating target so that drivetrain is around origin
        double xFromRobot = Math.cos(angleRadToTargetWaypoint);
        double yFromRobot = Math.sin(angleRadToTargetWaypoint);
        Vector2d targetDir = GeometryUtils.rotateVector(new Vector2d(xFromRobot, yFromRobot), -headingRad);
        double targetDirMag = Math.hypot(targetDir.x, targetDir.y);
        if(targetDirMag < 1e-6)
            return new Vector2d(0, 0);
        else
            return targetDir.div(targetDirMag);
    }
    // gets the corrective drive powers in the robot's coordinate plane
    // x: axial
    // y: -lateral
    private Vector2d getCorrectiveVector(Pose2d robotPose, Vector2d prevWaypointPosition, Vector2d targetWaypointPosition) {
        Vector2d curPos = robotPose.position;

        Vector2d prevToCur = curPos.minus(prevWaypointPosition);
        Vector2d prevToTargetDir = targetWaypointPosition.minus(prevWaypointPosition);
        prevToTargetDir = prevToTargetDir.div(MathUtils.vecMag(prevToTargetDir));
        Vector2d orthogonalDir = new Vector2d(-prevToTargetDir.y, 1*prevToTargetDir.x);

        double projectedPerpendicularOffset = orthogonalDir.dot(prevToCur);
        double correctiveMagnitude = projectedPerpendicularOffset * getCurParams().correctiveKp;

        // corrective vector in the fields coordinate plane
        Vector2d absoluteCorrectivePower = orthogonalDir.times(-correctiveMagnitude);

        // rotate vector by negative robot heading to get relative corrective powers
        Vector2d relativeCorrectivePower = GeometryUtils.rotateVector(absoluteCorrectivePower, -robotPose.heading.toDouble());
        return relativeCorrectivePower.times(getCurParams().correctiveStrength); // scale relative to drive vector
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        mostRecentPath = this;
        if (shouldUpdatePose) {
            drivetrain.updatePoseEstimate();
        }
        Pose2d robotPose = odo.getPose();

        if (first) {
            first = false;
            startPose = new Pose2d(robotPose.position, robotPose.heading);
            prevPositions.clear();
            prevPositions.add(startPose.position);
            resetToNewWaypoint();
        }
        prevPositions.add(robotPose.position);

        // note: error is calculated in field's coordinate plane
        ErrorInfo errorInfo = getWaypointErrorInfo(robotPose, getCurWaypoint());

        // tolerance
        boolean inPositionTolerance = getCurWaypoint().tolerance.inPositionTolerance(errorInfo.xError, errorInfo.yError);
        boolean inHeadingTolerance = getCurWaypoint().tolerance.inHeadingTolerance(errorInfo.headingRadError);
        // pass position
        if (getCurParams().passPosition && getDotProductToNextWaypoint(robotPose) > 0)
            inPositionTolerance = true;

        boolean inWaypointTolerance = inPositionTolerance && inHeadingTolerance;
        boolean reachedMinTime = waypointTimer.seconds() >= getCurParams().minTime;
        boolean reachedMaxTime = waypointTimer.seconds() >= getCurParams().maxTime;

        boolean currentCustomEndTriggered = getCurParams().customEndCondition.getAsBoolean();
        if (!prevCustomEndTriggered && currentCustomEndTriggered)
            customEndConfirmationTimer.reset();
        prevCustomEndTriggered = currentCustomEndTriggered;
        boolean meetsCustomEnd = currentCustomEndTriggered && customEndConfirmationTimer.seconds() >= getCurParams().customEndConfirmationTime;
        // in tolerance
        if (reachedMinTime && (inWaypointTolerance || reachedMaxTime || meetsCustomEnd)) {
            // completely finished drive path
            if (curWaypointIndex + 1 >= waypoints.size()) {
                if (getCurParams().slowDownPercent == 1)
                    drivetrain.stop();
                return false;
            }
            // finished current waypoint path, moving on to next waypoint
            curWaypointIndex++;
            // set new PID targets and targetPose
            resetToNewWaypoint();

            // recalculate new waypoint errors and tolerances
            errorInfo = getWaypointErrorInfo(robotPose, getCurWaypoint());
            inPositionTolerance = getCurWaypoint().tolerance.inPositionTolerance(errorInfo.xError, errorInfo.yError);
            inHeadingTolerance = getCurWaypoint().tolerance.inHeadingTolerance(errorInfo.headingRadError);
        }

        // calculate inputs to speed PIDs
        waypointDistanceError = errorInfo.distanceError;
        double totalDistanceAway = errorInfo.distanceError + getWaypointDistanceToTarget(curWaypointIndex);

        // calculate translational speed
        double linearPower = 0;
        double maxLinearPower = getCurParams().maxLinearPower;
        if (getCurParams().prioritizeHeadingInBeginning && Math.abs(errorInfo.headingRadError) > Math.toRadians(PathParams.defaultParams.prioritizeHeadingThresholdDeg))
            maxLinearPower = PathParams.defaultParams.maxLinearPowerWhilePrioritizingHeading;

        if (!inPositionTolerance) {
            linearPower = calculateLinearPower(getCurWaypoint(), totalDistanceAway, errorInfo.distanceError);
            if (linearPower > 0)
                linearPower = Range.clip(linearPower, getCurParams().minLinearPower, maxLinearPower);
            else if (linearPower < 0)
                linearPower = Range.clip(linearPower, -maxLinearPower, -getCurParams().minLinearPower);
        }

        // calculating drive vector
        double rawT = (getCurParams().tValueStartDist - errorInfo.distanceError) / (getCurParams().tValueStartDist - getCurParams().tValueFinishDist);
        splineT = Range.clip(rawT, 0, 1);
        Vector2d targetDriveDir = updateTargetDriveDir(robotPose);
        driveVector = new Vector2d(
                targetDriveDir.x * linearPower * getCurParams().axialWeight,
                targetDriveDir.y * linearPower * getCurParams().lateralWeight
        );

        // calculating corrective vector
        // for normal paths, default corrective strength is 1
        // for curved paths, default corrective strength is 0 (not applied)
        Vector2d correction = getCorrectiveVector(robotPose, prevWaypointPose.position, targetPose.position);
        correctiveVector = new Vector2d(correction.x * getCurParams().axialWeight, correction.y * getCurParams().lateralWeight);

        // optional custom force vector
        customForceVector = GeometryUtils.rotateVector(getCurParams().customForceVector, -robotPose.heading.toDouble());

        // final vector
        combinedDirectionVector = driveVector.plus(correctiveVector).plus(customForceVector);
        double powerMag = Math.hypot(combinedDirectionVector.x, combinedDirectionVector.y);
        if (powerMag > 1)
            combinedDirectionVector = combinedDirectionVector.div(powerMag);

        // calculate angular speed (heading)
        headingRadCloseErrorPID.setPIDValues(getCurParams().closeHeadingKp, getCurParams().closeHeadingKi, getCurParams().closeHeadingKd);
        headingRadFarErrorPID.setPIDValues(getCurParams().farHeadingKp, getCurParams().farHeadingKi, getCurParams().farHeadingKd);
        double headingPower = 0;
        if (!inHeadingTolerance) {
            if (Math.abs(errorInfo.headingRadError) < Math.toRadians(getCurParams().applyCloseHeadingPIDErrorDeg))
                headingPower = headingRadCloseErrorPID.update(Math.toDegrees(-errorInfo.headingRadError));
            else
                headingPower = headingRadFarErrorPID.update(Math.toDegrees(-errorInfo.headingRadError));
            double headingSign = Math.signum(errorInfo.headingRadError);
            double kfPower = -headingSign * getCurWaypoint().params.headingKf;
            headingPower += kfPower;
            headingPower = headingSign * Range.clip(Math.abs(headingPower), getCurParams().minHeadingPower, getCurParams().maxHeadingPower);
        }

        double filteredVoltage = drivetrain.getFilteredVoltage();
        Vector2d voltageScaledTranslationalPower = combinedDirectionVector.times(baseVoltage / filteredVoltage);
        double voltageScaledHeadingPower = headingPower * baseVoltage / filteredVoltage;

        drivetrain.setDrivePowers(new PoseVelocity2d(voltageScaledTranslationalPower, voltageScaledHeadingPower));

        if (telemetry != null) {
//            telemetry.addData("DP WAYPOINT TIMER", waypointTimer.seconds());
//            telemetry.addData("DP CUR WAYPOINT INDEX", curWaypointIndex);
//            telemetry.addData("DP target pose", MathUtils.formatPose1(targetPose));
//            telemetry.addData("DP current position", MathUtils.format3(robotPose.position.x) + " ," + MathUtils.format3(robotPose.position.y) + ", " + MathUtils.format3(Math.toDegrees(robotPose.heading.toDouble())));
//            telemetry.addData("DP heading waypoint error", MathUtils.format3(Math.toDegrees(errorInfo.headingRadError)));
//            telemetry.addData("DP in position tolerance", inPositionTolerance);
//            telemetry.addData("DP in heading tolerance", inHeadingTolerance);
//            telemetry.addData("WAYPOINT DIR", MathUtils.format3(Math.toDegrees(angleRadToTargetWaypoint)));
//            telemetry.addLine();
        }
        return true;
    }

    private void resetToNewWaypoint() {
        waypointTimer.reset();

        totalDistancePID = new PidDrivePidController(getCurParams().bigSpeedKp, getCurParams().speedKi, getCurParams().bigSpeedKd);
        totalDistancePID.reset();
        totalDistancePID.setTarget(0);
        totalDistancePID.setOutputBounds(0, 1);

        waypointDistancePID = new PidDrivePidController(getCurParams().bigSpeedKp, getCurParams().speedKi, getCurParams().bigSpeedKd);
        waypointDistancePID.reset();
        waypointDistancePID.setTarget(0);
        waypointDistancePID.setOutputBounds(0, 1);

        headingRadCloseErrorPID = new PidDrivePidController(getCurParams().closeHeadingKp, getCurParams().closeHeadingKi, getCurParams().closeHeadingKd);
        headingRadCloseErrorPID.reset();
        headingRadCloseErrorPID.setTarget(0);
        headingRadCloseErrorPID.setOutputBounds(-1, 1);

        headingRadFarErrorPID = new PidDrivePidController(getCurParams().farHeadingKp, getCurParams().farHeadingKi, getCurParams().farHeadingKd);
        headingRadFarErrorPID.reset();
        headingRadFarErrorPID.setTarget(0);
        headingRadFarErrorPID.setOutputBounds(-1, 1);

        prevWaypointPose = curWaypointIndex == 0 ? startPose : getWaypoint(curWaypointIndex - 1).pose;
        if (getCurParams().pathType == PathParams.PathType.CURVED)
            targetPose = getCurParams().controlPoint;
        else
            targetPose = getCurWaypoint().pose;
        splineT = 0;
    }
    // calculates all the relevant info about error between robot pose and waypoint
    private ErrorInfo getWaypointErrorInfo(Pose2d robotPose, Waypoint waypoint) {
        double rx = robotPose.position.x;
        double ry = robotPose.position.y;
        double rHeadingRad = robotPose.heading.toDouble();

        double xWaypointError = Math.abs(rx - waypoint.x());
        double yWaypointError = Math.abs(ry - waypoint.y());
        double waypointDistAway = Math.hypot(xWaypointError, yWaypointError);

        boolean setHeadingTangent = (waypoint.params.headingLerpType == PathParams.HeadingLerpType.TANGENT ||
                waypoint.params.headingLerpType == PathParams.HeadingLerpType.REVERSE_TANGENT)
                        && waypointDistAway > waypoint.params.tangentHeadingDeactivateDist;
        double targetHeading;
        if (setHeadingTangent) {
            if(waypoint.params.headingLerpType == PathParams.HeadingLerpType.TANGENT)
                targetHeading = angleRadToTargetWaypoint;
            else
                targetHeading = MathUtils.angleNormDeltaRad(angleRadToTargetWaypoint + Math.PI);
        }
        else
            targetHeading = targetPose.heading.toDouble();

        double headingRadWaypointError = MathUtils.angleNormDeltaRad(targetHeading - rHeadingRad);

        return new ErrorInfo(xWaypointError, yWaypointError, waypointDistAway, headingRadWaypointError);
    }
    private double calculateLinearPower(Waypoint waypoint, double totalDistanceAway, double waypointDistAway) {
        PathParams params = waypoint.params;
        if(totalDistanceAway < params.applyCloseSpeedPIDError)
            totalDistancePID.setPIDValues(params.smallSpeedKp, params.speedKi, params.smallSpeedKd);
        else
            totalDistancePID.setPIDValues(params.bigSpeedKp, params.speedKi, params.bigSpeedKd);

        if(waypointDistAway < params.applyCloseSpeedPIDError)
            waypointDistancePID.setPIDValues(params.smallSpeedKp, params.speedKi, params.smallSpeedKd);
        else
            waypointDistancePID.setPIDValues(params.bigSpeedKp, params.speedKi, params.bigSpeedKd);

        double a = totalDistancePID.update(totalDistanceAway);
        double b = waypointDistancePID.update(waypointDistAway);
        double slowDownT = params.slowDownPercent;
        double linearPower = a + (b - a) * slowDownT;
        linearPower += params.speedKf;
        return linearPower;
    }

    // calculates vector from previous waypoint to next waypoint
    // calculates vector from next waypoint to robot
    // returns the dot product of these 2 vectors
    // if dot product is positive, robot overshot waypoint
    // if dot is negative, robot has not yet overshot waypoint
    private double getDotProductToNextWaypoint(Pose2d pose) {
        Vector2d oldPosition = curWaypointIndex == 0 ? startPose.position : waypoints.get(curWaypointIndex - 1).pose.position;
        Vector2d targetWaypointPosition = getCurWaypoint().pose.position;

        Vector2d oldToTarget = targetWaypointPosition.minus(oldPosition);
        Vector2d targetToPose = pose.position.minus(targetWaypointPosition);
        return oldToTarget.x * targetToPose.x + oldToTarget.y * targetToPose.y;
    }
    public double getWaypointDistanceError() {
        return waypointDistanceError;
    }
    private void drawPath(Canvas fieldOverlay) {
        Pose2d curWaypointPose = getCurWaypoint().pose;

        boolean isCurved = getCurParams().pathType == PathParams.PathType.CURVED;
        fieldOverlay.setStroke("gray");
        if (pathDrawType == PathDrawType.IDEAL)
            fieldOverlay.strokeLine(prevWaypointPose.position.x, prevWaypointPose.position.y, targetPose.position.x, targetPose.position.y);
        else {
            for (int i=0; i<prevPositions.size() - 1; i++) {
                Vector2d cur = prevPositions.get(i);
                Vector2d next = prevPositions.get(i + 1);
                fieldOverlay.strokeLine(cur.x, cur.y, next.x, next.y);
            }
        }

        fieldOverlay.setStroke("black");
        Drawing.drawRobot(fieldOverlay, curWaypointPose);
        if (!isCurved)
            Drawing.drawRobot(fieldOverlay, prevWaypointPose);
        else {
            fieldOverlay.setStroke("gray");
            Drawing.drawRobot(fieldOverlay, targetPose);
            Drawing.drawRobot(fieldOverlay, getCurParams().controlPoint);
        }
    }
    private void drawRobotPose(Pose2d robotPose, Canvas fieldOverlay) {
        fieldOverlay.setStroke("green");
        Drawing.drawRobot(fieldOverlay, robotPose);
    }
}

