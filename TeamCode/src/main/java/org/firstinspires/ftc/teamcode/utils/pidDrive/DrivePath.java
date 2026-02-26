package org.firstinspires.ftc.teamcode.utils.pidDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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
    public static boolean showRobotPose = true;
    private final MecanumDrive drivetrain;
    private final PinpointLocalizer odo;
    private final Telemetry telemetry;
    private final ArrayList<Waypoint> waypoints; // list of all waypoints in drive path
    private int curWaypointIndex; // the waypoint index the drivetrain is currently trying to go to
    private PidDrivePidController totalDistancePID, waypointDistancePID, headingRadCloseErrorPID, headingRadFarErrorPID;
    private final ElapsedTime waypointTimer;
    private double angleToTargetWaypoint;
    private boolean first;
    private Pose2d startPose, prevWaypointPose;
    private double splineT;
    private double targetX, targetY, targetHeadingRad;
    private Vector2d driveVector, correctiveVector, combinedDirectionVector;
    private boolean followingCurvedPath;
    private boolean shouldUpdatePose = true;
    private double waypointDistanceError;
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
        angleToTargetWaypoint = 0;
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
            followingCurvedPath = true;
            targetX = MathUtils.lerp(getCurParams().controlPoint.position.x, getCurWaypoint().x(), splineT);
            targetY = MathUtils.lerp(getCurParams().controlPoint.position.y, getCurWaypoint().y(), splineT);
            targetHeadingRad = MathUtils.lerp(getCurParams().controlPoint.heading.toDouble(), getCurWaypoint().headingRad(), splineT);
        }
        else
            followingCurvedPath = false;
        angleToTargetWaypoint = Math.atan2(targetY - robotPos.y, targetX - robotPos.x);

        // translating target so that drivetrain is around origin
        double xFromRobot = Math.cos(angleToTargetWaypoint);
        double yFromRobot = Math.sin(angleToTargetWaypoint);
        Vector2d targetDir = GeometryUtils.rotateVector(new Vector2d(xFromRobot, yFromRobot), -headingRad);
        double targetDirMag = Math.hypot(targetDir.x, targetDir.y);
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
        if (shouldUpdatePose) {
            drivetrain.updatePoseEstimate();
        }
        Pose2d robotPose = odo.getPose();

        if (first) {
            first = false;
            startPose = new Pose2d(robotPose.position, robotPose.heading);
            resetToNewWaypoint();
        }

        // note: error is calculated in field's coordinate plane
        ErrorInfo errorInfo = getWaypointErrorInfo(robotPose, getCurWaypoint());

        // tolerance
        boolean inPositionTolerance = getCurWaypoint().tolerance.inPositionTolerance(errorInfo.xError, errorInfo.yError);
        boolean inHeadingTolerance = getCurWaypoint().tolerance.inHeadingTolerance(errorInfo.headingRadError);
        // pass position
        if(getCurParams().passPosition && getDotProductToNextWaypoint(robotPose) > 0)
                inPositionTolerance = true;

        boolean inWaypointTolerance = inPositionTolerance && inHeadingTolerance;
        boolean reachedMaxTime = getCurParams().hasMaxTime() && waypointTimer.seconds() >= getCurParams().maxTime;


        // in tolerance
        if (inWaypointTolerance || reachedMaxTime || getCurParams().customEndCondition.getAsBoolean()) {
            curWaypointIndex++;
            // completely finished drive path
            if (curWaypointIndex >= waypoints.size()) {
                if (getCurParams().slowDownPercent == 1)
                    drivetrain.stop();
                return false;
            }
            // finished current waypoint path, moving on to next waypoint
            else {
                // set new PID targets
                resetToNewWaypoint();

                // recalculate new waypoint errors
                errorInfo = getWaypointErrorInfo(robotPose, getCurWaypoint());

                // recalculate new tolerances
                inPositionTolerance = getCurWaypoint().tolerance.inPositionTolerance(errorInfo.xError, errorInfo.yError);
                inHeadingTolerance = getCurWaypoint().tolerance.inHeadingTolerance(errorInfo.headingRadError);
            }
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
//        splineT = Math.min(1, Math.max(waypointTimer.seconds() - getCurParams().tValueStartDist, 0) / getCurParams().tValueFinishDis);
        double rawT = (getCurParams().tValueStartDist - errorInfo.distanceError) / (getCurParams().tValueStartDist - getCurParams().tValueFinishDist);
        splineT = Range.clip(rawT, 0, 1);
        Vector2d targetDriveDir = updateTargetDriveDir(robotPose);
        driveVector = new Vector2d(
                targetDriveDir.x * linearPower * getCurParams().axialWeight,
                targetDriveDir.y * linearPower * getCurParams().lateralWeight
        );

        // calculating corrective vector
        if (!followingCurvedPath) {
            Vector2d correction = getCorrectiveVector(robotPose, prevWaypointPose.position, getCurWaypoint().pose.position);
            correctiveVector = new Vector2d(correction.x * getCurParams().axialWeight, correction.y * getCurParams().lateralWeight);
        }
        else
            correctiveVector = new Vector2d(0, 0);

        // final vector
        combinedDirectionVector = driveVector.plus(correctiveVector);
        double powerMag = Math.hypot(combinedDirectionVector.x, combinedDirectionVector.y);
        if(powerMag > 1)
            combinedDirectionVector = combinedDirectionVector.div(powerMag);

        // calculate angular speed (heading)
        double headingPower = 0;
        if (!inHeadingTolerance) {
            if(Math.abs(errorInfo.headingRadError) < Math.toRadians(getCurParams().applyCloseHeadingPIDErrorDeg))
                headingPower = headingRadCloseErrorPID.update(-errorInfo.headingRadError);
            else
                headingPower = headingRadFarErrorPID.update(-errorInfo.headingRadError);
            double headingSign = Math.signum(errorInfo.headingRadError);
            double kfPower = -headingSign * getCurWaypoint().params.headingKf;
            headingPower += kfPower;
            headingPower = headingSign * Range.clip(Math.abs(headingPower), getCurParams().minHeadingPower, getCurParams().maxHeadingPower);
            headingPower *= errorInfo.headingPowerDirFlip;
        }

        drivetrain.setDrivePowers(new PoseVelocity2d(combinedDirectionVector, headingPower));
        
        if (telemetry != null) {
            telemetry.addData("curved", getCurParams().pathType == PathParams.PathType.CURVED);
            telemetry.addData("splineT", splineT);
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("targetHeading", MathUtils.format3(Math.toDegrees(targetHeadingRad)));
            telemetry.addData("total dist away", totalDistanceAway);
            telemetry.addData("AAA waypoint dist away", errorInfo.distanceError);
//            telemetry.addData("slow down", getCurParams().slowDownPercent);
//            telemetry.addData("position current", MathUtils.format3(rx) + " ," + MathUtils.format3(ry) + ", " + MathUtils.format3(rHeadingDeg));
//            telemetry.addData("position target", MathUtils.format3(getCurWaypoint().x()) + " ," + MathUtils.format3(getCurWaypoint().y()) + ", " + MathUtils.format3(getCurWaypoint().headingDeg()));
//            telemetry.addData("position prev", MathUtils.formatPose3(prevWaypointPose));
//            telemetry.addData("target dir", MathUtils.format3(targetDir.x) + ", " + MathUtils.format3(targetDir.y));
//            telemetry.addData("x waypoint error", MathUtils.format3(xWaypointError));
//            telemetry.addData("y waypoint error", MathUtils.format3(yWaypointError));
            telemetry.addData("heading waypoint error", MathUtils.format3(Math.toDegrees(errorInfo.headingRadError)));
//            telemetry.addData("x waypoint tolerance", getCurWaypoint().tolerance.xTol);
//            telemetry.addData("y waypoint tolerance", getCurWaypoint().tolerance.yTol);
//            telemetry.addData("heading waypoint tolerance", getCurWaypoint().tolerance.headingDegTol);
            telemetry.addData("in position tolerance", inPositionTolerance);
            telemetry.addData("in heading tolerance", inHeadingTolerance);
//            telemetry.addData("waypoint dist PID proportional", waypointDistancePID.getProportional());
//            telemetry.addData("waypoint dist PID derivative", waypointDistancePID.getDerivative());
            telemetry.addData("WAYPOINT DIR", MathUtils.format3(Math.toDegrees(angleToTargetWaypoint)));
            telemetry.addLine();
//            telemetry.addData("speed", MathUtils.format3(linearPower));
//            telemetry.addData("powers (lat, ax, heading)", MathUtils.format3(lateralPower) + ", " + MathUtils.format3(axialPower) + ", " + MathUtils.format2(headingPower));
//            telemetry.addData("lat pow", MathUtils.format3(lateralPower));
//            telemetry.addData("ax pow", MathUtils.format3(axialPower));
//            telemetry.addData("heading pow", MathUtils.format3(headingPower));
//            telemetry.addData("power hypot", powerMag);

            telemetry.update();
        }
        if (showRobotPose) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d curWaypointPose = getCurWaypoint().pose;

            fieldOverlay.setStroke("gray");
            fieldOverlay.strokeLine(prevWaypointPose.position.x, prevWaypointPose.position.y, targetX, targetY);
            fieldOverlay.setStroke("black");
            Drawing.drawRobot(fieldOverlay, prevWaypointPose);
            Drawing.drawRobot(fieldOverlay, curWaypointPose);
            if (getCurParams().pathType == PathParams.PathType.CURVED) {
                fieldOverlay.setStroke("gray");
                Drawing.drawRobot(fieldOverlay, new Pose2d(targetX, targetY, targetHeadingRad));
                Drawing.drawRobot(fieldOverlay, getCurParams().controlPoint);
            }
            fieldOverlay.setStroke("green");
            Drawing.drawRobot(fieldOverlay, robotPose);
            Vector2d driveVectorToDraw = GeometryUtils.rotateVector(driveVector, robotPose.heading.toDouble()).times(10);
            fieldOverlay.strokeLine(robotPose.position.x, robotPose.position.y,
                    robotPose.position.x + driveVectorToDraw.x, robotPose.position.y + driveVectorToDraw.y);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
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

        targetX = getCurWaypoint().x();
        targetY = getCurWaypoint().y();
        targetHeadingRad = getCurWaypoint().headingRad();
        prevWaypointPose = curWaypointIndex == 0 ? startPose : getWaypoint(curWaypointIndex - 1).pose;

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
                        && waypointDistAway > waypoint.params.tangentHeadingDeactivateThreshold;
        double headingRadWaypointError;
        if(getCurParams().pathType == PathParams.PathType.CURVED)
            headingRadWaypointError = targetHeadingRad - rHeadingRad;
        else if (setHeadingTangent) {
            if(waypoint.params.headingLerpType == PathParams.HeadingLerpType.TANGENT)
                headingRadWaypointError = angleToTargetWaypoint - rHeadingRad;
            else
                headingRadWaypointError = MathUtils.angleNormDeltaRad(angleToTargetWaypoint + Math.PI) - rHeadingRad;
        }
        else
            headingRadWaypointError = waypoint.headingRad() - rHeadingRad;

        // flip heading error if necessary
        double absHeadingWaypointError = Math.abs(headingRadWaypointError);
        boolean flipHeadingDirection = absHeadingWaypointError > 180;
        if (flipHeadingDirection)
            headingRadWaypointError = Math.signum(headingRadWaypointError) * (360 - absHeadingWaypointError);

        return new ErrorInfo(xWaypointError, yWaypointError, waypointDistAway, headingRadWaypointError, flipHeadingDirection ? -1 : 1 );
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

        Vector2d relativeWaypoint = targetWaypointPosition.minus(oldPosition);
        Vector2d relativePositionToWaypoint = pose.position.minus(targetWaypointPosition);
        return relativeWaypoint.x * relativePositionToWaypoint.x + relativeWaypoint.y * relativePositionToWaypoint.y;
    }
    public double getWaypointDistanceError() {
        return waypointDistanceError;
    }
}

