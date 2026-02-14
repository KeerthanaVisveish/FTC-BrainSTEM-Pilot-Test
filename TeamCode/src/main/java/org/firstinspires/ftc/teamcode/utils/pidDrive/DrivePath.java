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
import org.firstinspires.ftc.teamcode.utils.math.GeometryUtils;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

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
    private double curWaypointDirRad;
    private boolean first;
    private Pose2d startPose, prevWaypointPose;
    private double splineT;
    private double targetX, targetY, targetHeadingRad;
    private Vector2d driveVector, correctiveVector, combinedDirectionVector;
    private boolean followingCurvedPath;
    public DrivePath(MecanumDrive drivetrain, Waypoint ...waypoints) {
        this(drivetrain, null, waypoints);
    }
    public DrivePath(MecanumDrive drivetrain, Telemetry telemetry, Waypoint ...waypoints) {
        this.drivetrain = drivetrain;
        this.odo = drivetrain.pinpoint();
        this.telemetry = telemetry;

        this.waypoints = new ArrayList<>();
        this.waypoints.addAll(Arrays.asList(waypoints));
        for(int i = 0; i < this.waypoints.size() - 1; i++) {
            Waypoint cur = this.waypoints.get(i);
            Waypoint next = this.waypoints.get(i+1);
            cur.setDistToNextWaypoint(Math.hypot(cur.x() - next.x(), cur.y() - next.y()));
        }

        curWaypointIndex = 0;

        waypointTimer = new ElapsedTime();
        first = true;
        startPose = new Pose2d(0, 0, 0);
        curWaypointDirRad = 0;
    }
    private Waypoint getWaypoint(int index) {
        return waypoints.get(index);
    }
    private Waypoint getCurWaypoint() {
        return waypoints.get(Math.min(waypoints.size() - 1, curWaypointIndex));
    }
    private PathParams getCurParams() {
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
    private Vector2d updateTargetDriveDir(double robotX, double robotY, double headingRad) {
        if(getCurParams().pathType == PathParams.PathType.CURVED) {
            followingCurvedPath = true;
            targetX = UtilFunctions.lerp(getCurParams().controlPoint.position.x, getCurWaypoint().x(), splineT);
            targetY = UtilFunctions.lerp(getCurParams().controlPoint.position.y, getCurWaypoint().y(), splineT);
            targetHeadingRad = UtilFunctions.lerp(getCurParams().controlPoint.heading.toDouble(), getCurWaypoint().headingRad(), splineT);

            // re-calculating waypoint dir for curve
            curWaypointDirRad = Math.atan2(targetY - robotY, targetX - robotX);
        }
        else
            followingCurvedPath = false;

        // translating target so that drivetrain is around origin
        double xFromRobot = Math.cos(curWaypointDirRad);
        double yFromRobot = Math.sin(curWaypointDirRad);
        Vector2d targetDir = GeometryUtils.fieldVectorToRobotVector(new Vector2d(xFromRobot, yFromRobot), headingRad);
        double targetDirMag = Math.hypot(targetDir.x, targetDir.y);
        return targetDir.div(targetDirMag);
    }
    // gets the corrective drive powers in the robot's coordinate plane
    // x: axial
    // y: -lateral
    private Vector2d getCorrectiveVector(double rx, double ry, double rHeadingRad) {
        Vector2d prevPos = prevWaypointPose.position;
        Vector2d targetPos = getCurWaypoint().pose.position;
        double[] info = GeometryUtils.pointToLineDistanceAndAngle(
                new double[] { prevPos.x, prevPos.y },
                new double[] { targetPos.x, targetPos.y },
                new double[] { rx, ry }
        );
        double distance = info[0];
        double angle = info[1];
        double correctiveMagnitude = distance * getCurParams().correctiveKp;

        // corrective vector in the fields coordinate plane
        double correctiveX = correctiveMagnitude * Math.cos(angle);
        double correctiveY = correctiveMagnitude * Math.sin(angle);

        // rotate vector by negative robot heading to get relative corrective powers
        Vector2d unscaled = GeometryUtils.fieldVectorToRobotVector(new Vector2d(correctiveX, correctiveY), rHeadingRad);
        return unscaled.times(getCurParams().correctiveStrength);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        drivetrain.updatePoseEstimate();
        Pose2d robotPose = odo.getPose();
        double rx = robotPose.position.x, ry = robotPose.position.y, rHeadingRad = MathUtils.angleNormRad(robotPose.heading.toDouble()), rHeadingDeg = Math.toDegrees(rHeadingRad);

        if (first) {
            first = false;
            startPose = new Pose2d(robotPose.position, robotPose.heading);
            resetToNewWaypoint();
        }

        // note: error is calculated in field's coordinate plane
        ErrorInfo errorInfo = getWaypointErrorInfo(robotPose, getCurWaypoint());
        double xWaypointError = errorInfo.xError;
        double yWaypointError = errorInfo.yError;
        double waypointDistAway = errorInfo.distanceError;
        double headingDegWaypointError = errorInfo.headingDegError;
        double headingPowerDirFlip = errorInfo.headingPowerDirFlip;

        // tolerance
        boolean inPositionTolerance = xWaypointError <= getCurWaypoint().tolerance.xTol && yWaypointError <= getCurWaypoint().tolerance.yTol;
        boolean inHeadingTolerance = Math.abs(headingDegWaypointError) <= getCurWaypoint().tolerance.headingDegTol;
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
                xWaypointError = errorInfo.xError;
                yWaypointError = errorInfo.yError;
                waypointDistAway = errorInfo.distanceError;
                headingDegWaypointError = errorInfo.headingDegError;
                headingPowerDirFlip = errorInfo.headingPowerDirFlip;

                // recalculate new tolerances
                inPositionTolerance = xWaypointError <= getCurWaypoint().tolerance.xTol && yWaypointError <= getCurWaypoint().tolerance.yTol;
                inHeadingTolerance = Math.abs(headingDegWaypointError) <= getCurWaypoint().tolerance.headingDegTol;
            }
        }

        // calculate inputs to speed PIDs
        double totalDistanceAway = waypointDistAway + getWaypointDistanceToTarget(curWaypointIndex);

        // calculate translational speed
        double linearPower = 0;
        double maxLinearPower = getCurParams().maxLinearPower;
        if (getCurParams().prioritizeHeadingInBeginning && Math.abs(headingDegWaypointError) > PathParams.defaultParams.prioritizeHeadingThresholdDeg)
            maxLinearPower = PathParams.defaultParams.maxLinearPowerWhilePrioritizingHeading;

        if (!inPositionTolerance) {
            if(totalDistanceAway < getCurParams().applyCloseSpeedPIDError)
                totalDistancePID.setPIDValues(getCurParams().smallSpeedKp, getCurParams().speedKi, getCurParams().smallSpeedKd);
            else
                totalDistancePID.setPIDValues(getCurParams().bigSpeedKp, getCurParams().speedKi, getCurParams().bigSpeedKd);

            if(waypointDistAway < getCurParams().applyCloseSpeedPIDError)
                waypointDistancePID.setPIDValues(getCurParams().smallSpeedKp, getCurParams().speedKi, getCurParams().smallSpeedKd);
            else
                waypointDistancePID.setPIDValues(getCurParams().bigSpeedKp, getCurParams().speedKi, getCurParams().bigSpeedKd);

            double a = totalDistancePID.update(totalDistanceAway);
            double b = waypointDistancePID.update(waypointDistAway);
            double slowDownT = getCurParams().slowDownPercent;
            linearPower = a + (b - a) * slowDownT;
            linearPower += getCurParams().speedKf;

            if (linearPower > 0)
                linearPower = Range.clip(linearPower, getCurParams().minLinearPower, maxLinearPower);
            else if (linearPower < 0)
                linearPower = Range.clip(linearPower, -maxLinearPower, -getCurParams().minLinearPower);
        }

        // calculating drive vector
        Vector2d targetDriveDir = updateTargetDriveDir(rx, ry, rHeadingRad);
        driveVector = new Vector2d(
                targetDriveDir.x * linearPower * getCurParams().axialWeight,
                targetDriveDir.y * linearPower * getCurParams().lateralWeight
        );

        // calculating corrective vector
        if (!followingCurvedPath)
            correctiveVector = getCorrectiveVector(rx, ry, rHeadingRad);
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
            if(Math.abs(headingDegWaypointError) < getCurParams().applyCloseHeadingPIDErrorDeg)
                headingPower = headingRadCloseErrorPID.update(-headingDegWaypointError);
            else
                headingPower = headingRadFarErrorPID.update(-headingDegWaypointError);
            double headingSign = Math.signum(headingDegWaypointError);
            double kfPower = -headingSign * getCurWaypoint().params.headingKf;
            headingPower += kfPower;
            headingPower = headingSign * Range.clip(Math.abs(headingPower), getCurParams().minHeadingPower, getCurParams().maxHeadingPower);
            headingPower *= headingPowerDirFlip;
        }

        drivetrain.setDrivePowers(new PoseVelocity2d(combinedDirectionVector, headingPower));
        
        if (telemetry != null) {
            telemetry.addData("curved", getCurParams().pathType == PathParams.PathType.CURVED);
            telemetry.addData("splineT", splineT);
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("targetHeading", MathUtils.format3(Math.toDegrees(targetHeadingRad)));
            telemetry.addData("total dist away", totalDistanceAway);
            telemetry.addData("slow down", getCurParams().slowDownPercent);
            telemetry.addData("position current", MathUtils.format3(rx) + " ," + MathUtils.format3(ry) + ", " + MathUtils.format3(rHeadingDeg));
            telemetry.addData("position target", MathUtils.format3(getCurWaypoint().x()) + " ," + MathUtils.format3(getCurWaypoint().y()) + ", " + MathUtils.format3(getCurWaypoint().headingDeg()));
            telemetry.addData("position prev", MathUtils.formatPose3(prevWaypointPose));
//            telemetry.addData("target dir", MathUtils.format3(targetDir.x) + ", " + MathUtils.format3(targetDir.y));
//            telemetry.addData("x waypoint error", MathUtils.format3(xWaypointError));
//            telemetry.addData("y waypoint error", MathUtils.format3(yWaypointError));
            telemetry.addData("heading waypoint error", MathUtils.format3(headingDegWaypointError));
//            telemetry.addData("x waypoint tolerance", getCurWaypoint().tolerance.xTol);
//            telemetry.addData("y waypoint tolerance", getCurWaypoint().tolerance.yTol);
//            telemetry.addData("heading waypoint tolerance", getCurWaypoint().tolerance.headingDegTol);
            telemetry.addData("in position tolerance", inPositionTolerance);
            telemetry.addData("in heading tolerance", inHeadingTolerance);
//            telemetry.addData("waypoint dist PID proportional", waypointDistancePID.getProportional());
//            telemetry.addData("waypoint dist PID derivative", waypointDistancePID.getDerivative());
            telemetry.addData("WAYPOINT DIR", MathUtils.format3(Math.toDegrees(curWaypointDirRad)));
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

            fieldOverlay.setStroke("black");
            Drawing.drawRobot(fieldOverlay, prevWaypointPose);
            Drawing.drawRobot(fieldOverlay, curWaypointPose);
            if(getCurParams().pathType == PathParams.PathType.CURVED) {
                fieldOverlay.setStroke("gray");
                Drawing.drawRobot(fieldOverlay, new Pose2d(targetX, targetY, targetHeadingRad));
                Drawing.drawRobot(fieldOverlay, getCurParams().controlPoint);
            }
            fieldOverlay.setStroke("green");
            Drawing.drawRobot(fieldOverlay, robotPose);
            fieldOverlay.strokeLine(robotPose.position.x, robotPose.position.y,
                    robotPose.position.x + Math.cos(curWaypointDirRad) * 10, robotPose.position.y + Math.sin(curWaypointDirRad));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        splineT = Math.min(1, Math.max(waypointTimer.seconds() - getCurParams().tValueStartTime, 0) / getCurParams().tValueMaxOutTime);
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
        curWaypointDirRad = Math.atan2(getCurWaypoint().pose.position.y - prevWaypointPose.position.y, getCurWaypoint().pose.position.x - prevWaypointPose.position.x);

        splineT = 0;
    }
    // calculates all the relevant info about error between robot pose and waypoint
    private ErrorInfo getWaypointErrorInfo(Pose2d robotPose, Waypoint waypoint) {
        double rx = robotPose.position.x;
        double ry = robotPose.position.y;
        double rHeadingDeg = Math.toDegrees(robotPose.heading.toDouble());

        double xWaypointError = Math.abs(rx - waypoint.x());
        double yWaypointError = Math.abs(ry - waypoint.y());
        double waypointDistAway = Math.hypot(xWaypointError, yWaypointError);

        boolean setHeadingTangent = (waypoint.params.headingLerpType == PathParams.HeadingLerpType.TANGENT ||
                waypoint.params.headingLerpType == PathParams.HeadingLerpType.REVERSE_TANGENT)
                        && waypointDistAway > waypoint.params.tangentHeadingDeactivateThreshold;
        double headingDegWaypointError;
        if(getCurParams().pathType == PathParams.PathType.CURVED)
            headingDegWaypointError = Math.toDegrees(targetHeadingRad) - rHeadingDeg;
        else if (setHeadingTangent) {
            if(waypoint.params.headingLerpType == PathParams.HeadingLerpType.TANGENT)
                headingDegWaypointError = Math.toDegrees(curWaypointDirRad) - rHeadingDeg;
            else
                headingDegWaypointError = Math.toDegrees(MathUtils.angleNormDeltaRad(curWaypointDirRad + Math.PI)) - rHeadingDeg;
        }
        else
            headingDegWaypointError = waypoint.headingDeg() - rHeadingDeg;

        // flip heading error if necessary
        double absHeadingWaypointError = Math.abs(headingDegWaypointError);
        boolean flipHeadingDirection = absHeadingWaypointError > 180;
        if (flipHeadingDirection)
            headingDegWaypointError = Math.signum(headingDegWaypointError) * (360 - absHeadingWaypointError);

        return new ErrorInfo(xWaypointError, yWaypointError, waypointDistAway, headingDegWaypointError, flipHeadingDirection ? -1 : 1 );
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
}

