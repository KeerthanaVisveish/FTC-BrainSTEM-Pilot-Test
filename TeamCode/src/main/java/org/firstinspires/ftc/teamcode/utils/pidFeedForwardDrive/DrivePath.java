package org.firstinspires.ftc.teamcode.utils.pidFeedForwardDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;

import java.util.ArrayList;
import java.util.Arrays;

public class DrivePath implements Action {
    public static double moveRightSign = -1, moveForwardSign = 1, turnLeftSign = -1;
    private final VoltageSensor voltageSensor;
    private final MecanumDrive drivetrain;
    private final PinpointLocalizer odo;
    private final Telemetry telemetry;
    private final ArrayList<Waypoint> waypoints; // list of all waypoints in drive path
    private int curWaypointIndex; // the waypoint index the drivetrain is currently trying to go to
    private final PIDController distancePID, headingRadPID;
    private final SimpleMotorFeedforward distanceFeedForward, headingRadFeedForward;
    private final ElapsedTime waypointTimer;
    private boolean first;
    private Pose2d startPose;

    public DrivePath(HardwareMap hardwareMap, MecanumDrive drivetrain, Waypoint...waypoints) {
        this(hardwareMap, drivetrain, null, waypoints);
    }
    public DrivePath(HardwareMap hardwareMap, MecanumDrive drivetrain, Telemetry telemetry, Waypoint...waypoints) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next(); // gets the first valid voltage sensor
        this.drivetrain = drivetrain;
        this.odo = drivetrain.pinpoint();
        this.telemetry = telemetry;

        this.waypoints = new ArrayList<>();
        this.waypoints.addAll(Arrays.asList(waypoints));
        curWaypointIndex = 0;

        waypointTimer = new ElapsedTime();
        first = true;
        startPose = new Pose2d(0, 0, 0);

        distancePID = new PIDController(PathParams.pid.speedKp, PathParams.pid.speedKi, PathParams.pid.speedKd);
        distancePID.reset();
        distancePID.setTarget(0);
        distancePID.setOutputBounds(0, 1);

        headingRadPID = new PIDController(PathParams.pid.headingKp, PathParams.pid.headingKi, PathParams.pid.headingKd);
        headingRadPID.reset();
        headingRadPID.setTarget(0);
        headingRadPID.setOutputBounds(-1, 1);

        distanceFeedForward = new SimpleMotorFeedforward(PathParams.ff.speedKs, PathParams.ff.speedKv, PathParams.ff.speedKa);
        headingRadFeedForward = new SimpleMotorFeedforward(PathParams.ff.headingKs, PathParams.ff.headingKv, PathParams.ff.headingKa);
    }

    public void addWaypoint(Waypoint waypoint) {
        waypoint.setDistToNextWaypoint(waypoints.get(waypoints.size() - 1));
        waypoints.add(waypoints.size() - 1, waypoint);
        if(waypoints.size() >= 3)
            waypoints.get(waypoints.size() - 3).setDistToNextWaypoint(waypoint);
    }
    public Waypoint getWaypoint(int index) {
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

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        drivetrain.updatePoseEstimate();
        Pose2d pose = odo.getPose();
        double rx = pose.position.x, ry = pose.position.y, rHeadingRad = MathUtils.angleNormRad(pose.heading.toDouble()), rHeadingDeg = Math.toDegrees(rHeadingRad);

        if (first) {
            first = false;
            resetToNewWaypoint();
            startPose = new Pose2d(pose.position, pose.heading);
        }

        // finding direction that motor powers should be applied in
        Vector2d targetDir = updateTargetDir(rx, ry, rHeadingRad);

        // note: error is calculated in field's coordinate plane
        double[] waypointErrorInfo = calculateWaypointErrorInfo(rx, ry, rHeadingDeg);
        double xWaypointError = waypointErrorInfo[0], yWaypointError = waypointErrorInfo[1], headingDegWaypointError = waypointErrorInfo[2], headingPowerMultiplier = waypointErrorInfo[3];

        // tolerance
        boolean[] reachedWaypointInfo = getReachedWaypointInfo(pose, xWaypointError, yWaypointError, headingDegWaypointError);
        boolean reachedWaypoint = reachedWaypointInfo[0], inPositionTolerance = reachedWaypointInfo[1], inHeadingTolerance = reachedWaypointInfo[2];

        // in tolerance
        if (reachedWaypoint) {
            curWaypointIndex++;
            // completely finished drive path
            if (curWaypointIndex >= waypoints.size()) {
                drivetrain.stop();
                telemetry.addLine("finished drive path");
                telemetry.update();
                return false;
            }
            // finished current waypoint path, moving on to next waypoint
            else {
                // set new PID targets
                resetToNewWaypoint();

                // recalculate new waypoint errors
                waypointErrorInfo = calculateWaypointErrorInfo(rx, ry, rHeadingDeg);
                xWaypointError = waypointErrorInfo[0];
                yWaypointError = waypointErrorInfo[1];
                headingDegWaypointError = waypointErrorInfo[2];
                headingPowerMultiplier = waypointErrorInfo[3];

                // recalculate new tolerances
                reachedWaypointInfo = getReachedWaypointInfo(pose, xWaypointError, yWaypointError, headingDegWaypointError);
                inPositionTolerance = reachedWaypointInfo[1];
                inHeadingTolerance = reachedWaypointInfo[2];
            }
        }

        // calculate inputs to speed PIDs
        double waypointDistAway = Math.sqrt(xWaypointError * xWaypointError + yWaypointError * yWaypointError);
        double totalDistanceAway = waypointDistAway + getWaypointDistanceToTarget(curWaypointIndex);

        // calculate translational speed
        double translationalVoltage = 0;
        if (!inPositionTolerance) {
            double a = totalDistanceAway;
            double b = waypointDistAway;
            double t = getCurParams().slowDownPercent;
            double dist = a + (b - a) * t;

            // velocity and acceleration feedforward
            DriveInfo driveInfo = getCurParams().translationalMotionProfile.calculate(dist);
            translationalVoltage = distanceFeedForward.calculate(driveInfo.vel, driveInfo.accel);

            // PID feedback control
            translationalVoltage += distancePID.update(dist);

            if (translationalVoltage > 0)
                translationalVoltage = Range.clip(translationalVoltage, getCurParams().minLinearVoltage, getCurParams().maxLinearVoltage);
            else if (translationalVoltage < 0)
                translationalVoltage = Range.clip(translationalVoltage, -getCurParams().maxLinearVoltage, -getCurParams().minLinearVoltage);
        }

        // calculate angular speed (heading)
        double headingVoltage = 0;
        if (!inHeadingTolerance) {
            // velocity and acceleration feedforward
            DriveInfo driveInfo = getCurParams().headingRadMotionProfile.calculate(Math.toRadians(headingDegWaypointError));
            headingVoltage = headingRadFeedForward.calculate(driveInfo.vel, driveInfo.accel);

            // PID feedback
            headingVoltage += headingRadPID.update(headingDegWaypointError);

            // heading correction
            double headingSign = Math.signum(headingVoltage);
            headingVoltage = -headingSign * Range.clip(Math.abs(headingVoltage), getCurParams().minHeadingPower, getCurParams().maxHeadingPower);
            headingVoltage *= headingPowerMultiplier * turnLeftSign;
        }

        // TODO - find how to actually get battery voltage
        double batteryVoltage = voltageSensor.getVoltage();
        double translationalPower = translationalVoltage / batteryVoltage;
        double headingPower = headingVoltage / batteryVoltage;

        double lateralPower = targetDir.x * translationalPower * getCurParams().lateralWeight * moveRightSign;
        double axialPower = targetDir.y * translationalPower * getCurParams().axialWeight * moveForwardSign;

        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(axialPower, lateralPower), headingPower));
        
        if (telemetry != null) {
            telemetry.addData("battery voltage", batteryVoltage);
            telemetry.addData("current position", MathUtils.format3(rx) + " ," + MathUtils.format3(ry) + ", " + MathUtils.format3(rHeadingDeg));
            telemetry.addData("target position", MathUtils.format3(getCurWaypoint().x()) + " ," + MathUtils.format3(getCurWaypoint().y()) + ", " + MathUtils.format3(getCurWaypoint().headingDeg()));
            telemetry.addData("target dir", MathUtils.format3(targetDir.x) + ", " + MathUtils.format3(targetDir.y));
            telemetry.addData("waypoint errors", MathUtils.format3(xWaypointError) + ", " + MathUtils.format3(yWaypointError) + ", " + MathUtils.format3(headingDegWaypointError));
            telemetry.addData("in position tolerance", inPositionTolerance);
            telemetry.addData("in heading tolerance", inHeadingTolerance);
            telemetry.addLine();
            telemetry.addData("translational power", MathUtils.format3(translationalPower));
            telemetry.addData("heading power", MathUtils.format3(headingPower));
            telemetry.addData("powers (lat, ax, heading)", MathUtils.format3(lateralPower) + ", " + MathUtils.format3(axialPower) + ", " + MathUtils.format2(headingVoltage));
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d prevWaypointPose = curWaypointIndex == 0 ? startPose : getWaypoint(curWaypointIndex - 1).pose;
            TelemetryHelper.radii[0] = 5;
            TelemetryHelper.radii[1] = 5;
            TelemetryHelper.radii[2] = 5;
            TelemetryHelper.numPosesToShow = 3;
            TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, pose, prevWaypointPose, getCurWaypoint().pose);
            fieldOverlay.setStroke("black");
            double angle = pose.heading.toDouble() + Math.atan2(targetDir.y, targetDir.x);
            fieldOverlay.strokeLine(pose.position.x, pose.position.y, pose.position.x + Math.sin(angle) * 10, pose.position.y - Math.cos(angle) * 10);

            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        return true;
    }
    private Vector2d updateTargetDir(double robotX, double robotY, double headingRad) {
        // translating target so that drivetrain is around origin
        double xFromRobot = getCurWaypoint().x() - robotX; // 48
        double yFromRobot = getCurWaypoint().y() - robotY; // 0
        // rotating target around origin
        double rotatedXFromRobot = xFromRobot * Math.sin(headingRad) - yFromRobot * Math.cos(headingRad);
        double rotatedYFromRobot = xFromRobot * Math.cos(headingRad) + yFromRobot * Math.sin(headingRad);
        // 48 * sin(0) - 0 * cos(0) = 0
        // 48 * cos(0) + 0 * sin(0) = 48
        // translating target back to absolute; this returns the direction to the next waypoint IN THE ROBOT'S COORDINATE PLANE
        Vector2d targetDir = new Vector2d(rotatedXFromRobot, rotatedYFromRobot);
        double targetDirMag = Math.hypot(targetDir.x, targetDir.y);
        return targetDir.div(targetDirMag); // normalize
    }
    private double[] calculateWaypointErrorInfo(double robotX, double robotY, double robotHeadingDeg) {
        double xWaypointError = Math.abs(robotX - getCurWaypoint().x());
        double yWaypointError = Math.abs(robotY - getCurWaypoint().y());

        double headingDegWaypointError = getCurWaypoint().headingDeg() - robotHeadingDeg;
        // flip heading error if necessary
        double absHeadingWaypointError = Math.abs(headingDegWaypointError);
        boolean flipHeadingDirection = absHeadingWaypointError > 180;
        if (flipHeadingDirection)
            headingDegWaypointError = Math.signum(headingDegWaypointError) * (360 - absHeadingWaypointError);
        double headingPowerMultiplier = flipHeadingDirection ? -1 : 1;
        return new double[] { xWaypointError, yWaypointError, headingDegWaypointError, headingPowerMultiplier };
    }

    private boolean[] getReachedWaypointInfo(Pose2d robotPose, double xWaypointError, double yWaypointError, double headingDegWaypointError) {
        boolean inPositionTolerance = (xWaypointError <= getCurWaypoint().tolerance.xTol && yWaypointError <= getCurWaypoint().tolerance.yTol) ||
                (getCurParams().passPosition && getDotProductToNextWaypoint(robotPose) > 0); // pass position
        boolean inHeadingTolerance = Math.abs(headingDegWaypointError) <= getCurWaypoint().tolerance.headingDegTol;

        boolean inWaypointTolerance = inPositionTolerance && inHeadingTolerance;
        boolean reachedMaxTime = getCurParams().hasMaxTime() && waypointTimer.seconds() >= getCurParams().maxTime;

        boolean reachedWaypoint = inWaypointTolerance || reachedMaxTime || getCurParams().customEndCondition.getAsBoolean();
        return new boolean[] { reachedWaypoint, inPositionTolerance, inHeadingTolerance };
    }
    // calculates vector from previous waypoint to next waypoint
    // calculates vector from next waypoint to robot
    // returns the dot product of these 2 vectors
    // if dot product is positive, robot overshot waypoint
    // if dot is negative, robot has not yet overshot waypoint
    private double getDotProductToNextWaypoint(Pose2d robotPose) {
        Vector2d oldPosition = curWaypointIndex == 0 ? startPose.position : waypoints.get(curWaypointIndex - 1).pose.position;
        Vector2d targetWaypointPosition = getCurWaypoint().pose.position;

        Vector2d relativeWaypoint = targetWaypointPosition.minus(oldPosition);
        Vector2d relativePositionToWaypoint = robotPose.position.minus(targetWaypointPosition);
        return relativeWaypoint.x * relativePositionToWaypoint.x + relativeWaypoint.y * relativePositionToWaypoint.y;
    }

    private void resetToNewWaypoint() {
        waypointTimer.reset();
        distancePID.reset();
        headingRadPID.reset();
    }
}

