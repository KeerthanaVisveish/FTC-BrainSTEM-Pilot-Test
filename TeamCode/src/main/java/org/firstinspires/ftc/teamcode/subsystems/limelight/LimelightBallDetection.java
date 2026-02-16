package org.firstinspires.ftc.teamcode.subsystems.limelight;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PathFinder;
import org.firstinspires.ftc.teamcode.utils.math.Vec;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.Arrays;

@Config
public class LimelightBallDetection extends LLParent {
    public static class Params {
        public int maxBlobs = 5;
        public int numPiecesOfInfoPerBlob = 3;
        public boolean showPythonOutputs = false;
        public double minDistFromFieldWall = 2.5;
        public boolean drawBalls = false;
        public boolean projectBallsInsideField = false;
    }
    public static class AutoCollectParams {
        public double cornerBallDistance = 10;
        public double minLinearPower = 0.1;
    }

    public static Params params = new Params();
    public static AutoCollectParams autoCollectParams = new AutoCollectParams();
    private double[] pythonOutputs;
    private int numBlobs;
    private Blob[] blobs;
    public LimelightBallDetection(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        pythonOutputs = new double[3];
        blobs = new Blob[0];
    }

    private static class Blob {
        public final double tx, ty, x, y, area;
        public Blob(double tx, double ty, double x, double y, double area) {
            this.tx = tx;
            this.ty = ty;
            this.x = x;
            this.y = y;
            this.area = area;
        }
        @NonNull
        @Override
        public String toString() {
            return "(" + MathUtils.format1(x) + " " + MathUtils.format1(y) + " " + MathUtils.format1(area) + ")";
        }
    }
    @Override
    public void update() {
        LLResult result = limelight.getLatestResult();
        pythonOutputs = result.getPythonOutput();

        int numNonZeroEntries = 0;
        while (numNonZeroEntries < pythonOutputs.length && pythonOutputs[numNonZeroEntries] != 0)
            numNonZeroEntries++;
        numBlobs = numNonZeroEntries / params.numPiecesOfInfoPerBlob;
        if (numBlobs > params.maxBlobs)
            numBlobs = params.maxBlobs;

        blobs = new Blob[numBlobs];
        for (int i=0; i<numBlobs; i++) {
            int index = i * params.numPiecesOfInfoPerBlob;
            double px = pythonOutputs[index];
            double py = pythonOutputs[index + 1];
            double area = pythonOutputs[index + 2];
            blobs[i] = createBlob(px, py, area);
        }
    }

    private Blob createBlob(double px, double py, double area) {
        double tx = Limelight.pixelXToTx(px);
        double ty = Limelight.pixelYToTy(py);
        Pose2d cameraPose = Limelight.getLimelightPose(robot.shootingSystem.turretPose);
        Vector2d fieldPosition = Limelight.calculateBallFieldPosition(cameraPose, tx, ty);
        if (params.projectBallsInsideField)
            fieldPosition = GeometryUtils.projectOntoField(robot.drive.localizer.getPose().position, fieldPosition, params.minDistFromFieldWall);

        return new Blob(tx, ty, fieldPosition.x, fieldPosition.y, area);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        if (params.showPythonOutputs)
            telemetry.addData("python outputs", Arrays.toString(pythonOutputs));
        telemetry.addData("num blobs", numBlobs);
        telemetry.addData("blobs", Arrays.toString(blobs));
        if (blobs.length > 0) {
            telemetry.addData("primary blob", blobs[0]);
            telemetry.addData("primary tx", blobs[0].tx);
            telemetry.addData("primary ty", blobs[0].ty);
            telemetry.addData("primary field position", MathUtils.format3(blobs[0].x) + ", " + MathUtils.format3(blobs[0].y));
        }
        else {
            telemetry.addData("primary blob", "null");
            telemetry.addData("primary tx", "null");
            telemetry.addData("primary ty", "null");
        }
    }

    public void addBallInfo(Canvas fieldOverlay) {
        if (params.drawBalls) {
            fieldOverlay.setStroke("gray");
            for (Blob blob : blobs)
                fieldOverlay.strokeCircle(blob.x, blob.y, 2.5);
        }
    }

    public Vector2d[] getShortestPath(int maxBlobsInPath) {
        Vector2d[] nodes = new Vector2d[blobs.length];
        for (int i=0; i<blobs.length; i++)
            nodes[i] = new Vector2d(blobs[i].x, blobs[i].y);
        return PathFinder.findShortestPath(robot.drive.localizer.getPose().position, nodes, maxBlobsInPath);
    }

    public DrivePath generateDrivePath(Vector2d[] path, boolean shouldUpdatePose) {
        Pose2d robotPose = robot.drive.localizer.getPose();
        DrivePath drivePath = new DrivePath(robot.drive);
        drivePath.setShouldUpdatePose(shouldUpdatePose);
        Vector2d cornerPosition = new Vector2d(72, BrainSTEMRobot.alliance == Alliance.RED ? 72 : -72);

        for (int i=0; i<path.length; i++) {
            Vector2d point = path[i];
            Vector2d prev = i > 0 ? drivePath.getWaypoint(i - 1).pose.position : robotPose.position;

            // constrain the robot to enter parallel to the field wall if the ball is in the corner
            double distFromCorner = Math.hypot(cornerPosition.x - point.x, cornerPosition.y - point.y);

            if (distFromCorner < autoCollectParams.cornerBallDistance) {
                // figure out which way to approach the ball
                Vector2d p1 = new Vector2d(point.x - 10, point.y);
                Vector2d p2 = new Vector2d(point.x, point.y + (BrainSTEMRobot.alliance == Alliance.RED ? -10 : 10));
                double d1 = Math.hypot(point.x - p1.x, point.y - p1.y) + Math.hypot(p1.x - prev.x, p1.y - prev.y);
                double d2 = Math.hypot(point.x - p2.x, point.y - p2.y) + Math.hypot(p2.x - prev.x, p2.y - prev.y);
                Vector2d setupPoint = d1 < d2 ? p1 : p2;
                double angle = Math.atan2(setupPoint.y - prev.y, setupPoint.x - prev.x);
                Waypoint w = new Waypoint(new Pose2d(setupPoint.x, setupPoint.y, angle))
                        .prioritizeHeadingInBeginning()
                        .setMinLinearPower(autoCollectParams.minLinearPower);
                drivePath.addWaypoint(w);
                prev = setupPoint;
            }

            double angle = Math.atan2(point.y - prev.y, point.x - prev.x);
            Vector2d position = getCollectPosition(point, angle);
            Waypoint waypoint = new Waypoint(new Pose2d(position.x, position.y, angle))
                    .prioritizeHeadingInBeginning()
                    .setMinLinearPower(autoCollectParams.minLinearPower);
            drivePath.addWaypoint(waypoint);
        }
        return drivePath;
    }
    private Vector2d getCollectPosition(Vector2d ballPosition, double angle) {
        double offsetAmount = 8;
        double dx = Math.cos(angle) * offsetAmount;
        double dy = Math.sin(angle) * offsetAmount;
        return new Vector2d(ballPosition.x - dx, ballPosition.y - dy);
    }
}
