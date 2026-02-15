package org.firstinspires.ftc.teamcode.subsystems.limelight;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PathFinder;
import org.firstinspires.ftc.teamcode.utils.math.Vec;
import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;

import java.util.Arrays;

@Config
public class LimelightBallDetection extends LLParent {
    public static class Params {
        public int maxBlobs = 5;
        public int numPiecesOfInfoPerBlob = 3;
        public boolean showPythonOutputs = false;
        public double minDistFromFieldWall = 2.5;
    }

    public static Params params = new Params();
    private double[] pythonOutputs;
    private int numBlobs;
    private Blob[] blobs;
    public LimelightBallDetection(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        pythonOutputs = new double[3];
    }

    private static class Blob {
        public final double x, y, area;
        public Blob(double x, double y, double area) {
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
        Vector2d projectedFieldPosition = GeometryUtils.projectOntoField(robot.drive.localizer.getPose().position, fieldPosition, params.minDistFromFieldWall);

        return new Blob(projectedFieldPosition.x, projectedFieldPosition.y, area);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        if (params.showPythonOutputs)
            telemetry.addData("python outputs", Arrays.toString(pythonOutputs));
        telemetry.addData("num blobs", numBlobs);
        telemetry.addData("blobs", Arrays.toString(blobs));
        if (blobs.length > 0) {
            telemetry.addData("primary blob", blobs[0]);
            telemetry.addData("primary field position", MathUtils.format3(blobs[0].x) + ", " + MathUtils.format3(blobs[0].y));
        }
        else {
            telemetry.addData("primary blob", "null");
            telemetry.addData("primary tx", "null");
            telemetry.addData("primary ty", "null");
        }
    }

    public void addBallInfo(Canvas fieldOverlay) {
        fieldOverlay.setStroke("gray");
        for (Blob blob : blobs) {
            fieldOverlay.strokeCircle(blob.x, blob.y, 2.5);
        }
    }

    public Vector2d getBlobPosition(int i) {
        if (i >= blobs.length)
            return null;
        return new Vector2d(blobs[i].x, blobs[i].y);
    }

    public Vector2d[] getShortestPath(int maxBlobsInPath) {
        Vector2d[] nodes = new Vector2d[blobs.length];
        for (int i=0; i<blobs.length; i++)
            nodes[i] = new Vector2d(blobs[i].x, blobs[i].y);
        return PathFinder.findShortestPath(robot.drive.localizer.getPose().position, nodes, maxBlobsInPath);
    }
}
