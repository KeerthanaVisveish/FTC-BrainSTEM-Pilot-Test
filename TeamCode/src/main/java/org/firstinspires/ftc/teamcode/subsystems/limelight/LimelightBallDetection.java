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
import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;

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

    public static Params params = new Params();
    private double[] pythonOutputs;
    private int numBlobs;
    private Blob[] blobs;
    public LimelightBallDetection(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        pythonOutputs = new double[3];
        blobs = new Blob[0];
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
    public Blob[] getBlobs() {
        return blobs;
    }
}
