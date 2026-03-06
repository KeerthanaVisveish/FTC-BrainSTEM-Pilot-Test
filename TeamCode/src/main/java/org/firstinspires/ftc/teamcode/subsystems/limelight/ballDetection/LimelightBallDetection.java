package org.firstinspires.ftc.teamcode.subsystems.limelight.ballDetection;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LLParent;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.GeometryUtils;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class LimelightBallDetection extends LLParent {
    public static class Params {
        public int maxBlobs = 5;
        public int numPiecesOfInfoPerBlob = 4;
        public double minDistFromFieldWall = 2.5;
        public boolean projectBallsInsideField = true;
        public int numImagesPerSnapshot = 1;
        public double maxDistToCombineSnapshotBlobs = 1.5;
        public boolean showPythonOutputs = true;
        public boolean drawBalls = true;
        public double waitToScanAfterTurretMove = 2;
    }
    public static Params params = new Params();
    private double[] pythonOutputs;
    private int currentNumBlobs;
    private ArrayList<Blob> currentBlobs;
    private ArrayList<ArrayList<Blob>> previousSnapshots;
    private int numImagesLeft;
    public LimelightBallDetection(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        pythonOutputs = new double[3];
        currentBlobs = new ArrayList<>();
        previousSnapshots = new ArrayList<>();
        numImagesLeft = 0;
    }
    @Override
    public void update() {
        LLResult result = limelight.getLatestResult();
        pythonOutputs = result.getPythonOutput();
        robot.telemetry.addLine("UPDATING PYTHON OUTPUTS IN LIMELIGHT");

        int numNonZeroEntries = 0;
        while (numNonZeroEntries < pythonOutputs.length && pythonOutputs[numNonZeroEntries] != 0)
            numNonZeroEntries++;
        currentNumBlobs = numNonZeroEntries / params.numPiecesOfInfoPerBlob;
        if (currentNumBlobs > params.maxBlobs)
            currentNumBlobs = params.maxBlobs;

        currentBlobs = new ArrayList<>();
        for (int i = 0; i < currentNumBlobs; i++) {
            int index = i * params.numPiecesOfInfoPerBlob;
            double px = pythonOutputs[index];
            double py = pythonOutputs[index + 1];
            double area = pythonOutputs[index + 2];
            boolean isGiantClump = pythonOutputs[index + 3] > 0;
            currentBlobs.add(createBlob(px, py, area, isGiantClump));
        }
        if (numImagesLeft > 0) {
            numImagesLeft--;
            previousSnapshots.add(new ArrayList<>(currentBlobs));
        }
    }

    private Blob createBlob(double px, double py, double area, boolean isGiantClump) {
        double tx = Limelight.pixelXToTx(px);
        double ty = Limelight.pixelYToTy(py);
        Pose2d cameraPose = Limelight.getLimelightPose(robot.shootingSystem.turretPose);
        Vector2d fieldPosition = Limelight.calculateBallFieldPosition(cameraPose, tx, ty);
        if (params.projectBallsInsideField)
            fieldPosition = GeometryUtils.projectOntoField(robot.drive.localizer.getPose().position, fieldPosition, params.minDistFromFieldWall);

        return new Blob(tx, ty, fieldPosition.x, fieldPosition.y, area, isGiantClump);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        if (params.showPythonOutputs)
            telemetry.addData("python outputs", Arrays.toString(pythonOutputs));
        telemetry.addData("num blobs", currentNumBlobs);
        telemetry.addData("blobs", currentBlobs);
        if (!currentBlobs.isEmpty()) {
            telemetry.addData("primary blob", currentBlobs.get(0));
            telemetry.addData("primary tx", currentBlobs.get(0).tx);
            telemetry.addData("primary ty", currentBlobs.get(0).ty);
            telemetry.addData("primary field position", MathUtils.formatVec3(currentBlobs.get(0).pos()));
        }
        else {
            telemetry.addData("primary blob", "null");
            telemetry.addData("primary tx", "null");
            telemetry.addData("primary ty", "null");
        }
        telemetry.addData("giant clump", MathUtils.formatVec2(getGiantClumpPosition()));
    }
    public void addBallInfo(Canvas fieldOverlay) {
        if (params.drawBalls) {
            ArrayList<Vector2d> balls = new ArrayList<>();
            for (int i=0; i<currentBlobs.size(); i++)
                balls.add(currentBlobs.get(i).pos());
            drawBalls(fieldOverlay, balls);
        }
    }

    public void drawBalls(Canvas fieldOverlay, ArrayList<Vector2d> balls) {
        fieldOverlay.setFill("purple");
        for (Vector2d ball : balls)
            fieldOverlay.fillCircle(ball.x, ball.y, 2.5);
    }
    public void drawPath(Canvas fieldOverlay, Pose2d startPose, ArrayList<Pose2d> autoCollectPathPoses) {
        ArrayList<Pose2d> posesToDraw = new ArrayList<>(autoCollectPathPoses);
        posesToDraw.add(0, startPose);
        for (int i = 0; i < posesToDraw.size() - 1; i++) {
            Vector2d start = posesToDraw.get(i).position;
            Vector2d end = posesToDraw.get(i + 1).position;
            fieldOverlay.setStroke("black");
            fieldOverlay.strokeLine(start.x, start.y, end.x, end.y);
            if (i > 0) {
                fieldOverlay.setStroke("gray");
                Drawing.drawRobotSimple(fieldOverlay, posesToDraw.get(i), 3);
            }
        }
        if (!posesToDraw.isEmpty()) {
            fieldOverlay.setStroke("gray");
            Pose2d last = autoCollectPathPoses.get(autoCollectPathPoses.size() - 1);
            Drawing.drawRobotSimple(fieldOverlay, last, 3);
        }
    }
    public Vector2d getGiantClumpPosition(ArrayList<Blob> blobs) {
        for (Blob blob : currentBlobs)
            if (blob.isGiantClump)
                return blob.pos();
        return null;
    }

    public ArrayList<Blob> getCurrentBlobs() {
        return currentBlobs;
    }
    public ArrayList<Blob> getCombinedBlobs() {
        ArrayList<Blob> allBlobs = new ArrayList<>();
        for (ArrayList<Blob> snapshotBlobs : previousSnapshots)
            allBlobs.addAll(snapshotBlobs);

        ArrayList<Blob> combined = new ArrayList<>();
        ArrayList<Integer> indexesToSkip = new ArrayList<>();
        for (int i=0; i<allBlobs.size(); i++) {
            if (indexesToSkip.contains(i))
                continue;

            Blob blob = allBlobs.get(i);
            boolean merge = false;
            for (int j = 0; j < allBlobs.size(); j++) {
                if (i == j)
                    continue;

                Blob check = allBlobs.get(j);
                if (MathUtils.vecDist(blob.pos(), check.pos()) <= params.maxDistToCombineSnapshotBlobs) {
                    double newTx = (blob.tx + check.tx) * 0.5;
                    double newTy = (blob.ty + check.ty) * 0.5;
                    Vector2d newPos = blob.pos().plus(check.pos()).times(0.5);
                    double newArea = (blob.area + check.area) * 0.5;
                    boolean isGiantClump = blob.isGiantClump || check.isGiantClump;
                    combined.add(new Blob(newTx, newTy, newPos.x, newPos.y, newArea, isGiantClump));
                    indexesToSkip.add(j);
                    merge = true;
                    break;
                }
            }
            if (!merge) {
                combined.add(blob);
            }
        }
        return combined;
    }
    public ArrayList<Vector2d> getBlobPositions(ArrayList<Blob> blobs) {
        ArrayList<Vector2d> combinedPositions = new ArrayList<>();
        for (Blob blob : blobs)
            combinedPositions.add(blob.pos());
        return combinedPositions;
    }
    public Action clearBallSnapshots() {
        return new InstantAction(() -> {
            previousSnapshots.clear();
            numImagesLeft = 0;
        });
    }
    public Action takeBallSnapshotAction() {
        return new SequentialAction(
                new InstantAction(() -> numImagesLeft = params.numImagesPerSnapshot),
                packet -> numImagesLeft > 0
        );
    }
}
