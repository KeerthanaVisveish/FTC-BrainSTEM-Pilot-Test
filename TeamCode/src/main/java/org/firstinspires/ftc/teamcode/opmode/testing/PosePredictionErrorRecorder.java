package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;

import java.util.ArrayList;

@Config
@TeleOp(name="Pose Prediction Error Recorder", group="TestingParams")
public class PosePredictionErrorRecorder extends OpMode {
    public static int currentMode = 0;
    public static boolean showError = false, showAcceleration = true, showVelocity = false;
    public static int numDeciPlaces = 4;
    // double[] format = time, x, y, heading
    public static final ArrayList<OdoInfo> acceleration = new ArrayList<>(), velocity = new ArrayList<>();
    public static final ArrayList<OdoInfo> predictionErrorsSimple = new ArrayList<>();
    public static final ArrayList<OdoInfo> predictionErrorsAdvanced = new ArrayList<>();
    public static final ArrayList<OdoInfo> controlGroupError = new ArrayList<>(); // control group
    private GamepadTracker g1;
    private int lastCurrentMode;
    public static void clearData() {
        acceleration.clear();
        velocity.clear();
        predictionErrorsSimple.clear();
        predictionErrorsAdvanced.clear();
        controlGroupError.clear();
    }
    @Override
    public void init() {
        g1 = new GamepadTracker(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lastCurrentMode = currentMode;
        printCurrentInfo();
    }

    @Override
    public void loop() {
        g1.update();

        if (g1.isFirstY())
            clearData();

        if (g1.isFirstB())
            currentMode = (currentMode + 1) % 4;
        else if (g1.isFirstX())
            currentMode = (currentMode + 3) % 4;

        if (currentMode != lastCurrentMode) {

            printCurrentInfo();

            telemetry.update();
        }
        lastCurrentMode = currentMode;
    }

    private void printCurrentInfo() {
        telemetry.addLine("===CONTROLS===");
        telemetry.addData("increment/decrement mode", "B/X");
        telemetry.addData("clear data", "Y");
        telemetry.addLine();
        telemetry.addData("available modes", "simple, advanced, comparison");
        telemetry.addLine();
        switch (currentMode) {
            case 0: telemetry.addLine("=== 0: CONTROL GROUP ERROR ========="); break;
            case 1: telemetry.addLine("=== 1: SIMPLE ESTIMATION ERROR ====="); break;
            case 2: telemetry.addLine("=== 2: ADVANCED ESTIMATION ERROR ==="); break;
            case 3: telemetry.addLine("=== 3: COMPARISON =================="); break;
        }
        telemetry.addLine();

        if (currentMode != 3) {
            ArrayList<OdoInfo> errorsToShow = controlGroupError;
            if (currentMode == 1)
                errorsToShow = predictionErrorsSimple;
            else if (currentMode == 2)
                errorsToShow = predictionErrorsAdvanced;

            for (int i = 0; i < errorsToShow.size(); i++) {
                String error = showError ? "x:" + MathUtils.format(errorsToShow.get(i).x, numDeciPlaces) +
                        "y:" + MathUtils.format(errorsToShow.get(i).y, numDeciPlaces) +
                        "h:" + MathUtils.format(errorsToShow.get(i).headingRad, numDeciPlaces) : "";
                String vel = showVelocity ? " |v" + velocity.get(i).toString(numDeciPlaces) : "";

                String accel = "";
                if (showAcceleration && i > 0)
                    accel = " |a" + acceleration.get(i - 1).toString(numDeciPlaces);
                telemetry.addLine(error + vel + accel);
            }
        }
        else {
            OdoInfo avgErrorControl = calculateAverageError(controlGroupError);
            OdoInfo avgErrorSimple = calculateAverageError(predictionErrorsSimple);
            OdoInfo avgErrorAdvanced = calculateAverageError(predictionErrorsAdvanced);
            OdoInfo maxLinearErrorControl = calculateMaxLinearError(controlGroupError);
            OdoInfo maxHeadingErrorControl = calculateMaxHeadingError(controlGroupError);
            OdoInfo maxLinearErrorSimple = calculateMaxLinearError(predictionErrorsSimple);
            OdoInfo maxHeadingErrorSimple = calculateMaxHeadingError(predictionErrorsSimple);
            OdoInfo maxLinearErrorAdvanced = calculateMaxLinearError(predictionErrorsAdvanced);
            OdoInfo maxHeadingErrorAdvanced = calculateMaxHeadingError(predictionErrorsAdvanced);

            OdoInfo avgStdControl = calculateAvgStandardDev(controlGroupError);
            OdoInfo avgStdSimple = calculateAvgStandardDev(predictionErrorsSimple);
            OdoInfo avgStdAdvanced = calculateAvgStandardDev(predictionErrorsAdvanced);

            telemetry.addLine("x error, y error, heading error");
            telemetry.addLine("average error control group: " + avgErrorControl.toString(numDeciPlaces));
            telemetry.addLine("average error simple method: " + avgErrorSimple.toString(numDeciPlaces));
            telemetry.addLine("average error advanced method: " + avgErrorAdvanced.toString(numDeciPlaces));
            telemetry.addLine();
            telemetry.addLine("max error control group: " + maxLinearErrorControl.toStringPosition(numDeciPlaces)  + " " + maxHeadingErrorControl.toStringHeading(numDeciPlaces));
            telemetry.addLine("max error simple method: " + maxLinearErrorSimple.toStringPosition(numDeciPlaces)  + " " + maxHeadingErrorSimple.toStringHeading(numDeciPlaces));
            telemetry.addLine("max error advanced method: " + maxLinearErrorAdvanced.toStringPosition(numDeciPlaces)  + " " + maxHeadingErrorAdvanced.toStringHeading(numDeciPlaces));
            telemetry.addLine();
            telemetry.addLine("average standard dev control group: " + avgStdControl.toString(numDeciPlaces));
            telemetry.addLine("average standard dev simple method: " + avgStdSimple.toString(numDeciPlaces));
            telemetry.addLine("average standard dev advanced method: " + avgStdAdvanced.toString(numDeciPlaces));
        }
    }

    private OdoInfo calculateAverageError(ArrayList<OdoInfo> errors) {
        if (errors.isEmpty())
            return new OdoInfo(0, 0, 0);
        OdoInfo avgErrors = new OdoInfo();
        for (int i=1; i<errors.size(); i++) {
            avgErrors.x += Math.abs(errors.get(i).x);
            avgErrors.y += Math.abs(errors.get(i).y);
            avgErrors.headingRad += Math.abs(errors.get(i).headingRad);
        }
        avgErrors.x /= errors.size();
        avgErrors.y /= errors.size();
        avgErrors.headingRad /= errors.size();
        return avgErrors;
    }
    // variance = sum of the squared differences from the mean
    // standard dev = sqrt(variance)
    private OdoInfo calculateAvgStandardDev(ArrayList<OdoInfo> errors) {
        if (errors.isEmpty())
            return new OdoInfo(0, 0, 0);
        OdoInfo avgErrors = calculateAverageError(errors);
        OdoInfo stds = new OdoInfo();

        for (int i=1; i<errors.size(); i++) { // exclude first column b/c first column is time
            double xDiff = Math.abs(errors.get(i).x) - avgErrors.x;
            stds.x += xDiff * xDiff;

            double yDiff = Math.abs(errors.get(i).y) - avgErrors.y;
            stds.y += yDiff * yDiff;

            double headingRadDiff = Math.abs(errors.get(i).headingRad) - avgErrors.headingRad;
            stds.headingRad += headingRadDiff * headingRadDiff;
        }
        stds.x /= errors.size();
        stds.y /= errors.size();
        stds.headingRad /= errors.size();
        stds.x = Math.sqrt(stds.x);
        stds.y = Math.sqrt(stds.y);
        stds.headingRad = Math.sqrt(stds.headingRad);
        return stds;
    }
    private OdoInfo calculateMaxLinearError(ArrayList<OdoInfo> errors) {
        if (errors.isEmpty())
            return new OdoInfo(0, 0, 0);
        OdoInfo max = errors.get(0);
        for (OdoInfo info : errors)
            if (info.x * info.x + info.y * info.y > max.x * max.x + max.y * max.y)
                max = info;
        return max;
    }
    private OdoInfo calculateMaxHeadingError(ArrayList<OdoInfo> errors) {
        if (errors.isEmpty())
            return new OdoInfo(0, 0, 0);
        OdoInfo max = errors.get(0);
        for (OdoInfo info : errors)
            if (Math.abs(info.headingRad) > Math.abs(max.headingRad))
                max = info;
        return max;
    }
}
