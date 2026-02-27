package org.firstinspires.ftc.teamcode.utils.shootingRecording;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

import java.util.ArrayList;

//@Config
//@TeleOp(name="Automatic Shooter Speed Recorder", group="Data Recording")
public class AutomaticShooterSpeedRecorder extends OpMode {
    public static class ShowDataOptions {
        public boolean showTime = false;
        public boolean showNumBallsShot = true;
        public boolean showShooterVel = true;
        public boolean showShooterPower = false;
        public boolean showHood = false;
        public boolean showTurret = false;
    }
    public static ShowDataOptions showDataOptions = new ShowDataOptions();
    public static int bigScrollAmount = 4;
    public static int shotBufferFrames = 10;
    private static final ArrayList<ShotData> rawData = new ArrayList<>();
    private static final ArrayList<ArrayList<ShotData>> data = new ArrayList<>();

    public static void resetData() {
        rawData.clear();
        data.clear();
    }
    public static void addShotData(ShotData shot) {
        rawData.add(shot);
    }
    private static void parseRawData() {
        data.clear();
        ArrayList<Integer> shotInts = new ArrayList<>();
        for (int i=1; i<rawData.size(); i++)
            if (rawData.get(i).numBallsShot > rawData.get(i - 1).numBallsShot)
                shotInts.add(i);

        for (Integer shotInt : shotInts) {
            ArrayList<ShotData> shotDataList = new ArrayList<>();
            for (int i=Math.max(0, shotInt - shotBufferFrames); i<Math.min(rawData.size(), shotInt + shotBufferFrames); i++)
                shotDataList.add(rawData.get(i));
            data.add(shotDataList);
        }
    }
    private GamepadTracker g1;
    private int currentShownShot = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);
    }

    @Override
    public void start() {
        parseRawData();
        updateTelemetry();
    }

    @Override
    public void loop() {
        int oldCurrentShownShot = currentShownShot;
        g1.update();

        if (gamepad1.y)
            resetData();

        if (!data.isEmpty()) {
            if (g1.isFirstDpadUp())
                currentShownShot++;
            else if (g1.isFirstDpadDown())
                currentShownShot--;
            else if (g1.isFirstDpadRight())
                currentShownShot += bigScrollAmount;
            else if (g1.isFirstDpadLeft())
                currentShownShot -= bigScrollAmount;

            currentShownShot = (currentShownShot + data.size()) % data.size(); // wrap around
        }

        boolean newShot = currentShownShot != oldCurrentShownShot;
        if (newShot)
            updateTelemetry();
    }
    private void updateTelemetry() {
        telemetry.addLine("===CONTROLS===");
        telemetry.addData("parse raw data", "A");
        telemetry.addData("reset speeds", "Y");
        telemetry.addData("scroll through recorded shots", "dpad up/down");
        telemetry.addData("big scroll through recorded shots", "dpad right/left");
        telemetry.addLine();
        telemetry.addLine("===HYPER PARAMS===");
        telemetry.addData("current shot index", currentShownShot);
        telemetry.addData("num shots recorded", data.size());
        telemetry.addLine();
        telemetry.addLine("===DATA (time, target spd, actual spd, power, hood) ===");

        ArrayList<ShotData> shot = data.get(currentShownShot);
        for (ShotData shotData : shot) {
            String time = showDataOptions.showTime ? "tm:" + MathUtils.format3(shotData.timestamp) + "| " : "";
            String numBalls = showDataOptions.showNumBallsShot ? "bs: " + shotData.numBallsShot + "| " : "";
            String shooterVel = showDataOptions.showShooterVel ?
                    "spd: " + MathUtils.format1(shotData.avgVel) + " ttSpd: " + MathUtils.format1(shotData.theoreticalTargetVel) +
                    " attSpd: " + MathUtils.format1(shotData.adjustedTargetVel) + "| " : "";
            String shooterPow = showDataOptions.showShooterPower ? "pwr: " + MathUtils.format3(shotData.motorPower) + "| " : "";
            String hood = "angle: " + MathUtils.format3(shotData.ballExitAngleDeg);
            if (shotData.ballExitAngleDeg > ShootingMath.hoodSystemParams.maxAngleDeg || shotData.ballExitAngleDeg < ShootingMath.hoodSystemParams.minAngleDeg)
                    hood += ", OUT OF HOOD RANGE";
            hood += "| ";
            if (!showDataOptions.showHood)
                hood = "";
            String turret = showDataOptions.showTurret ? "trrt: " + shotData.turretEncoder + ", " + shotData.targetTurretEncoder : "";

            telemetry.addLine(time + numBalls + shooterVel + shooterPow + hood + turret);
        }

        telemetry.update();
    }
}
