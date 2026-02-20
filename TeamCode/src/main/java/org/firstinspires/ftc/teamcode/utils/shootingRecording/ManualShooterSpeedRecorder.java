package org.firstinspires.ftc.teamcode.utils.shootingRecording;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.pidDrive.MathUtils;

@Config
@TeleOp(name="Manual Shooter Speed Recorder", group="Data Recording")
public class ManualShooterSpeedRecorder extends OpMode {
    public static double recordIntervalMs = 20;
    public static int numDataEntries = 5;
    public static int numShotsToRecord = 30; // if you shoot 3 consecutively, it only counts as 1 shot - so this is 45 balls if you collect and shoot 3 everytime
    public static int recordAmountForEachShot = 150;
    public static double waitToDisplayTime = 0.2;
    public static double[][][] data = new double[numShotsToRecord][recordAmountForEachShot][numDataEntries];

    private GamepadTracker g1;

    public static void resetData() {
        data = new double[numShotsToRecord][recordAmountForEachShot][numDataEntries];
        currentShot = 0;
    }

    private static int currentShot = 0;
    public static int getCurrentShot() {
        return currentShot;
    }
    public static void incrementCurrentShot() {
        currentShot++;
    }
    private int currentShownShot = 0;
    private final ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);
    }

    @Override
    public void start() {
        timer.reset();
    }
    @Override
    public void loop() {
        g1.update();

        if (gamepad1.y)
            resetData();
        if (g1.isFirstDpadUp()) {
            currentShownShot++;
            timer.reset();
        }
        if (g1.isFirstDpadDown()) {
            currentShownShot--;
            timer.reset();
        }

        currentShownShot = Math.max(0, Math.min(currentShot - 1, currentShownShot));

        telemetry.addLine("===CONTROLS===");
        telemetry.addData("reset speeds", "Y");
        telemetry.addData("scroll through recorded shots", "dpad up/down");
        telemetry.addLine();
        telemetry.addLine("===HYPER PARAMS===");
        telemetry.addData("current shot index", currentShownShot);
        telemetry.addData("last recorded shot index", Math.min(currentShot - 1, numShotsToRecord));

        if (timer.seconds() > waitToDisplayTime) {
            telemetry.addLine();
            telemetry.addLine("===DATA (time, target spd, actual spd, power, hood) ===");
            double[][] shot = data[currentShownShot];
            for (double[] datum : shot)
                telemetry.addLine("t: " + MathUtils.format2(datum[0]) +
                        ", ts: " + MathUtils.format2(datum[1]) +
                        ", s: " + MathUtils.format2(datum[2]) +
                        ", p: " + MathUtils.format3(datum[3]) +
                        ", h: " + MathUtils.format3(datum[4]));
        }
        telemetry.update();
    }
}
